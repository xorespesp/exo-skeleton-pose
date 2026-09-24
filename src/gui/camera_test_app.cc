#include "camera_test_app.hh"

#include "frame_texture.hh"
#include "log_console.hh"
#include "app_config.hh"
#include "hw/device_enumeration.hh"
#include "hw/sensor_frame_provider.hh"
#include "io/frame_recorder.hh"
#include "pose/marker_tracker.hh"
#include "pose/synced_tracker.hh"
#include "pose/tag_detector.hh"
#include <imfilebrowser.h>
#include <implot.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <format>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

// 동기화 검증용 임시 하네스. 핵심 모듈(provider, synchronizer, synced_tracker, frame_recorder, 장치
// 열거)을 그대로 쓰기만 하고, 여기에만 있는 로직은 UI 뿐이다. 검증이 끝나면 통째로 걷어 낸다.
namespace gui
{
    namespace
    {
        constexpr std::size_t kMaxCameraRows = 4;
        constexpr std::size_t kSkewHistoryLength = 256;
        constexpr double kTagSizeM = 0.05; // 검출 수와 시간만 보므로 포즈 스케일은 아무 값이어도 된다

        // 코덱 콤보 항목. `io::kImageCodecs` 와 같은 순서.
        constexpr std::array<const char*, io::kImageCodecs.size()> kCodecLabels{
            "JPEG (compressed)",
            "Raw (lossless)",
        };

        float to_ms(const std::chrono::nanoseconds ns)
        {
            return std::chrono::duration<float, std::milli>{ ns }.count();
        }

        // 녹화 파일 이름에 넣을 지역 시각.
        std::string local_stamp()
        {
            const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
            try {
                return std::format("{:%y%m%d%H%M%S}", std::chrono::zoned_time{ std::chrono::current_zone(), now });
            }
            catch (const std::exception&) {
                return std::format("{:%y%m%d%H%M%S}", now); // 시간대 DB 가 없으면 UTC
            }
        }

        // 워커가 슬롯별 스냅샷 하나와 최근 Δt 이력을 발행하고, GUI 가 텍스처 작업을 전부 소유한다.
        // 검출 테스트가 켜져 있으면 frameset 을 통째로 `synced_tracker` 에도 넘긴다.
        class preview_observer final : public hw::synced_frameset_observer
        {
        public:
            struct snapshot_t
            {
                std::vector<std::shared_ptr<hw::sensor_frame>> frames; // 슬롯별
                std::vector<float> pair_skew_ms;                       // 최근 frameset 들의 max_pair_skew. 오래된 것부터
                std::optional<hw::stream_end_reason_t> end;
            };

            snapshot_t snapshot() const
            {
                std::scoped_lock lock{ _mutex };
                snapshot_t copy;
                copy.frames = _frames;
                copy.pair_skew_ms.assign(_pair_skew_ms.begin(), _pair_skew_ms.end());
                copy.end = _end;
                return copy;
            }

            void set_synced_tracker(std::shared_ptr<pose::synced_tracker_base> synced_tracker)
            {
                std::scoped_lock lock{ _mutex };
                _synced_tracker = std::move(synced_tracker);
            }

            void on_synced_frameset_update(const hw::synced_frameset& frameset) override
            {
                std::shared_ptr<pose::synced_tracker_base> synced_tracker;
                {
                    std::scoped_lock lock{ _mutex };
                    _frames.assign(frameset.stream_count(), nullptr);
                    for (std::size_t stream_idx = 0; stream_idx < frameset.stream_count(); ++stream_idx)
                    {
                        if (const hw::sensor_frameset* capture = frameset.stream_frameset(stream_idx)) {
                            _frames[stream_idx] = capture->frame();
                        }
                    }
                    _end.reset();

                    // 스트림이 둘 이상 기여한 frameset 만 스큐를 말한다.
                    if (frameset.present_stream_count() >= 2)
                    {
                        _pair_skew_ms.push_back(to_ms(frameset.max_pair_skew()));
                        if (_pair_skew_ms.size() > kSkewHistoryLength) { _pair_skew_ms.pop_front(); }
                    }
                    synced_tracker = _synced_tracker;
                }

                // `synced_tracker` 에 넘기는 것은 락 밖에서. submit 은 막히지 않는다.
                if (synced_tracker) { synced_tracker->submit(frameset); }
            }

            void on_sensor_stream_reset() override
            {
                std::scoped_lock lock{ _mutex };
                _frames.clear();
                _pair_skew_ms.clear();
                _end.reset();
            }

            void on_sensor_stream_end(const hw::stream_end_reason_t reason) override
            {
                std::scoped_lock lock{ _mutex };
                _end = reason;
            }

        private:
            mutable std::mutex _mutex;
            std::vector<std::shared_ptr<hw::sensor_frame>> _frames;
            std::deque<float> _pair_skew_ms;
            std::optional<hw::stream_end_reason_t> _end;
            std::shared_ptr<pose::synced_tracker_base> _synced_tracker;
        };

        // 카메라 한 대를 어떻게 열지. 하네스 UI 의 행 하나.
        struct camera_row_t
        {
            int device_choice{ -1 }; // 열거 목록의 인덱스. -1: 인덱스로 직접
            int device_index{ 0 };
            int format_index{ 0 };   // 0 = BGR8, 1 = GRAY8
            bool manual_exposure{ false };
            int exposure_us{ 8000 };
            bool manual_gain{ false };
            int gain{ 0 };
            bool manual_frame_rate{ true }; // VZ 만. 꺼져 있으면 카메라 상한으로 프리런
            float frame_rate_fps{ 30.0f };
        };

        // 스트림 하나의 ROI 편집 칸. x, y, width, height.
        struct roi_editor_t
        {
            int fields[4]{ 0, 0, 0, 0 };
        };
    }

    struct camera_test_app::context_t
    {
        log_console console;
        std::shared_ptr<spdlog::logger> logger{ spdlog::default_logger() };
        std::shared_ptr<preview_observer> observer;
        std::unique_ptr<hw::sensor_frame_provider> provider;

        // 슬롯별 표시 상태. 소스를 열 때 스트림 수만큼 잡는다.
        std::vector<std::unique_ptr<frame_texture>> textures;
        std::vector<std::optional<uint64_t>> displayed_frame_ids;
        std::vector<hw::timestamp_t> displayed_timestamps;
        std::vector<roi_editor_t> roi_editors;

        // AprilTag 검출 테스트. 스트림마다 트래커 하나를 `synced_tracker` 가 frameset 단위로 돌린다.
        bool apriltag_test{ false };
        float tag_quad_decimate{ 2.0f };
        std::shared_ptr<pose::synced_tracker_base> synced_tracker;

        // 녹화. 열린 스트림 전부를 파일 하나에 담는다.
        std::shared_ptr<io::frame_recorder> recorder; // 녹화 중에만 non-null
        int record_codec{ 0 }; // kCodecLabels 인덱스
        int record_jpeg_quality{ 90 };

        ImGui::FileBrowser browser;
        std::filesystem::path recording;
        int source_mode{ 0 }; // 0 = 카메라, 1 = MCAP 녹화
        std::vector<camera_row_t> camera_rows{ camera_row_t{} };

        // 한 번에 여는 카메라들은 같은 백엔드다. 목록도 그 백엔드의 것 하나다.
        int backend_kind{ 0 }; // 0 = K4A, 1 = VZ
        std::vector<hw::device_info_t> devices;
        bool devices_enumerated{ false };
        int reference_stream_idx{ 0 };
        float max_pair_skew_ms{ 0.0f }; // 0: 기준 스트림 간격의 절반으로 자동
        bool playback{ false };
        std::string error;

        context_t()
        {
            browser.SetTitle("Open camera recording");
            browser.SetTypeFilters({ ".mcap" });
            console.sink()->set_level(spdlog::level::trace);
            logger->sinks().push_back(console.sink());
        }

        ~context_t()
        {
            close();
            std::erase(logger->sinks(), console.sink());
        }

        void stop_apriltag_test()
        {
            if (observer) { observer->set_synced_tracker(nullptr); }
            synced_tracker.reset(); // 그 스레드와 풀이 join 된다
            apriltag_test = false;
        }

        void start_apriltag_test()
        {
            if (!provider || !observer) { return; }
            pose::tag_detector::options_t options;
            options.quad_decimate = std::max(1.0f, tag_quad_decimate);

            std::vector<pose::synced_tracker_base::stream_entry_t> stream_entries;
            for (std::size_t stream_idx = 0; stream_idx < provider->stream_count(); ++stream_idx) {
                stream_entries.push_back({
                    .tracker = std::make_shared<pose::apriltag_tracker>(options, kTagSizeM, std::nullopt),
                    .view = pose::camera_view_t::frontal,
                });
            }
            // 검출 수와 시간만 보므로 측정치의 타입도 뷰도 아무 쪽이어도 된다.
            synced_tracker = std::make_shared<pose::synced_tracker<pose::joint_2d_measurement_t>>(
                std::move(stream_entries), true);
            observer->set_synced_tracker(synced_tracker);
            apriltag_test = true;
        }

        void stop_recording()
        {
            if (!recorder) { return; }

            if (provider) { provider->remove_observer(recorder); }
            recorder->stop();

            const io::recording_stats_t stats = recorder->stats();
            spdlog::info("camera-test: recording stopped ({} frames over {:.1f} s, {} dropped, {:.1f} MB)"
                , stats.frames_written
                , std::chrono::duration<double>{ stats.duration }.count()
                , stats.frames_dropped
                , static_cast<double>(stats.file_bytes) / (1024.0 * 1024.0)
            );
            recorder.reset();
        }

        void start_recording()
        {
            error.clear();
            if (!provider || recorder) { return; }
            if (playback) {
                error = "A playback source is not recorded.";
                return;
            }

            // 출처는 descriptor 가 말한다. 노출·게인은 그 스트림을 연 카메라 행의 수동값이고, auto 로 둔 것은
            // 비워 둔다.
            std::vector<io::camera_stream_info_t> stream_infos;
            for (std::size_t stream_idx = 0; stream_idx < provider->stream_count(); ++stream_idx)
            {
                const hw::stream_descriptor_t descriptor = provider->get_stream_descriptor(stream_idx);
                const camera_row_t* row = stream_idx < camera_rows.size() ? &camera_rows[stream_idx] : nullptr;
                stream_infos.push_back(io::camera_stream_info_t{
                    .calibration = provider->get_calibration(stream_idx),
                    .color_format = provider->get_frame_format(stream_idx),
                    .sensor_backend = descriptor.sensor_backend,
                    .device_serial = descriptor.device_serial,
                    .exposure_us = (row && row->manual_exposure) ? std::optional<double>{ row->exposure_us } : std::nullopt,
                    .gain = (row && row->manual_gain) ? std::optional<double>{ row->gain } : std::nullopt,
                });
            }

            const std::size_t codec_idx = std::clamp<std::size_t>(
                static_cast<std::size_t>(record_codec), 0, io::kImageCodecs.size() - 1);
            const io::recording_options_t options{
                .codec = io::kImageCodecs[codec_idx].codec,
                .encode = { .jpeg_quality = record_jpeg_quality },
            };

            std::error_code ec;
            const std::filesystem::path dir = app::project_dir("recordings");
            std::filesystem::create_directories(dir, ec); // 실패는 아래 start 가 알린다
            const std::filesystem::path path = dir / std::format("camera-test-{}.mcap", local_stamp());
            if (std::filesystem::exists(path, ec)) {
                // 이름이 초 단위라, 방금 닫은 파일을 같은 초 안에 다시 열면 덮어쓰게 된다.
                error = "A recording of this second already exists; try again.";
                return;
            }

            // 관찰자로 붙이기 전에 시작해 두어, 처음 보는 frameset 부터 쓸 수 있게 한다.
            auto next_recorder = std::make_shared<io::frame_recorder>(options);
            if (!next_recorder->start(path, stream_infos)) {
                error = "Could not start the recording. See the log for details.";
                return;
            }
            provider->add_observer(next_recorder);
            recorder = std::move(next_recorder);
            spdlog::info("camera-test: recording {} stream(s) to '{}'", stream_infos.size(), path.string());
        }

        void close()
        {
            stop_recording();
            stop_apriltag_test();
            // 캡처 워커를 먼저 join 하고 그 관찰자와 GUI 로그 싱크를 놓는다.
            provider.reset();
            observer.reset();
            textures.clear();
            displayed_frame_ids.clear();
            displayed_timestamps.clear();
            roi_editors.clear();
            playback = false;
        }

        void refresh_devices()
        {
            const auto backend = (backend_kind == 1) ? hw::sensor_backend_t::vz : hw::sensor_backend_t::k4a;
            devices = hw::enumerate_devices(backend);
            devices_enumerated = true;
            spdlog::info("camera-test: found {} {} device(s)", devices.size(), hw::sensor_backend_to_str(backend));

            // 아직 고르지 않은 행에는 열거 순서대로 하나씩 배정한다. 행 둘이 같은 장치를 가리키는 기본
            // 상태를 피하기 위한 것이고, 이미 고른 행은 건드리지 않는다.
            std::size_t next = 0;
            for (camera_row_t& row : camera_rows)
            {
                if (row.device_choice < 0 && next < devices.size()) { row.device_choice = static_cast<int>(next); }
                ++next;
            }
        }

        hw::source_config_t make_camera_config(const camera_row_t& row) const
        {
            const auto format = row.format_index == 0 ? hw::frame_format_t::bgr8 : hw::frame_format_t::gray8;

            // 열거 목록에서 고른 장치는 시리얼로 고정한다. 고르지 않은 행은 손으로 적은 인덱스로 연다.
            hw::device_selector_t device_selector = hw::device_index_t{ static_cast<uint32_t>(std::max(0, row.device_index)) };
            if (row.device_choice >= 0 && row.device_choice < static_cast<int>(devices.size())) {
                device_selector = hw::device_serial_t{ devices[row.device_choice].device_serial };
            }

            if (backend_kind == 1)
            {
                hw::vz_device_config_t camera;
                camera.device_selector = device_selector;
                camera.frame_format = format;
                if (row.manual_exposure) { camera.exposure_us = row.exposure_us; }
                if (row.manual_gain) { camera.gain = row.gain; }
                if (row.manual_frame_rate) { camera.frame_rate_fps = row.frame_rate_fps; }
                return camera;
            }

            hw::k4a_device_config_t camera;
            camera.device_selector = device_selector;
            camera.frame_format = format;
            if (row.manual_exposure) { camera.exposure_us = row.exposure_us; }
            if (row.manual_gain) { camera.gain = row.gain; }
            return camera;
        }

        void load_roi_editor(const std::size_t stream_idx)
        {
            if (!provider || stream_idx >= roi_editors.size()) { return; }
            const std::optional<hw::roi_t> roi = provider->get_effective_roi(stream_idx);
            const Eigen::Vector2i full = provider->get_full_frame_resolution(stream_idx);
            int* fields = roi_editors[stream_idx].fields;
            fields[0] = roi ? roi->x : 0;
            fields[1] = roi ? roi->y : 0;
            fields[2] = roi ? roi->width : full.x();
            fields[3] = roi ? roi->height : full.y();
        }

        void open()
        {
            error.clear();

            std::vector<hw::source_config_t> configs;
            if (source_mode == 1)
            {
                if (recording.empty()) {
                    error = "Choose an MCAP recording.";
                    return;
                }
                configs.push_back(hw::recording_config_t{ .file = recording });
            }
            else
            {
                for (const camera_row_t& row : camera_rows) { configs.push_back(make_camera_config(row)); }

                // 같은 장치를 두 행이 가리키면 두 번째 열기가 SDK 에서 막힌다. 여기서 먼저 잡아 준다.
                for (std::size_t a = 0; a < configs.size(); ++a)
                {
                    for (std::size_t b = a + 1; b < configs.size(); ++b)
                    {
                        if (hw::describe(configs[a]) == hw::describe(configs[b])) {
                            error = std::format("Camera {} and {} point at the same device ({}).", a, b, hw::describe(configs[a]));
                            return;
                        }
                    }
                }
            }

            auto next_observer = std::make_shared<preview_observer>();
            auto next_provider = std::make_unique<hw::sensor_frame_provider>();
            next_provider->set_auto_repeat(false);
            next_provider->add_observer(next_observer);

            bool opened = false;
            if (configs.size() == 1)
            {
                opened = next_provider->open(configs.front());
            }
            else
            {
                hw::sync_options_t options;
                options.reference_stream_idx = static_cast<std::size_t>(
                    std::clamp(reference_stream_idx, 0, static_cast<int>(configs.size()) - 1));
                if (max_pair_skew_ms > 0.0f) {
                    options.max_pair_skew = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<float, std::milli>{ max_pair_skew_ms });
                }
                opened = next_provider->open_synced(configs, options);
            }
            if (!opened) {
                error = "Could not open the source. See the log for details.";
                return;
            }

            observer = std::move(next_observer);
            provider = std::move(next_provider);
            playback = source_mode == 1;

            const std::size_t stream_count = provider->stream_count();
            textures.clear();
            textures.resize(stream_count); // 첫 프레임에서 렌더러와 함께 만든다
            displayed_frame_ids.assign(stream_count, std::nullopt);
            displayed_timestamps.assign(stream_count, hw::timestamp_t{});
            roi_editors.assign(stream_count, roi_editor_t{});
            for (std::size_t stream_idx = 0; stream_idx < stream_count; ++stream_idx) { load_roi_editor(stream_idx); }

            spdlog::info("camera-test: opened {} ({} stream(s))", provider->get_source_name(), stream_count);
        }

        // 행이 지워졌으면 true.
        bool draw_camera_row(const std::size_t row_idx)
        {
            camera_row_t& row = camera_rows[row_idx];
            bool removed = false;

            ImGui::PushID(static_cast<int>(row_idx));
            const std::string header = std::format("Camera {}", row_idx);
            if (ImGui::CollapsingHeader(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                const bool chosen = row.device_choice >= 0 && row.device_choice < static_cast<int>(devices.size());
                const std::string preview = chosen ? devices[row.device_choice].display_name : std::string{ "(by index)" };
                if (ImGui::BeginCombo("Device", preview.c_str()))
                {
                    if (ImGui::Selectable("(by index)", !chosen)) { row.device_choice = -1; }
                    for (int choice = 0; choice < static_cast<int>(devices.size()); ++choice)
                    {
                        if (ImGui::Selectable(devices[choice].display_name.c_str(), row.device_choice == choice)) {
                            row.device_choice = choice;
                        }
                    }
                    ImGui::EndCombo();
                }
                if (!chosen)
                {
                    ImGui::InputInt("Device index", &row.device_index);
                    row.device_index = std::max(0, row.device_index);
                }

                ImGui::Combo("Pixel format", &row.format_index, "BGR8\0GRAY8\0");
                ImGui::Checkbox("Manual exposure", &row.manual_exposure);
                ImGui::BeginDisabled(!row.manual_exposure);
                ImGui::InputInt("Exposure (us)", &row.exposure_us);
                row.exposure_us = std::max(1, row.exposure_us);
                ImGui::EndDisabled();
                ImGui::Checkbox("Manual gain", &row.manual_gain);
                ImGui::BeginDisabled(!row.manual_gain);
                ImGui::InputInt("Gain", &row.gain);
                ImGui::EndDisabled();
                if (backend_kind == 1)
                {
                    ImGui::Checkbox("Manual frame rate", &row.manual_frame_rate);
                    ImGui::BeginDisabled(!row.manual_frame_rate);
                    ImGui::InputFloat("Frame rate (fps)", &row.frame_rate_fps);
                    row.frame_rate_fps = std::max(1.0f, row.frame_rate_fps);
                    ImGui::EndDisabled();
                }

                if (camera_rows.size() > 1 && ImGui::Button("Remove camera")) { removed = true; }
            }
            ImGui::PopID();
            return removed;
        }

        void draw_stream_panel(const std::size_t stream_idx)
        {
            ImGui::PushID(static_cast<int>(1000 + stream_idx));

            const hw::stream_descriptor_t descriptor = provider->get_stream_descriptor(stream_idx);
            const Eigen::Vector2i resolution = provider->get_frame_resolution(stream_idx);
            const Eigen::Vector2i full = provider->get_full_frame_resolution(stream_idx);
            const auto format = hw::frame_format_to_str(provider->get_frame_format(stream_idx));

            ImGui::Separator();
            const std::string_view backend = hw::sensor_backend_to_str(descriptor.sensor_backend);
            ImGui::Text("Stream %zu | %.*s", stream_idx, static_cast<int>(backend.size()), backend.data());
            if (!descriptor.device_serial.empty()) { ImGui::Text("S/N %s", descriptor.device_serial.c_str()); }
            ImGui::Text("%d x %d of %d x %d | %.*s", resolution.x(), resolution.y(), full.x(), full.y(),
                static_cast<int>(format.size()), format.data());
            ImGui::Text("Delivered: %llu | %.1f fps"
                , static_cast<unsigned long long>(provider->get_frames_delivered(stream_idx))
                , provider->get_current_update_rate(stream_idx));
            if (stream_idx < displayed_frame_ids.size() && displayed_frame_ids[stream_idx])
            {
                const auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    displayed_timestamps[stream_idx].time_since_epoch()).count();
                ImGui::Text("Displayed timestamp (us): %lld", static_cast<long long>(timestamp_us));
            }

            const std::optional<hw::roi_t> roi = provider->get_effective_roi(stream_idx);
            ImGui::Text("ROI: %s", roi
                ? std::format("{}x{}+{}+{}", roi->width, roi->height, roi->x, roi->y).c_str()
                : "whole frame");
            if (stream_idx < roi_editors.size())
            {
                // 녹화 파일은 스트림마다 프레임 크기를 한 번만 선언하므로 녹화 중에는 ROI 를 못 움직인다.
                const bool recording = recorder != nullptr;
                ImGui::BeginDisabled(recording);
                ImGui::InputInt4("x y w h", roi_editors[stream_idx].fields);
                const int* fields = roi_editors[stream_idx].fields;
                if (ImGui::Button("Apply ROI") && !recording) {
                    provider->set_roi(stream_idx, hw::roi_t{ fields[0], fields[1], fields[2], fields[3] });
                }
                ImGui::SameLine();
                if (ImGui::Button("Whole frame") && !recording) {
                    provider->set_roi(stream_idx, std::nullopt);
                }
                ImGui::EndDisabled();
                if (recording) { ImGui::TextUnformatted("ROI is locked while recording"); }
            }

            ImGui::PopID();
        }

        void draw_controls()
        {
            ImGui::TextUnformatted("Camera input");
            ImGui::BeginDisabled(provider != nullptr);

            ImGui::RadioButton("Cameras", &source_mode, 0);
            ImGui::SameLine();
            ImGui::RadioButton("MCAP recording", &source_mode, 1);

            bool wants_vz = false;
            if (source_mode == 1)
            {
                if (ImGui::Button("Choose recording...")) { browser.Open(); }
                ImGui::TextWrapped("%s", recording.empty() ? "No recording selected" : recording.string().c_str());
            }
            else
            {
                // 한 번에 여는 카메라들은 같은 백엔드다. 바꾸면 목록도 고른 것도 그 백엔드의 것이 아니다.
                if (ImGui::Combo("Backend", &backend_kind, "K4A\0VZ\0"))
                {
                    devices.clear();
                    devices_enumerated = false;
                    for (camera_row_t& row : camera_rows) { row.device_choice = -1; }
                }

                if (ImGui::Button("Refresh devices")) { refresh_devices(); }
                ImGui::SameLine();
                if (devices_enumerated) {
                    ImGui::Text("%zu found", devices.size());
                } else {
                    ImGui::TextUnformatted("not enumerated");
                }

                for (std::size_t row_idx = 0; row_idx < camera_rows.size(); ++row_idx)
                {
                    if (draw_camera_row(row_idx)) {
                        camera_rows.erase(camera_rows.begin() + static_cast<std::ptrdiff_t>(row_idx));
                        break;
                    }
                }
                if (camera_rows.size() < kMaxCameraRows && ImGui::Button("Add camera")) {
                    // 새 행은 다음 인덱스를 가리킨다. 열거 목록이 있으면 그 순서로 배정한다.
                    camera_row_t row;
                    row.device_index = static_cast<int>(camera_rows.size());
                    camera_rows.push_back(row);
                    if (devices_enumerated) { refresh_devices(); }
                }

                if (camera_rows.size() >= 2)
                {
                    ImGui::Separator();
                    ImGui::TextUnformatted("Pairing");
                    ImGui::SliderInt("Reference stream", &reference_stream_idx, 0, static_cast<int>(camera_rows.size()) - 1);
                    ImGui::InputFloat("Max pair skew (ms, 0 = auto)", &max_pair_skew_ms);
                    max_pair_skew_ms = std::max(0.0f, max_pair_skew_ms);
                }

                wants_vz = (backend_kind == 1);
            }

#ifndef EXO_HAS_VZ_BACKEND
            if (wants_vz) { ImGui::TextWrapped("VZ support is unavailable in this build."); }
            ImGui::BeginDisabled(wants_vz);
#else
            (void)wants_vz;
#endif
            if (ImGui::Button("Open")) { open(); }
#ifndef EXO_HAS_VZ_BACKEND
            ImGui::EndDisabled();
#endif
            ImGui::EndDisabled();

            if (provider)
            {
                ImGui::SameLine();
                if (ImGui::Button("Close")) { close(); }
            }
            if (!error.empty()) { ImGui::TextWrapped("%s", error.c_str()); }
            if (!provider) { return; }

            ImGui::Separator();
            ImGui::TextWrapped("%s", provider->get_source_name().c_str());
            ImGui::Text("Framesets: %u | %.1f fps", provider->get_current_frameset_seq(), provider->get_current_frameset_rate());

            const auto state = observer->snapshot();
            if (state.end) {
                ImGui::TextUnformatted(*state.end == hw::stream_end_reason_t::completed ? "End of stream" : "Stream failed");
            }
            else if (state.frames.empty()) {
                ImGui::TextUnformatted("Waiting for frames");
            }

            if (playback)
            {
                const bool paused = provider->is_paused();
                if (ImGui::Button(paused ? "Play" : "Pause")) {
                    if (paused) { provider->play(); }
                    else { provider->pause(); }
                }
                ImGui::SameLine();
                if (ImGui::Button("Rewind")) { provider->seek_recording_to_begin(); }
                bool repeat = provider->is_auto_repeat_enabled();
                if (ImGui::Checkbox("Repeat", &repeat)) { provider->set_auto_repeat(repeat); }
            }

            ImGui::Separator();
            ImGui::TextUnformatted("AprilTag detection test");
            ImGui::BeginDisabled(apriltag_test);
            ImGui::InputFloat("quad_decimate", &tag_quad_decimate);
            tag_quad_decimate = std::max(1.0f, tag_quad_decimate);
            ImGui::EndDisabled();
            bool test_on = apriltag_test;
            if (ImGui::Checkbox("Run a tracker per stream", &test_on))
            {
                if (test_on) { start_apriltag_test(); }
                else { stop_apriltag_test(); }
            }

            draw_recording_controls();

            const std::size_t stream_count = provider->stream_count();
            for (std::size_t stream_idx = 0; stream_idx < stream_count; ++stream_idx) { draw_stream_panel(stream_idx); }
        }

        void draw_recording_controls()
        {
            // 스트림이 끝나거나 기하가 바뀌면 녹화기가 스스로 파일을 닫는다. 그것을 보고 관찰자를 뗀다.
            if (recorder && !recorder->is_started()) { stop_recording(); }

            ImGui::Separator();
            ImGui::TextUnformatted("Recording");

            if (!recorder)
            {
                ImGui::Combo("Codec", &record_codec, kCodecLabels.data(), static_cast<int>(kCodecLabels.size()));
                const bool is_jpeg = io::kImageCodecs[static_cast<std::size_t>(record_codec)].codec == io::image_codec_t::jpeg;
                ImGui::BeginDisabled(!is_jpeg);
                ImGui::SliderInt("JPEG quality", &record_jpeg_quality, 1, 100);
                ImGui::EndDisabled();

                ImGui::BeginDisabled(playback);
                if (ImGui::Button("Start recording")) { start_recording(); }
                ImGui::EndDisabled();
                if (playback) { ImGui::TextUnformatted("A playback source is not recorded"); }
                return;
            }

            const io::recording_stats_t stats = recorder->stats();
            const double seconds = std::chrono::duration<double>{ stats.duration }.count();
            const double megabytes = static_cast<double>(stats.file_bytes) / (1024.0 * 1024.0);
            ImGui::TextColored(ImVec4{ 0.90f, 0.30f, 0.30f, 1.0f }, "REC %s", recorder->path().filename().string().c_str());
            ImGui::Text("%zu stream(s) | %.1f s | %llu frames written | %.1f MB (%.1f MB/s)"
                , recorder->stream_count()
                , seconds
                , static_cast<unsigned long long>(stats.frames_written)
                , megabytes
                , seconds > 0.0 ? megabytes / seconds : 0.0);
            if (stats.frames_dropped > 0) {
                ImGui::TextColored(ImVec4{ 0.90f, 0.60f, 0.20f, 1.0f }, "Dropped: %llu frame(s)"
                    , static_cast<unsigned long long>(stats.frames_dropped));
            }
            if (ImGui::Button("Stop recording")) { stop_recording(); }
        }

        void draw_preview(SDL_Renderer* renderer)
        {
            if (!observer || textures.empty())
            {
                ImGui::TextUnformatted("Open a camera or recording to view frames.");
                return;
            }

            const auto state = observer->snapshot();

            // 스트림이 리셋되면 슬롯 전부가 옛 위치를 말하므로 텍스처를 내린다.
            if (state.frames.empty())
            {
                for (auto& texture : textures) { texture.reset(); }
                for (auto& id : displayed_frame_ids) { id.reset(); }
            }

            const std::size_t stream_count = textures.size();
            const ImVec2 available = ImGui::GetContentRegionAvail();
            if (available.x <= 0.0f || available.y <= 0.0f) { return; }
            const float spacing = ImGui::GetStyle().ItemSpacing.x;
            const float pane_width = std::max(1.0f,
                (available.x - spacing * static_cast<float>(stream_count - 1)) / static_cast<float>(stream_count));

            for (std::size_t stream_idx = 0; stream_idx < stream_count; ++stream_idx)
            {
                if (stream_idx > 0) { ImGui::SameLine(); }
                ImGui::PushID(static_cast<int>(2000 + stream_idx));
                ImGui::BeginChild("pane", ImVec2{ pane_width, available.y });

                const std::shared_ptr<hw::sensor_frame> frame =
                    stream_idx < state.frames.size() ? state.frames[stream_idx] : nullptr;
                if (frame) { displayed_timestamps[stream_idx] = frame->timestamp(); }

                // 검출 테스트 중에는 트래커가 그린 사본을, 아니면 원본을 올린다.
                cv::Mat image_to_show;
                std::optional<uint64_t> image_id;
                if (apriltag_test && synced_tracker)
                {
                    cv::Mat annotated, source;
                    uint64_t annotated_id = displayed_frame_ids[stream_idx].value_or(0);
                    if (synced_tracker->try_get_annotated(stream_idx, annotated, source, annotated_id)) {
                        image_to_show = annotated;
                        image_id = annotated_id;
                    }
                }
                else if (frame)
                {
                    image_to_show = frame->image();
                    image_id = frame->id();
                }

                if (image_id && (!displayed_frame_ids[stream_idx] || *displayed_frame_ids[stream_idx] != *image_id))
                {
                    if (!textures[stream_idx]) { textures[stream_idx] = std::make_unique<frame_texture>(renderer); }
                    if (textures[stream_idx]->update(image_to_show)) {
                        displayed_frame_ids[stream_idx] = image_id;
                    } else {
                        textures[stream_idx].reset();
                        displayed_frame_ids[stream_idx].reset();
                    }
                }

                ImGui::Text("Stream %zu", stream_idx);
                frame_texture* texture = textures[stream_idx].get();
                if (texture && texture->valid())
                {
                    const ImVec2 pane = ImGui::GetContentRegionAvail();
                    if (pane.x > 0.0f && pane.y > 0.0f)
                    {
                        const float scale = std::min(pane.x / static_cast<float>(texture->width()),
                                                     pane.y / static_cast<float>(texture->height()));
                        ImGui::Image(texture->id(), ImVec2{ texture->width() * scale, texture->height() * scale });
                    }
                }
                else
                {
                    ImGui::TextUnformatted("Waiting for frames");
                }

                ImGui::EndChild();
                ImGui::PopID();
            }
        }

        void draw_diagnostics()
        {
            if (!provider)
            {
                ImGui::TextUnformatted("Open a source to see pairing diagnostics.");
                return;
            }

            const hw::sync_stats_t stats = provider->get_sync_stats();
            ImGui::Text("Framesets emitted %llu | dropped incomplete %llu | out-of-order %llu"
                , static_cast<unsigned long long>(stats.framesets_emitted)
                , static_cast<unsigned long long>(stats.framesets_dropped_incomplete)
                , static_cast<unsigned long long>(stats.framesets_dropped_out_of_order));
            if (stats.pair_tolerance) {
                ImGui::Text("Pair tolerance: %.2f ms", to_ms(*stats.pair_tolerance));
            } else {
                ImGui::TextUnformatted("Pair tolerance: deriving from the reference interval");
            }

            for (std::size_t stream_idx = 0; stream_idx < stats.per_stream.size(); ++stream_idx)
            {
                const hw::sync_stats_t::per_stream_t& per_stream = stats.per_stream[stream_idx];
                ImGui::Text("Stream %zu: fetched %llu | dropped %llu | last skew %.2f ms"
                    , stream_idx
                    , static_cast<unsigned long long>(per_stream.frames_fetched)
                    , static_cast<unsigned long long>(per_stream.frames_dropped)
                    , to_ms(per_stream.last_pair_skew));
            }

            if (apriltag_test && synced_tracker)
            {
                // frameset 단위: 슬롯 전부가 끝나기까지의 시간이라 30fps 예산과 바로 견줄 수 있다.
                const pose::synced_tracker_base::stats_t tracker_stats = synced_tracker->stats();
                ImGui::Text("AprilTag framesets: %.1f ms (last %.1f) | %.1f fps | dropped %llu of %llu | failed %llu"
                    , tracker_stats.process_ms_ema
                    , tracker_stats.last_process_ms
                    , tracker_stats.process_rate_fps
                    , static_cast<unsigned long long>(tracker_stats.framesets_dropped)
                    , static_cast<unsigned long long>(tracker_stats.framesets_submitted)
                    , static_cast<unsigned long long>(tracker_stats.framesets_failed));

                for (std::size_t stream_idx = 0; stream_idx < synced_tracker->stream_count(); ++stream_idx)
                {
                    const pose::synced_tracker_base::stream_stats_t stream_stats = synced_tracker->stream_stats(stream_idx);
                    ImGui::Text("AprilTag %zu: %zu tags | %.1f ms (last %.1f)"
                        , stream_idx
                        , synced_tracker->tracker(stream_idx).last_detection_count()
                        , stream_stats.process_ms_ema
                        , stream_stats.last_process_ms);
                }
            }

            // 표시 중인 프레임끼리의 시각 차. 프레임 싱크가 실제로 무엇을 나란히 놓았는지 보여 준다.
            for (std::size_t stream_idx = 1; stream_idx < displayed_frame_ids.size(); ++stream_idx)
            {
                if (!displayed_frame_ids[0] || !displayed_frame_ids[stream_idx]) { continue; }
                const auto delta_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    displayed_timestamps[stream_idx] - displayed_timestamps[0]).count();
                ImGui::Text("Displayed stream %zu vs 0: %+lld us", stream_idx, static_cast<long long>(delta_us));
            }

            const auto state = observer->snapshot();
            const std::vector<float>& skew = state.pair_skew_ms;
            if (skew.size() < 2)
            {
                ImGui::TextUnformatted("Pair skew: needs two streams contributing");
                return;
            }

            std::vector<float> sorted{ skew };
            std::sort(sorted.begin(), sorted.end());
            const float skew_min = sorted.front();
            const float skew_median = sorted[sorted.size() / 2];
            const float skew_max = sorted.back();
            ImGui::Text("Pair skew over last %zu: min %.2f | median %.2f | max %.2f ms",
                skew.size(), skew_min, skew_median, skew_max);

            float y_max = skew_max;
            std::optional<float> tolerance_ms;
            if (stats.pair_tolerance) {
                tolerance_ms = to_ms(*stats.pair_tolerance);
                y_max = std::max(y_max, *tolerance_ms);
            }

            if (ImPlot::BeginPlot("##pair_skew", ImVec2{ -1.0f, -1.0f }, ImPlotFlags_NoLegend))
            {
                ImPlot::SetupAxes("frameset", "skew (ms)", 0, 0);
                ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, static_cast<double>(skew.size() - 1), ImPlotCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, static_cast<double>(y_max) * 1.2 + 0.1, ImPlotCond_Always);
                ImPlot::PlotLine("skew", skew.data(), static_cast<int>(skew.size()));
                if (tolerance_ms)
                {
                    const float xs[2] = { 0.0f, static_cast<float>(skew.size() - 1) };
                    const float ys[2] = { *tolerance_ms, *tolerance_ms };
                    ImPlot::PlotLine("tolerance", xs, ys, 2);
                }
                ImPlot::EndPlot();
            }
        }
    };

    camera_test_app::camera_test_app() : _ctx{ std::make_unique<context_t>() } {}
    camera_test_app::~camera_test_app() = default;

    int camera_test_app::run()
    {
        if (!create("exo-skeleton-pose camera-test", 1600, 900)) {
            spdlog::error("camera-test: failed to create window");
            return -1;
        }
        spdlog::info("camera-test: ready");
        app_base::run();
        _ctx->close();
        destroy();
        return 0;
    }

    void camera_test_app::render_ui()
    {
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        constexpr auto flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove;
        if (ImGui::Begin("Camera test", nullptr, flags))
        {
            const float dpi = renderer().dpi_scale();
            const ImVec2 available = ImGui::GetContentRegionAvail();
            const float top_height = std::max(120.0f * dpi, available.y * 0.55f);
            const float controls_width = 380.0f * dpi;

            ImGui::BeginChild("Controls", ImVec2{ controls_width, top_height });
            _ctx->draw_controls();
            ImGui::EndChild();
            ImGui::SameLine();
            ImGui::BeginChild("Preview", ImVec2{ 0.0f, top_height });
            _ctx->draw_preview(renderer().sdl_renderer());
            ImGui::EndChild();

            ImGui::Separator();
            const float bottom_height = ImGui::GetContentRegionAvail().y;
            ImGui::BeginChild("Diagnostics", ImVec2{ available.x * 0.5f, bottom_height });
            _ctx->draw_diagnostics();
            ImGui::EndChild();
            ImGui::SameLine();
            ImGui::BeginChild("Log", ImVec2{ 0.0f, bottom_height });
            _ctx->console.draw();
            ImGui::EndChild();
        }
        ImGui::End();

        _ctx->browser.Display();
        if (_ctx->browser.HasSelected()) {
            if (!_ctx->provider) { _ctx->recording = _ctx->browser.GetSelected(); }
            _ctx->browser.ClearSelected();
        }
    }
}
