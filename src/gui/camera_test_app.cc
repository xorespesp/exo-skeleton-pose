#include "camera_test_app.hh"

#include "frame_texture.hh"
#include "log_console.hh"
#include "hw/sensor_frame_provider.hh"

#include <imfilebrowser.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace gui
{
    namespace
    {
        // The worker publishes one snapshot; the GUI owns all texture operations.
        class preview_observer final : public hw::sensor_frame_observer
        {
        public:
            struct snapshot_t
            {
                std::shared_ptr<hw::sensor_frame> frame;
                std::optional<hw::stream_end_reason_t> end;
            };

            snapshot_t snapshot() const {
                std::scoped_lock lock{ _mutex };
                return _snapshot;
            }

            void on_sensor_frame_update(const std::shared_ptr<hw::sensor_frame>& frame) override {
                std::scoped_lock lock{ _mutex };
                _snapshot = { frame, std::nullopt };
            }

            void on_sensor_stream_reset() override {
                std::scoped_lock lock{ _mutex };
                _snapshot = {};
            }

            void on_sensor_stream_end(hw::stream_end_reason_t reason) override {
                std::scoped_lock lock{ _mutex };
                _snapshot.end = reason;
            }

        private:
            mutable std::mutex _mutex;
            snapshot_t _snapshot;
        };
    }

    struct camera_test_app::context_t
    {
        log_console console;
        std::shared_ptr<spdlog::logger> logger{ spdlog::default_logger() };
        std::shared_ptr<preview_observer> observer;
        std::unique_ptr<hw::sensor_frame_provider> provider;
        std::optional<frame_texture> texture;
        std::optional<uint64_t> displayed_id;
        hw::timestamp_t displayed_timestamp{};
        ImGui::FileBrowser browser;
        std::filesystem::path recording;
        int source_kind{ 0 };
        int device_index{ 0 };
        int format_index{ 0 };
        bool manual_exposure{ false };
        int exposure_us{ 8000 };
        bool manual_gain{ false };
        int gain{ 0 };
        bool playback{ false };
        std::string error;

        context_t() {
            browser.SetTitle("Open camera recording");
            browser.SetTypeFilters({ ".mcap" });
            console.sink()->set_level(spdlog::level::trace);
            logger->sinks().push_back(console.sink());
        }

        ~context_t() {
            close();
            std::erase(logger->sinks(), console.sink());
        }

        void close() {
            // Join the capture worker before releasing its observer and the GUI log sink.
            provider.reset();
            observer.reset();
            texture.reset();
            displayed_id.reset();
            displayed_timestamp = {};
            playback = false;
        }

        void open() {
            error.clear();
            hw::source_config_t config;
            const auto format = format_index == 0 ? hw::frame_format_t::bgr8 : hw::frame_format_t::gray8;
            if (source_kind == 2) {
                if (recording.empty()) {
                    error = "Choose an MCAP recording.";
                    return;
                }
                config = hw::recording_config_t{ .file = recording };
            }
            else if (source_kind == 1) {
                hw::vz_device_config_t camera;
                camera.device_index = static_cast<uint32_t>(device_index);
                camera.frame_format = format;
                if (manual_exposure) { camera.exposure_us = exposure_us; }
                if (manual_gain) { camera.gain = gain; }
                config = camera;
            }
            else {
                hw::k4a_device_config_t camera;
                camera.device_index = static_cast<uint32_t>(device_index);
                camera.frame_format = format;
                if (manual_exposure) { camera.exposure_us = exposure_us; }
                if (manual_gain) { camera.gain = gain; }
                config = camera;
            }

            auto next_observer = std::make_shared<preview_observer>();
            auto next_provider = std::make_unique<hw::sensor_frame_provider>();
            next_provider->set_auto_repeat(false);
            next_provider->add_observer(next_observer);
            if (!next_provider->open(config)) {
                error = "Could not open the source. See the log for details.";
                return;
            }
            observer = std::move(next_observer);
            provider = std::move(next_provider);
            playback = source_kind == 2;
            spdlog::info("camera-test: opened {}", provider->get_source_name());
        }

        void draw_controls() {
            ImGui::TextUnformatted("Camera input");
            ImGui::BeginDisabled(provider != nullptr);
            ImGui::Combo("Source", &source_kind, "K4A\0VZ\0MCAP recording\0");
            if (source_kind == 2) {
                if (ImGui::Button("Choose recording...")) { browser.Open(); }
                ImGui::TextWrapped("%s", recording.empty() ? "No recording selected" : recording.string().c_str());
            }
            else {
                ImGui::InputInt("Device index", &device_index);
                device_index = std::max(0, device_index);
                ImGui::Combo("Pixel format", &format_index, "BGR8\0GRAY8\0");
                ImGui::Checkbox("Manual exposure", &manual_exposure);
                ImGui::BeginDisabled(!manual_exposure);
                ImGui::InputInt("Exposure (us)", &exposure_us);
                exposure_us = std::max(1, exposure_us);
                ImGui::EndDisabled();
                ImGui::Checkbox("Manual gain", &manual_gain);
                ImGui::BeginDisabled(!manual_gain);
                ImGui::InputInt("Gain", &gain);
                ImGui::EndDisabled();
                ImGui::TextWrapped("Unchecked controls use the backend defaults. Device indices start at zero.");
            }
#ifndef EXO_HAS_VZ_BACKEND
            if (source_kind == 1) { ImGui::TextWrapped("VZ support is unavailable in this build."); }
            ImGui::BeginDisabled(source_kind == 1);
#endif
            if (ImGui::Button("Open")) { open(); }
#ifndef EXO_HAS_VZ_BACKEND
            ImGui::EndDisabled();
#endif
            ImGui::EndDisabled();
            if (provider) {
                ImGui::SameLine();
                if (ImGui::Button("Close")) { close(); }
            }
            if (!error.empty()) { ImGui::TextWrapped("%s", error.c_str()); }
            if (!provider) { return; }

            ImGui::Separator();
            ImGui::TextWrapped("%s", provider->get_source_name().c_str());
            const auto resolution = provider->get_frame_resolution();
            const auto format = hw::frame_format_to_str(provider->get_frame_format());
            ImGui::Text("%d x %d | %.*s", resolution.x(), resolution.y(),
                static_cast<int>(format.size()), format.data());
            ImGui::Text("Delivered: %u | %.1f fps", provider->get_current_frame_seq(), provider->get_current_update_rate());
            if (displayed_id) {
                const auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    displayed_timestamp.time_since_epoch()).count();
                ImGui::Text("Displayed timestamp (us): %lld", static_cast<long long>(timestamp_us));
            }

            const auto state = observer->snapshot();
            if (state.end) {
                ImGui::TextUnformatted(*state.end == hw::stream_end_reason_t::completed
                    ? "End of stream" : "Stream failed");
            }
            else if (!state.frame) { ImGui::TextUnformatted("Waiting for frames"); }

            if (playback) {
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
        }

        void draw_preview(SDL_Renderer* renderer) {
            if (!observer) {
                ImGui::TextUnformatted("Open a camera or recording to view frames.");
                return;
            }
            const auto state = observer->snapshot();
            if (!state.frame) {
                texture.reset();
                displayed_id.reset();
            }
            else if (!displayed_id || *displayed_id != state.frame->id()) {
                if (!texture) { texture.emplace(renderer); }
                if (texture->update(state.frame->image())) {
                    displayed_id = state.frame->id();
                    displayed_timestamp = state.frame->timestamp();
                }
                else {
                    texture.reset();
                    displayed_id.reset();
                }
            }
            if (!texture || !texture->valid()) { return; }
            const ImVec2 available = ImGui::GetContentRegionAvail();
            if (available.x <= 0.0f || available.y <= 0.0f) { return; }
            const float scale = std::min(available.x / texture->width(), available.y / texture->height());
            ImGui::Image(texture->id(), ImVec2{ texture->width() * scale, texture->height() * scale });
        }
    };

    camera_test_app::camera_test_app() : _ctx{ std::make_unique<context_t>() } {}
    camera_test_app::~camera_test_app() = default;

    int camera_test_app::run() {
        if (!create("exo-skeleton-pose camera-test", 1280, 800)) {
            spdlog::error("camera-test: failed to create window");
            return -1;
        }
        spdlog::info("camera-test: ready");
        app_base::run();
        _ctx->close();
        destroy();
        return 0;
    }

    void camera_test_app::render_ui() {
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        constexpr auto flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove;
        if (ImGui::Begin("Camera test", nullptr, flags)) {
            const float dpi = renderer().dpi_scale();
            const float height = std::max(100.0f * dpi, ImGui::GetContentRegionAvail().y * 0.7f);
            ImGui::BeginChild("Controls", ImVec2{ 350.0f * dpi, height });
            _ctx->draw_controls();
            ImGui::EndChild();
            ImGui::SameLine();
            ImGui::BeginChild("Preview", ImVec2{ 0.0f, height });
            _ctx->draw_preview(renderer().sdl_renderer());
            ImGui::EndChild();
            ImGui::Separator();
            ImGui::BeginChild("Log");
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
