#include "exo_pose_pipeline.hh"

#include "hw/frameset_observer.hh"
#include "io/calibration_io.hh"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <filesystem>
#include <format>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace net
{
    namespace
    {
        // How often poll() summarizes throughput while a source streams.
        constexpr auto kStatsInterval = std::chrono::seconds{ 5 };

        // The camera controls arrive as integers;
        // the VZ camera states its exposure and gain in fractional units.
        // NOTE: a VZ gain therefore lands on whole dB where the camera steps in 0.1. Widening the
        //       config's type reaches K4A too, which takes integers, so it is a decision first.
        std::optional<double> to_optional_double(const std::optional<int32_t> value)
        {
            if (!value.has_value()) { return std::nullopt; }
            return static_cast<double>(*value);
        }

        // 카메라 엔트리 하나의 hw 소스 설정.
        hw::source_config_t make_camera_source_config(
            const app::camera_config_t& cam,
            const hw::frame_format_t frame_format)
        {
            if (cam.source.device_backend() == hw::sensor_backend_t::k4a)
            {
                return hw::k4a_device_config_t{
                    .device_selector = cam.source.device_serial(),
                    .exposure_us = cam.exposure_us,
                    .gain = cam.gain,
                    .frame_format = frame_format,
                    .roi = cam.roi,
                };
            }

            hw::vz_device_config_t vz{
                .device_selector = cam.source.device_serial(),
                .exposure_us = to_optional_double(cam.exposure_us),
                .gain = to_optional_double(cam.gain),
                .frame_format = frame_format,
                .roi = cam.roi,
                .frame_rate_fps = cam.frame_rate_fps,
            };

            if (!cam.intrinsics_file.empty())
            {
                const std::filesystem::path path{ cam.intrinsics_file };
                hw::intrinsic_t intr{};
                hw::distortion_t dist{};
                if (std::string err; io::load_camera_calibration(path, intr, dist, err)) {
                    vz.intrinsic = intr;
                    vz.distortion = dist;
                    spdlog::info("pipeline: read intrinsics from '{}' ({}x{})"
                        , path.string()
                        , intr.calib_resolution.x(), intr.calib_resolution.y()
                    );
                } else {
                    // frontal estimator will not solve tag poses, but the sagittal estimator still works off 2D tag centers
                    spdlog::warn("pipeline: '{}': {}; opening without intrinsics", path.string(), err);
                }
            }
            return vz;
        }

        std::string describe_setting(const std::optional<int32_t> value, const char* unit)
        {
            return value.has_value() ? std::format("{}{}", *value, unit) : std::string{ "auto" };
        }

        // The intrinsics a tag pose solve may run on. Only a frontal run consumes the poses, and a
        // zero focal length solves to plausible nonsense rather than failing, so either case answers
        // nothing and leaves the detection at tag centers.
        std::optional<hw::intrinsic_t> pose_solve_intrinsics(
            const pose::view_plane_t view_plane,
            const hw::calibration_t& calib)
        {
            if (view_plane != pose::view_plane_t::frontal) { return std::nullopt; }
            if (calib.intrinsic.fx <= 0.0f || calib.intrinsic.fy <= 0.0f) { return std::nullopt; }
            return calib.intrinsic;
        }

    } // namespace

    // --- observer (worker thread) ------------------------------------------------
    // Hands each arriving frameset to the frameset tracker and latches the stream events for poll().
    // What detection means and what it produces are the tracker's business, which the frameset
    // tracker publishes to the loop thread; this class holds no knowledge of any marker technology.
    class pose_frame_observer final : public hw::synced_frameset_observer
    {
    public:
        explicit pose_frame_observer(std::shared_ptr<pose::synced_tracker_base> synced_tracker)
            : _synced_tracker{ std::move(synced_tracker) }
        { }

        // Why the stream ended, once per stream-end signal (consumes the latched flag).
        // Empty when none has been raised since the last call.
        std::optional<hw::stream_end_reason_t> consume_stream_end_signal() noexcept {
            if (!_stream_ended.exchange(false)) { return std::nullopt; }
            return _stream_end_reason.load();
        }

        // True once per stream-jump signal (consumes the latched flag).
        bool consume_stream_reset_signal() noexcept {
            return _stream_reset.exchange(false);
        }

        // True once per geometry-change signal (consumes the latched flag).
        bool consume_frame_geometry_signal() noexcept {
            return _frame_geometry_changed.exchange(false);
        }

    public:
        void on_synced_frameset_update(const hw::synced_frameset& new_frameset) override
        {
            _synced_tracker->submit(new_frameset);
        }

        void on_sensor_stream_reset() override {
            // Called from the worker thread; what the jump invalidates is dropped on the estimator thread,
            // so this only raises the flag that poll() acts on.
            _stream_reset = true;
        }

        void on_sensor_frame_geometry_changed(std::size_t /*stream_idx*/) override {
            // 스트림을 가리지 않는다: rest 는 모든 스트림을 한 번에 캡처한 것이다.
            _frame_geometry_changed = true;
        }

        void on_sensor_stream_end(const hw::stream_end_reason_t reason) override {
            spdlog::debug("pipeline: worker signalled end of stream");
            _stream_end_reason = reason;
            _stream_ended = true; // set last, so a reader that sees it finds the reason in place
        }

    private:
        const std::shared_ptr<pose::synced_tracker_base> _synced_tracker;

        std::atomic<bool> _stream_ended{ false }; // set by the worker thread on stream end
        std::atomic<hw::stream_end_reason_t> _stream_end_reason{ hw::stream_end_reason_t::completed };
        std::atomic<bool> _stream_reset{ false }; // set by the worker thread when the position jumps
        std::atomic<bool> _frame_geometry_changed{ false }; // ... and when the ROI changes
    };

    // --- exo_pose_pipeline -------------------------------------------------------
    exo_pose_pipeline::exo_pose_pipeline(bool annotate_frames)
        : _annotate_frames{ annotate_frames }
    { }

    exo_pose_pipeline::~exo_pose_pipeline() = default;

    void exo_pose_pipeline::_build_estimator(pose::view_plane_t view_plane)
    {
        _frontal.reset();
        _sagittal.reset();

        switch (view_plane)
        {
        case pose::view_plane_t::sagittal:
            _sagittal.emplace();
            _active = &_sagittal.value();
            break;
        case pose::view_plane_t::frontal:
        default:
            _frontal.emplace();
            _active = &_frontal.value();
            break;
        }
        _view_plane = view_plane;
    }

    bool exo_pose_pipeline::_holds_any_camera_of(const std::span<const app::camera_config_t> cameras) const
    {
        if (!_provider) { return false; }

        for (std::size_t stream_idx = 0; stream_idx < _provider->stream_count(); ++stream_idx)
        {
            const hw::stream_descriptor_t open_stream = _provider->get_stream_descriptor(stream_idx);
            if (open_stream.device_serial.empty()) { continue; } // 대조할 이름이 없는 장치

            for (const app::camera_config_t& cam : cameras)
            {
                if (cam.source.is_recording()) { continue; }

                // 같은 장치라는 말은 백엔드와 시리얼이 함께 같다는 뜻이다(`source_address::operator==`).
                if (cam.source.device_backend() == open_stream.sensor_backend
                    && cam.source.device_serial().value == open_stream.device_serial)
                {
                    return true;
                }
            }
        }
        return false;
    }

    bool exo_pose_pipeline::open_source(const app::app_config_t& config)
    {
        _status_changed = true; // opening a source changes the reported status (even on failure)

        // The same answer a config file is held to, asked of whatever assembled this one. A control
        // panel reaches every rule here that an authored profile does.
        if (std::string err; !app::validate_config(config, err)) {
            spdlog::error("pipeline: {}", err);
            return false;
        }

        const std::vector<app::camera_config_t>& cameras = config.cameras;
        if (cameras.empty()) {
            spdlog::error("pipeline: the config names no camera to open");
            return false;
        }

        // 검증이 보장한다: 엔트리들은 한 평면이고, 녹화면 모두 같은 파일이다.
        const pose::view_plane_t view_plane = app::view_plane_of(cameras).value();
        const app::marker_kind_t marker_kind = config.pose.detector.kind;
        const bool playback = cameras.front().source.is_recording();

        // A recording ignores this and replays the layout it was written with, which is what makes
        // a gray recording opened under a color profile something the tracker has to refuse.
        const auto kCameraFrameFormat = app::marker_frame_format(marker_kind);

        spdlog::info("pipeline: opening {} ({} view, {} markers, {} frames)"
            , playback ? "a recording" : (cameras.size() == 1 ? "a camera" : std::format("{} synced cameras", cameras.size()))
            , pose::view_plane_name(view_plane)
            , app::marker_kind_name(marker_kind)
            , hw::frame_format_to_str(kCameraFrameFormat)
        );
        for (std::size_t stream_idx = 0; stream_idx < cameras.size(); ++stream_idx)
        {
            const app::camera_config_t& cam = cameras[stream_idx];
            spdlog::info("pipeline:   stream {}: '{}' seen from {}, exposure {}, gain {}"
                , stream_idx
                , cam.source.to_string()
                , pose::camera_view_name(cam.view)
                , describe_setting(cam.exposure_us, " us")
                , describe_setting(cam.gain, "")
            );
        }

        // 프리런하는 카메라들은 각자 자기 상한으로 도므로 그 속도가 서로 다를 수 있다. 열리기는 하지만
        // frameset 이 덜 맞춰지므로, 한 번은 경고한다.
        if (!playback && cameras.size() >= 2
            && std::ranges::none_of(cameras, [](const app::camera_config_t& cam) { return cam.frame_rate_fps.has_value(); })
            && cameras.front().source.device_backend() != hw::sensor_backend_t::k4a)
        {
            spdlog::warn("pipeline: no frame rate is pinned, so each camera free-runs at its own ceiling; "
                         "captures pair less often when the two differ");
        }

        if (_provider) {
            spdlog::info("pipeline: replacing the open source '{}'", _provider->get_source_name());
        }

        // 카메라는 배타적으로 열린다. 지금 쥐고 있는 장치를 다시 지목했으면 먼저 놓아야 열 수 있다.
        // 재생은 장치를 쥐지 않으므로 대조하지 않는다(녹화 스트림도 촬영에 쓰인 시리얼을 싣는다).
        if (_provider && !_is_playback_source && !playback && this->_holds_any_camera_of(cameras))
        {
            // 먼저 닫는 것은 되돌릴 수 없다. 녹화 중이라면 파일까지 끝나므로, 그 선택은 사람이 한다.
            if (this->is_recording()) {
                spdlog::error("pipeline: '{}' is being recorded and the new profile names the same camera; "
                              "stop the recording first", _provider->get_source_name());
                return false;
            }

            spdlog::info("pipeline: the new profile names a camera this source holds; closing it first");
            this->close_source();
        }

        auto new_provider = std::make_shared<hw::sensor_frame_provider>();
        bool opened = false;
        if (playback)
        {
            // 엔트리 i 는 파일의 스트림 i 이고, ROI 만 재생에 걸린다.
            hw::recording_config_t recording{ .file = cameras.front().source.recording_path() };
            for (const app::camera_config_t& cam : cameras) { recording.stream_rois.push_back(cam.roi); }
            opened = new_provider->open(std::move(recording));
        }
        else if (cameras.size() == 1)
        {
            opened = new_provider->open(make_camera_source_config(cameras.front(), kCameraFrameFormat));
        }
        else
        {
            std::vector<hw::source_config_t> member_configs;
            for (const app::camera_config_t& cam : cameras) {
                member_configs.push_back(make_camera_source_config(cam, kCameraFrameFormat));
            }
            const app::sync_config_t& sync = config.sync.value(); // 검증이 라이브 둘 이상에 요구한다
            hw::sync_options_t sync_options{ .reference_stream_idx = sync.reference_stream_idx };
            if (sync.max_pair_skew.has_value()) {
                sync_options.max_pair_skew = std::chrono::duration_cast<std::chrono::nanoseconds>(*sync.max_pair_skew);
            }
            opened = new_provider->open_synced(member_configs, sync_options);
        }
        if (!opened) {
            spdlog::error("pipeline: failed to open the configured source");
            return false;
        }

        // 스트림 수는 열어 봐야 안다.
        if (new_provider->stream_count() != cameras.size())
        {
            spdlog::error("pipeline: '{}' delivers {} stream(s) but the profile describes {} camera(s); "
                          "a {} profile needs a source with exactly that many"
                , new_provider->get_source_name(), new_provider->stream_count(), cameras.size()
                , pose::view_plane_name(view_plane));
            return false; // new_provider 가 여기서 닫힌다
        }

        // The one place that names a concrete tracker. It is built around the source it will read,
        // so it stands for exactly as long as that source does.
        //
        // An open installs the config, so what is in effect right afterwards is what the file says.
        // Edits made from the control panel are scratch until they are saved back.
        std::vector<pose::synced_tracker_base::stream_entry_t> stream_entries;
        for (std::size_t stream_idx = 0; stream_idx < cameras.size(); ++stream_idx)
        {
            const app::camera_config_t& cam = cameras[stream_idx];

            std::shared_ptr<pose::marker_tracker_base> tracker;
            if (marker_kind == app::marker_kind_t::color_marker)
            {
                // The colour and the blob filters were measured together on site and sit together in
                // the profile. Absent, the detector runs on defaults that carry no colour and finds
                // nothing, which it says once when it is built.
                const std::optional<app::color_marker_calibration_t>& calibration =
                    config.pose.detector.color_marker.calibration[stream_idx];

                // The blob gates and the search radius are counted in pixels, so they only mean what they
                // meant if a marker still covers as many of them. A different frame size moves every one
                // of them at once.
                if (const Eigen::Vector2i frame_resolution = new_provider->get_frame_resolution(stream_idx);
                    calibration.has_value() && calibration->frame_resolution != frame_resolution)
                {
                    spdlog::warn("pipeline: the color was measured on {}x{} frames but stream {} "
                                 "delivers {}x{}; the blob size gates and the search radius were sized for the other one",
                        calibration->frame_resolution.x(), calibration->frame_resolution.y(),
                        stream_idx, frame_resolution.x(), frame_resolution.y());
                }

                tracker = std::make_shared<pose::color_marker_tracker>(
                    cam.view,
                    calibration.has_value() ? calibration->detector : pose::color_marker_detector::options_t{},
                    calibration.has_value() ? calibration->assigner : pose::color_marker_assigner::options_t{}
                );
            }
            else
            {
                // Read now because only an opened source reports them.
                std::optional<hw::intrinsic_t> intrinsics = pose_solve_intrinsics(
                    view_plane, new_provider->get_calibration(stream_idx));
                if (view_plane == pose::view_plane_t::frontal && !intrinsics.has_value())
                {
                    spdlog::warn("pipeline: stream {} of '{}' reports no intrinsics, so marker poses cannot be "
                                 "solved and the frontal estimator will track nothing",
                        stream_idx, new_provider->get_source_name());
                }
                tracker = std::make_shared<pose::apriltag_tracker>(
                    config.pose.detector.apriltag.detector,
                    config.pose.detector.apriltag.tag_size_m,
                    std::move(intrinsics)
                );
            }

            stream_entries.push_back({ .tracker = std::move(tracker), .view = cam.view });
        }

        // frameset 트래커가 꺼내는 측정치는 추정기가 소비하는 것이다: frontal 은 3D, sagittal 은 2D.
        std::shared_ptr<pose::synced_tracker<pose::joint_3d_measurement_t>> new_frontal_synced_tracker;
        std::shared_ptr<pose::synced_tracker<pose::joint_2d_measurement_t>> new_sagittal_synced_tracker;
        std::shared_ptr<pose::synced_tracker_base> new_synced_tracker;
        if (view_plane == pose::view_plane_t::frontal) {
            new_frontal_synced_tracker = std::make_shared<pose::synced_tracker<pose::joint_3d_measurement_t>>(
                std::move(stream_entries), _annotate_frames);
            new_synced_tracker = new_frontal_synced_tracker;
        }
        else {
            new_sagittal_synced_tracker = std::make_shared<pose::synced_tracker<pose::joint_2d_measurement_t>>(
                std::move(stream_entries), _annotate_frames);
            new_synced_tracker = new_sagittal_synced_tracker;
        }

        auto new_observer = std::make_shared<pose_frame_observer>(new_synced_tracker);
        new_provider->add_observer(new_observer);

        std::vector<stream_settings_t> streams;
        for (const app::camera_config_t& cam : cameras) {
            streams.push_back(stream_settings_t{ .view = cam.view, .exposure_us = cam.exposure_us, .gain = cam.gain });
        }

        // 여기부터 교체가 확정된다. 녹화 중지는 옛 provider 에서 옵저버를 떼는 일이라 교체 전에 끝낸다.
        this->stop_recording();

        _provider = std::move(new_provider); // old provider closes/joins here
        _observer = std::move(new_observer);
        _synced_tracker = std::move(new_synced_tracker);
        _frontal_synced_tracker = std::move(new_frontal_synced_tracker);
        _sagittal_synced_tracker = std::move(new_sagittal_synced_tracker);
        _is_playback_source = playback;
        _streams = std::move(streams);
        _has_pose = false; // whatever the previous source produced does not describe this one
        this->_build_estimator(view_plane); // rest 도 트래킹 상태도 새로 시작한다

        if (_frontal) {
            _frontal->options() = config.pose.estimator.frontal;
        }

        if (_sagittal) {
            _sagittal->options() = config.pose.estimator.sagittal;
        }

        _frame_log.reset();

        std::string resolutions;
        for (std::size_t stream_idx = 0; stream_idx < _provider->stream_count(); ++stream_idx)
        {
            const Eigen::Vector2i res = _provider->get_frame_resolution(stream_idx);
            if (!resolutions.empty()) { resolutions += ", "; }
            resolutions += std::format("{}x{}", res.x(), res.y());
        }
        spdlog::info("pipeline: '{}' opened ({} stream(s): {}; {} estimator); rest pose cleared, awaiting first frame",
            _provider->get_source_name(), _provider->stream_count(), resolutions, pose::view_plane_name(_view_plane));
        return true;
    }

    void exo_pose_pipeline::close_source()
    {
        if (!_provider) { return; } // nothing open; keep the status flag and the log quiet

        _status_changed = true;
        spdlog::info("pipeline: closing source '{}' after {} frames"
            , _provider->get_source_name()
            , _provider->get_current_frameset_seq()
        );

        this->stop_recording();

        _provider.reset(); // stops/joins the worker thread
        _observer.reset();

        // 트래커는 자기가 읽을 소스를 기준으로 지어지므로 그 소스와 함께 내린다.
        _synced_tracker.reset();
        _frontal_synced_tracker.reset();
        _sagittal_synced_tracker.reset();

        _is_playback_source = false;
        _streams.clear();
        _frame_log.reset();
    }

    bool exo_pose_pipeline::is_source_open() const { return static_cast<bool>(_provider); }
    bool exo_pose_pipeline::is_playback_source() const { return _is_playback_source; }

    std::size_t exo_pose_pipeline::stream_count() const
    {
        return _provider ? _provider->stream_count() : 0;
    }

    bool exo_pose_pipeline::start_recording(
        const std::filesystem::path& path,
        const io::recording_options_t& options)
    {
        _status_changed = true;

        if (_is_playback_source) {
            spdlog::error("pipeline: cannot record a playback source");
            return false;
        }

        if (!_provider) {
            spdlog::error("pipeline: cannot record without an open source");
            return false;
        }
        // A recorder that finalized on its own (stream end, geometry change) is released here as
        // well as in poll(), so a start issued before the next poll is not turned away.
        if (_recorder && !_recorder->is_started()) { this->stop_recording(); }
        if (_recorder) {
            spdlog::warn("pipeline: already recording to '{}'", _recorder->path().string());
            return false;
        }

        std::vector<io::camera_stream_info_t> stream_infos;
        for (std::size_t stream_idx = 0; stream_idx < _provider->stream_count(); ++stream_idx)
        {
            const hw::stream_descriptor_t descriptor = _provider->get_stream_descriptor(stream_idx);
            const stream_settings_t& settings = _streams.at(stream_idx);
            stream_infos.push_back(io::camera_stream_info_t{
                .calibration = _provider->get_calibration(stream_idx),
                .color_format = _provider->get_frame_format(stream_idx),
                .sensor_backend = descriptor.sensor_backend,
                .device_serial = descriptor.device_serial,
                .exposure_us = settings.exposure_us,
                .gain = settings.gain,
            });
        }

        // Start the recorder before it is subscribed,
        // so the first frame it sees is one it can already write.
        auto recorder = std::make_shared<io::frame_recorder>(options);
        if (!recorder->start(path, stream_infos)) { return false; }

        _provider->add_observer(recorder);
        _recorder = std::move(recorder);

        spdlog::info("pipeline: recording '{}' to '{}'", _provider->get_source_name(), path.string());
        return true;
    }

    void exo_pose_pipeline::stop_recording()
    {
        if (!_recorder) { return; }

        _status_changed = true;

        if (_provider) { _provider->remove_observer(_recorder); }
        _recorder->stop();

        const io::recording_stats_t stats = _recorder->stats();
        if (stats.frames_dropped > 0) {
            spdlog::warn("pipeline: {} frame(s) dropped while recording; the disk or the encoder could not keep up"
                , stats.frames_dropped);
        }
        spdlog::info("pipeline: recording stopped ({} frames over {:.1f} s)"
            , stats.frames_written
            , std::chrono::duration<double>{ stats.duration }.count()
        );

        _recorder.reset();
    }

    bool exo_pose_pipeline::is_recording() const
    {
        return _recorder && _recorder->is_started();
    }

    io::recording_stats_t exo_pose_pipeline::recording_stats() const
    {
        return _recorder ? _recorder->stats() : io::recording_stats_t{};
    }

    std::filesystem::path exo_pose_pipeline::recording_path() const
    {
        return _recorder ? _recorder->path() : std::filesystem::path{};
    }

    bool exo_pose_pipeline::calibrate_rest_pose()
    {
        if (!_active || !this->is_source_open()) {
            spdlog::warn("pipeline: cannot calibrate a rest pose without an open source");
            return false;
        }

        // 추정기가 거부하면 트래커의 기준도 그대로 둔다.
        if (!_active->calibrate_rest_pose()) { return false; }

        _status_changed = true;

        // Whatever the tracker latches at calibration belongs to this same moment: it is the one
        // point where an operator is watching the annotated frame and vouching for what it shows.
        //
        // 소스가 열려 있으면 frameset 트래커도 서 있다. 둘은 같은 자리에서 설치되고 함께 내려간다.
        for (std::size_t stream_idx = 0; stream_idx < _synced_tracker->stream_count(); ++stream_idx) {
            _synced_tracker->tracker(stream_idx).on_rest_pose_captured();
        }

        spdlog::info("pipeline: rest pose calibrated");
        return true;
    }

    void exo_pose_pipeline::clear_rest_pose()
    {
        if (!_active) { return; }
        _status_changed = true;
        spdlog::info("pipeline: rest pose cleared");
        _active->clear_rest_pose();
        if (_synced_tracker) // captured together, so dropped together
        {
            for (std::size_t stream_idx = 0; stream_idx < _synced_tracker->stream_count(); ++stream_idx) {
                _synced_tracker->tracker(stream_idx).on_rest_pose_cleared();
            }
        }
    }

    bool exo_pose_pipeline::has_rest_pose() const
    {
        return _active && _active->has_rest_pose();
    }

    pose::marker_tracker_base* exo_pose_pipeline::tracker(const std::size_t stream_idx)
    {
        if (!_synced_tracker || stream_idx >= _synced_tracker->stream_count()) { return nullptr; }
        return &_synced_tracker->tracker(stream_idx);
    }

    const pose::marker_tracker_base* exo_pose_pipeline::tracker(const std::size_t stream_idx) const
    {
        if (!_synced_tracker || stream_idx >= _synced_tracker->stream_count()) { return nullptr; }
        return &_synced_tracker->tracker(stream_idx);
    }

    pose::pose_estimator_base* exo_pose_pipeline::estimator() { return _active; }
    const pose::pose_estimator_base* exo_pose_pipeline::estimator() const { return _active; }

    std::optional<pose::frontal_pose_estimator::options_t> exo_pose_pipeline::frontal_options() const
    {
        if (!_frontal) { return std::nullopt; }
        return _frontal->options();
    }

    void exo_pose_pipeline::set_frontal_options(const pose::frontal_pose_estimator::options_t& opt)
    {
        if (_frontal) { _frontal->options() = opt; }
    }

    std::optional<pose::sagittal_pose_estimator::options_t> exo_pose_pipeline::sagittal_options() const
    {
        if (!_sagittal) { return std::nullopt; }
        return _sagittal->options();
    }

    void exo_pose_pipeline::set_sagittal_options(const pose::sagittal_pose_estimator::options_t& opt)
    {
        if (_sagittal) { _sagittal->options() = opt; }
    }

    const pose::sagittal_pose_estimator* exo_pose_pipeline::sagittal_estimator() const
    {
        return _sagittal ? &_sagittal.value() : nullptr;
    }

    bool exo_pose_pipeline::_consume_source_signals()
    {
        bool stream_jumped = false;

        if (_observer && _observer->consume_stream_reset_signal())
        {
            spdlog::debug("pipeline: the stream position jumped; dropping what described the last one");
            if (_synced_tracker)
            {
                for (std::size_t stream_idx = 0; stream_idx < _synced_tracker->stream_count(); ++stream_idx) {
                    _synced_tracker->tracker(stream_idx).on_stream_reset();
                }
            }
            if (_active) { _active->reset_tracking(); }
            stream_jumped = true;
        }

        if (_observer && _observer->consume_frame_geometry_signal())
        {
            spdlog::debug("pipeline: the ROI changed; dropping what described the last one");
            if (_active) { _active->on_frame_geometry_changed(); }

            // The principal point moved with the window, and a tag pose is solved against it.
            // Handing the retargeted one over is what keeps a solve in the same metric frame
            // it was in before, which is why an estimator working in metres loses nothing.
            if (_synced_tracker)
            {
                for (std::size_t stream_idx = 0; stream_idx < _synced_tracker->stream_count(); ++stream_idx)
                {
                    auto* tag_tracker = dynamic_cast<pose::apriltag_tracker*>(&_synced_tracker->tracker(stream_idx));
                    if (!tag_tracker) { continue; }
                    tag_tracker->set_intrinsics(
                        _provider ? pose_solve_intrinsics(_view_plane, _provider->get_calibration(stream_idx))
                                  : std::nullopt);
                }
            }

            // The reported frame size moved with it, and a client has no other way to hear
            // that: the images it would notice on are not on the wire.
            _status_changed = true;
        }

        return stream_jumped;
    }

    template <typename Measurement, typename Estimator>
    bool exo_pose_pipeline::_take_and_step(
        pose::synced_tracker<Measurement>& synced_tracker,
        Estimator& estimator,
        std::size_t& detections_this_frame)
    {
        // Take the newest bundle the frameset tracker published, ahead of stepping the estimator on it.
        pose::synced_measurements_t<Measurement> bundle;
        const bool took = synced_tracker.try_take_measurements(bundle);

        // The stream jumped. Reading this after the take is what makes the two agree: the frame
        // thread raises it before publishing anything of the new position, so a step that took the
        // first such frame is a step that sees it. What it took goes with it.
        const bool stream_jumped = this->_consume_source_signals();
        if (!took || stream_jumped) { return false; }

        bool any_measured = false;
        for (const auto& slot : bundle.slots)
        {
            if (!slot.measured) { continue; }
            any_measured = true;
            detections_this_frame += slot.detection_count;
        }
        if (!any_measured) { return false; }

        _last_timestamp = bundle.timestamp;
        if constexpr (std::is_same_v<Estimator, pose::sagittal_pose_estimator>) {
            estimator.update(bundle);
        }
        else {
            estimator.update(bundle.slots.front().measurements, bundle.timestamp); // frontal 은 카메라 한 대다
        }
        return true;
    }

    exo_pose_pipeline::poll_result_t exo_pose_pipeline::poll()
    {
        poll_result_t r{};
        std::size_t detections_this_frame = 0; // 이번 frameset 의 모든 스트림에서 찾은 마커 수

        // NOTE: `update()` is not part of the estimator base; each one takes the input its algorithm
        // needs, so the frameset tracker is asked for the shape the built estimator consumes.
        if (_frontal && _frontal_synced_tracker) {
            r.new_pose = this->_take_and_step(*_frontal_synced_tracker, *_frontal, detections_this_frame);
        }
        else if (_sagittal && _sagittal_synced_tracker) {
            r.new_pose = this->_take_and_step(*_sagittal_synced_tracker, *_sagittal, detections_this_frame);
        }

        if (r.new_pose)
        {
            _has_pose = true;

            spdlog::trace("pipeline: frame #{} (t={:%H:%M:%S}) with {} detection(s)"
                , this->current_frame_seq()
                , _last_timestamp
                , detections_this_frame
            );

            std::string stream_rates;
            for (std::size_t stream_idx = 0; stream_idx < this->stream_count(); ++stream_idx) {
                if (!stream_rates.empty()) { stream_rates += ", "; }
                stream_rates += std::format("{:.1f}", this->source_fps(stream_idx));
            }
            _frame_log.log_transitions(*_synced_tracker, *_active);
            _frame_log.log_throughput(detections_this_frame, *_active, stream_rates, this->has_rest_pose());
        }

        // Stream end: consume the one-shot signal the worker thread raises at end of stream.
        r.stream_end_reason = _observer ? _observer->consume_stream_end_signal() : std::nullopt;
        if (r.stream_end_reason.has_value())
        {
            // A recording hitting EOF is expected; a live device going quiet is a loss.
            if (_is_playback_source) { spdlog::info("pipeline: recording '{}' reached the end of its stream", this->source_name()); }
            else { spdlog::warn("pipeline: device '{}' stopped streaming", this->source_name()); }

            // A failed source has nothing left to give, so it is released and the reported status follows.
            // One that ran out stays open, where a seek starts it going again.
            if (r.stream_end_reason == hw::stream_end_reason_t::failed) { this->close_source(); }
        }

        // The recorder finalizes its file on its own when the stream ends or a frame geometry
        // changes. Releasing it here is what lets the next start_recording() go through.
        if (_recorder && !_recorder->is_started()) { this->stop_recording(); }

        // Status: consume the flag set by the last source/rest command.
        r.status_changed = std::exchange(_status_changed, false);
        if (r.status_changed) { 
            spdlog::trace("pipeline: status changed (source open: {}, rest pose: {})",
                this->is_source_open(), this->has_rest_pose());
        }
        return r;
    }

    // The tracker losing its markers, and joints gaining or losing their local rotation, are what
    // explain a stalled or jumpy skeleton. What a marker technology has to say about its own
    // detections it says itself.
    void exo_pose_pipeline::frame_logger::log_transitions(
        const pose::synced_tracker_base& synced_tracker,
        const pose::pose_estimator_base& estimator)
    {
        // Whether the tracker has identified its markers is what a run whose markers are anonymous
        // hangs on: until it has, no joint is named and every one reads untracked for a reason that
        // is not the detector's.
        _stream_was_tracking.resize(synced_tracker.stream_count(), false);
        for (std::size_t stream_idx = 0; stream_idx < synced_tracker.stream_count(); ++stream_idx)
        {
            const bool tracking = synced_tracker.tracker(stream_idx).is_tracking();
            if (tracking == _stream_was_tracking[stream_idx]) { continue; }

            if (tracking) { spdlog::debug("pipeline: stream {}: the tracker identified its markers", stream_idx); }
            else { spdlog::debug("pipeline: stream {}: the tracker lost its markers and is searching again", stream_idx); }
            _stream_was_tracking[stream_idx] = tracking;
        }

        // A marker can be visible while its joint still has no local rotation (the parent's is
        // missing), so joint tracking is reported on its own rather than inferred from the markers.
        for (const auto& def : pose::get_joint_defs())
        {
            const bool tracked = estimator.get_joint_state(def.joint_id).position.has_value();
            bool& was_tracked = _joint_tracked[static_cast<size_t>(def.joint_id)];
            if (tracked == was_tracked) { continue; }

            if (tracked) { spdlog::debug("pipeline: joint '{}' tracking acquired", def.name); }
            else { spdlog::debug("pipeline: joint '{}' tracking lost", def.name); }
            was_tracked = tracked;
        }
    }

    // Periodic throughput line: the cheap way to see the pipeline is alive and keeping up
    // without a per-frame log. Detection rate matters as much as fps, since a stream at full
    // fps with no markers looks identical to a healthy one from the outside.
    void exo_pose_pipeline::frame_logger::log_throughput(
        const std::size_t detections,
        const pose::pose_estimator_base& estimator,
        const std::string_view stream_rates,
        const bool has_rest_pose)
    {
        ++_frames;
        _detections += static_cast<uint32_t>(detections);

        const auto now = std::chrono::steady_clock::now();
        if (_since.time_since_epoch().count() == 0) { _since = now; return; }

        const auto elapsed = now - _since;
        if (elapsed < kStatsInterval) { return; }

        const double sec = std::chrono::duration<double>{ elapsed }.count();
        size_t tracked = 0;
        for (const auto& def : pose::get_joint_defs())
        {
            if (estimator.get_joint_state(def.joint_id).position.has_value()) { ++tracked; }
        }

        spdlog::debug("pipeline: {} frames in {:.1f} s ({:.1f} fps polled, streams at {} fps), "
                      "{:.1f} detection(s)/frame, {}/{} joint(s) tracked, rest pose {}"
            , _frames, sec, _frames / sec, stream_rates
            , _frames > 0 ? static_cast<double>(_detections) / _frames : 0.0
            , tracked, pose::kNumJoints
            , has_rest_pose ? "captured" : "not captured"
        );

        _since = now;
        _frames = 0;
        _detections = 0;
    }

    void exo_pose_pipeline::frame_logger::reset()
    {
        *this = frame_logger{};
    }

    bool exo_pose_pipeline::try_get_annotated_frame(
        const std::size_t stream_idx,
        cv::Mat& out_img,
        cv::Mat& out_source,
        uint64_t& last_frame_id)
    {
        return _synced_tracker
            && _synced_tracker->try_get_annotated(stream_idx, out_img, out_source, last_frame_id);
    }

    hw::sensor_backend_t exo_pose_pipeline::sensor_backend(const std::size_t stream_idx) const
    {
        if (!_provider) { throw std::logic_error{ "pipeline: no source is open" }; }
        if (stream_idx >= _provider->stream_count()) {
            throw std::out_of_range{ std::format("pipeline: stream {} is not among the {} open", stream_idx, _provider->stream_count()) };
        }
        return _provider->get_stream_descriptor(stream_idx).sensor_backend;
    }

    pose::camera_view_t exo_pose_pipeline::camera_view(const std::size_t stream_idx) const
    {
        if (!_provider) { throw std::logic_error{ "pipeline: no source is open" }; }
        return _streams.at(stream_idx).view;
    }

    std::string exo_pose_pipeline::source_name() const
    {
        return _provider ? _provider->get_source_name() : std::string{};
    }

    Eigen::Vector2i exo_pose_pipeline::source_resolution(const std::size_t stream_idx) const
    {
        if (!_provider || stream_idx >= _provider->stream_count()) { return Eigen::Vector2i::Zero(); }
        return _provider->get_frame_resolution(stream_idx);
    }

    Eigen::Vector2i exo_pose_pipeline::source_full_resolution(const std::size_t stream_idx) const
    {
        if (!_provider || stream_idx >= _provider->stream_count()) { return Eigen::Vector2i::Zero(); }
        return _provider->get_full_frame_resolution(stream_idx);
    }

    std::optional<hw::roi_t> exo_pose_pipeline::effective_roi(const std::size_t stream_idx) const
    {
        if (!_provider || stream_idx >= _provider->stream_count()) { return std::nullopt; }
        return _provider->get_effective_roi(stream_idx);
    }

    void exo_pose_pipeline::set_roi(const std::size_t stream_idx, const std::optional<hw::roi_t>& roi)
    {
        if (!_provider || stream_idx >= _provider->stream_count()) { return; }

        // A recording declares one calibration per stream up front, frame size included, so the
        // frames that follow have to keep it.
        if (this->is_recording()) {
            spdlog::error("pipeline: cannot move the ROI while a recording is being written");
            return;
        }
        _provider->set_roi(stream_idx, roi);
    }

    float exo_pose_pipeline::source_fps(const std::size_t stream_idx) const
    {
        if (!_provider || stream_idx >= _provider->stream_count()) { return 0.0f; }
        return _provider->get_current_update_rate(stream_idx);
    }

    std::optional<hw::intrinsic_t> exo_pose_pipeline::intrinsics(const std::size_t stream_idx) const
    {
        // Color intrinsics of the open source; lets a diagnostic dump reproject
        // corners independently of whatever the pipeline computed.
        if (!_provider || !_provider->is_opened() || stream_idx >= _provider->stream_count()) { return std::nullopt; }
        return _provider->get_calibration(stream_idx).intrinsic;
    }

    uint32_t exo_pose_pipeline::current_frame_seq() const
    {
        return _provider ? _provider->get_current_frameset_seq() : 0;
    }

    bool exo_pose_pipeline::is_source_paused() const
    {
        return _provider && _provider->is_paused();
    }

    void exo_pose_pipeline::set_source_paused(bool paused)
    {
        if (!_provider) { return; }
        spdlog::info("pipeline: source {}", paused ? "paused" : "resumed");
        paused ? _provider->pause() : _provider->play();
    }

    bool exo_pose_pipeline::is_auto_repeat_enabled() const
    {
        return _provider && _provider->is_auto_repeat_enabled();
    }

    void exo_pose_pipeline::set_auto_repeat(bool enable)
    {
        if (!_provider) { return; }
        spdlog::info("pipeline: playback {} at the end of the recording",
            enable ? "starts over" : "stops");
        _provider->set_auto_repeat(enable);
    }

    void exo_pose_pipeline::seek_to_begin()
    {
        if (!_provider) { return; }
        spdlog::info("pipeline: seek to the beginning of the recording");
        _provider->seek_recording_to_begin();
    }

    void exo_pose_pipeline::seek_to_end()
    {
        if (!_provider) { return; }
        spdlog::info("pipeline: seek to the end of the recording");
        _provider->seek_recording_to_end();
    }

} // namespace net
