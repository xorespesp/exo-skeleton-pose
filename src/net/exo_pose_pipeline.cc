#include "exo_pose_pipeline.hh"

#include "hw/sensor_frame_observer.hh"
#include "pose/tag_detector.hh"

#include <spdlog/spdlog.h>

#include <atomic>
#include <charconv>
#include <mutex>
#include <utility>

namespace net
{
    // --- observer (worker thread) ------------------------------------------------
    // Worker-thread tag detection; latches detections & annotated frame for the loop thread to pull.
    class pose_frame_observer final : public hw::sensor_frame_observer
    {
    public:
        pose_frame_observer(const hw::sensor_frame_provider& provider, double tag_size_m, bool annotate)
            : _provider{ provider }, _tag_size_m{ tag_size_m }, _annotate{ annotate }
        { }

        // Returns false if nothing new since `last_seq`, else copies out + advances it.
        bool try_get(
            std::vector<pose::tag_detection_t>& out_dets,
            std::chrono::microseconds& out_timestamp,
            uint64_t& last_seq)
        {
            std::scoped_lock lk{ _mtx };
            if (_seq == last_seq) { return false; }
            out_dets = _detections;
            out_timestamp = _timestamp;
            last_seq = _seq;
            return true;
        }

        // Like try_get, plus the annotated frame image.
        // Empty image if annotation is off.
        bool try_get_frame(
            cv::Mat& out_img,
            std::vector<pose::tag_detection_t>& out_dets,
            std::chrono::microseconds& out_timestamp,
            uint64_t& last_seq)
        {
            std::scoped_lock lk{ _mtx };
            if (_seq == last_seq) { return false; }
            out_img = _annotated;
            out_dets = _detections;
            out_timestamp = _timestamp;
            last_seq = _seq;
            return true;
        }

        // True once per stream-end signal (consumes the latched flag).
        bool consume_stream_ended_signal() noexcept {
            return _stream_ended.exchange(false);
        }

    public:
        void on_sensor_frame_update(const std::shared_ptr<hw::sensor_frame>& frame) override
        {
            if (!_detector.has_value()) // built once intrinsics are known (after open)
            {
                pose::tag_detector::options_t opt;
                opt.intrinsics = _provider.get_calibration().color_intr;
                opt.tag_size_m = _tag_size_m;
                _detector.emplace(opt);
            }

            cv::Mat annotated;
            std::vector<pose::tag_detection_t> detections;
            if (_annotate) {
                annotated = frame->color_image.clone();
                detections = _detector.value().detect(annotated);
                pose::draw_tag_detections(annotated, detections);
            } else {
                detections = _detector.value().detect(frame->color_image);
            }

            std::scoped_lock lk{ _mtx };
            _annotated = std::move(annotated);
            _detections = std::move(detections);
            _timestamp = frame->timestamp;
            ++_seq;
        }

        void on_sensor_stream_reset() override {}
        void on_sensor_stream_end() override {
            _stream_ended = true;
        }

    private:
        const hw::sensor_frame_provider& _provider;
        double _tag_size_m{};
        bool _annotate{ false }; // keep an annotated frame copy for a monitor GUI
        std::optional<pose::tag_detector> _detector; // built once intrinsics are known (after open)
        std::mutex _mtx;
        cv::Mat _annotated; // annotated frame
        std::vector<pose::tag_detection_t> _detections;
        std::chrono::microseconds _timestamp{ 0 }; // device timestamp of the latched frame
        uint64_t _seq{ 0 };
        std::atomic<bool> _stream_ended{ false }; // set by the worker thread on stream end
    };

    // --- exo_pose_pipeline -------------------------------------------------------
    exo_pose_pipeline::exo_pose_pipeline(double default_tag_size_m, bool annotate_frames)
        : _default_tag_size_m{ default_tag_size_m }, _annotate_frames{ annotate_frames }
    { }

    exo_pose_pipeline::~exo_pose_pipeline() = default;

    bool exo_pose_pipeline::open_source(
        const std::string& source,
        double tag_size_m,
        std::optional<int32_t> exposure_us,
        std::optional<int32_t> gain)
    {
        _status_changed = true; // opening a source changes the reported status (even on failure)

        auto new_provider = std::make_shared<hw::sensor_frame_provider>();
        auto new_observer = std::make_shared<pose_frame_observer>(*new_provider, tag_size_m, _annotate_frames);
        new_provider->add_observer(new_observer);

        // Parse: a full unsigned integer is a device index, anything else a path.
        uint32_t device_index{};
        const auto [ptr, ec] = std::from_chars(source.data(), source.data() + source.size(), device_index);
        const bool is_device = (ec == std::errc{} && ptr == source.data() + source.size());

        const bool ok = is_device
            ? new_provider->open_device(device_index, exposure_us, gain)
            : new_provider->open_recording(source);

        if (!ok) {
            spdlog::error("failed to open source '{}'", source);
            return false;
        }

        _provider = std::move(new_provider); // old provider closes/joins here
        _observer = std::move(new_observer);
        _is_recording = !is_device;
        _last_seq = 0;
        _estimator.clear_rest_pose();
        spdlog::info("source '{}' opened", _provider->get_source_name());
        return true;
    }

    bool exo_pose_pipeline::open_device(uint32_t index, std::optional<int32_t> exposure_us, std::optional<int32_t> gain)
    {
        return this->open_source(std::to_string(index), _default_tag_size_m, exposure_us, gain);
    }

    bool exo_pose_pipeline::open_recording(const std::string& path)
    {
        return this->open_source(path, _default_tag_size_m, std::nullopt, std::nullopt);
    }

    void exo_pose_pipeline::close_source()
    {
        _status_changed = true;
        _provider.reset(); // stops/joins the worker thread
        _observer.reset();
        _is_recording = false;
        _last_seq = 0;
    }

    bool exo_pose_pipeline::is_source_open() const { return static_cast<bool>(_provider); }
    bool exo_pose_pipeline::is_source_recording() const { return _is_recording; }

    bool exo_pose_pipeline::calibrate_rest_pose()
    {
        _status_changed = true;
        return _estimator.calibrate_rest_pose();
    }

    void exo_pose_pipeline::clear_rest_pose()
    {
        _status_changed = true;
        _estimator.clear_rest_pose();
    }

    pose::exo_pose_estimator& exo_pose_pipeline::estimator() { return _estimator; }
    const pose::exo_pose_estimator& exo_pose_pipeline::estimator() const { return _estimator; }

    exo_pose_pipeline::poll_result exo_pose_pipeline::poll()
    {
        poll_result r{};

        // Detections: pull the newest latched frame and recompute joint states.
        if (_observer && _observer->try_get(_detections, _last_timestamp, _last_seq))
        {
            _estimator.update(_detections, _last_timestamp);
            r.new_pose = true;
        }

        // Stream end: consume the one-shot signal the worker thread raises at end of stream.
        r.stream_ended = _observer && _observer->consume_stream_ended_signal();

        // Status: consume the flag set by the last source/rest command.
        r.status_changed = std::exchange(_status_changed, false);
        return r;
    }

    bool exo_pose_pipeline::try_get_annotated_frame(
        cv::Mat& out_img,
        std::vector<pose::tag_detection_t>& out_dets,
        std::chrono::microseconds& out_ts,
        uint64_t& last_seq)
    {
        return _observer && _observer->try_get_frame(out_img, out_dets, out_ts, last_seq);
    }

    std::string exo_pose_pipeline::source_name() const
    {
        return _provider ? _provider->get_source_name() : std::string{};
    }

    Eigen::Vector2i exo_pose_pipeline::source_resolution() const
    {
        return _provider ? _provider->get_color_camera_resolution() : Eigen::Vector2i::Zero();
    }

    uint32_t exo_pose_pipeline::current_frame_id() const
    {
        return _provider ? _provider->get_current_frame_id() : 0;
    }

} // namespace net
