#include "hw/sensor_frame_provider.hh"
#include "hw/sensor_frame_observer.hh"
#include "pose/tag_detector.hh"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <exception>
#include <format>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace
{
    class tag_detector_demo_observer final : public hw::sensor_frame_observer
    {
    public:
        tag_detector_demo_observer(const hw::sensor_frame_provider& provider, double tag_size_m)
            : _provider{ provider }
            , _tag_size_m{ tag_size_m }
        { }

        bool stream_ended() const {
            return _stream_ended.load();
        }

        // Returns the newest annotated frame if it changed since last_seq.
        bool try_get_annotated(cv::Mat& out, uint64_t& last_seq)
        {
            std::scoped_lock lk{ _mtx };
            if (_seq == last_seq || _latest.empty()) { return false; }
            out = _latest;
            last_seq = _seq;
            return true;
        }

    public:
        // ----- hw::sensor_frame_observer interfaces -----

        void on_sensor_frame_update(const std::shared_ptr<hw::sensor_frame>& frame) override
        {
            // Lazily build the detector once intrinsics are known (after open).
            if (!_detector.has_value()) {
                pose::tag_detector::options_t opt;
                opt.intrinsics = _provider.get_calibration().color_intr;
                opt.tag_size_m = _tag_size_m;
                _detector.emplace(opt);
            }

            cv::Mat annotated = frame->color_image.clone();
            const auto detections = _detector->detect(annotated);
            pose::draw_tag_detections(annotated, detections);

            thread_local std::array<char, 64> hud{};
            std::format_to_n(hud.data(), hud.size() - 1, "tags: {}  fps: {:.0f}",
                detections.size(), _provider.get_current_update_rate());
            cv::putText(annotated, hud.data(), { 10, 30 }, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar{ 0, 0, 255 }, 2);

            {
                std::scoped_lock lk{ _mtx };
                _latest = std::move(annotated);
                ++_seq;
            }

            if ((frame->id() == 1 || frame->id() % 30 == 0)) {
                if (!detections.empty() && detections.front().pose) {
                    const Eigen::Vector3d t = detections.front().pose->translation();
                    spdlog::info("frame #{:04d} tags={} id{} pos=({:.3f}, {:.3f}, {:.3f})m",
                        frame->id(), detections.size(), detections.front().id, t.x(), t.y(), t.z());
                }
                else {
                    spdlog::info("frame #{:04d} tags={} fps={:.1f}",
                        frame->id(), detections.size(), _provider.get_current_update_rate());
                }
            }
        }

        void on_sensor_stream_reset() override { _stream_ended.store(false); }
        void on_sensor_stream_end() override { _stream_ended.store(true); }

    private:
        const hw::sensor_frame_provider& _provider;
        double _tag_size_m{};
        std::optional<pose::tag_detector> _detector;
        std::mutex _mtx;
        cv::Mat _latest;
        uint64_t _seq{ 0 };
        std::atomic<bool> _stream_ended{ false };
    };

} // namespace

int main(int argc, char** argv)
{
    int retval{ -1 };

    CLI::App app{ "exo-skeleton-pose: pose detection demo (Orbbec K4A backend, temporary)" };
    uint32_t device_index{ 0 };
    std::string input_path;
    double tag_size_m{ 0.05 };
    std::optional<int32_t> exposure_us;
    std::optional<int32_t> gain;
    app.add_option("-d,--device", device_index, "Device index to open")->default_val(0);
    app.add_option("-i,--input", input_path, "MKV recording file path to open");
    app.add_option("-s,--tag-size", tag_size_m, "AprilTag black-square edge length [m]")->default_val(0.05);
    app.add_option("-e,--exposure-us", exposure_us, "Manual color exposure [us] (default: auto)");
    app.add_option("-g,--gain", gain, "Manual color gain (default: auto)");
    CLI11_PARSE(app, argc, argv);

    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    spdlog::set_level(spdlog::level::info);
    spdlog::flush_on(spdlog::level::info);

    try
    {
        auto provider = std::make_shared<hw::sensor_frame_provider>();
        auto observer = std::make_shared<tag_detector_demo_observer>(*provider, tag_size_m);
        provider->add_observer(observer);

        // Stop on Ctrl+C.
        static std::atomic_flag stop_requested;
        std::signal(SIGINT, [](int) { stop_requested.test_and_set(); });

        const bool is_recording = !input_path.empty();
        const bool opened = is_recording
            ? provider->open_recording(input_path)
            : provider->open_device(device_index, exposure_us, gain);

        if (!opened)
        {
            spdlog::error("failed to open camera source");
            return retval;
        }

        const auto res = provider->get_color_camera_resolution();
        const auto fov = provider->get_color_camera_fov();
        const auto& ci = provider->get_calibration().color_intr;
        spdlog::info("source '{}' opened", provider->get_source_name());
        spdlog::info("Color {}x{}  intrinsic fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}  FOV {:.1f}x{:.1f} deg"
            , res.x(), res.y()
            , ci.fx, ci.fy
            , ci.cx, ci.cy
            , fov.x(), fov.y()
        );

        const std::string window_name{ "tag detector demo (q to quit)" };
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);

        cv::Mat frame;
        uint64_t last_seq = 0;
        while (!stop_requested.test())
        {
            if (is_recording && observer->stream_ended()) { break; }

            if (observer->try_get_annotated(frame, last_seq)) {
                cv::imshow(window_name, frame);
            }

            const int key = cv::waitKey(15);
            if (key == 27 || key == 'q') { break; } // ESC / q
        }

        spdlog::info("shutting down...");
        cv::destroyAllWindows();
        provider->close();
        retval = 0;
    }
    catch (const std::exception& e)
    {
        spdlog::error("fatal: {}", e.what());
    }

    return retval;
}
