#include "k4a_frame_source.hh"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <spdlog/spdlog.h>

#include <format>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace hw
{
    namespace
    {
        template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

        // SDK 가 붙이는 널 종단을 떼고 돌려준다. 읽지 못하면 비어 있다.
        std::string read_serialnum(k4a_device_t device)
        {
            std::string serialnum;
            size_t needed = 0;
            if (::k4a_device_get_serialnum(device, nullptr, &needed) == K4A_BUFFER_RESULT_TOO_SMALL && needed > 1)
            {
                serialnum.resize(needed);
                if (::k4a_device_get_serialnum(device, &serialnum[0], &needed) == K4A_BUFFER_RESULT_SUCCEEDED
                    && !serialnum.empty() && serialnum.back() == '\0')
                {
                    serialnum.pop_back();
                }
                else
                {
                    serialnum.clear();
                }
            }
            return serialnum;
        }

        ::k4a_wired_sync_mode_t to_sdk_wired_sync_mode(const k4a_sync_role_t role)
        {
            switch (role) {
            case k4a_sync_role_t::master:      return K4A_WIRED_SYNC_MODE_MASTER;
            case k4a_sync_role_t::subordinate: return K4A_WIRED_SYNC_MODE_SUBORDINATE;
            case k4a_sync_role_t::standalone:  break;
            }
            return K4A_WIRED_SYNC_MODE_STANDALONE;
        }

        const char* sync_role_name(const k4a_sync_role_t role)
        {
            switch (role) {
            case k4a_sync_role_t::master:      return "master";
            case k4a_sync_role_t::subordinate: return "subordinate";
            case k4a_sync_role_t::standalone:  break;
            }
            return "standalone";
        }
    } // namespace

    // Copy the color camera parameters into the SDK-agnostic calibration_t.
    calibration_t k4a_to_calibration(const k4a_calibration_t& k4a_calib)
    {
        const k4a_calibration_camera_t& cc = k4a_calib.color_camera_calibration;
        const auto& p = cc.intrinsics.parameters.param; // {cx,cy,fx,fy,k1..k6,codx,cody,p2,p1,...}

        calibration_t out{};
        out.intrinsic = intrinsic_t{
            p.fx, p.fy, p.cx, p.cy,
            Eigen::Vector2i{ cc.resolution_width, cc.resolution_height }
        };
        out.distortion = distortion_t{
            p.k1, p.k2, p.k3, p.k4, p.k5, p.k6, p.p1, p.p2
        };
        out.frame_resolution = Eigen::Vector2i{ cc.resolution_width, cc.resolution_height };

        return out;
    }

    cv::Mat k4a_color_to_mat(
        const k4a::image& color,
        const frame_format_t format,
        const std::optional<roi_t> roi)
    {
        if (!color.is_valid() || color.get_size() == 0) {
            throw std::runtime_error{ "k4a_color_to_mat: color image is invalid" };
        }

        const bool want_gray = (format == frame_format_t::gray8);

        // A view of `full` restricted to the ROI, or `full` itself when the whole frame is wanted.
        const auto narrow = [&roi](const cv::Mat& full) {
            return roi.has_value()
                ? full(cv::Rect{ roi->x, roi->y, roi->width, roi->height })
                : full;
        };

        cv::Mat out;
        switch (color.get_format()) {
        case K4A_IMAGE_FORMAT_COLOR_BGRA32: {
            const cv::Mat bgra_view( // Zero-copy view over the k4a BGRA buffer
                color.get_height_pixels(),
                color.get_width_pixels(),
                CV_8UC4,
                const_cast<void*>(static_cast<const void*>(color.get_buffer())),
                static_cast<size_t>(color.get_stride_bytes())
            );
            // Narrowing ahead of the conversion is the whole point: it reads and writes only
            // the pixels that will be delivered, and `out` comes out owning them.
            cv::cvtColor(narrow(bgra_view), out,
                want_gray ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGRA2BGR
            );
            break;
        }
        case K4A_IMAGE_FORMAT_COLOR_MJPG: {
            const cv::Mat jpeg_view( // Zero-copy view of the k4a JPEG buffer
                1,
                static_cast<int>(color.get_size()),
                CV_8UC1,
                const_cast<void*>(static_cast<const void*>(color.get_buffer()))
            );
            // The decoder needs the whole frame, so here the ROI can only follow it.
            const cv::Mat decoded = cv::imdecode(jpeg_view,
                want_gray ? cv::IMREAD_GRAYSCALE : cv::IMREAD_COLOR
            );
            out = narrow(decoded);
            break;
        }
        default:
            throw std::runtime_error{
                "k4a_color_to_mat: unsupported color image format (only MJPG / BGRA32)"
            };
        }

        // The MJPG path can leave a view of the decoded frame, which dies here.
        return out.isSubmatrix() ? out.clone() : out;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // k4a_device_capturer
    ///////////////////////////////////////////////////////////////////////////////////////////////

    k4a_device_capturer::~k4a_device_capturer()
    {
        this->close();
    }

    bool k4a_device_capturer::open(const k4a_device_config_t& config) noexcept try
    {
        std::scoped_lock lk{ _mtx };
        if (_device) { throw std::runtime_error{ "k4a_device_capturer: already opened" }; }

        k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        device_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        device_config.depth_mode = K4A_DEPTH_MODE_OFF; // RGB only
        device_config.synchronized_images_only = false; // no depth to sync with

        // 유선 싱크. subordinate 의 지연은 그 역할에서만 의미를 가지고, SDK 도 그때만 받는다.
        device_config.wired_sync_mode = to_sdk_wired_sync_mode(config.wired_sync_mode);
        device_config.subordinate_delay_off_master_usec =
            (config.wired_sync_mode == k4a_sync_role_t::subordinate) ? config.subordinate_delay_off_master_usec : 0;

        std::string serialnum;
        const k4a_device_t device = std::visit(overloaded{
            [&serialnum](const device_index_t& by_index)
            {
                k4a_device_t opened = nullptr;
                if (K4A_FAILED(::k4a_device_open(by_index.value, &opened))) {
                    throw std::runtime_error{ "k4a_device_capturer: failed to open device" };
                }
                serialnum = read_serialnum(opened);
                return opened;
            },
            [&serialnum](const device_serial_t& by_serial)
            {
                // C API 는 인덱스로만 열리므로 하나씩 열어 시리얼을 맞춰 본다.
                const uint32_t count = ::k4a_device_get_installed_count();
                for (uint32_t device_index = 0; device_index < count; ++device_index)
                {
                    k4a_device_t candidate = nullptr;
                    if (K4A_FAILED(::k4a_device_open(device_index, &candidate))) { continue; }
                    serialnum = read_serialnum(candidate);
                    if (serialnum == by_serial.value) { return candidate; }
                    ::k4a_device_close(candidate);
                }
                throw std::runtime_error{ std::format("k4a_device_capturer: no device with serial '{}'", by_serial.value) };
            },
        }, config.device_selector);

        k4a_calibration_t k4a_calib{};
        if (K4A_FAILED(::k4a_device_get_calibration(
            device, device_config.depth_mode, device_config.color_resolution, &k4a_calib)))
        {
            ::k4a_device_close(device);
            throw std::runtime_error{ "k4a_device_capturer: failed to get calibration" };
        }

        if (K4A_FAILED(::k4a_device_start_cameras(device, &device_config))) {
            ::k4a_device_close(device);
            throw std::runtime_error{ "k4a_device_capturer: failed to start cameras" };
        }

        // Color controls (non-fatal if unsupported). The device retains previous
        // settings across opens while it stays connected, so nullopt must actively
        // reset the control to AUTO rather than leave a stale manual value in place.
        const auto apply_color_control = [device](k4a_color_control_command_t cmd,
                                                  std::optional<int32_t> value, const char* name)
        {
            const auto mode = value.has_value() ? K4A_COLOR_CONTROL_MODE_MANUAL : K4A_COLOR_CONTROL_MODE_AUTO;
            if (K4A_FAILED(::k4a_device_set_color_control(device, cmd, mode, value.value_or(0)))) {
                spdlog::warn("k4a: failed to set {} {}", name, value.has_value() ? "manual" : "auto");
            } else if (value.has_value()) {
                spdlog::info("k4a: manual {} {}", name, *value);
            } else {
                spdlog::info("k4a: auto {}", name);
            }
        };
        apply_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, config.exposure_us, "exposure");
        apply_color_control(K4A_COLOR_CONTROL_GAIN, config.gain, "gain");

        _device = device;
        _config = device_config;
        _calib = k4a_to_calibration(k4a_calib);
        _serialnum = std::move(serialnum);
        _frame_format = config.frame_format;
        _roi.reset();
        _clock_anchor.reset();
        _clock_warmup_samples = 0;

        spdlog::info("k4a device opened (S/N: {}, {}, sync {})"
            , _serialnum.empty() ? "<unknown>" : _serialnum
            , frame_format_to_str(_frame_format)
            , sync_role_name(config.wired_sync_mode)
        );
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("k4a_device_capturer::open failed: {}", e.what());
        return false;
    }

    bool k4a_device_capturer::is_valid() const
    {
        std::scoped_lock lk{ _mtx };
        return _device != nullptr;
    }

    stream_descriptor_t k4a_device_capturer::get_stream_descriptor() const
    {
        std::scoped_lock lk{ _mtx };
        return stream_descriptor_t{
            .sensor_backend = sensor_backend_t::k4a,
            .device_serial = _serialnum,
        };
    }

    void k4a_device_capturer::close()
    {
        std::scoped_lock lk{ _mtx };
        if (_device) {
            ::k4a_device_stop_cameras(_device);
            ::k4a_device_close(_device);
            _device = nullptr;
        }
    }

    std::optional<roi_t> k4a_device_capturer::try_set_roi(const roi_t& roi)
    {
        std::scoped_lock lk{ _mtx };

        const roi_t clipped = clamp_roi(roi,
            _calib.frame_resolution.x(),
            _calib.frame_resolution.y()
        );

        if (clipped.is_empty()) { return std::nullopt; }

        _roi = clipped;
        return _roi;
    }

    std::optional<sensor_frameset> k4a_device_capturer::fetch_next_sensor_frameset()
    {
        std::scoped_lock lk{ _mtx };
        if (!_device) { return std::nullopt; }

        // 다음 컬러 이미지를 기다린다. 타임아웃이면 비어 있다.
        const auto wait_color_image = [this]() -> std::optional<k4a::image>
        {
            k4a::image color;
            do
            {
                k4a_capture_t capture_handle = nullptr;
                const k4a_wait_result_t wait_result = ::k4a_device_get_capture(
                    _device,
                    &capture_handle,
                    1000 /* ms; finite so the polling thread can wake for join */
                );

                if (wait_result == K4A_WAIT_RESULT_FAILED) {
                    throw std::runtime_error{ "k4a_device_capturer: failed to get capture" };
                }
                if (wait_result == K4A_WAIT_RESULT_TIMEOUT) {
                    return std::nullopt; // provider retries
                }

                const k4a::capture capture{ capture_handle };
                if (!capture.is_valid()) { return std::nullopt; }

                color = capture.get_color_image();
            }
            while (!color.is_valid() || color.get_size() == 0); // 컬러 없는 캡처(depth 만 온 것)는 넘긴다

            return color;
        };

        // 이 캡처의 Unix 시각. 앵커가 서기 전의 캡처는 warmup 표본으로 쓰이고 비어 있는 값이 돌아온다.
        const auto resolve_capture_timestamp = [this](const k4a::image& color) -> std::optional<timestamp_t>
        {
            const std::chrono::nanoseconds device_ts = color.get_device_timestamp();
            if (_clock_warmup_samples < kClockWarmupSamples)
            {
                // 표본 중 offset 이 가장 작은 앵커가 남는다.
                const clock_anchor_t candidate{ device_ts };
                if (!_clock_anchor.has_value() || candidate.offset() < _clock_anchor->offset()) { _clock_anchor = candidate; }
                ++_clock_warmup_samples;
                return std::nullopt;
            }

            return _clock_anchor->to_unix(device_ts);
        };

        // 시각을 받은 캡처가 나올 때까지 받는다.
        std::optional<k4a::image> color;
        std::optional<timestamp_t> timestamp;
        do
        {
            color = wait_color_image();
            if (!color.has_value()) { return std::nullopt; }

            timestamp = resolve_capture_timestamp(*color);
        }
        while (!timestamp.has_value());

        // Make a new sensor frameset and return it. (deep copied)
        return sensor_frameset{ std::make_shared<sensor_frame>(
            k4a_color_to_mat(*color, _frame_format, _roi),
            _frame_format,
            *timestamp
        ) };
    }

} // namespace hw
