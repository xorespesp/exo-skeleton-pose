#include "k4a_frame_source.hh"

#include "k4a_frameset.hh"

#include <k4a/k4a.hpp>
#include <k4arecord/playback.h>

#include <spdlog/spdlog.h>

#include <cmath>
#include <optional>
#include <stdexcept>

namespace hw
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        // Copy the color camera parameters into the SDK-agnostic calibration_t.
        calibration_t to_calibration_t(const k4a_calibration_t& k4a_calib)
        {
            const k4a_calibration_camera_t& cc = k4a_calib.color_camera_calibration;
            const auto& p = cc.intrinsics.parameters.param; // {cx,cy,fx,fy,k1..k6,codx,cody,p2,p1,...}

            calibration_t out{};
            out.color_intr = intrinsic_t{
                p.fx, p.fy, p.cx, p.cy,
                cc.resolution_width, cc.resolution_height
            };
            out.color_dist = distortion_t{
                p.k1, p.k2, p.k3, p.k4, p.k5, p.k6, p.p1, p.p2
            };
            out.color_resolution = Eigen::Vector2i{ cc.resolution_width, cc.resolution_height };

            const float h_fov = (p.fx > 0.0f)
                ? static_cast<float>(2.0 * std::atan(cc.resolution_width / (2.0 * p.fx)) * 180.0 / kPi)
                : 0.0f;
            const float v_fov = (p.fy > 0.0f)
                ? static_cast<float>(2.0 * std::atan(cc.resolution_height / (2.0 * p.fy)) * 180.0 / kPi)
                : 0.0f;
            out.color_fov = Eigen::Vector2f{ h_fov, v_fov };

            return out;
        }
    } // namespace

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // k4a_device_capturer
    ///////////////////////////////////////////////////////////////////////////////////////////////

    k4a_device_capturer::~k4a_device_capturer()
    {
        this->close();
    }

    bool k4a_device_capturer::open(
        const uint32_t device_index, 
        const color_controls& controls) noexcept try
    {
        std::scoped_lock lk{ _mtx };
        if (_device) { throw std::runtime_error{ "k4a_device_capturer: already opened" }; }

        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        config.depth_mode = K4A_DEPTH_MODE_OFF; // RGB only
        config.synchronized_images_only = false; // no depth to sync with

        k4a_device_t device = nullptr;
        if (K4A_FAILED(::k4a_device_open(device_index, &device))) {
            throw std::runtime_error{ "k4a_device_capturer: failed to open device" };
        }

        k4a_calibration_t k4a_calib{};
        if (K4A_FAILED(::k4a_device_get_calibration(
            device, config.depth_mode, config.color_resolution, &k4a_calib)))
        {
            ::k4a_device_close(device);
            throw std::runtime_error{ "k4a_device_capturer: failed to get calibration" };
        }

        if (K4A_FAILED(::k4a_device_start_cameras(device, &config))) {
            ::k4a_device_close(device);
            throw std::runtime_error{ "k4a_device_capturer: failed to start cameras" };
        }

        // Manual color controls (non-fatal if unsupported).
        if (controls.exposure_us.has_value()) {
            if (K4A_FAILED(::k4a_device_set_color_control(device,
                K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, 
                K4A_COLOR_CONTROL_MODE_MANUAL,
                *controls.exposure_us))) {
                spdlog::warn("k4a: failed to set manual exposure {}us", *controls.exposure_us);
            } else {
                spdlog::info("k4a: manual exposure {}us", *controls.exposure_us);
            }
        }
        if (controls.gain.has_value()) {
            if (K4A_FAILED(::k4a_device_set_color_control(device,
                K4A_COLOR_CONTROL_GAIN, 
                K4A_COLOR_CONTROL_MODE_MANUAL, 
                *controls.gain))) {
                spdlog::warn("k4a: failed to set manual gain {}", *controls.gain);
            } else {
                spdlog::info("k4a: manual gain {}", *controls.gain);
            }
        }

        // serial number (for logging; non-fatal if it fails)
        std::string serialnum;
        {
            size_t needed = 0;
            if (::k4a_device_get_serialnum(device, nullptr, &needed) == K4A_BUFFER_RESULT_TOO_SMALL && needed > 1) {
                serialnum.resize(needed);
                if (::k4a_device_get_serialnum(device, &serialnum[0], &needed) == K4A_BUFFER_RESULT_SUCCEEDED
                    && !serialnum.empty() && serialnum.back() == '\0') {
                    // std::string expects there to not be as null terminator at the end of its data but
                    // k4a_device_get_serialnum adds a null terminator, so we drop the last character of the string after we
                    // get the result back.
                    serialnum.pop_back();
                }
            }
        }

        _device = device;
        _config = config;
        _calib = to_calibration_t(k4a_calib);
        _serialnum = std::move(serialnum);

        spdlog::info("k4a device opened (S/N: {})", _serialnum.empty() ? "<unknown>" : _serialnum);
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

    void k4a_device_capturer::close()
    {
        std::scoped_lock lk{ _mtx };
        if (_device) {
            ::k4a_device_stop_cameras(_device);
            ::k4a_device_close(_device);
            _device = nullptr;
        }
    }

    std::unique_ptr<sensor_frameset> k4a_device_capturer::fetch_next_sensor_frameset()
    {
        std::scoped_lock lk{ _mtx };
        if (!_device) { return nullptr; }

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
            return nullptr; // provider retries
        }

        k4a::capture capture{ capture_handle };
        if (!capture.is_valid()) { return nullptr; }

        return std::make_unique<k4a_frameset>(std::move(capture));
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // k4a_record_player
    ///////////////////////////////////////////////////////////////////////////////////////////////

    namespace
    {
        // Step in the given direction until a capture with a valid color image is found,
        // and return its device timestamp.
        std::optional<std::chrono::microseconds> seek_color_timestamp(
            k4a_playback_t playback, bool backward)
        {
            for (;;) {
                k4a_capture_t handle = nullptr;
                const k4a_stream_result_t result = backward
                    ? ::k4a_playback_get_previous_capture(playback, &handle)
                    : ::k4a_playback_get_next_capture(playback, &handle);

                if (result != K4A_STREAM_RESULT_SUCCEEDED) { return std::nullopt; }

                k4a::capture capture{ handle };
                const k4a::image color = capture.get_color_image();
                if (color.is_valid() && color.get_size() > 0) {
                    return color.get_device_timestamp();
                }
            }
        }
    } // namespace

    k4a_record_player::~k4a_record_player()
    {
        this->close();
    }

    bool k4a_record_player::open(
        const std::filesystem::path& recording_file,
        const std::optional<k4a_image_format_t> color_conversion_format
    ) noexcept try
    {
        std::scoped_lock lk{ _mtx };
        if (_playback) { throw std::runtime_error{ "k4a_record_player: already opened" }; }

        if (!std::filesystem::is_regular_file(recording_file)) {
            throw std::invalid_argument{ "k4a_record_player: invalid recording file path" };
        }

        k4a_playback_t playback = nullptr;
        if (K4A_FAILED(::k4a_playback_open(recording_file.string().c_str(), &playback))) {
            throw std::runtime_error{ "k4a_record_player: failed to open recording" };
        }

        k4a_record_configuration_t record_config{};
        if (K4A_FAILED(::k4a_playback_get_record_configuration(playback, &record_config))) {
            ::k4a_playback_close(playback);
            throw std::runtime_error{ "k4a_record_player: failed to get record configuration" };
        }

        if (!record_config.color_track_enabled) {
            ::k4a_playback_close(playback);
            throw std::runtime_error{ "k4a_record_player: recording has no color track" };
        }

        k4a_calibration_t k4a_calib{};
        if (K4A_FAILED(::k4a_playback_get_calibration(playback, &k4a_calib))) {
            ::k4a_playback_close(playback);
            throw std::runtime_error{ "k4a_record_player: failed to get calibration" };
        }

        if (color_conversion_format.has_value()) {
            if (K4A_FAILED(::k4a_playback_set_color_conversion(playback, color_conversion_format.value()))) {
                ::k4a_playback_close(playback);
                throw std::runtime_error{ "k4a_record_player: failed to set color conversion" };
            }
        }

        // Resolve first/last color timestamps.
        std::chrono::microseconds first_ts{ 0 }, last_ts{ 0 };
        {
            ::k4a_playback_seek_timestamp(
                playback, record_config.start_timestamp_offset_usec, K4A_PLAYBACK_SEEK_DEVICE_TIME);
            const auto first = seek_color_timestamp(playback, /*backward*/false);

            ::k4a_playback_seek_timestamp(playback, 0, K4A_PLAYBACK_SEEK_END);
            const auto last = seek_color_timestamp(playback, /*backward*/true);

            if (!first.has_value() || !last.has_value()) {
                ::k4a_playback_close(playback);
                throw std::runtime_error{ "k4a_record_player: failed to resolve record timestamps" };
            }
            first_ts = first.value();
            last_ts = last.value();

            // Rewind to the start, ready to play.
            ::k4a_playback_seek_timestamp(playback, first_ts.count(), K4A_PLAYBACK_SEEK_DEVICE_TIME);
        }

        _playback = playback;
        _record_config = record_config;
        _calib = to_calibration_t(k4a_calib);
        _first_ts = first_ts;
        _last_ts = last_ts;

        spdlog::info("k4a recording opened (length: {} ms)",
            std::chrono::duration_cast<std::chrono::milliseconds>(_last_ts - _first_ts).count());
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("k4a_record_player::open failed: {}", e.what());
        return false;
    }

    bool k4a_record_player::is_valid() const
    {
        std::scoped_lock lk{ _mtx };
        return _playback != nullptr;
    }

    void k4a_record_player::close()
    {
        std::scoped_lock lk{ _mtx };
        if (_playback) {
            ::k4a_playback_close(_playback);
            _playback = nullptr;
        }
    }

    std::unique_ptr<sensor_frameset> k4a_record_player::fetch_next_sensor_frameset()
    {
        std::scoped_lock lk{ _mtx };
        if (!_playback) { return nullptr; }

        k4a_capture_t handle = nullptr;
        k4a_stream_result_t result = ::k4a_playback_get_next_capture(_playback, &handle);

        if (result == K4A_STREAM_RESULT_EOF) {
            if (!_auto_repeat) { return nullptr; }
            // auto-repeat: seek to start and retry
            ::k4a_playback_seek_timestamp(_playback, _first_ts.count(), K4A_PLAYBACK_SEEK_DEVICE_TIME);
            result = ::k4a_playback_get_next_capture(_playback, &handle);
        }

        if (result == K4A_STREAM_RESULT_FAILED) {
            throw std::runtime_error{ "k4a_record_player: failed to get next capture" };
        }
        if (result != K4A_STREAM_RESULT_SUCCEEDED) { return nullptr; }

        k4a::capture capture{ handle };
        if (!capture.is_valid()) { return nullptr; }

        return std::make_unique<k4a_frameset>(std::move(capture));
    }

    void k4a_record_player::seek_begin()
    {
        std::scoped_lock lk{ _mtx };
        if (_playback) {
            ::k4a_playback_seek_timestamp(_playback, _first_ts.count(), K4A_PLAYBACK_SEEK_DEVICE_TIME);
        }
    }

    void k4a_record_player::seek_end()
    {
        std::scoped_lock lk{ _mtx };
        if (_playback) {
            ::k4a_playback_seek_timestamp(_playback, _last_ts.count(), K4A_PLAYBACK_SEEK_DEVICE_TIME);
        }
    }

    void k4a_record_player::seek_timestamp(std::chrono::microseconds offset)
    {
        std::scoped_lock lk{ _mtx };
        if (_playback) {
            ::k4a_playback_seek_timestamp(_playback, offset.count(), K4A_PLAYBACK_SEEK_DEVICE_TIME);
        }
    }

    bool k4a_record_player::auto_repeat_enabled() const
    {
        std::scoped_lock lk{ _mtx };
        return _auto_repeat;
    }

    void k4a_record_player::enable_auto_repeat(bool enable)
    {
        std::scoped_lock lk{ _mtx };
        _auto_repeat = enable;
    }

} // namespace hw
