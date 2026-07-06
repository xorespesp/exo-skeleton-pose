#pragma once
#include "sensor_frame_source.hh"

#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include <filesystem>
#include <mutex>
#include <optional>
#include <string>

namespace hw
{
    ////////////////////////////////////////////////////////////////////////////////////////
    // Orbbec K4A Wrapper backend.
    ////////////////////////////////////////////////////////////////////////////////////////

    // Live camera source.
    class k4a_device_capturer final : public sensor_frame_source {
    public:
        // Manual color controls; nullopt leaves the camera on auto.
        struct color_controls {
            std::optional<int32_t> exposure_us; // manual exposure time [us]
            std::optional<int32_t> gain; // manual gain
        };

        k4a_device_capturer() = default;
        ~k4a_device_capturer() override;

        [[nodiscard]] bool open(
            uint32_t device_index, 
            const color_controls& controls = {}
        ) noexcept;

        bool is_valid() const override;
        void close() override;

        const calibration_t& get_calibration() const override { return _calib; }
        Eigen::Vector2i get_color_camera_resolution() const override { return _calib.color_resolution; }
        Eigen::Vector2f get_color_camera_fov() const override { return _calib.color_fov; }

        [[nodiscard]] std::unique_ptr<sensor_frameset> fetch_next_sensor_frameset() override;

    private:
        mutable std::mutex _mtx;
        k4a_device_t _device{ nullptr };
        k4a_device_configuration_t _config{ K4A_DEVICE_CONFIG_INIT_DISABLE_ALL };
        calibration_t _calib{};
        std::string _serialnum;
    };

    // Recording playback source.
    class k4a_record_player final : public record_player_source {
    public:
        k4a_record_player() = default;
        ~k4a_record_player() override;

        // Requests color as color_conversion_format (default BGRA32); fails if there is no color track.
        [[nodiscard]] bool open(
            const std::filesystem::path& recording_file,
            std::optional<k4a_image_format_t> color_conversion_format = K4A_IMAGE_FORMAT_COLOR_BGRA32
        ) noexcept;

        bool is_valid() const override;
        void close() override;

        const calibration_t& get_calibration() const override { return _calib; }
        Eigen::Vector2i get_color_camera_resolution() const override { return _calib.color_resolution; }
        Eigen::Vector2f get_color_camera_fov() const override { return _calib.color_fov; }

        [[nodiscard]] std::unique_ptr<sensor_frameset> fetch_next_sensor_frameset() override;

        std::chrono::microseconds get_recording_length() const override { return _last_ts - _first_ts; }
        std::chrono::microseconds get_first_record_timestamp() const override { return _first_ts; }
        std::chrono::microseconds get_last_record_timestamp() const override { return _last_ts; }

        void seek_begin() override;
        void seek_end() override;
        void seek_timestamp(std::chrono::microseconds offset) override;

        bool auto_repeat_enabled() const override;
        void enable_auto_repeat(bool enable) override;

    private:
        mutable std::mutex _mtx;
        k4a_playback_t _playback{ nullptr };
        k4a_record_configuration_t _record_config{};
        calibration_t _calib{};
        std::chrono::microseconds _first_ts{ 0 };
        std::chrono::microseconds _last_ts{ 0 };
        bool _auto_repeat{ false };
    };

} // namespace hw
