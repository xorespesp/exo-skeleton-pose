#pragma once
#include "calibration.hh"
#include "sensor_frame_source.hh"
#include "sensor_frame_observer.hh"

#include <Eigen/Core>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace hw
{
    // Owns a camera backend and runs a background polling thread that pulls
    // framesets, builds sensor_frame, and pushes them to observers.
    // SDK-agnostic: the concrete backend is created only in the .cc.
    class sensor_frame_provider final {
    public:
        sensor_frame_provider() = default;
        ~sensor_frame_provider();

        sensor_frame_provider(const sensor_frame_provider&) = delete;
        sensor_frame_provider& operator=(const sensor_frame_provider&) = delete;
        sensor_frame_provider(sensor_frame_provider&&) = delete;
        sensor_frame_provider& operator=(sensor_frame_provider&&) = delete;

        void add_observer(std::shared_ptr<sensor_frame_observer> observer);
        void remove_observer(const std::shared_ptr<sensor_frame_observer>& observer);

        bool is_opened() const;

        [[nodiscard]] bool open_device(
            uint32_t device_index,
            std::optional<int32_t> exposure_us = std::nullopt,
            std::optional<int32_t> gain = std::nullopt
        ) noexcept;
        
        [[nodiscard]] bool open_recording(const std::filesystem::path& recording_file) noexcept;

        void close();

        const std::string& get_source_name() const { return _source_name; }
        const calibration_t& get_calibration() const { return _calib; }
        Eigen::Vector2i get_color_camera_resolution() const { return _color_resolution; }
        Eigen::Vector2f get_color_camera_fov() const { return _color_fov; }

        float get_current_update_rate() const { return _update_rate.load(); } // EMA-smoothed fps
        uint32_t get_current_frame_id() const { return _frame_id.load(); }

        bool is_paused() const { return _paused.load(); }
        void play();
        void pause();

        // Recording sources only (no-op / 0 otherwise).
        void seek_recording_to_begin();
        void seek_recording_to_end();
        void seek_recording_timeline(std::chrono::microseconds timestamp);

        std::chrono::microseconds get_recording_length() const;
        std::chrono::microseconds get_first_record_timestamp() const;
        std::chrono::microseconds get_last_record_timestamp() const;

        float get_update_speed() const { return _speed.load(); }
        void  set_update_speed(float factor);

        bool is_auto_repeat_enabled() const;
        void set_auto_repeat(bool enable);

    private:
        void _install_source(
            std::unique_ptr<sensor_frame_source> source,
            record_player_source* player,
            std::string source_name
        );

        void _start_thread();
        void _stop_thread();
        void _polling_thread_proc();

        std::vector<std::shared_ptr<sensor_frame_observer>> _snapshot_observers() const;
        void _notify_sensor_frame_update(const std::shared_ptr<sensor_frame>& frame);
        void _notify_sensor_stream_reset();
        void _notify_sensor_stream_end();

    private:
        std::unique_ptr<sensor_frame_source> _source;
        record_player_source* _player{ nullptr }; // non-owning; set only for recording sources
        mutable std::mutex _source_mtx;

        std::thread _thread;
        std::atomic<bool> _running{ false };
        std::atomic<bool> _paused{ false };
        std::atomic<bool> _need_repace{ false }; // request playback pacing anchor reset

        std::vector<std::shared_ptr<sensor_frame_observer>> _observers;
        mutable std::mutex _observers_mtx;

        // Cached at open(); immutable while streaming.
        calibration_t _calib{};
        Eigen::Vector2i _color_resolution{ Eigen::Vector2i::Zero() };
        Eigen::Vector2f _color_fov{ Eigen::Vector2f::Zero() };
        std::string _source_name;

        std::atomic<uint32_t> _frame_id{ 0 };
        std::atomic<float> _update_rate{ 0.0f };
        std::atomic<float> _speed{ 1.0f };
    }; // class

} // namespace hw
