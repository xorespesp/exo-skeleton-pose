#pragma once
#include "recording_reader.hh"

#include "hw/sensor_frame_source.hh"

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <span>
#include <vector>

namespace io
{
    // Playback source for our own recordings. (SDK-agnostic)
    class mcap_record_player final
        : public hw::record_player_source
        , public std::enable_shared_from_this<mcap_record_player>
    {
    private:
        class recording_stream_impl;

    public:
        [[nodiscard]] static std::shared_ptr<mcap_record_player> create();
        ~mcap_record_player() override;

        mcap_record_player(const mcap_record_player&) = delete;
        mcap_record_player& operator=(const mcap_record_player&) = delete;

        // Fails if the file is not one of our recordings, or carries no camera stream.
        [[nodiscard]] bool open(const std::filesystem::path& recording_file) noexcept;

        bool is_opened() const;
        void close();

        std::span<const std::shared_ptr<hw::sensor_frame_source>> get_recording_streams() const override { return _recording_streams; }
        std::chrono::nanoseconds get_recording_length() const override { return _last_timestamp - _first_timestamp; }
        hw::timestamp_t get_first_record_timestamp() const override { return _first_timestamp; }
        hw::timestamp_t get_last_record_timestamp() const override { return _last_timestamp; }

        void seek_begin() override;
        void seek_end() override;
        void seek_timestamp(hw::timestamp_t timestamp) override;

    private:
        mcap_record_player() = default;

    private:
        mutable std::mutex _mtx;
        recording_reader _reader;
        bool _opened{ false };
        std::vector<std::shared_ptr<hw::sensor_frame_source>> _recording_streams; // 파일의 순서대로
        hw::timestamp_t _first_timestamp{};
        hw::timestamp_t _last_timestamp{};
    };

} // namespace io
