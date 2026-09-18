#pragma once
#include "calibration.hh"
#include "frame_format.hh"
#include "roi.hh"
#include "sensor_frameset.hh"
#include "source_backend.hh"
#include "timestamp.hh"

#include <chrono>
#include <cstddef>
#include <format>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace hw
{
    struct stream_descriptor_t
    {
        std::string stream_name;
        std::string device_serial; // optional
        source_backend_t source_backend{};
    };

    inline std::string default_stream_name(const std::size_t stream_idx)
    {
        return std::format("color{}", stream_idx);
    }

    // Abstract camera backend interface. SDK-agnostic.
    class sensor_frame_source {
    public:
        virtual ~sensor_frame_source() = default;

        virtual bool is_valid() const = 0;
        virtual void close() = 0;

        // ---- 전체 프레임 기준 (ROI 가 걸려 있어도 바뀌지 않음) ----
        virtual const calibration_t& get_calibration() const = 0;
        virtual frame_format_t get_frame_format() const = 0;
        virtual stream_descriptor_t get_stream_descriptor() const = 0;

        // 전달되는 이미지를 `roi`(전체 프레임 픽셀 좌표)로 좁힌다.
        // NOTE:
        //   - 소스가 자기 프레임에 맞게 자르거나 센서 증분에 스냅할 수 있으므로, 실제로 걸린 창을 돌려준다.
        //   - 비어 있으면 아무것도 걸리지 않은 것이다. (좁히기를 지원하지 않거나 `roi` 가 프레임을 벗어남)
        //   - `get_calibration()` 은 계속 전체 프레임 기준이므로, 좁힌 창에 맞추는 것은 호출자의 몫이다.
        virtual std::optional<roi_t> try_set_roi(const roi_t& /*roi*/) { return std::nullopt; }

        // Blocking. Returns the frames of one capture, already converted and narrowed.
        // Empty on EOF / timeout / disconnect.
        [[nodiscard]] virtual std::optional<sensor_frameset> fetch_next_sensor_frameset() = 0;
    };

    // Recording playback backend interface.
    class record_player_source {
    public:
        virtual ~record_player_source() = default;

        virtual std::span<const std::shared_ptr<sensor_frame_source>> get_recording_streams() const = 0;
        virtual std::chrono::nanoseconds get_recording_length() const = 0;
        virtual timestamp_t get_first_record_timestamp() const = 0;
        virtual timestamp_t get_last_record_timestamp() const = 0;

        virtual void seek_begin() = 0;
        virtual void seek_end() = 0;
        virtual void seek_timestamp(timestamp_t timestamp) = 0;
    };

} // namespace hw
