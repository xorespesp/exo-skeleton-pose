#pragma once
#include "sensor_frameset.hh"
#include "timestamp.hh"

#include <chrono>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace hw
{
    class synced_frameset final
    {
    public:
        synced_frameset() = default;

        explicit synced_frameset(sensor_frameset stream_frameset)
        {
            _stream_framesets.emplace_back(std::move(stream_frameset));
            this->_summarize();
        }

        explicit synced_frameset(std::vector<std::optional<sensor_frameset>> stream_framesets)
            : _stream_framesets{ std::move(stream_framesets) }
        {
            this->_summarize();
        }

        std::size_t stream_count() const noexcept { return _stream_framesets.size(); }

        // 그 스트림이 이 순간의 캡처를 내지 못했으면 null.
        const sensor_frameset* stream_frameset(std::size_t stream_idx) const noexcept
        {
            if (stream_idx >= _stream_framesets.size()) { return nullptr; }
            const std::optional<sensor_frameset>& slot = _stream_framesets[stream_idx];
            return slot.has_value() ? &slot.value() : nullptr;
        }

        std::size_t present_stream_count() const noexcept { return _present_stream_count; }

        // 여기 든 캡처 중 가장 이른 타임스탬프.
        // 기여한 스트림이 없으면 0.
        timestamp_t timestamp() const noexcept { return _timestamp; }

        // 여기 든 캡처 중 가장 이른 것과 가장 늦은 것의 간격.
        // 기여한 스트림이 둘 미만이면 0.
        std::chrono::nanoseconds max_pair_skew() const noexcept { return _max_pair_skew; }

    private:
        void _summarize() noexcept
        {
            _present_stream_count = 0;
            _timestamp = timestamp_t{};
            _max_pair_skew = std::chrono::nanoseconds{ 0 };

            timestamp_t earliest{};
            timestamp_t latest{};
            for (const std::optional<sensor_frameset>& slot : _stream_framesets)
            {
                if (!slot.has_value()) { continue; }

                const timestamp_t captured_at = slot->timestamp();
                if (_present_stream_count == 0) { earliest = latest = captured_at; }
                else if (captured_at < earliest) { earliest = captured_at; }
                else if (captured_at > latest)   { latest = captured_at; }
                ++_present_stream_count;
            }

            if (_present_stream_count == 0) { return; }
            _timestamp = earliest;
            _max_pair_skew = latest - earliest;
        }

        std::vector<std::optional<sensor_frameset>> _stream_framesets;
        timestamp_t _timestamp{};
        std::chrono::nanoseconds _max_pair_skew{ 0 };
        std::size_t _present_stream_count{ 0 };
    };

} // namespace hw
