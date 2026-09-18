#pragma once
#include "timestamp.hh"

#include <chrono>
#include <cstddef>

namespace hw
{
    // 소스의 단조 클럭을 Unix 시각으로 옮긴다.
    //
    // 소스 시각의 읽기 하나가 지금 막 호스트에 닿았다고 보고, 그 순간의 차(Unix - 소스)를 offset 으로 잡는다.
    // 그 뒤로는 그 값을 더할 뿐이므로 프레임 간격은 소스가 보고한 그대로 나온다.
    //
    // 소스 시각이 이미 Unix 인 소스는 이것이 필요 없다.
    class clock_anchor_t final
    {
    public:
        explicit clock_anchor_t(const std::chrono::nanoseconds source_ts)
        {
            const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch());
            _offset = now - source_ts;
        }

        timestamp_t to_unix(const std::chrono::nanoseconds source_ts) const noexcept
        {
            return timestamp_t{ source_ts + _offset };
        }

        std::chrono::nanoseconds offset() const noexcept { return _offset; }

    private:
        std::chrono::nanoseconds _offset; // Unix 시각 - 소스 시각
    };

    // 앵커를 세우기 전에 모으는 읽기 수.
    inline constexpr std::size_t kClockWarmupSamples = 12;

} // namespace hw
