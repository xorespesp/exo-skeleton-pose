#pragma once
#include <opencv2/core.hpp>

#include <chrono>
#include <cstdint>

namespace hw
{
    // Owned frame delivered to observers.
    // (deep-copied; outlives the frameset)
    class sensor_frame final {
    private:
        const uint32_t _id;

    public:
        std::chrono::microseconds timestamp{}; // device timestamp
        cv::Mat color_image; // 8-bit BGR

    public:
        sensor_frame(uint32_t id) : _id{ id } {}
        ~sensor_frame() = default;
        sensor_frame(const sensor_frame&) = delete;
        sensor_frame& operator=(const sensor_frame&) = delete;

        uint32_t id() const noexcept {
            return _id;
        }

        double timestamp_in_sec() const {
            return std::chrono::duration<double>{ timestamp }.count();
        }
    };

} // namespace hw
