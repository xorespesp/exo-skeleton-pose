#pragma once
#include <cstdint>
#include <optional>
#include <string_view>

namespace hw
{
    // 센서를 구동하는 SDK.
    enum class sensor_backend_t : uint8_t {
        k4a,
        vz,
    };

    constexpr std::string_view sensor_backend_to_str(sensor_backend_t backend)
    {
        switch (backend) {
        case sensor_backend_t::k4a: return "k4a";
        case sensor_backend_t::vz:  return "vz";
        }
        return "?";
    }

    // 위 백엔드가 아니면 nullopt.
    constexpr std::optional<sensor_backend_t> sensor_backend_from_str(std::string_view str)
    {
        if (str == sensor_backend_to_str(sensor_backend_t::k4a)) { return sensor_backend_t::k4a; }
        if (str == sensor_backend_to_str(sensor_backend_t::vz))  { return sensor_backend_t::vz; }
        return std::nullopt;
    }

} // namespace hw
