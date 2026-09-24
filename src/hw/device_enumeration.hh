#pragma once
#include "hw/sensor_backend.hh"

#include <cstdint>
#include <string>
#include <vector>

namespace hw
{
    struct device_info_t
    {
        sensor_backend_t backend{ sensor_backend_t::k4a };
        std::string device_serial;
        std::string display_name;
    };

    // NOTE: 다른 프로세스가 먼저 open한 device는 아예 열거되지 않거나 open 실패할 수 있음
    [[nodiscard]] std::vector<device_info_t> enumerate_devices(sensor_backend_t backend);

} // namespace hw
