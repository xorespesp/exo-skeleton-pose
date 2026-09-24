#include "hw/device_enumeration.hh"

#include "hw/backends/k4a_frame_source.hh"
#ifdef EXO_HAS_VZ_BACKEND
#include "hw/backends/vz_device.hh"
#endif

#include <spdlog/spdlog.h>

#include <format>

namespace hw
{
    namespace
    {
#ifdef EXO_HAS_VZ_BACKEND
        // 인터페이스를 훑는 데 주는 시간. 그 안에 응답하지 못한 카메라는 없는 것으로 친다.
        constexpr uint32_t kVzEnumerateTimeoutMs = 500;

        std::vector<device_info_t> enumerate_vz_devices()
        {
            std::string err_msg;
            const std::vector<vz::device_info_t> found = vz::device::enumerate(kVzEnumerateTimeoutMs, &err_msg);
            if (!err_msg.empty()) { spdlog::warn("hw: vz enumeration: {}", err_msg); }

            std::vector<device_info_t> out;
            out.reserve(found.size());
            for (uint32_t device_index = 0; device_index < found.size(); ++device_index)
            {
                if (found[device_index].serial.empty()) {
                    spdlog::warn("vz: device #{} did not report a serial and is left out", device_index);
                    continue;
                }

                out.push_back(device_info_t{
                    .backend = sensor_backend_t::vz,
                    .device_serial = found[device_index].serial,
                    .display_name = std::format("vz device #{} (S/N {})", device_index, found[device_index].serial),
                });
            }
            return out;
        }
#endif
    } // namespace

    std::vector<device_info_t> enumerate_devices(const sensor_backend_t backend)
    {
        switch (backend)
        {
        case sensor_backend_t::k4a:
            return k4a_enumerate_devices();
        case sensor_backend_t::vz:
#ifdef EXO_HAS_VZ_BACKEND
            return enumerate_vz_devices();
#else
            spdlog::warn("hw: this build carries no VZ backend, so no VZ camera can be found");
            return {};
#endif
        }
        return {};
    }

} // namespace hw
