#include "source_config.hh"

#include "backends/k4a_frame_source.hh"
#ifdef EXO_HAS_VZ_BACKEND
#include "backends/vz_frame_source.hh"
#endif

#include <format>
#include <variant>

namespace hw
{
    namespace
    {
        template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

        // "#0" 또는 "'VZ12345'".
        std::string describe_device_selector(const device_selector_t& device_selector)
        {
            return std::visit(overloaded{
                [](const device_index_t& by_index)   { return std::format("#{}", by_index.value); },
                [](const device_serial_t& by_serial) { return std::format("'{}'", by_serial.value); },
            }, device_selector);
        }
    } // namespace

    source_backend_t get_source_backend(const source_config_t& config)
    {
        return std::visit(overloaded{
            [](const k4a_device_config_t&) { return source_backend_t::k4a; },
            [](const vz_device_config_t&)  { return source_backend_t::vz; },
            [](const recording_config_t&)  { return source_backend_t::recording; },
        }, config);
    }

    std::string describe(const source_config_t& config)
    {
        return std::visit(overloaded{
            [](const k4a_device_config_t& c) {
                return std::format("k4a device {}", describe_device_selector(c.device_selector));
            },
            [](const vz_device_config_t& c) {
                return std::format("vz device {}", describe_device_selector(c.device_selector));
            },
            [](const recording_config_t& c) {
                return std::format("recording '{}'", c.file.filename().string());
            },
        }, config);
    }

    std::vector<device_info_t> enumerate_devices(const source_backend_t backend)
    {
        switch (backend) {
        case source_backend_t::k4a:
            return k4a_device_capturer::enumerate();
        case source_backend_t::vz:
#ifdef EXO_HAS_VZ_BACKEND
            return vz_frame_source::enumerate();
#else
            return {};
#endif
        case source_backend_t::recording:
            break;
        }
        return {};
    }

} // namespace hw
