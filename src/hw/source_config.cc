#include "source_config.hh"

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

} // namespace hw
