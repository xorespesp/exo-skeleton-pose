#include "source_address.hh"

#include <format>
#include <utility>

namespace app
{
    namespace
    {
        // 백엔드 이름 뒤에 이것이 오고, 그 뒤가 시리얼이다. 드라이브 문자는 백엔드 이름이 아니므로
        // 윈도 경로는 그대로 경로로 읽힌다.
        constexpr std::string_view kSerialPrefix{ "sn:" };

        template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
    } // namespace

    source_address source_address::device(const hw::sensor_backend_t backend, hw::device_serial_t serial)
    {
        source_address out;
        out._value = device_t{ backend, std::move(serial) };
        return out;
    }

    source_address source_address::recording(std::filesystem::path file)
    {
        source_address out;
        out._value = std::move(file);
        return out;
    }

    std::optional<source_address> source_address::try_parse(const std::string_view text)
    {
        if (text.empty()) { return std::nullopt; }

        if (const std::size_t colon = text.find(':'); colon != std::string_view::npos)
        {
            if (const std::optional<hw::sensor_backend_t> backend =
                    hw::sensor_backend_from_str(text.substr(0, colon)))
            {
                const std::string_view rest = text.substr(colon + 1);
                if (!rest.starts_with(kSerialPrefix)) { return std::nullopt; }

                const std::string_view serial = rest.substr(kSerialPrefix.size());
                if (serial.empty()) { return std::nullopt; }
                return source_address::device(*backend, hw::device_serial_t{ std::string{ serial } });
            }
        }

        return source_address::recording(std::filesystem::path{ text });
    }

    bool source_address::is_device() const noexcept
    {
        return std::holds_alternative<device_t>(_value);
    }

    bool source_address::is_recording() const noexcept
    {
        return std::holds_alternative<std::filesystem::path>(_value);
    }

    hw::sensor_backend_t source_address::device_backend() const
    {
        return std::get<device_t>(_value).backend;
    }

    const hw::device_serial_t& source_address::device_serial() const
    {
        return std::get<device_t>(_value).serial;
    }

    const std::filesystem::path& source_address::recording_path() const
    {
        return std::get<std::filesystem::path>(_value);
    }

    std::string source_address::to_string() const
    {
        return std::visit(overloaded{
            [](const device_t& d) {
                return std::format("{}:{}{}", hw::sensor_backend_to_str(d.backend), kSerialPrefix, d.serial.value);
            },
            [](const std::filesystem::path& p) { return p.string(); },
        }, _value);
    }

    bool source_address::operator==(const source_address& other) const noexcept
    {
        if (_value.index() != other._value.index()) { return false; }
        return std::visit(overloaded{
            [&](const device_t& d) {
                const device_t& rhs = std::get<device_t>(other._value);
                return d.backend == rhs.backend && d.serial == rhs.serial;
            },
            [&](const std::filesystem::path& p) { return p == std::get<std::filesystem::path>(other._value); },
        }, _value);
    }

} // namespace app
