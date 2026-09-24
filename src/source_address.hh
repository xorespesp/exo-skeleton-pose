#pragma once
#include "hw/sensor_backend.hh"
#include "hw/source_config.hh"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

namespace app
{
    // Where frames come from: a camera or a recording file, never both or neither.
    class source_address
    {
    public:
        struct device_t {
            hw::sensor_backend_t backend{ hw::sensor_backend_t::k4a };
            hw::device_serial_t serial;
        };

        source_address() = default; // 시리얼 없는 카메라. 검증이 거부한다

        static source_address device(hw::sensor_backend_t backend, hw::device_serial_t serial);
        static source_address recording(std::filesystem::path file);

        // Text form:
        //
        //   "k4a:sn:<serial>"  a K4A camera
        //   "vz:sn:<serial>"   a VZ camera
        //   anything else      a recording file path
        //
        [[nodiscard]] static std::optional<source_address> try_parse(std::string_view text);

        bool is_device() const noexcept;
        bool is_recording() const noexcept;

        // Preconditions: the matching is_*() holds.
        hw::sensor_backend_t device_backend() const;
        const hw::device_serial_t& device_serial() const;
        const std::filesystem::path& recording_path() const;

        std::string to_string() const;

        // 같은 장치를 가리키는지. 카메라는 백엔드와 시리얼이 같을 때, 녹화는 같은 파일일 때.
        bool operator==(const source_address& other) const noexcept;

    private:
        std::variant<device_t, std::filesystem::path> _value{ device_t{} };
    };

} // namespace app
