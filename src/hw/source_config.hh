#pragma once
#include "calibration.hh"
#include "frame_format.hh"
#include "roi.hh"
#include "source_backend.hh"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace hw
{
    // 무엇을 열고 어떻게 스트리밍할지.
    // 모든 config 가 `roi` 를 갖고, provider 는 그것을 종류와 무관하게 읽는다.

    // K4A 유선 싱크 잭의 역할.
    enum class k4a_sync_role_t : uint8_t
    {
        standalone,  // 싱크 잭을 쓰지 않는다
        master,      // Sync Out 으로 트리거를 낸다. subordinate 들을 연 뒤에 연다
        subordinate, // Sync In 의 트리거에 맞춰 찍는다
    };

    struct device_index_t  { uint32_t value{ 0 }; };
    struct device_serial_t { std::string value; };
    using device_selector_t = std::variant<device_index_t, device_serial_t>;

    // Orbbec K4A Wrapper 카메라.
    struct k4a_device_config_t
    {
        device_selector_t device_selector{ device_index_t{ 0 } };
        std::optional<int32_t> exposure_us; // nullopt: 자동 노출
        std::optional<int32_t> gain;        // nullopt: 자동 게인

        frame_format_t frame_format{ frame_format_t::bgr8 };
        std::optional<roi_t> roi;     // nullopt: 전체 프레임

        k4a_sync_role_t wired_sync_mode{ k4a_sync_role_t::standalone };
        uint32_t subordinate_delay_off_master_usec{ 0 }; // subordinate 만. SDK 이름 그대로
    };

    // Vieworks VZ-5MU-C79H00 카메라.
    struct vz_device_config_t
    {
        device_selector_t device_selector{ device_index_t{ 0 } };
        std::optional<double> exposure_us; // nullopt: 카메라의 현재 설정을 둔다
        std::optional<double> gain;        // nullopt: 카메라의 현재 설정을 둔다

        frame_format_t frame_format{ frame_format_t::gray8 };
        std::optional<roi_t> roi;    // nullopt: 전체 프레임

        // VZ 카메라는 intrinsic 을 주지 않으므로 오프라인에서 잰 캘리브레이션을 여기로 넣는다.
        // NOTE: 비워 두면 태그 포즈를 풀 수 없다. 2D 태그 중심으로 일하는 추정기는 필요 없다.
        std::optional<intrinsic_t>  intrinsic;
        std::optional<distortion_t> distortion; // intrinsic 과 함께 비어 있으면 왜곡 없음

        std::optional<double> frame_rate_fps; // nullopt: fps 제한 없음
    };

    // 우리 녹화 하나. 카메라 자리에서 재생된다.
    struct recording_config_t
    {
        std::filesystem::path file;
        std::optional<roi_t> roi; // nullopt: 전체 프레임
    };

    using source_config_t = std::variant<k4a_device_config_t, vz_device_config_t, recording_config_t>;

    // 이 config 가 고르는 백엔드.
    source_backend_t get_source_backend(const source_config_t& config);

    struct device_info_t
    {
        source_backend_t source_backend{};
        uint32_t device_index{ 0 };
        std::string device_serial; // SDK 가 주지 않으면 비어 있다
        std::string display_name;
    };

    // `backend` 의 카메라를 나열한다. 여는 경로가 아니라 무엇을 열 수 있는지 묻는 경로다: 설정을 만드는
    // 쪽이 시리얼을 고르거나, 설정의 시리얼이 꽂혀 있는지 미리 확인할 때 쓴다. SDK 헤더가 `hw/` 밖으로
    // 새지 않도록 백엔드의 열거를 여기서 한 번 감싼다.
    // 스트리밍 중이 아닐 때 부른다. K4A 는 시리얼을 읽으려면 장치를 열어야 하므로, 이미 열려 있는 장치는
    // 시리얼 없이 인덱스만 보고된다.
    std::vector<device_info_t> enumerate_devices(source_backend_t backend);

    // 사람이 읽는 짧은 라벨.
    // 예: "k4a device #0", "vz device 'VZ12345'", "recording 'walk.mcap'".
    std::string describe(const source_config_t& config);

} // namespace hw
