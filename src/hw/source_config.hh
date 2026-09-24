#pragma once
#include "calibration.hh"
#include "frame_format.hh"
#include "roi.hh"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace hw
{
    // 무엇을 열고 어떻게 스트리밍할지.
    // 카메라 config 는 자기 스트림 하나의 `roi` 를, 녹화 config 는 파일이 가진 스트림마다의 ROI 를 든다.

    // K4A 유선 싱크 잭의 역할.
    enum class k4a_sync_role_t : uint8_t
    {
        standalone,  // 싱크 잭을 쓰지 않는다
        master,      // Sync Out 으로 트리거를 낸다. subordinate 들을 연 뒤에 연다
        subordinate, // Sync In 의 트리거에 맞춰 찍는다
    };

    struct device_index_t  { uint32_t value{ 0 }; bool operator==(const device_index_t&) const = default; };
    struct device_serial_t { std::string value;   bool operator==(const device_serial_t&) const = default; };
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

        // 스트림 i 의 ROI. 파일의 스트림 수보다 짧으면 나머지는 전체 프레임이고, 길면 열기 실패다.
        std::vector<std::optional<roi_t>> stream_rois;
    };

    using source_config_t = std::variant<k4a_device_config_t, vz_device_config_t, recording_config_t>;

    // 사람이 읽는 짧은 라벨.
    // 예: "k4a device 'S12345'", "vz device 'VZ12345'", "recording 'walk.mcap'".
    std::string describe(const source_config_t& config);

} // namespace hw
