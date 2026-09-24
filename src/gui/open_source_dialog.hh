#pragma once
#include "app_config.hh"

#include "hw/device_enumeration.hh"
#include "pose/joints_def.hh"
#include "pose/view_plane.hh"

#include <imgui.h> // imfilebrowser.h requires it first
#include <imfilebrowser.h>

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace gui
{
    // 무엇을 여는가: 카메라들, 또는 녹화 하나.
    enum class source_kind_t { cameras, recording };

    // The Open Source form: a view over `app_config_t` in both directions. `fill` reads the
    // settings in force into the fields, `render` writes them back on Open.
    //
    // 폼은 뷰 평면으로 엔트리 수를 정한다(frontal 하나, sagittal 둘). config 에 쓰이는 것은 엔트리마다의
    // `view` 이고 평면은 거기서 유도된다.
    class open_source_dialog
    {
    public:
        struct result_t
        {
            bool open_source{ false }; // the form is in the config; open a source with it
            // Picked under Load Config...; reading it over the settings is the owner's, not the form's.
            std::optional<std::filesystem::path> load_config;
        };

        open_source_dialog();

        // An Open writes the whole form back, so a form older than the config would undo whatever
        // moved it. The owner fills before opening the dialog, and again when the config changes
        // under an open one.
        void fill(const app::app_config_t& config);

        void show() { _show = true; }

        // Draws the dialog and the file browsers it owns. A recording with no file picked leaves
        // the dialog standing and raises nothing.
        //
        // `source_open` 이면 장치 열거를 막는다. 열려 있는 카메라는 자기 시리얼을 대지 못해 목록에서
        // 빠지므로, 그때의 목록은 고를 수 있는 것을 말하지 않는다.
        result_t render(app::app_config_t& config, bool source_open);

    private:
        // 카메라 한 대의 폼. 어느 쪽에 선 카메라인지는 폼이 놓인 자리가 말한다.
        struct camera_form_t
        {
            std::string device_serial;

            bool manual_exposure{ false };
            int exposure_us{ 8000 };
            bool manual_gain{ false };
            int gain{ 0 };

            std::string intrinsics; // VZ 만. calibration file for a camera that reports none
        };

        std::size_t _entry_count() const; // 뷰 평면이 정한다

        // 엔트리 i 는 스트림 i 라 그 순서는 config 가 정하고, 폼은 늘 왼쪽·오른쪽 순으로 선다.
        pose::joint_side_t _side_of(std::size_t entry_idx) const;
        pose::camera_view_t _view_of(std::size_t entry_idx) const;
        std::size_t _entry_of(pose::joint_side_t side) const;
        std::size_t _form_of(pose::joint_side_t side) const; // frontal 은 카메라가 하나라 늘 앞의 것

        bool _apply(app::app_config_t& config) const; // false when there is nothing to open
        void _render_camera(std::size_t form_idx, bool source_open);
        void _render_device_picker(camera_form_t& form, bool source_open);
        void _render_browsers(result_t& result);

    private:
        bool _show{ false };

        source_kind_t _kind{ source_kind_t::cameras };
        pose::view_plane_t _view_plane{ pose::view_plane_t::frontal };
        app::marker_kind_t _marker_kind{ app::marker_kind_t::apriltag };

        // 한 현장은 같은 기종으로 맞춘다. 프레임 레이트도 함께 여는 스트림이 같은 값을 써야 한다.
        hw::sensor_backend_t _backend{ hw::sensor_backend_t::k4a };
        bool _limit_frame_rate{ false }; // VZ 만. 꺼져 있으면 카메라 상한으로 프리런
        float _frame_rate_fps{ 30.0f };

        // `_backend` 를 열거한 결과. 스캔을 눌러야 채워진다(K4A 열거는 장치를 하나씩 열어 본다).
        std::vector<hw::device_info_t> _devices;
        bool _devices_enumerated{ false };

        // `_cameras[0]` 이 왼쪽, `[1]` 이 오른쪽 카메라다. frontal 은 앞의 하나만 쓴다.
        std::array<camera_form_t, 2> _cameras{};
        std::string _recording; // kept alongside the cameras, so switching kind discards neither

        // 엔트리 0 의 카메라가 선 쪽. 나머지 하나는 그 반대쪽이다.
        pose::joint_side_t _first_entry_side{ pose::joint_side_t::left };

        // 카메라 둘을 묶을 때.
        pose::joint_side_t _reference_side{ pose::joint_side_t::left };
        bool _limit_pair_skew{ false }; // 꺼져 있으면 기준 간격의 절반으로 자동
        float _max_pair_skew_ms{ 8.0f };

        ImGui::FileBrowser _recording_browser;
        ImGui::FileBrowser _intrinsics_browser;
        std::size_t _intrinsics_for{ 0 }; // 캘리브레이션 브라우저를 연 폼 자리
        ImGui::FileBrowser _config_load_browser; // picks an existing file, so no new-filename flag
    };

} // namespace gui
