#include "open_source_dialog.hh"

#include <imgui.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstdint>
#include <format>
#include <optional>
#include <utility>
#include <vector>

namespace gui
{
    namespace
    {
        // ImGui 의 InputText 가 std::string 을 받을 수 있게 하는 크기 고정 버퍼.
        constexpr std::size_t kSerialBufferSize = 64;
    } // namespace

    open_source_dialog::open_source_dialog()
    {
        _recording_browser.SetTitle("Open recording file");
        _recording_browser.SetTypeFilters({ ".mcap" });
        _recording_browser.SetPwd(app::project_dir("recordings"));

        _intrinsics_browser.SetTitle("Open camera calibration");
        _intrinsics_browser.SetTypeFilters({ ".yml", ".yaml", ".xml" });
        _intrinsics_browser.SetPwd(app::project_dir("configs"));

        _config_load_browser.SetTitle("Load config");
        _config_load_browser.SetTypeFilters({ ".json" });
    }

    std::size_t open_source_dialog::_entry_count() const
    {
        return (_view_plane == pose::view_plane_t::sagittal) ? 2 : 1;
    }

    pose::joint_side_t open_source_dialog::_side_of(const std::size_t entry_idx) const
    {
        // 엔트리 순서는 읽어 온 config 가 정한 것을 지킨다. 둘은 서로 반대쪽이므로 첫 엔트리만 알면 된다.
        if (entry_idx == 0) { return _first_entry_side; }
        return (_first_entry_side == pose::joint_side_t::left)
            ? pose::joint_side_t::right
            : pose::joint_side_t::left;
    }

    pose::camera_view_t open_source_dialog::_view_of(const std::size_t entry_idx) const
    {
        if (_view_plane == pose::view_plane_t::frontal) { return pose::camera_view_t::frontal; }
        return (this->_side_of(entry_idx) == pose::joint_side_t::right)
            ? pose::camera_view_t::sagittal_right
            : pose::camera_view_t::sagittal_left;
    }

    std::size_t open_source_dialog::_entry_of(const pose::joint_side_t side) const
    {
        return (this->_side_of(0) == side) ? 0 : 1;
    }

    std::size_t open_source_dialog::_form_of(const pose::joint_side_t side) const
    {
        if (_view_plane == pose::view_plane_t::frontal) { return 0; }
        return (side == pose::joint_side_t::left) ? 0 : 1;
    }

    void open_source_dialog::fill(const app::app_config_t& config)
    {
        _marker_kind = config.pose.detector.kind;
        _view_plane = app::view_plane_of(config.cameras).value_or(_view_plane);

        // Only what the config names, so the other kind keeps what was typed into it.
        if (config.cameras.empty()) { return; }
        _kind = config.cameras.front().source.is_recording() ? source_kind_t::recording : source_kind_t::cameras;

        // 엔트리 0 의 뷰가 그 자리의 쪽을 말한다. 이것이 정해져야 엔트리를 폼 자리로 옮길 수 있다.
        _first_entry_side = pose::viewed_leg_of(config.cameras.front().view).value_or(pose::joint_side_t::left);

        if (config.sync.has_value())
        {
            const std::size_t reference = std::min<std::size_t>(
                config.sync->reference_stream_idx, config.cameras.size() - 1);
            _reference_side = this->_side_of(reference);
            _limit_pair_skew = config.sync->max_pair_skew.has_value();
            if (config.sync->max_pair_skew.has_value()) {
                _max_pair_skew_ms = static_cast<float>(config.sync->max_pair_skew->count());
            }
        }
        else
        {
            // 이 프로파일이 말하지 않는 값이므로 이전 프로파일의 것을 들고 있지 않는다.
            _reference_side = pose::joint_side_t::left;
            _limit_pair_skew = false;
        }

        // 카메라 폼은 이 config 가 채우는 만큼만 남는다. 엔트리가 줄면 남은 자리의 옛 시리얼이 다음 Open
        // 에서 열릴 수 있다.
        if (_kind == source_kind_t::cameras) { _cameras = {}; }

        for (std::size_t entry_idx = 0; entry_idx < config.cameras.size() && entry_idx < _cameras.size(); ++entry_idx)
        {
            const app::camera_config_t& cam = config.cameras[entry_idx];
            camera_form_t& form = _cameras[this->_form_of(this->_side_of(entry_idx))];

            if (cam.source.is_recording())
            {
                _recording = cam.source.recording_path().string();
                continue;
            }

            if (const hw::sensor_backend_t backend = cam.source.device_backend(); backend != _backend)
            {
                _backend = backend;
                _devices.clear(); // 목록은 한 백엔드의 것이다
                _devices_enumerated = false;
            }
            form.device_serial = cam.source.device_serial().value;

            form.manual_exposure = cam.exposure_us.has_value();
            if (cam.exposure_us.has_value()) { form.exposure_us = *cam.exposure_us; }
            form.manual_gain = cam.gain.has_value();
            if (cam.gain.has_value()) { form.gain = *cam.gain; }
            _limit_frame_rate = cam.frame_rate_fps.has_value();
            if (cam.frame_rate_fps.has_value()) { _frame_rate_fps = static_cast<float>(*cam.frame_rate_fps); }
            form.intrinsics = cam.intrinsics_file;
        }
    }

    bool open_source_dialog::_apply(app::app_config_t& config) const
    {
        const std::size_t count = this->_entry_count();
        if (_kind == source_kind_t::recording && _recording.empty()) {
            spdlog::warn("no recording file selected");
            return false;
        }

        app::app_config_t candidate_config = config;

        // ROI 는 프레임 도구가 정한다. 같은 자리의 엔트리가 있던 값을 잇는다.
        std::vector<app::camera_config_t> cameras(count);
        for (std::size_t entry_idx = 0; entry_idx < count; ++entry_idx)
        {
            app::camera_config_t& cam = cameras[entry_idx];
            if (entry_idx < candidate_config.cameras.size()) { cam.roi = candidate_config.cameras[entry_idx].roi; }
            cam.view = this->_view_of(entry_idx);

            if (_kind == source_kind_t::recording)
            {
                // A recording carries the settings it was shot with, so the exposure, the gain and the
                // calibration have nowhere to land here.
                cam.source = app::source_address::recording(_recording);
                continue;
            }

            const pose::joint_side_t side = this->_side_of(entry_idx);
            const camera_form_t& form = _cameras[this->_form_of(side)];

            // 시리얼이 카메라를 가리키는 유일한 이름이라, 비어 있으면 열 대상이 없다.
            if (form.device_serial.empty()) {
                spdlog::warn("no serial for the {} camera", pose::joint_side_name(side));
                return false;
            }

            cam.source = app::source_address::device(_backend, hw::device_serial_t{ form.device_serial });
            cam.exposure_us = form.manual_exposure ? std::optional<int32_t>{ form.exposure_us } : std::nullopt;
            cam.gain = form.manual_gain ? std::optional<int32_t>{ form.gain } : std::nullopt;
            if (_backend == hw::sensor_backend_t::vz)
            {
                cam.intrinsics_file = form.intrinsics;
                cam.frame_rate_fps = _limit_frame_rate ? std::optional<double>{ _frame_rate_fps } : std::nullopt;
            }
        }

        candidate_config.cameras = std::move(cameras);

        // 라이브 카메라 둘일 때만 sync 가 있다.
        if (_kind == source_kind_t::cameras && count >= 2)
        {
            app::sync_config_t sync;
            sync.reference_stream_idx = static_cast<uint32_t>(this->_entry_of(_reference_side));
            sync.max_pair_skew = _limit_pair_skew ? std::optional<pose::millis_f64>{ pose::millis_f64{ _max_pair_skew_ms } } : std::nullopt;
            candidate_config.sync = sync;
        }
        else
        {
            candidate_config.sync.reset();
        }

        // 스트림마다 한 칸. 늘어난 자리는 안 잰 상태이고, 줄어든 자리의 측정치는 버려진다.
        candidate_config.pose.detector.color_marker.calibration.resize(count);
        candidate_config.pose.detector.kind = _marker_kind;

        if (std::string err; !app::validate_config(candidate_config, err)) {
            spdlog::error("open dialog: {}", err);
            return false;
        }

        config = std::move(candidate_config);
        return true;
    }

    void open_source_dialog::_render_device_picker(camera_form_t& form, const bool source_open)
    {
        char buffer[kSerialBufferSize]{};
        const std::size_t copied = std::min(form.device_serial.size(), kSerialBufferSize - 1);
        form.device_serial.copy(buffer, copied);
        ImGui::SetNextItemWidth(220.0f);
        if (ImGui::InputText("Serial", buffer, kSerialBufferSize)) { form.device_serial = buffer; }
        ImGui::SetItemTooltip("The camera this entry opens. Pick one below, or type the serial of a\n"
                              "camera that is not attached yet.");

        // 열거는 이 프로세스가 아무것도 열지 않았을 때만 뜻이 선다.
        ImGui::BeginDisabled(source_open);
        ImGui::SetNextItemWidth(220.0f);
        if (ImGui::BeginCombo("##attached", _devices_enumerated
            ? (_devices.empty() ? "(none found)" : "(pick a camera)")
            : "(not scanned)"))
        {
            for (const hw::device_info_t& device : _devices)
            {
                if (device.backend != _backend) { continue; }
                if (ImGui::Selectable(device.display_name.c_str(), device.device_serial == form.device_serial)) {
                    form.device_serial = device.device_serial;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::EndDisabled();
        if (source_open) {
            ImGui::SetItemTooltip("Close the source to scan: a camera being held does not name itself.");
        }
    }

    void open_source_dialog::_render_camera(const std::size_t form_idx, const bool source_open)
    {
        camera_form_t& form = _cameras[form_idx];
        const bool color_markers = (_marker_kind == app::marker_kind_t::color_marker);

        ImGui::PushID(static_cast<int>(form_idx));

        if (_view_plane == pose::view_plane_t::sagittal) {
            ImGui::SeparatorText(form_idx == 0 ? "Left camera" : "Right camera");
            ImGui::SetItemTooltip("The camera standing on that side of the exo. It reads that leg.");
        } else {
            ImGui::SeparatorText("Camera");
        }

        this->_render_device_picker(form, source_open);

        // A fitted colour sits at one brightness and the open refuses a camera free to
        // leave it, so only the choice of auto goes away. The values stay editable.
        if (color_markers) {
            form.manual_exposure = true;
            form.manual_gain = true;
        }

        ImGui::BeginDisabled(color_markers);
        ImGui::Checkbox("Manual exposure [us]", &form.manual_exposure);
        ImGui::EndDisabled();
        if (form.manual_exposure)
        {
            ImGui::SameLine();
            ImGui::InputInt("##exposure", &form.exposure_us);
        }

        ImGui::BeginDisabled(color_markers);
        ImGui::Checkbox("Manual gain", &form.manual_gain);
        ImGui::EndDisabled();
        if (form.manual_gain)
        {
            ImGui::SameLine();
            ImGui::InputInt("##gain", &form.gain);
            ImGui::SetItemTooltip("K4A: raw gain. VZ: gain in dB.");
        }

        if (color_markers) {
            ImGui::TextDisabled("Color markers are measured at one brightness, so both are fixed.");
        }

        if (_backend == hw::sensor_backend_t::vz)
        {
            ImGui::TextUnformatted("Calibration");
            if (ImGui::Button("Browse...##intr")) { _intrinsics_for = form_idx; _intrinsics_browser.Open(); }
            ImGui::SameLine();
            if (ImGui::Button("Clear##intr")) { form.intrinsics.clear(); }
            ImGui::SameLine();
            ImGui::TextWrapped("%s", form.intrinsics.empty()
                ? "(none: tag poses will not be solved)"
                : form.intrinsics.c_str());
            ImGui::SetItemTooltip(
                "OpenCV FileStorage (.yml/.xml) from a chessboard calibration.\n"
                "Must have been measured at the camera's own frame size.");
        }

        ImGui::PopID();
    }

    open_source_dialog::result_t open_source_dialog::render(app::app_config_t& config, const bool source_open)
    {
        result_t result;
        if (!_show)
        {
            this->_render_browsers(result); // a pick made before the dialog was dismissed still lands
            return result;
        }

        ImGui::SetNextWindowSize(ImVec2(460, 0), ImGuiCond_Appearing);
        if (ImGui::Begin("Open Source", &_show, ImGuiWindowFlags_NoCollapse))
        {
            if (ImGui::Button("Load Config..."))
            {
                _config_load_browser.SetPwd(app::project_dir("configs"));
                _config_load_browser.Open();
            }
            ImGui::SetItemTooltip(
                "Reads a saved config over the current settings and fills this dialog from it.\n"
                "An open source is reopened with it at once; with none open it takes effect on\n"
                "Open. Either way it replaces what the control panel has been tuned to since the\n"
                "last save.");

            ImGui::Separator();

            // The marker kind and the viewing plane decide which detector and estimator run, and
            // swapping either mid-stream would invalidate the rest pose, so both are picked here.
            const auto marker_kind_radio = [this](const char* label, app::marker_kind_t val) {
                if (ImGui::RadioButton(label, _marker_kind == val)) { _marker_kind = val; }
            };
            ImGui::TextUnformatted("Markers");
            marker_kind_radio("AprilTag", app::marker_kind_t::apriltag);
            ImGui::SameLine();
            marker_kind_radio("Color", app::marker_kind_t::color_marker);
            ImGui::SetItemTooltip("AprilTag: each tag states its own id, so a detection names its joint.\n"
                                  "Color:    plain discs, named by their order down the leg. The camera\n"
                                  "          streams colour, and the colour itself is measured on site.");

            const bool color_markers = (_marker_kind == app::marker_kind_t::color_marker);
            if (color_markers)
            {
                // A disc solves no distance, so colour is a sagittal setup.
                _view_plane = pose::view_plane_t::sagittal;
            }

            const auto view_plane_radio = [this](const char* label, pose::view_plane_t val) {
                if (ImGui::RadioButton(label, _view_plane == val)) { _view_plane = val; }
            };
            ImGui::TextUnformatted("Viewing plane");
            ImGui::BeginDisabled(color_markers);
            view_plane_radio("Frontal", pose::view_plane_t::frontal);
            ImGui::SameLine();
            view_plane_radio("Sagittal", pose::view_plane_t::sagittal);
            ImGui::EndDisabled();
            ImGui::SetItemTooltip("Frontal:  one camera faces the exo; both legs tagged, rig solved in 3D.\n"
                                  "Sagittal: two cameras, one at each side of the exo; each reads its own\n"
                                  "          leg's angles off the image plane (no tag pose solve).");

            ImGui::Separator();

            const auto kind_radio = [this](const char* label, source_kind_t val) {
                if (ImGui::RadioButton(label, _kind == val)) { _kind = val; }
            };
            ImGui::TextUnformatted("Source");
            kind_radio("Cameras", source_kind_t::cameras);
            ImGui::SameLine();
            kind_radio("Recording", source_kind_t::recording);
            ImGui::Separator();

            const std::size_t count = this->_entry_count();
            if (_kind == source_kind_t::recording)
            {
                if (ImGui::Button("Browse...")) { _recording_browser.Open(); }
                ImGui::SameLine();
                ImGui::TextUnformatted(_recording.empty() ? "(no file selected)" : _recording.c_str());

                if (_view_plane == pose::view_plane_t::sagittal)
                {
                    // 파일은 어느 쪽에서 찍힌 스트림인지 말하지 않는다. 둘은 서로 반대쪽이므로 첫
                    // 스트림만 정하면 나머지가 따라온다.
                    ImGui::TextUnformatted("First stream was shot from");
                    ImGui::SameLine();
                    if (ImGui::RadioButton("Left", _first_entry_side == pose::joint_side_t::left)) {
                        _first_entry_side = pose::joint_side_t::left;
                    }
                    ImGui::SameLine();
                    if (ImGui::RadioButton("Right", _first_entry_side == pose::joint_side_t::right)) {
                        _first_entry_side = pose::joint_side_t::right;
                    }
                }
            }
            else
            {
                // 한 현장은 같은 기종으로 맞추고, 함께 여는 스트림은 한 레이트로 돈다.
                const auto backend_radio = [this](const char* label, const hw::sensor_backend_t val) {
                    if (ImGui::RadioButton(label, _backend == val) && _backend != val)
                    {
                        _backend = val;
                        _devices.clear(); // 목록은 한 백엔드의 것이다
                        _devices_enumerated = false;
                    }
                };
                ImGui::TextUnformatted("Camera model");
                backend_radio("K4A", hw::sensor_backend_t::k4a);
                ImGui::SameLine();
                backend_radio("VZ", hw::sensor_backend_t::vz);

                if (_backend == hw::sensor_backend_t::vz)
                {
                    ImGui::Checkbox("Limit frame rate [fps]", &_limit_frame_rate);
                    ImGui::SetItemTooltip("Cameras opened together are fixed to one rate so their frames pair up.\n"
                                          "Unlimited, the camera free-runs at its own ceiling.");
                    if (_limit_frame_rate)
                    {
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(120.0f);
                        ImGui::InputFloat("##fps", &_frame_rate_fps, 1.0f, 5.0f, "%.1f");
                        _frame_rate_fps = std::max(1.0f, _frame_rate_fps);
                    }
                }

                ImGui::BeginDisabled(source_open);
                if (ImGui::Button("Scan for cameras"))
                {
                    _devices = hw::enumerate_devices(_backend);
                    _devices_enumerated = true;
                    spdlog::info("open dialog: {} {} camera(s) found"
                        , _devices.size(), hw::sensor_backend_to_str(_backend));
                }
                ImGui::EndDisabled();
                ImGui::SameLine();
                if (source_open) {
                    ImGui::TextDisabled("a source is open");
                } else if (_devices_enumerated) {
                    ImGui::Text("%zu found", _devices.size());
                } else {
                    ImGui::TextDisabled("not scanned");
                }

                for (std::size_t form_idx = 0; form_idx < count; ++form_idx) {
                    this->_render_camera(form_idx, source_open);
                }

                if (count >= 2)
                {
                    ImGui::SeparatorText("Sync");
                    ImGui::TextUnformatted("Reference camera");
                    ImGui::SameLine();
                    if (ImGui::RadioButton("Left##ref", _reference_side == pose::joint_side_t::left)) {
                        _reference_side = pose::joint_side_t::left;
                    }
                    ImGui::SameLine();
                    if (ImGui::RadioButton("Right##ref", _reference_side == pose::joint_side_t::right)) {
                        _reference_side = pose::joint_side_t::right;
                    }
                    ImGui::SetItemTooltip("The other camera's captures are matched to this one's moments.");
                    ImGui::Checkbox("Limit pair skew [ms]", &_limit_pair_skew);
                    ImGui::SetItemTooltip("How far apart two captures may be and still count as one moment.\n"
                                          "Unset, half the reference camera's frame interval is used.");
                    if (_limit_pair_skew)
                    {
                        ImGui::SameLine();
                        ImGui::InputFloat("##skew", &_max_pair_skew_ms, 0.5f, 2.0f, "%.1f");
                        _max_pair_skew_ms = std::max(0.1f, _max_pair_skew_ms);
                    }
                }
            }

            ImGui::Separator();
            if (ImGui::Button("Open", ImVec2(90, 0)) && this->_apply(config))
            {
                result.open_source = true;
                _show = false;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(90, 0))) { _show = false; }
        }
        ImGui::End();

        this->_render_browsers(result);
        return result;
    }

    void open_source_dialog::_render_browsers(result_t& result)
    {
        _config_load_browser.Display();
        if (_config_load_browser.HasSelected())
        {
            result.load_config = _config_load_browser.GetSelected();
            _config_load_browser.ClearSelected();
        }

        _recording_browser.Display();
        if (_recording_browser.HasSelected())
        {
            _recording = _recording_browser.GetSelected().string();
            _kind = source_kind_t::recording;
            _recording_browser.ClearSelected();
        }

        _intrinsics_browser.Display();
        if (_intrinsics_browser.HasSelected())
        {
            if (_intrinsics_for < _cameras.size()) {
                _cameras[_intrinsics_for].intrinsics = _intrinsics_browser.GetSelected().string();
            }
            _intrinsics_browser.ClearSelected();
        }
    }

} // namespace gui
