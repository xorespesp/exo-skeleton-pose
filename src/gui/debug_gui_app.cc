#include "debug_gui_app.hh"

#include <spdlog/spdlog.h>

#include <imgui.h>
#include <implot3d.h>

#include <algorithm>
#include <format>
#include <numbers>

namespace gui
{
    namespace
    {
        // Selectable euler decomposition order (Eigen axis indices: 0=X, 1=Y, 2=Z).
        struct euler_order_t { const char* name; int a0, a1, a2; };
        constexpr std::array<euler_order_t, 6> kEulerOrders{ {
            { "XYZ", 0, 1, 2 }, { "XZY", 0, 2, 1 }, { "YXZ", 1, 0, 2 },
            { "YZX", 1, 2, 0 }, { "ZXY", 2, 0, 1 }, { "ZYX", 2, 1, 0 },
        } };

        // Euler angles [deg] of a rotation in the given order (readout only; euler
        // angles wrap and hit gimbal singularities, so drive skeletons from the quaternion).
        Eigen::Vector3d to_euler_deg(const Eigen::Quaterniond& q, const euler_order_t& order)
        {
            return q.toRotationMatrix().eulerAngles(order.a0, order.a1, order.a2) * (180.0 / std::numbers::pi);
        }

        // RGB axis triad (X=red, Y=green, Z=blue) for `q`, into the current ImPlot3D plot.
        void draw_axes(std::string_view prefix, const Eigen::Quaterniond& q, float thickness)
        {
            const Eigen::Vector3d ex = q * Eigen::Vector3d::UnitX();
            const Eigen::Vector3d ey = q * Eigen::Vector3d::UnitY();
            const Eigen::Vector3d ez = q * Eigen::Vector3d::UnitZ();

            const double xx[2]{ 0.0, ex.x() }, xy[2]{ 0.0, ex.y() }, xz[2]{ 0.0, ex.z() };
            const double yx[2]{ 0.0, ey.x() }, yy[2]{ 0.0, ey.y() }, yz[2]{ 0.0, ey.z() };
            const double zx[2]{ 0.0, ez.x() }, zy[2]{ 0.0, ez.y() }, zz[2]{ 0.0, ez.z() };

            ImPlot3DSpec spec;
            spec.LineWeight = thickness;
            spec.LineColor = ImVec4(1, 0, 0, 1);
            ImPlot3D::PlotLine(std::format("{}-X", prefix).c_str(), xx, xy, xz, 2, spec);
            spec.LineColor = ImVec4(0, 1, 0, 1);
            ImPlot3D::PlotLine(std::format("{}-Y", prefix).c_str(), yx, yy, yz, 2, spec);
            spec.LineColor = ImVec4(0, 0, 1, 1);
            ImPlot3D::PlotLine(std::format("{}-Z", prefix).c_str(), zx, zy, zz, 2, spec);
        }
    } // namespace

    void debug_gui_observer::on_sensor_frame_update(const std::shared_ptr<hw::sensor_frame>& frame)
    {
        if (!_detector.has_value()) // built once intrinsics are known (after open)
        {
            pose::tag_detector::options_t opt;
            opt.intrinsics = _provider.get_calibration().color_intr;
            opt.tag_size_m = _tag_size_m;
            _detector.emplace(opt);
        }

        cv::Mat annotated = frame->color_image.clone();
        auto detections = _detector.value().detect(annotated);
        pose::draw_tag_detections(annotated, detections);

        std::scoped_lock lk{ _mtx };
        _latest = std::move(annotated);
        _detections = std::move(detections);
        ++_seq;
    }

    debug_gui_app::debug_gui_app(const app::source_options& opt)
        : _opt{ opt }
    {
        _dlg_device = static_cast<int>(opt.device_index);
        if (opt.exposure_us.has_value()) { _dlg_manual_exposure = true; _dlg_exposure = opt.exposure_us.value(); }
        if (opt.gain.has_value()) { _dlg_manual_gain = true; _dlg_gain = opt.gain.value(); }
        _dlg_recording = opt.input_path;
        _open_kind = opt.is_recording() ? 1 : 0;

        _file_dialog.SetTitle("Open recording (.mkv)");
        _file_dialog.SetTypeFilters({ ".mkv" });
    }

    int debug_gui_app::run()
    {
        if (!this->create("exo-skeleton-pose", 1440, 900))
        {
            spdlog::error("failed to create GUI window");
            return -1;
        }

        this->app_base::run();
        this->destroy();
        return 0;
    }

    void debug_gui_app::render_ui()
    {
        if (!_texture.has_value()) { _texture.emplace(this->renderer().sdl_renderer()); }
        this->_poll_observer();

        if (ImGui::IsKeyPressed(ImGuiKey_F11, false)) { _camera_fullscreen = !_camera_fullscreen; }
        this->_render_menu_bar();

        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        constexpr ImGuiWindowFlags host_flags =
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

        ImGui::Begin("##host", nullptr, host_flags);
        if (!_provider)
        {
            // No source: centered call-to-action only.
            const char* msg = "Open a source to start.   (File > Open...)";
            const ImVec2 avail = ImGui::GetContentRegionAvail();
            const ImVec2 sz = ImGui::CalcTextSize(msg);
            const ImVec2 cur = ImGui::GetCursorPos();
            ImGui::SetCursorPos(ImVec2{ cur.x + (avail.x - sz.x) * 0.5f, cur.y + (avail.y - sz.y) * 0.5f });
            ImGui::TextDisabled("%s", msg);
        }
        else if (_camera_fullscreen)
        {
            // Fullscreen: sensor frame scaled to fit, centered.
            if (!_texture.value().valid()) { ImGui::TextUnformatted("Waiting for frames...  (F11 to exit)"); }
            else
            {
                const ImVec2 avail = ImGui::GetContentRegionAvail();
                const float tw = static_cast<float>(_texture.value().width());
                const float th = static_cast<float>(_texture.value().height());
                const float scale = std::min(avail.x / tw, avail.y / th);
                const ImVec2 sz{ tw * scale, th * scale };
                const ImVec2 cur = ImGui::GetCursorPos();
                ImGui::SetCursorPos(ImVec2{ cur.x + (avail.x - sz.x) * 0.5f, cur.y + (avail.y - sz.y) * 0.5f });
                ImGui::Image(_texture.value().id(), sz);
            }
        }
        else
        {
            // Normal: plot panel (left) + control panel and angle table (right).
            const float left_w = ImGui::GetContentRegionAvail().x * 0.62f;
            ImGui::BeginChild("plots", ImVec2(left_w, 0), ImGuiChildFlags_Borders);
            this->_render_plot_panel();
            ImGui::EndChild();

            ImGui::SameLine();

            ImGui::BeginChild("side", ImVec2(0, 0), ImGuiChildFlags_Borders);
            this->_render_control_panel();
            ImGui::Separator();
            this->_render_angle_table();
            ImGui::EndChild();
        }
        ImGui::End();

        this->_render_open_dialog();

        _file_dialog.Display();
        if (_file_dialog.HasSelected())
        {
            _dlg_recording = _file_dialog.GetSelected().string();
            _open_kind = 1;
            _file_dialog.ClearSelected();
        }
    }

    void debug_gui_app::_open_device(uint32_t index)
    {
        auto provider = std::make_shared<hw::sensor_frame_provider>();
        auto observer = std::make_shared<debug_gui_observer>(*provider, _opt.tag_size_m);
        provider->add_observer(observer);
        if (!provider->open_device(index, _opt.exposure_us, _opt.gain))
        {
            spdlog::error("failed to open device #{}", index);
            return;
        }
        _opt.input_path.clear();
        _provider = std::move(provider); // old provider closes/joins here
        _observer = std::move(observer);
        _last_seq = 0;
        _estimator.clear_rest_pose();
        spdlog::info("source '{}' opened", _provider->get_source_name());
    }

    void debug_gui_app::_open_recording(const std::string& path)
    {
        auto provider = std::make_shared<hw::sensor_frame_provider>();
        auto observer = std::make_shared<debug_gui_observer>(*provider, _opt.tag_size_m);
        provider->add_observer(observer);
        if (!provider->open_recording(path))
        {
            spdlog::error("failed to open recording '{}'", path);
            return;
        }
        _opt.input_path = path; // marks the source as a recording (enables playback controls)
        _provider = std::move(provider); // old provider closes/joins here
        _observer = std::move(observer);
        _last_seq = 0;
        _estimator.clear_rest_pose();
        spdlog::info("source '{}' opened", _provider->get_source_name());
    }

    void debug_gui_app::_do_open_source()
    {
        if (_open_kind == 1)
        {
            if (_dlg_recording.empty()) { spdlog::warn("no recording file selected"); return; }
            _opt.exposure_us.reset();
            _opt.gain.reset();
            this->_open_recording(_dlg_recording);
        }
        else
        {
            _opt.exposure_us = _dlg_manual_exposure ? std::optional<int32_t>{ _dlg_exposure } : std::nullopt;
            _opt.gain = _dlg_manual_gain ? std::optional<int32_t>{ _dlg_gain } : std::nullopt;
            this->_open_device(static_cast<uint32_t>(_dlg_device));
        }
        _show_open = false;
    }

    void debug_gui_app::_do_close_source()
    {
        _provider.reset(); // stops/joins the worker thread
        _observer.reset();
        _opt.input_path.clear();
        _last_seq = 0;
    }

    void debug_gui_app::_poll_observer()
    {
        if (_observer && _observer->try_get(_frame, _detections, _last_seq))
        {
            _texture.value().update(_frame);
            _estimator.update(_detections);
        }
    }

    std::optional<Eigen::Quaterniond> debug_gui_app::_display_rot(const pose::joint_state_t& st) const
    {
        if (_relative_rot) { return st.local_anim_rot; } // local: relative to parent (delta from rest)
        if (st.is_visible()) { return Eigen::Quaterniond{ st.view_pose.value().rotation() }.normalized(); }
        return std::nullopt; // absolute: tag orientation in the camera frame
    }

    void debug_gui_app::_render_menu_bar()
    {
        if (!ImGui::BeginMainMenuBar()) { return; }
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open...")) { _show_open = true; }
            if (ImGui::MenuItem("Close", nullptr, false, _provider != nullptr)) { this->_do_close_source(); }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) { SDL_Event e{}; e.type = SDL_EVENT_QUIT; ::SDL_PushEvent(&e); }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Fullscreen", "F11", &_camera_fullscreen);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    void debug_gui_app::_render_control_panel()
    {
        if (_provider)
        {
            ImGui::TextUnformatted(std::format("Source : {}", _provider->get_source_name()).c_str());
            const auto res = _provider->get_color_camera_resolution();
            ImGui::TextUnformatted(std::format("Color  : {}x{}", res.x(), res.y()).c_str());
            ImGui::TextUnformatted(std::format("FPS    : {:.1f}", _provider->get_current_update_rate()).c_str());

            if (_opt.is_recording())
            {
                if (ImGui::Button(_provider->is_paused() ? "Play" : "Pause"))
                {
                    _provider->is_paused() ? _provider->play() : _provider->pause();
                }
                ImGui::SameLine();
                if (ImGui::Button("|< Begin")) { _provider->seek_recording_to_begin(); }
                ImGui::SameLine();
                if (ImGui::Button("End >|")) { _provider->seek_recording_to_end(); }
            }
        }
        else
        {
            ImGui::TextUnformatted("No source opened. (File > Open...)");
        }

        ImGui::SeparatorText("Rest Pose");

        ImGui::TextUnformatted(_estimator.has_rest_pose() ? "rest pose: calibrated" : "rest pose: N/A");
        ImGui::SameLine();
        if (ImGui::Button("Calibrate")) {
            if (!_estimator.calibrate_rest_pose()) {
                spdlog::warn("calibrate: no joint had a computable local rotation");
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear")) { _estimator.clear_rest_pose(); }

        ImGui::SeparatorText("Visualization");
        ImGui::Checkbox("Relative Rotation", &_relative_rot);
        if (ImGui::BeginCombo("Euler Order", kEulerOrders[_euler_order].name))
        {
            for (int i = 0; i < static_cast<int>(kEulerOrders.size()); ++i)
            {
                const bool selected = (i == _euler_order);
                if (ImGui::Selectable(kEulerOrders[i].name, selected)) { _euler_order = i; }
                if (selected) { ImGui::SetItemDefaultFocus(); }
            }
            ImGui::EndCombo();
        }
        ImGui::Checkbox("Auto-fit Subplots", &_subplot_autofit);
        if (!_subplot_autofit) {
            ImGui::SliderFloat("Subplot Size", &_subplot_size, 80.0f, 400.0f, "%.0f px");
        }

        ImGui::SeparatorText("Sensor Frame");
        if (_texture.value().valid())
        {
            const float scale = ImGui::GetContentRegionAvail().x / _texture.value().width();
            ImGui::Image(_texture.value().id(), ImVec2{ _texture.value().width() * scale, _texture.value().height() * scale });
        }
        else
        {
            ImGui::TextUnformatted("Waiting for sensor frames...");
        }
    }

    void debug_gui_app::_render_plot_panel()
    {
        const int n = static_cast<int>(pose::kNumJoints);
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        const ImVec2 avail = ImGui::GetContentRegionAvail();

        int cols = 1;
        float cell_sz = 1.0f;
        if (_subplot_autofit)
        {
            // Pick the column count whose square cell best fills the panel area.
            for (int c = 1; c <= n; ++c)
            {
                const int r = (n + c - 1) / c;
                const float cw = (avail.x - spacing * (c - 1)) / c;
                const float ch = (avail.y - spacing * (r - 1)) / r;
                if (const float s = std::min(cw, ch); s > cell_sz) { cell_sz = s; cols = c; }
            }
        }
        else
        {
            cell_sz = _subplot_size * this->renderer().dpi_scale(); // DPI-aware px
            cols = std::max(1, static_cast<int>((avail.x + spacing) / (cell_sz + spacing)));
        }
        const ImVec2 cell{ cell_sz, cell_sz };

        int col = 0;
        for (const auto& info : pose::kJointsInfo)
        {
            const auto& st = _estimator.get_joint_state(info.id);
            const char* ref = (!_relative_rot || pose::is_root_joint(info.id))
                ? "camera" : pose::joint_info(info.parent).name.data();
            const auto rot = this->_display_rot(st);

            if (col != 0) { ImGui::SameLine(); }
            ImGui::BeginGroup();
            const std::string title = std::format("{} (ref: {})##giz", info.name, ref);
            if (ImPlot3D::BeginPlot(title.c_str(), cell, ImPlot3DFlags_Equal | ImPlot3DFlags_NoClip))
            {
                ImPlot3D::SetupAxesLimits(-1.2, 1.2, -1.2, 1.2, -1.2, 1.2, ImPlot3DCond_Always);
                if (rot.has_value()) { draw_axes(info.name, rot.value(), 3.0f); }
                ImPlot3D::EndPlot();
            }
            ImGui::EndGroup();

            if (++col >= cols) { col = 0; }
        }
    }

    void debug_gui_app::_render_angle_table()
    {
        constexpr ImGuiTableFlags flags =
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
        if (!ImGui::BeginTable("angles", 5, flags)) { return; }

        ImGui::TableSetupColumn("Joint ID");
        ImGui::TableSetupColumn("Visible");
        ImGui::TableSetupColumn("X deg");
        ImGui::TableSetupColumn("Y deg");
        ImGui::TableSetupColumn("Z deg");
        ImGui::TableHeadersRow();

        for (const auto& info : pose::kJointsInfo)
        {
            const auto& st = _estimator.get_joint_state(info.id);
            const auto rot = this->_display_rot(st);
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(std::format("{} (#{})", info.name, info.tag_id).c_str());
            ImGui::TableNextColumn(); ImGui::TextUnformatted(st.is_visible() ? "o" : "-");
            if (rot.has_value())
            {
                const Eigen::Vector3d e = to_euler_deg(rot.value(), kEulerOrders[_euler_order]);
                ImGui::TableNextColumn(); ImGui::TextUnformatted(std::format("{:6.1f}", e.x()).c_str());
                ImGui::TableNextColumn(); ImGui::TextUnformatted(std::format("{:6.1f}", e.y()).c_str());
                ImGui::TableNextColumn(); ImGui::TextUnformatted(std::format("{:6.1f}", e.z()).c_str());
            }
            else
            {
                ImGui::TableNextColumn(); ImGui::TextUnformatted("-");
                ImGui::TableNextColumn(); ImGui::TextUnformatted("-");
                ImGui::TableNextColumn(); ImGui::TextUnformatted("-");
            }
        }
        ImGui::EndTable();
    }

    void debug_gui_app::_render_open_dialog()
    {
        if (!_show_open) { return; }
        ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_Appearing);
        if (ImGui::Begin("Open Source", &_show_open, ImGuiWindowFlags_NoCollapse))
        {
            ImGui::RadioButton("Device", &_open_kind, 0);
            ImGui::SameLine();
            ImGui::RadioButton("Recording", &_open_kind, 1);
            ImGui::Separator();

            if (_open_kind == 0)
            {
                ImGui::InputInt("Device index", &_dlg_device);
                if (_dlg_device < 0) { _dlg_device = 0; }
                ImGui::Checkbox("Manual exposure [us]", &_dlg_manual_exposure);
                if (_dlg_manual_exposure)
                {
                    ImGui::SameLine(); ImGui::SetNextItemWidth(120);
                    ImGui::InputInt("##exposure", &_dlg_exposure);
                }
                ImGui::Checkbox("Manual gain", &_dlg_manual_gain);
                if (_dlg_manual_gain)
                {
                    ImGui::SameLine(); ImGui::SetNextItemWidth(120);
                    ImGui::InputInt("##gain", &_dlg_gain);
                }
            }
            else
            {
                if (ImGui::Button("Browse...")) { _file_dialog.Open(); }
                ImGui::SameLine();
                ImGui::TextUnformatted(_dlg_recording.empty() ? "(no file selected)" : _dlg_recording.c_str());
            }

            ImGui::Separator();
            if (ImGui::Button("Open", ImVec2(90, 0))) { this->_do_open_source(); }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(90, 0))) { _show_open = false; }
        }
        ImGui::End();
    }

} // namespace gui
