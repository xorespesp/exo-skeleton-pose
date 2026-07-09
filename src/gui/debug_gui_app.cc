#include "debug_gui_app.hh"

#include "net/exo_pose_server.hh"
#include "hw/sensor_frame_observer.hh"

#include <spdlog/spdlog.h>

#include <imgui.h>
#include <implot.h>
#include <implot3d.h>
#include <implot3d_internal.h> // GetCurrentPlot / ImPlot3DPlot for axis-frame sync

#include <algorithm>
#include <cmath>
#include <format>
#include <mutex>
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

        Eigen::Vector3d to_euler_deg(const Eigen::Quaterniond& q, const euler_order_t& order)
        {
            return q.toRotationMatrix().eulerAngles(order.a0, order.a1, order.a2) * (180.0 / std::numbers::pi);
        }

        std::optional<Eigen::Quaterniond> try_get_joint_rot(const pose::joint_state_t& st, bool relative)
        {
            if (relative) { return st.local_anim_rot; }
            if (st.view_pose.has_value()) { return Eigen::Quaterniond{ st.view_pose.value().rotation() }.normalized(); }
            return std::nullopt;
        }

        // RGB axis triad (X=red, Y=green, Z=blue) for `q`, into the current ImPlot3D plot.
        void draw_axes(const Eigen::Quaterniond& q, float thickness)
        {
            const Eigen::Vector3d ex = q * Eigen::Vector3d::UnitX();
            const Eigen::Vector3d ey = q * Eigen::Vector3d::UnitY();
            const Eigen::Vector3d ez = q * Eigen::Vector3d::UnitZ();

            const double xx[2]{ 0.0, ex.x() }, xy[2]{ 0.0, ex.y() }, xz[2]{ 0.0, ex.z() };
            const double yx[2]{ 0.0, ey.x() }, yy[2]{ 0.0, ey.y() }, yz[2]{ 0.0, ey.z() };
            const double zx[2]{ 0.0, ez.x() }, zy[2]{ 0.0, ez.y() }, zz[2]{ 0.0, ez.z() };

            // NOTE: Use short legend names; the caller wraps each subplot in PushID/PopID so ids stay unique.
            ImPlot3DSpec spec;
            spec.LineWeight = thickness;
            spec.LineColor = ImVec4(1, 0, 0, 1);
            ImPlot3D::PlotLine("X", xx, xy, xz, 2, spec);
            spec.LineColor = ImVec4(0, 1, 0, 1);
            ImPlot3D::PlotLine("Y", yx, yy, yz, 2, spec);
            spec.LineColor = ImVec4(0, 0, 1, 1);
            ImPlot3D::PlotLine("Z", zx, zy, zz, 2, spec);
        }

        // Splitter grip thickness [px]. It doubles as the inter-panel gap: surrounding
        // ItemSpacing is zeroed so the visible border-to-border gap equals this on both
        // axes, and the whole gap is the drag hit-target (same width for v/h splitters).
        constexpr float kSplitHit = 6.0f;
        constexpr float kLogMinH = 60.0f;  // min height for both the content and log panes [px]
        constexpr float kPlotMinW = 200.0f; // min width for the plots pane [px]
        constexpr float kSideMinW = 200.0f; // min width for the control pane [px]

        constexpr float kWindowSec = 10.0f; // scrolling line-plot window [s]
        constexpr float kEulerYLo = -190.0f, kEulerYHi = 190.0f; // euler deg range (all subplots)
        constexpr float kQuatYLo = -1.1f, kQuatYHi = 1.1f; // quaternion range (all subplots)

        // Scrolling line plot of a buffer's channels over the newest `window` seconds.
        // x is the device time (`v.xs`); channel k is `v.ys.data() + k`, both strided by `v.stride`.
        // x always auto-scrolls; only y obeys `y_cond` (Always locks, Once leaves it mouse-free) and
        // `sync` (links y to the shared `sy` so all subplots share one y range).
        template <typename _Scalar>
        void draw_lines(
            const char* title,
            const plot_buffer_view<_Scalar>& v,
            float window,
            float y_lo,
            float y_hi,
            ImPlotCond y_cond,
            bool sync,
            double* sy,
            const ImVec4* colors,
            const char* const* names,
            const ImVec2& size)
        {
            // legend shown (short names); the caller wraps each subplot in PushID/PopID for unique ids.
            if (!ImPlot::BeginPlot(title, size, ImPlotFlags_None)) { return; }

            ImPlot::SetupAxes(nullptr, nullptr, 0, 0);
            ImPlot::SetupLegend(ImPlotLocation_NorthWest);
            if (sync) { ImPlot::SetupAxisLinks(ImAxis_Y1, &sy[0], &sy[1]); } // sync y only
            // x always tracks the newest `window` seconds; y follows lock/sync.
            ImPlot::SetupAxisLimits(ImAxis_X1, v.t_hi - window, v.t_hi, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, y_lo, y_hi, y_cond);

            for (std::size_t k = 0; k < v.ys.size(); ++k)
            {
                ImPlotSpec spec;
                spec.LineColor = colors[k];
                spec.LineWeight = 2.0f;
                spec.Offset = v.offset;
                spec.Stride = v.stride;
                ImPlot::PlotLine(names[k], v.xs, v.ys.data() + k, v.count, spec);
            }
            ImPlot::EndPlot();
        }

    } // namespace

    // Worker-thread tag detection; latches the annotated image + detections for the UI.
    class debug_gui_observer final : public hw::sensor_frame_observer
    {
    public:
        debug_gui_observer(const hw::sensor_frame_provider& provider, double tag_size_m)
            : _provider{ provider }, _tag_size_m{ tag_size_m }
        { }

        bool try_get(
            cv::Mat& out_img,
            std::vector<pose::tag_detection_t>& out_dets,
            std::chrono::microseconds& out_ts,
            uint64_t& last_seq)
        {
            std::scoped_lock lk{ _mtx };
            if (_seq == last_seq || _latest.empty()) { return false; }
            out_img = _latest;
            out_dets = _detections;
            out_ts = _timestamp;
            last_seq = _seq;
            return true;
        }

    public:
        void on_sensor_frame_update(const std::shared_ptr<hw::sensor_frame>& frame) override;
        void on_sensor_stream_reset() override {}
        void on_sensor_stream_end() override {}

    private:
        const hw::sensor_frame_provider& _provider;
        double _tag_size_m{};
        std::optional<pose::tag_detector> _detector;
        std::mutex _mtx;
        cv::Mat _latest;
        std::vector<pose::tag_detection_t> _detections;
        std::chrono::microseconds _timestamp{ 0 }; // device timestamp of the latched frame
        uint64_t _seq{ 0 };
    };

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
        _timestamp = frame->timestamp;
        ++_seq;
    }

    debug_gui_app::debug_gui_app(
        const app::source_options& opt, 
        net::exo_pose_server* server)
        : _opt{ opt }
        , _server{ server }
    {
        _ui.device = static_cast<int>(opt.device_index);
        if (opt.exposure_us.has_value()) { _ui.manual_exposure = true; _ui.exposure = opt.exposure_us.value(); }
        if (opt.gain.has_value()) { _ui.manual_gain = true; _ui.gain = opt.gain.value(); }
        _ui.recording = opt.input_path;
        _ui.open_kind = opt.is_recording() ? source_kind_t::recording : source_kind_t::device;
        _ui.show_log = true; // surface the log console by default

        _file_dialog.SetTitle("Open recording file");
        _file_dialog.SetTypeFilters({ ".mkv" });

        // Mirror spdlog output into the in-GUI log console. Registered here (main thread,
        // before any capture worker exists) so appending to the sink list is race-free.
        spdlog::default_logger()->sinks().push_back(_log_console.sink());
    }

    pose::exo_pose_estimator& debug_gui_app::_est()
    {
        return _server ? _server->estimator() : _estimator;
    }

    bool debug_gui_app::_is_source_recording() const
    {
        return _server ? _server->is_source_recording() : _opt.is_recording();
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

        // Advance the server one tick: services the listener when up, and always pumps the
        // pipeline so device/algorithm testing works whether or not it's running.
        if (_server) { _server->poll(); }
        this->_poll_observer();

        if (ImGui::IsKeyPressed(ImGuiKey_F11, false)) { _ui.camera_fullscreen = !_ui.camera_fullscreen; }
        this->_render_menu_bar();

        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        constexpr ImGuiWindowFlags host_flags =
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

        // Drop the host's rounded corners and outer border.
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::Begin("##host", nullptr, host_flags);
        ImGui::PopStyleVar(2);

        // plots, control, and log are all direct host siblings (no wrapper child), so
        // every inter-panel gap is the same kSplitHit-wide grip drawn over the same host
        // background: the log/plots gap matches the control/plots gap exactly.
        // `row_h` is the height of the main content row above the (optional) log panel.
        const bool show_log = _ui.show_log && !_ui.camera_fullscreen;
        const float row_h = show_log ? this->_log_split_height() : ImGui::GetContentRegionAvail().y;

        if (!_provider)
        {
            // No source: centered call-to-action, bounded to the content row.
            ImGui::BeginChild("content", ImVec2(0, row_h), ImGuiChildFlags_None);
            const char* msg = "Open a source to start.   (File > Open...)";
            const ImVec2 avail = ImGui::GetContentRegionAvail();
            const ImVec2 sz = ImGui::CalcTextSize(msg);
            const ImVec2 cur = ImGui::GetCursorPos();
            ImGui::SetCursorPos(ImVec2{ cur.x + (avail.x - sz.x) * 0.5f, cur.y + (avail.y - sz.y) * 0.5f });
            ImGui::TextDisabled("%s", msg);
            ImGui::EndChild();
        }
        else if (_ui.camera_fullscreen)
        {
            // Fullscreen: sensor frame scaled to fit, centered (log panel is hidden here).
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
            // Normal: plot panel (left) + control panel (right), split by a grip whose
            // width equals the log splitter's so every gap looks identical.
            const float avail_x = ImGui::GetContentRegionAvail().x;
            const float max_side = std::max(kSideMinW, avail_x - kSplitHit - kPlotMinW);
            _ui.side_w = std::clamp(_ui.side_w, kSideMinW, max_side);
            const float plots_w = avail_x - _ui.side_w - kSplitHit;

            ImGui::BeginChild("plots", ImVec2(plots_w, row_h), ImGuiChildFlags_Borders);
            this->_render_plot_panel();
            ImGui::EndChild();

            // Vertical resize grip (no visible line): flush to both panes (zero spacing),
            // so the whole inter-panel gap is grabbable. Drag left to grow the control pane.
            ImGui::SameLine(0.0f, 0.0f);
            ImGui::InvisibleButton("##side_split", ImVec2(kSplitHit, row_h));
            if (ImGui::IsItemActive()) { _ui.side_w -= ImGui::GetIO().MouseDelta.x; }
            if (ImGui::IsItemHovered() || ImGui::IsItemActive()) { ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW); }
            ImGui::SameLine(0.0f, 0.0f);

            ImGui::BeginChild("side", ImVec2(0, row_h), ImGuiChildFlags_Borders);
            this->_render_control_panel();
            ImGui::EndChild();
        }

        if (show_log) { this->_render_log_panel(); }

        ImGui::End();

        this->_render_open_dialog();

        _file_dialog.Display();
        if (_file_dialog.HasSelected())
        {
            _ui.recording = _file_dialog.GetSelected().string();
            _ui.open_kind = source_kind_t::recording;
            _file_dialog.ClearSelected();
        }
    }

    void debug_gui_app::_open_device(uint32_t index)
    {
        if (_server) // monitor mode: the server owns the source
        {
            _server->open_device(index, _opt.exposure_us, _opt.gain);
            _last_seq = 0;
            _euler_bufs.clear();
            _quat_bufs.clear();
            return;
        }

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
        _euler_bufs.clear();
        _quat_bufs.clear();
        spdlog::info("source '{}' opened", _provider->get_source_name());
    }

    void debug_gui_app::_open_recording(const std::string& path)
    {
        if (_server) // monitor mode: the server owns the source
        {
            _server->open_recording(path);
            _last_seq = 0;
            _euler_bufs.clear();
            _quat_bufs.clear();
            return;
        }

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
        _euler_bufs.clear();
        _quat_bufs.clear();
        spdlog::info("source '{}' opened", _provider->get_source_name());
    }

    void debug_gui_app::_do_open_source()
    {
        if (_ui.open_kind == source_kind_t::recording)
        {
            if (_ui.recording.empty()) { spdlog::warn("no recording file selected"); return; }
            _opt.exposure_us.reset();
            _opt.gain.reset();
            this->_open_recording(_ui.recording);
        }
        else
        {
            _opt.exposure_us = _ui.manual_exposure ? std::optional<int32_t>{ _ui.exposure } : std::nullopt;
            _opt.gain = _ui.manual_gain ? std::optional<int32_t>{ _ui.gain } : std::nullopt;
            this->_open_device(static_cast<uint32_t>(_ui.device));
        }
        _ui.show_open = false;
    }

    void debug_gui_app::_do_close_source()
    {
        if (_server) { _server->close_source(); } // the server owns the source
        else
        {
            _provider.reset(); // stops/joins the worker thread
            _observer.reset();
            _opt.input_path.clear();
        }
        _last_seq = 0;
        _euler_bufs.clear();
        _quat_bufs.clear();
    }

    void debug_gui_app::_poll_observer()
    {
        std::chrono::microseconds ts{ 0 };
        if (_server)
        {
            // Monitor mode: mirror the server's provider (for sensor info + playback) and
            // pull its annotated frame. The server owns and updates the estimator.
            _provider = _server->provider_shared();
            if (!_server->try_get_annotated_frame(_frame, _detections, ts, _last_seq)) { return; }
        }
        else
        {
            if (!_observer || !_observer->try_get(_frame, _detections, ts, _last_seq)) { return; }
            _estimator.update(_detections, ts);
        }

        _texture.value().update(_frame);

        // Append the current per-joint euler/quat samples, stamped with the device frame time.
        const double ts_sec = std::chrono::duration<double>{ ts }.count();
        _euler_bufs.advance(ts_sec);
        _quat_bufs.advance(ts_sec);

        int ji = 0;
        for (const auto& info : pose::kJointsInfo)
        {
            const auto& st = _est().get_joint_state(info.id);
            const auto rot = try_get_joint_rot(st, _ui.relative_rot);
            if (rot.has_value())
            {
                const Eigen::Vector3d e = to_euler_deg(rot.value(), kEulerOrders[_ui.euler_order]);
                _euler_bufs.push(ji, e.cast<float>());
                _quat_bufs.push(ji, rot.value().cast<float>());
            }
            else // gap while the joint has no rotation this frame; NaN breaks the line
            {
                Eigen::Quaternionf qn;
                qn.coeffs().setConstant(std::nanf(""));
                _euler_bufs.push(ji, Eigen::Vector3f::Constant(std::nanf("")));
                _quat_bufs.push(ji, qn);
            }
            ++ji;
        }
    }

    void debug_gui_app::_render_menu_bar()
    {
        if (!ImGui::BeginMainMenuBar()) { return; }
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open...")) { _ui.show_open = true; }
            if (ImGui::MenuItem("Close", nullptr, false, _provider != nullptr)) { this->_do_close_source(); }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) { SDL_Event e{}; e.type = SDL_EVENT_QUIT; ::SDL_PushEvent(&e); }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Fullscreen", "F11", &_ui.camera_fullscreen);
            ImGui::MenuItem("Log Panel", nullptr, &_ui.show_log);
            ImGui::EndMenu();
        }
        if (_server && ImGui::BeginMenu("Server"))
        {
            const bool running = _server->is_listening();
            if (ImGui::MenuItem("Start Server", nullptr, false, !running)) { _server->start(); }
            if (ImGui::MenuItem("Stop Server", nullptr, false, running)) { _server->stop(); }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    void debug_gui_app::_render_control_panel()
    {
        // Sensor info section
        if (ImGui::CollapsingHeader("Sensor Info", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (_provider)
            {
                // annotated sensor frame (texture) at the top of the section
                if (_texture.value().valid())
                {
                    const float scale = ImGui::GetContentRegionAvail().x / _texture.value().width();
                    ImGui::Image(_texture.value().id(), ImVec2{ _texture.value().width() * scale, _texture.value().height() * scale });
                }
                else
                {
                    ImGui::TextUnformatted("Waiting for sensor frames...");
                }

                ImGui::TextUnformatted(std::format("Source : {}", _provider->get_source_name()).c_str());
                const auto res = _provider->get_color_camera_resolution();
                ImGui::TextUnformatted(std::format("Color  : {}x{}", res.x(), res.y()).c_str());
                ImGui::TextUnformatted(std::format("FPS    : {:.1f}", _provider->get_current_update_rate()).c_str());

                if (this->_is_source_recording())
                {
                    if (ImGui::Button(_provider->is_paused() ? "Play" : "Pause")) {
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
        }

        // Visualization section
        if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Checkbox("Relative Rotation", &_ui.relative_rot)) {
                // A rotation-basis change invalidates both histories
                _euler_bufs.clear();
                _quat_bufs.clear();
            }

            if (ImGui::BeginCombo("Euler Order", kEulerOrders[_ui.euler_order].name)) {
                for (int i = 0; i < static_cast<int>(kEulerOrders.size()); ++i) {
                    const bool selected = (i == _ui.euler_order);
                    if (ImGui::Selectable(kEulerOrders[i].name, selected) && i != _ui.euler_order) {
                        _ui.euler_order = i;
                        _euler_bufs.clear(); // quats are order-independent; keep their history
                    }
                    if (selected) { ImGui::SetItemDefaultFocus(); }
                }
                ImGui::EndCombo();
            }

            constexpr std::array<const char*, 3> plot_types{ "Axis Frame", "Euler Angles", "Quaternion" };
            if (ImGui::BeginCombo("Plot Type", plot_types[static_cast<int>(_ui.plot_type)])) {
                for (size_t i = 0; i < plot_types.size(); ++i) {
                    const plot_type_t curr_plot_type = static_cast<plot_type_t>(i);
                    const bool selected = (curr_plot_type == _ui.plot_type);
                    if (ImGui::Selectable(plot_types[i], selected) && !selected) {
                        _ui.plot_type = curr_plot_type;
                        _reset_plots = true;
                    }
                    if (selected) { ImGui::SetItemDefaultFocus(); }
                }
                ImGui::EndCombo();
            }

            ImGui::Checkbox("Lock Plots", &_ui.lock_plots);
            ImGui::SameLine();
            if (ImGui::Checkbox("Sync Plots", &_ui.sync_plots)) { _reset_plots = true; }
            ImGui::SameLine();
            if (ImGui::Button("Reset Plots")) { _reset_plots = true; }

            ImGui::Checkbox("Auto-size Plots", &_ui.autosize_plots);
            if (!_ui.autosize_plots) {
                ImGui::SliderFloat("Plots Size", &_ui.plot_size_px, 80.0f, 400.0f, "%.0f px");
            }
        }

        // Control section
        if (ImGui::CollapsingHeader("Control", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // ----- Rest Pose calibration options -----
            ImGui::SeparatorText("Rest Pose");
            {
                ImGui::TextUnformatted(_est().has_rest_pose() ? "Rest Pose: calibrated" : "Rest Pose: N/A");
                ImGui::SameLine();
                if (ImGui::Button("Calibrate")) {
                    const bool ok = _server ? _server->calibrate_rest_pose() : _estimator.calibrate_rest_pose();
                    if (!ok) { spdlog::warn("calibrate: no joint had a computable local rotation"); }
                }
                ImGui::SameLine();
                if (ImGui::Button("Clear")) {
                    if (_server) { _server->clear_rest_pose(); } else { _estimator.clear_rest_pose(); }
                }
            }

            // ----- Rotation filter options -----
            ImGui::SeparatorText("Rotation Filter");
            {
                constexpr auto kFlags = ImGuiSliderFlags_AlwaysClamp;
                // Small double-DragScalar helper (params are double; avoids float temporaries).
                const auto drag = [](const char* label, double& v, double lo, double hi, double step, const char* fmt) {
                    return ImGui::DragScalar(label, ImGuiDataType_Double, &v, static_cast<float>(step), &lo, &hi, fmt, kFlags);
                };

                auto& opt = _est().options();

                ImGui::Checkbox("Enable smoothing", &opt.enable_smoothing);

                // Kernel selector
                const char* const kernel_kinds[] = { "One Euro" };
                int curr_kind = static_cast<int>(opt.filter.kind);
                if (ImGui::Combo("Kernel", &curr_kind, kernel_kinds, IM_ARRAYSIZE(kernel_kinds))) {
                    opt.filter.kind = static_cast<pose::rotation_filter_kind>(curr_kind);
                }

                if (opt.filter.kind == pose::rotation_filter_kind::one_euro)
                {
                    ImGui::BeginDisabled(!opt.enable_smoothing);
                    auto& oe = opt.filter.one_euro;
                    drag("Min cutoff [Hz]", oe.min_cutoff_hz, 0.01, 10.0, 0.01, "%.2f");
                    drag("Beta", oe.beta, 0.0, 1.0, 0.001, "%.3f");
                    drag("Deriv cutoff [Hz]", oe.dcutoff_hz, 0.01, 10.0, 0.01, "%.2f");
                    ImGui::EndDisabled();
                }

                // Occlusion hold (independent of the smoothing on/off switch).
                double hold_ms = opt.max_hold.count();
                if (drag("Max hold [ms]", hold_ms, 0.0, 1000.0, 1.0, "%.0f")) { opt.max_hold = pose::millis_f64{ hold_ms }; }
                double reset_ms = opt.reset_gap.count();
                if (drag("Reset gap [ms]", reset_ms, 0.0, 2000.0, 1.0, "%.0f")) { opt.reset_gap = pose::millis_f64{ reset_ms }; }
            }

            // ----- Leg-hinge constraint options -----
            ImGui::SeparatorText("Hinge Constraint");
            {
                auto& opt = _est().options();
                ImGui::Checkbox("Enable hinge constraint", &opt.enable_hinge_constraint);
                ImGui::SetItemTooltip("Reject the planar-ambiguity flip and constrain each joint to its\n"
                                      "1-DOF hinge axis. Needs a captured rest pose.");
            }
        }
    }

    // Axis-frame sync/reset via the internal ImPlot3DPlot (implot3d has no public links).
    // Called inside each BeginPlot/EndPlot, after SetupAxesLimits.
    void debug_gui_app::_sync_axis_frame()
    {
        ImPlot3DPlot* plot = ImPlot3D::GetCurrentPlot();
        if (!plot) { return; }

        // One-shot reset: return to the home rotation and default ranges.
        if (_reset_plots)
        {
            plot->Rotation = plot->InitialRotation;
            for (int a = 0; a < 3; ++a) { plot->Axes[a].SetRange(-1.2, 1.2); }
        }

        if (!_ui.sync_plots) { return; } // subplots independent

        // The hovered/held plot (unless locked) is the master; the first sync frame and any reset
        // (re)seed the shared reference from it. Everyone else follows the shared reference.
        const bool master = !_sync_init || _reset_plots
            || (!_ui.lock_plots && (plot->Hovered || plot->Held));
        if (master)
        {
            _sync_rot[0] = plot->Rotation.x; _sync_rot[1] = plot->Rotation.y;
            _sync_rot[2] = plot->Rotation.z; _sync_rot[3] = plot->Rotation.w;
            for (int a = 0; a < 3; ++a)
            {
                _sync_range[a][0] = plot->Axes[a].Range.Min;
                _sync_range[a][1] = plot->Axes[a].Range.Max;
            }
            _sync_init = true;
        }
        else
        {
            plot->Rotation = ImPlot3DQuat{ _sync_rot[0], _sync_rot[1], _sync_rot[2], _sync_rot[3] };
            for (int a = 0; a < 3; ++a) { plot->Axes[a].SetRange(_sync_range[a][0], _sync_range[a][1]); }
        }
    }

    void debug_gui_app::_render_plot_panel()
    {
        const int n = static_cast<int>(pose::kNumJoints);
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        const ImVec2 avail = ImGui::GetContentRegionAvail();

        int cols = 1;
        float cell_sz = 1.0f;
        if (_ui.autosize_plots)
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
            cell_sz = _ui.plot_size_px * this->renderer().dpi_scale(); // DPI-aware px
            cols = std::max(1, static_cast<int>((avail.x + spacing) / (cell_sz + spacing)));
        }

        const ImVec2 plot_sz{ cell_sz, cell_sz };
        const char* order = kEulerOrders[_ui.euler_order].name;

        // Lines: x always auto-scrolls; Lock (or a one-shot reset) forces the default y range,
        // otherwise y is mouse-adjustable. Axis frame: rotation/range sync handled in _sync_axis_frame().
        const ImPlotCond y_cond = (_ui.lock_plots || _reset_plots) ? ImPlotCond_Always : ImPlotCond_Once;

        int col = 0;
        int ji = 0;
        for (const auto& info : pose::kJointsInfo)
        {
            const auto& st = _est().get_joint_state(info.id);
            const char* ref = (!_ui.relative_rot || pose::is_root_joint(info.id))
                ? "camera" : pose::joint_info(info.parent).name.data();
            const auto rot = try_get_joint_rot(st, _ui.relative_rot);
            const std::optional<Eigen::Vector3d> e = rot.has_value()
                ? std::optional{ to_euler_deg(rot.value(), kEulerOrders[_ui.euler_order]) }
                : std::nullopt;

            // Second title line: euler for the euler-line plot, else the quaternion.
            std::string readout;
            if (_ui.plot_type == plot_type_t::euler_line)
            {
                readout = e.has_value()
                    ? std::format("Euler{}: {:.1f}, {:.1f}, {:.1f}", order, e->x(), e->y(), e->z())
                    : std::format("Euler{}: -", order);
            }
            else
            {
                readout = rot.has_value()
                    ? std::format("Q: {:.3f}, {:.3f}, {:.3f}, {:.3f}", rot->x(), rot->y(), rot->z(), rot->w())
                    : "Q: -";
            }
            // `###name` = stable id so the per-frame readout doesn't reset the plot's zoom/rotation.
            const std::string title = std::format("{} (ref: {})\n{}###{}", info.name, ref, readout, info.name);

            if (col != 0) { ImGui::SameLine(); }
            // Scope every id in this subplot (plot, legend, context menus) so implot3d/implot
            // items don't clash across subplots. Lets the plot items keep short, plain labels.
            ImGui::PushID(ji);
            ImGui::BeginGroup();

            if (_ui.plot_type == plot_type_t::axis_frame)
            {
                // Axis-frame view: RGB triad. Limits fixed; Lock/Sync/Reset act on the view rotation.
                ImPlot3DFlags f3d = ImPlot3DFlags_Equal | ImPlot3DFlags_NoClip;
                if (_ui.lock_plots) { f3d |= ImPlot3DFlags_NoRotate | ImPlot3DFlags_NoPan | ImPlot3DFlags_NoZoom; }
                if (ImPlot3D::BeginPlot(title.c_str(), plot_sz, f3d))
                {
                    ImPlot3D::SetupAxesLimits(-1.2, 1.2, -1.2, 1.2, -1.2, 1.2, ImPlot3DCond_Once);
                    ImPlot3D::SetupLegend(ImPlot3DLocation_West);
                    this->_sync_axis_frame(); // read-back rotation/range sync across subplots + reset
                    if (rot.has_value()) { draw_axes(rot.value(), 3.0f); }
                    ImPlot3D::EndPlot();
                }
            }
            else if (_ui.plot_type == plot_type_t::euler_line)
            {
                // Rolling history of the three euler angles (matches the triad colors).
                const ImVec4 col[3]{ { 1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 0, 0, 1, 1 } };
                const char* const nm[3]{ "X", "Y", "Z" };
                draw_lines(title.c_str(), _euler_bufs.view(ji), kWindowSec, kEulerYLo, kEulerYHi,
                    y_cond, _ui.sync_plots, _sync_y, col, nm, plot_sz);
            }
            else
            {
                // Rolling history of the quaternion components.
                const ImVec4 col[4]{ { 1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 0, 0, 1, 1 }, { 0.85f, 0.85f, 0.2f, 1 } };
                const char* const nm[4]{ "X", "Y", "Z", "W" };
                draw_lines(title.c_str(), _quat_bufs.view(ji), kWindowSec, kQuatYLo, kQuatYHi,
                    y_cond, _ui.sync_plots, _sync_y, col, nm, plot_sz);
            }

            ImGui::EndGroup();
            ImGui::PopID();

            if (++col >= cols) { col = 0; }
            ++ji;
        }

        _reset_plots = false; // one-shot: the ranges were forced this frame
    }

    float debug_gui_app::_log_split_height()
    {
        const float avail_y = ImGui::GetContentRegionAvail().y;
        const float max_log = std::max(kLogMinH, avail_y - kSplitHit - kLogMinH);
        _ui.log_h = std::clamp(_ui.log_h, kLogMinH, max_log);
        return avail_y - _ui.log_h - kSplitHit;
    }

    void debug_gui_app::_render_log_panel()
    {
        // Starting a new line after the content row already advanced the cursor by one
        // ItemSpacing.y; undo it so the grip sits flush against the row. Without this the
        // vertical gap would be ItemSpacing.y wider than the (SameLine-flush) side splitter.
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() - ImGui::GetStyle().ItemSpacing.y);

        // Horizontal resize grip (no visible line): zero spacing keeps it flush to both
        // panes, so the inter-panel gap matches the vertical splitter's width and the
        // whole gap is grabbable. Drag up to grow the panel, down to shrink it.
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
        ImGui::InvisibleButton("##log_split", ImVec2(-1.0f, kSplitHit));
        if (ImGui::IsItemActive()) { _ui.log_h -= ImGui::GetIO().MouseDelta.y; }
        if (ImGui::IsItemHovered() || ImGui::IsItemActive()) { ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS); }

        ImGui::BeginChild("logpanel", ImVec2(0.0f, _ui.log_h), ImGuiChildFlags_Borders);
        ImGui::PopStyleVar(); // restore spacing for the console's own contents
        _log_console.draw();
        ImGui::EndChild();
    }

    void debug_gui_app::_render_open_dialog()
    {
        if (!_ui.show_open) { return; }
        ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_Appearing);
        if (ImGui::Begin("Open Source", &_ui.show_open, ImGuiWindowFlags_NoCollapse))
        {
            const auto kind_radio = [this](const char* label, source_kind_t val) {
                if (ImGui::RadioButton(label, _ui.open_kind == val)) { _ui.open_kind = val; }
            };
            kind_radio("Device", source_kind_t::device);
            ImGui::SameLine();
            kind_radio("Recording", source_kind_t::recording);
            ImGui::Separator();

            if (_ui.open_kind == source_kind_t::device)
            {
                ImGui::InputInt("Device index", &_ui.device);
                if (_ui.device < 0) { _ui.device = 0; }
                ImGui::Checkbox("Manual exposure [us]", &_ui.manual_exposure);
                if (_ui.manual_exposure)
                {
                    ImGui::SameLine(); ImGui::SetNextItemWidth(120);
                    ImGui::InputInt("##exposure", &_ui.exposure);
                }
                ImGui::Checkbox("Manual gain", &_ui.manual_gain);
                if (_ui.manual_gain)
                {
                    ImGui::SameLine(); ImGui::SetNextItemWidth(120);
                    ImGui::InputInt("##gain", &_ui.gain);
                }
            }
            else
            {
                if (ImGui::Button("Browse...")) { _file_dialog.Open(); }
                ImGui::SameLine();
                ImGui::TextUnformatted(_ui.recording.empty() ? "(no file selected)" : _ui.recording.c_str());
            }

            ImGui::Separator();
            if (ImGui::Button("Open", ImVec2(90, 0))) { this->_do_open_source(); }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(90, 0))) { _ui.show_open = false; }
        }
        ImGui::End();
    }

} // namespace gui
