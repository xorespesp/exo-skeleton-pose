#pragma once
#include "app_base.hh"
#include "app_renderer_sdl3.hh"
#include "frame_texture.hh"
#include "log_console.hh"
#include "cli_options.hh"

#include "hw/sensor_frame_provider.hh"
#include "pose/tag_detector.hh"
#include "pose/exo_pose_estimator.hh"
#include "plot_buffer.hh"

#include <imfilebrowser.h>
#include <imgui.h>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace net { class exo_pose_server; }

namespace gui
{
    // forward-declaration of worker thread tag detection observer
    class debug_gui_observer;

    enum class plot_type_t { axis_frame, euler_line, quat_line };

    enum class source_kind_t { device, recording };

    // Debugger GUI app
    class debug_gui_app final : public app_base<app_renderer_sdl3>
    {
    public:
        // `server` null: own the source. non-null: monitor/control the server's source.
        explicit debug_gui_app(
            const app::source_options& opt, 
            net::exo_pose_server* server = nullptr
        );

        int run(); // create window, open the initial source, loop, destroy

    public:
        void render_ui() override;

    private:
        // Active pipeline: the server's when monitoring, else this GUI's own.
        pose::exo_pose_estimator& _est();
        bool _is_source_recording() const;

        void _open_device(uint32_t index);
        void _open_recording(const std::string& path);

        void _do_open_source();
        void _do_close_source();
        void _poll_observer();

        void _render_menu_bar();
        void _render_control_panel();
        void _render_plot_panel();
        void _render_open_dialog();
        void _render_log_panel();  // bottom dock: resize grip + log console child
        float _log_split_height(); // clamps `_ui.log_h`; returns the main content height above the panel
        void _sync_axis_frame(); // read-back rotation/range sync + reset for the current implot3d plot

    private:
        // gui control states
        struct ui_state_t
        {
            // view / visualization
            bool  camera_fullscreen{ false };
            bool  relative_rot{ true }; // true = local (vs parent), false = global (camera frame)
            int   euler_order{ 0 };     // index into kEulerOrders for the euler readout
            plot_type_t plot_type{ plot_type_t::axis_frame };
            bool  autosize_plots{ true }; // pack subplots to fill the panel
            float plot_size_px{ 150.0f }; // manual subplot cell size [px], DPI-scaled at use
            bool  lock_plots{ false }; // true = force default ranges (live); false = mouse-adjustable
            bool  sync_plots{ true };  // share one range across all subplots
            float side_w{ 460.0f };    // right control panel width [px], splitter-adjustable
            bool  show_log{ false };   // spdlog output console: bottom dock panel
            float log_h{ 200.0f };     // bottom log panel height [px], splitter-adjustable

            // open-source dialog
            bool  show_open{ false };
            source_kind_t open_kind{ source_kind_t::device };
            int   device{ 0 };
            bool  manual_exposure{ false };
            int   exposure{ 8000 };
            bool  manual_gain{ false };
            int   gain{ 0 };
            std::string recording;
        };

        app::source_options _opt;
        net::exo_pose_server* _server{ nullptr }; // non-null -> monitor mode

        std::shared_ptr<hw::sensor_frame_provider> _provider;
        std::shared_ptr<debug_gui_observer> _observer;
        pose::exo_pose_estimator _estimator;

        std::optional<frame_texture> _texture;
        ImGui::FileBrowser _file_dialog;
        log_console _log_console;

        cv::Mat _frame;
        std::vector<pose::tag_detection_t> _detections;
        uint64_t _last_seq{ 0 };

        ui_state_t _ui;

        // scrolling plot buffer per joint (euler xyz / quat xyzw)
        plot_buffer<Eigen::Vector3f, pose::kNumJoints> _euler_bufs;
        plot_buffer<Eigen::Quaternionf, pose::kNumJoints> _quat_bufs;

        // plot limit controls (one-shot reset, shared by line + axis-frame plots)
        bool _reset_plots{ false };
        // line plots: shared y-range link vars for "sync" (x always auto-scrolls)
        double _sync_y[2]{ 0.0, 0.0 };
        // axis-frame plots: shared rotation (quat xyzw) + per-axis range, captured from the hovered plot
        double _sync_rot[4]{ 0.0, 0.0, 0.0, 1.0 };
        double _sync_range[3][2]{ { -1.2, 1.2 }, { -1.2, 1.2 }, { -1.2, 1.2 } };
        bool _sync_init{ false };
    };

} // namespace gui
