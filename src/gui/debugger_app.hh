#pragma once
#include "app_base.hh"
#include "app_renderer_sdl3.hh"
#include "frame_texture.hh"
#include "log_console.hh"
#include "pose_trace_recorder.hh"
#include "app_config.hh"

#include "hw/timestamp.hh"
#include "io/recording_writer.hh"
#include "pose/marker_tracker.hh"
#include "pose/frontal_pose_estimator.hh"
#include "pose/sagittal_pose_estimator.hh"
#include "pose/view_plane.hh"
#include "pose_plot_panel.hh"
#include "open_source_dialog.hh"

#include <imfilebrowser.h>
#include <imgui.h>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace net { class exo_pose_server; }

namespace gui
{
    // What the camera window is open for, and what it dispatches on. One canvas and one mouse, so
    // one at a time: each tool takes the whole frame, and two of them live would fight over every
    // drag.
    enum class view_tool_t
    {
        none,         // the window is closed
        color_sample, // dragging feeds the pixels under the cursor to the colour sampler
        roi_edit,     // the pending ROI is drawn over the frame and dragged into place
    };

    // Debugger GUI for the pose server: starts/stops the WebSocket listener and drives the pose
    // pipeline (source open/close, rest-pose calibration) while visualizing each stream's annotated
    // frame and the per-joint 3D positions / reconstructed skeleton. Owns the server; the listener
    // starts stopped.
    class debugger_app final : public app_base<app_renderer_sdl3>
    {
    public:
        explicit debugger_app(const app::app_config_t& config);
        ~debugger_app();

        int run(); // create window, loop, destroy

    public:
        void render_ui() override;

    private:
        // Opens the pipeline on the config as it stands and restarts what described the last source.
        void _open_source();
        void _do_close_source();

        // 열려 있던 소스를 설명하던 것들(프레임, 플롯, 트레이스, 색 샘플, 고른 스트림)을 버린다.
        void _clear_session_state();

        // Reads `path` over the settings this window edits, discarding what the control panel has
        // been tuned to since the last save. It fills the open dialog and reopens an open source,
        // so what streams is what the settings say. A file that fails to load changes nothing.
        void _do_load_config(const std::filesystem::path& path);
        void _do_save_config(const std::filesystem::path& path);

        void _do_start_recording();
        void _do_stop_recording();
        void _update_pose_frame();
        void _dump_pose_trace(); // write the trace ring to a timestamped .json under dumps/

        void _render_menu_bar();
        void _render_control_panel();

        // The ROI in force, and the button that opens the camera window to place a new one.
        void _render_roi_control();

        // `selected_stream_idx` 콤보. 스트림이 하나면 그리지 않는다. `id` 는 한 창 안의 둘을 가른다.
        void _render_stream_selector(const char* id);

        // 고른 스트림에 매달린 상태를 그 스트림의 것으로 다시 맞춘다. 색 샘플은 그 카메라의 픽셀이고,
        // ROI 사각형은 그 프레임의 좌표이며, 트레이스의 태그는 그 트래커가 찾은 것이다.
        void _on_selected_stream_changed();

        // 편집할 사각형을 고른 스트림에 걸린 ROI(없으면 전체 프레임)로 놓는다.
        void _seed_roi_edit_rect();

        // 스트림마다의 어노테이트 프레임을 `area` 에 나란히 그린다. `area.y` 가 0 이면 높이는 프레임
        // 비율에서 잡는다.
        void _render_stream_views(const ImVec2& area);

        // Estimator tuning, one function per viewing plane.
        void _render_frontal_estimator_control(pose::frontal_pose_estimator::options_t& opt);
        void _render_sagittal_estimator_control(pose::sagittal_pose_estimator::options_t& opt);

        // Blob filters, joint assignment, and what the last frame found.
        void _render_color_marker_control(pose::color_marker_tracker& tracker);

        // The colour in force, and the button that opens the camera window to measure a new one.
        // Nothing here saves: a fit lands on the running tracker and reaches the file through the
        // config profile, like every other control on this panel.
        void _render_color_model_section(pose::color_marker_tracker& tracker);

        // The one place a drag lands on the frame: the panel's preview and the fullscreen view
        // only display. Renders whichever tool it was opened with.
        void _render_camera_window();
        void _render_color_sample_tools(pose::color_marker_tracker& tracker);
        bool _render_roi_tools(); // false once there is no source left to place an ROI in

        // Feeds the pixels under the cursor to the colour sampler, over the camera view drawn
        // between `img_min` and `img_max`.
        void _handle_color_sample_click(const ImVec2& img_min, const ImVec2& img_max);

        // The pending ROI over that same view: the interior moves it, an edge or a corner resizes
        // it, and the overlay dims what it would cut away. The view shows the ROI in force while
        // the pending one is in full-frame pixels, so `_shown_window()` rebases between them.
        void _handle_roi_interaction(const ImVec2& img_min, const ImVec2& img_max);
        void _draw_roi_overlay(const ImVec2& img_min, const ImVec2& img_max);
        hw::roi_t _shown_window() const; // the part of the full frame the camera view covers

        // Fits the collected samples and installs the result on the running tracker, so the next
        // frame is classified by what was just measured.
        void _do_fit_color_model(pose::color_marker_tracker& tracker);
        void _render_recording_status(); // live counters while a recording is being written

        void _render_record_dialog();
        void _render_log_panel();  // bottom dock: resize grip + log console child
        float _log_split_height(); // clamps `_ui.log_panel_height`; returns the main content height above the panel

    private:
        // gui control states
        struct ui_state_t
        {
            // window layout (splitter-adjustable panels)
            float side_panel_width{ 460.0f };  // right control panel width [px]
            bool show_log{ false };            // spdlog output console: bottom dock panel
            float log_panel_height{ 200.0f };  // bottom log panel height [px]

            // view / visualization
            bool camera_fullscreen{ false };

            // 도구가 작용하는 스트림. 열 때 0 이 되고 스트림 수 안으로 묶인다.
            std::size_t selected_stream_idx{ 0 };

            // What the camera window is open for; `none` is what closed means.
            view_tool_t view_tool{ view_tool_t::none };

            // colour sampling, driven from the camera window's sampler tool
            int color_sample_radius{ 6 }; // pixels collected around each click [px]
            double color_max_distance{ 3.0 }; // how far into the fitted ellipse still counts
            int color_backdrop{ 0 };      // 0 camera, 1 mask, 2 membership score

            // ROI being composed, in full-frame pixels. Live only while `roi_edit` is the tool:
            // entering seeds them from what the source took, and Apply or Cancel leaves.
            int roi_size[2]{ 0, 0 };
            int roi_offset[2]{ 0, 0 };

            // record dialog
            bool record_dlg_show{ false };
            int record_dlg_codec{ 0 }; // index into `io::kImageCodecs`
            int record_dlg_jpeg_quality{ 90 };
            std::string record_dlg_path;

            // diagnostic pose trace (rolling ring dumped to JSON on demand)
            bool trace_enabled{ true }; // capture each pose frame into the ring
            int trace_capacity{ 600 }; // ring length [frames]
        };

        // Owns the installation settings this window edits; `_server->config()` is the one copy,
        // so a client command and this window open a source with the same thing.
        std::unique_ptr<net::exo_pose_server> _server;

        ImGui::FileBrowser _recording_save_browser{ ImGuiFileBrowserFlags_EnterNewFilename | ImGuiFileBrowserFlags_CreateNewDir };
        ImGui::FileBrowser _config_save_browser{ ImGuiFileBrowserFlags_EnterNewFilename | ImGuiFileBrowserFlags_CreateNewDir };
        log_console _log_console;

        // Per stream: the last annotated frame pulled from the pipeline, the same capture undrawn, and
        // its frame id. The undrawn one is what a colour sample is taken from.
        struct stream_view_t
        {
            std::unique_ptr<frame_texture> texture;
            cv::Mat annotated;
            cv::Mat source; // the same capture undrawn, for anything reading original pixels
            uint64_t last_frame_id{ 0 };
        };
        std::vector<stream_view_t> _stream_views;

        // 지난 프레임에 소스가 열려 있었는지. 이 창을 거치지 않은 close 를 알아채는 자리다.
        bool _source_open{ false };

        // 트레이스 링과 플롯에 마지막으로 넣은 포즈의 시각. 한 순간이 한 번만 들어가게 한다.
        hw::timestamp_t _last_plotted_ts{};

        // Samples this installation's colour, accumulated across clicks and frames. What it holds
        // is a measurement in progress, discarded whenever the operator says so and no part of
        // detecting anything, so it belongs to the panel that collects it.
        pose::color_sampler _color_sampler;

        pose_trace_recorder _trace; // rolling per-frame diagnostic trace (dumped to JSON on demand)

        // The frame geometry the trace ring and the plot buffers were filled under. Both are read
        // in image coordinates one way or another, so a move invalidates what they already hold.
        std::vector<std::optional<hw::roi_t>> _history_rois;

        pose_plot_panel _plot_panel; // left pane: the joint-state views and their own controls
        open_source_dialog _open_dialog;

        ui_state_t _ui;
    };

} // namespace gui
