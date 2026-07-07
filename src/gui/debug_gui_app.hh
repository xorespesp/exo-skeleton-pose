#pragma once
#include "app_base.hh"
#include "app_renderer_sdl3.hh"
#include "frame_texture.hh"
#include "cli_options.hh"

#include "hw/sensor_frame_provider.hh"
#include "hw/sensor_frame_observer.hh"
#include "pose/tag_detector.hh"
#include "pose/pose_estimator.hh"

#include <imfilebrowser.h>
#include <opencv2/core.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace gui
{
    // Worker-thread tag detection; latches the annotated image + detections for the UI.
    class debug_gui_observer final : public hw::sensor_frame_observer
    {
    public:
        debug_gui_observer(const hw::sensor_frame_provider& provider, double tag_size_m)
            : _provider{ provider }, _tag_size_m{ tag_size_m }
        { }

        bool try_get(cv::Mat& out_img, std::vector<pose::tag_detection_t>& out_dets, uint64_t& last_seq)
        {
            std::scoped_lock lk{ _mtx };
            if (_seq == last_seq || _latest.empty()) { return false; }
            out_img = _latest;
            out_dets = _detections;
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
        uint64_t _seq{ 0 };
    };

    // Debug GUI: live tag detection + per-joint pose visualization (imgui/implot3d).
    class debug_gui_app final : public app_base<app_renderer_sdl3>
    {
    public:
        explicit debug_gui_app(const app::source_options& opt);

        int run(); // create window, open the initial source, loop, destroy

    public:
        void render_ui() override;

    private:
        void _open_device(uint32_t index);
        void _open_recording(const std::string& path);

        void _do_open_source();
        void _do_close_source();
        void _poll_observer();

        void _render_menu_bar();
        void _render_control_panel();
        void _render_plot_panel();
        void _render_angle_table();
        void _render_open_dialog();

        // Rotation shown per joint: local (relative to parent) or absolute (camera frame).
        std::optional<Eigen::Quaterniond> _display_rot(const pose::joint_state_t& st) const;

    private:
        app::source_options _opt;

        std::shared_ptr<hw::sensor_frame_provider> _provider;
        std::shared_ptr<debug_gui_observer> _observer;
        pose::pose_estimator _estimator;

        std::optional<frame_texture> _texture;
        ImGui::FileBrowser _file_dialog;

        cv::Mat _frame;
        std::vector<pose::tag_detection_t> _detections;
        uint64_t _last_seq{ 0 };

        bool _camera_fullscreen{ false };
        bool _relative_rot{ true }; // true = local (vs parent), false = absolute (camera frame)
        int _euler_order{ 0 }; // index into kEulerOrders for the angle-table readout
        bool _subplot_autofit{ true }; // pack subplots to fill the panel
        float _subplot_size{ 150.0f }; // manual subplot cell size [px], DPI-scaled at use

        // Open-source dialog state.
        bool _show_open{ false };
        int _open_kind{ 0 }; // 0 = device, 1 = recording
        int _dlg_device{ 0 };
        bool _dlg_manual_exposure{ false };
        int _dlg_exposure{ 8000 };
        bool _dlg_manual_gain{ false };
        int _dlg_gain{ 0 };
        std::string _dlg_recording;
    };

} // namespace gui
