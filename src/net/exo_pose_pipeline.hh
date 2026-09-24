#pragma once
#include "app_config.hh"

#include "hw/calibration.hh"
#include "hw/roi.hh"
#include "hw/sensor_frame_provider.hh"
#include "hw/sensor_backend.hh"
#include "io/frame_recorder.hh"
#include "pose/frontal_pose_estimator.hh"
#include "pose/marker_tracker.hh"
#include "pose/sagittal_pose_estimator.hh"
#include "pose/pose_estimator_base.hh"
#include "pose/synced_tracker.hh"
#include "pose/view_plane.hh"

#include <opencv2/core.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace net
{
    // forward declaration of the worker-thread frameset observer
    class pose_frame_observer;

    // The pose pipeline: owns the source, the frameset tracker, and the joint estimator, and
    // steps them independently of any network transport. A server or GUI holds one and drives
    // it via poll(). Not thread-safe: call from a single thread (the loop/GUI thread). Only the
    // provider's worker crosses threads: it hands framesets to the frameset tracker, which publishes
    // measurements and drawn frames under its own lock.
    class exo_pose_pipeline final
    {
    public:
        explicit exo_pose_pipeline(bool annotate_frames);
        ~exo_pose_pipeline();

        exo_pose_pipeline(const exo_pose_pipeline&) = delete;
        exo_pose_pipeline& operator=(const exo_pose_pipeline&) = delete;

        // --- source control -----------------------------------------------------------
        bool open_source(const app::app_config_t& config);
        void close_source();

        bool is_source_open() const;
        bool is_playback_source() const; // the open source is a recording file, not a live camera

        // 열린 소스의 스트림 수. 소스가 없으면 0.
        std::size_t stream_count() const;

        // --- recording ----------------------------------------------------------------
        // Captures the live source's frames to a recording file.
        // (Refused without an open live source)
        bool start_recording(const std::filesystem::path& path, const io::recording_options_t& options);
        void stop_recording(); // drains what is queued, then finalizes the file

        bool is_recording() const;
        io::recording_stats_t recording_stats() const;
        std::filesystem::path recording_path() const;

        // --- rest pose ----------------------------------------------------------------
        // All three are no-ops / false until a source has been opened, since the estimator holding
        // the reference is created then. Closing a source leaves the estimator in place, so a
        // captured rest survives until the next open replaces it.
        bool has_rest_pose() const;
        bool calibrate_rest_pose();
        void clear_rest_pose();

        // --- estimator ----------------------------------------------------------------
        // The active estimator, null until the first source is opened (the viewing plane it is built
        // for comes from that source). It outlives close_source(), so the last frame's joint state
        // stays readable. Enough for anything that only reads that state.
        pose::pose_estimator_base* estimator();
        const pose::pose_estimator_base* estimator() const;

        // Which plane the active estimator solves in; meaningful only once one exists.
        pose::view_plane_t view_plane() const { return _view_plane; }

        // Live tuning of the active estimator. At most one answers, matching the plane, so a caller
        // renders the right controls without testing the plane itself. Read a copy, edit it, hand
        // it back: every open builds a fresh estimator, so no handle into one escapes. A set for
        // the idle plane is ignored.
        std::optional<pose::frontal_pose_estimator::options_t> frontal_options() const;
        void set_frontal_options(const pose::frontal_pose_estimator::options_t& opt);

        std::optional<pose::sagittal_pose_estimator::options_t> sagittal_options() const;
        void set_sagittal_options(const pose::sagittal_pose_estimator::options_t& opt);

        // The sagittal estimator's readouts (joint angles); null in a frontal run.
        const pose::sagittal_pose_estimator* sagittal_estimator() const;

        // --- marker tracking ----------------------------------------------------------
        // A stream's tracker owns everything that differs between marker technologies. It stands
        // for exactly as long as the source it reads, so it is null whenever none is open.
        //
        // A caller wanting one technology's own controls asks for that type:
        //
        //   if (auto* t = dynamic_cast<pose::apriltag_tracker*>(pipe.tracker(stream_idx))) { ... }
        //
        // which reads null both when nothing is open and when another technology is running. The
        // running tracker's type is the one record of which is which, so adding a technology leaves
        // this class alone.
        pose::marker_tracker_base* tracker(std::size_t stream_idx);
        const pose::marker_tracker_base* tracker(std::size_t stream_idx) const;

        // --- stepping -----------------------------------------------------------------
        // Advance one step: pull the newest published measurements and recompute joint states.
        // Each flag reports something that happened this step and is cleared once returned.
        struct poll_result_t {
            bool new_pose{ false };
            std::optional<hw::stream_end_reason_t> stream_end_reason; // set when the stream ended
            bool status_changed{ false };
        };
        poll_result_t poll();

        // Latest annotated frame for display, with the source frame it was drawn over; false if
        // nothing new since `last_frame_id`. The detections drawn on it are the tracker's to hand out,
        // since only it knows their shape.
        //
        // The two describe the same capture, so a point picked off the drawn image names the same
        // pixel in the source. Sampling a colour reads that source: the overlay would otherwise
        // contribute its own pixels to what a marker is measured to be.
        bool try_get_annotated_frame(std::size_t stream_idx, cv::Mat& out_img, cv::Mat& out_source, uint64_t& last_frame_id);

        // --- source metadata ----------------------------------------------------------
        // 스트림을 묻는 접근자는 소스가 열려 있을 때만 답이 있다. 없음을 담을 수 있는 것은 그것을 내고
        // (해상도 0, 빈 ROI), 그럴 자리가 없는 것은 던진다.
        //
        // The sensor whose frames stream `stream_idx` reads, which on playback is the sensor the
        // recording was made with. Throws std::logic_error without an open source.
        hw::sensor_backend_t sensor_backend(std::size_t stream_idx) const;
        pose::camera_view_t camera_view(std::size_t stream_idx) const; // 그 스트림의 카메라가 장비를 보는 자리
        std::string source_name() const;
        Eigen::Vector2i source_resolution(std::size_t stream_idx) const;
        Eigen::Vector2i source_full_resolution(std::size_t stream_idx) const;
        float source_fps(std::size_t stream_idx) const;
        std::optional<hw::intrinsic_t> intrinsics(std::size_t stream_idx) const; // color intrinsics of the open source (empty if none)

        // --- ROI ----------------------------------------------------------------------

        // The window in force, which a camera may have snapped to its own increments or refused
        // outright. nullopt: whole frames.
        std::optional<hw::roi_t> effective_roi(std::size_t stream_idx) const;

        // Narrows delivered images, or restores whole frames when empty. Takes effect between
        // frames, and the estimator drops whatever it held in the old pixel frame.
        // Refused while a recording is being written, which declares each stream's frame size once.
        void set_roi(std::size_t stream_idx, const std::optional<hw::roi_t>& roi);

        // Position of the newest frame in the open source's stream; restarts on every open.
        uint32_t current_frame_seq() const;

        // Capture time of the frame the estimator last stepped on, which is the moment the joint
        // state describes. Meaningful only once `has_pose()`.
        hw::timestamp_t last_timestamp() const { return _last_timestamp; }

        // True once the estimator has stepped on a frame. Frames arriving is not the same thing:
        // a tracker can refuse every one of them, as the color one does for a gray source.
        bool has_pose() const { return _has_pose; }

        // --- recording playback (no-op without an open recording source) ---------------
        bool is_source_paused() const;
        void set_source_paused(bool paused);

        // Whether reaching the end starts the recording over. Off, playback stops there and a seek
        // is what starts it going again.
        bool is_auto_repeat_enabled() const;
        void set_auto_repeat(bool enable);

        void seek_to_begin();
        void seek_to_end();

    private:
        // Says what changed rather than what is, so a steady stream stays silent, plus a throughput
        // line on a timer. Every field here remembers the previous frame, which is why it sits apart
        // from the state the pipeline runs on.
        class frame_logger
        {
        public:
            // Markers identified or lost, and joints gaining or losing tracking, as edges.
            void log_transitions(
                const pose::synced_tracker_base& synced_tracker,
                const pose::pose_estimator_base& estimator
            );

            // One line per interval, or nothing.
            void log_throughput(
                std::size_t detections,
                const pose::pose_estimator_base& estimator,
                std::string_view stream_rates,
                bool has_rest_pose
            );

            void reset(); // called whenever the source changes

        private:
            std::vector<bool> _stream_was_tracking; // per stream: the tracker had identified its markers last frame
            std::array<bool, pose::kNumJoints> _joint_tracked{}; // per joint: had a position last frame
            std::chrono::steady_clock::time_point _since{}; // start of the current summary window
            uint32_t _frames{ 0 };     // frames polled in the window
            uint32_t _detections{ 0 }; // markers detected across those frames
        };

        // 열린 소스의 스트림 하나에 대해 config 가 말한 것. 인덱스 = stream_idx.
        struct stream_settings_t
        {
            pose::camera_view_t view{ pose::camera_view_t::frontal };
            std::optional<int32_t> exposure_us; // 재생 소스는 비어 있다
            std::optional<int32_t> gain;
        };

        // Build the estimator for `view_plane` and point _active at it. Every open builds a fresh one.
        void _build_estimator(pose::view_plane_t view_plane);

        // 열려 있는 스트림 중 하나라도 `cameras` 가 지목한 카메라인지. 시리얼로 대조한다.
        bool _holds_any_camera_of(std::span<const app::camera_config_t> cameras) const;

        // frameset 트래커가 발행한 가장 새 묶음을 꺼내고, 그 사이 온 스트림 사건을 처리한 뒤 추정기를 한 번
        // 밟는다. 밟았으면 true 이고, 그 묶음의 검출 수를 `detections_this_frame` 에 더한다.
        template <typename Measurement, typename Estimator>
        bool _take_and_step(
            pose::synced_tracker<Measurement>& synced_tracker,
            Estimator& estimator,
            std::size_t& detections_this_frame
        );

        // provider 워커가 올린 스트림 점프와 기하 변경을 처리한다. 점프였으면 true.
        bool _consume_source_signals();

    private:
        bool _annotate_frames; // the frameset tracker keeps an annotated frame for a monitor GUI

        std::shared_ptr<hw::sensor_frame_provider> _provider;
        std::shared_ptr<pose_frame_observer> _observer;

        // Shared with the observer, which calls into it from the provider's worker thread.
        std::shared_ptr<pose::synced_tracker_base> _synced_tracker;

        // 같은 `synced_tracker` 를 그 측정치 타입으로. 열린 평면의 것 하나만 있다.
        std::shared_ptr<pose::synced_tracker<pose::joint_3d_measurement_t>> _frontal_synced_tracker;
        std::shared_ptr<pose::synced_tracker<pose::joint_2d_measurement_t>> _sagittal_synced_tracker;

        // 스스로 멎을 수 있어 non-null 이 곧 녹화 중은 아니다. 그것은 `is_recording()` 이 답한다.
        std::shared_ptr<io::frame_recorder> _recorder;

        // At most one estimator is engaged at a time and _active points at it, so the readers of
        // joint state never learn which plane produced it. Both are empty until a source is opened.
        pose::view_plane_t _view_plane{ pose::view_plane_t::frontal };
        std::optional<pose::frontal_pose_estimator> _frontal;
        std::optional<pose::sagittal_pose_estimator> _sagittal;
        pose::pose_estimator_base* _active{ nullptr };
        hw::timestamp_t _last_timestamp{}; // capture time of the frame the estimator last stepped on
        bool _has_pose{ false };           // that frame exists; cleared when a new source is opened
        bool _is_playback_source{ false }; // the open source is a recording file (vs a live camera)
        bool _status_changed{ false }; // a source/rest command changed the reported status; consumed by poll()

        std::vector<stream_settings_t> _streams; // 열린 소스의 스트림마다. 닫으면 비운다

        frame_logger _frame_log;
    };

} // namespace net
