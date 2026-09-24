#pragma once
#include "source_address.hh"

#include "hw/frame_format.hh"
#include "hw/roi.hh"
#include "pose/color_marker_detector.hh"
#include "pose/frontal_pose_estimator.hh"
#include "pose/sagittal_pose_estimator.hh"
#include "pose/tag_detector.hh"
#include "pose/view_plane.hh"
#include "utils/serializable.hh"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace app
{
    // An installation's own settings, persisted as JSON. Fixed for one physical setup, hence off
    // the wire; what an operator adjusts during a run (the rest pose) is session state and stays out.

    // The sensor: which one to open, where it views the exo from, and what gets programmed into it.
    // `cameras[i]` is stream i; a playback names the same file in every entry, entry i being its stream i.
    struct camera_config_t
    {
        source_address source;
        pose::camera_view_t view{ pose::camera_view_t::frontal }; // 이 카메라가 장비를 보는 자리

        std::optional<int32_t> exposure_us; // nullopt: auto
        std::optional<int32_t> gain;        // nullopt: auto
        std::optional<hw::roi_t> roi;       // nullopt: whole frames

        // Calibration measured off-line, for a camera reporting none of its own.
        // Empty leaves tag poses unsolvable, which the 2D estimators do not mind. Only a VZ camera
        // takes one: a K4A reports its own and a recording carries the one it was shot with.
        std::string intrinsics_file;

        // 카메라가 찍는 속도. 같은 순간으로 묶이는 카메라들은 같은 값으로 고정한다. nullopt: 제한 없음
        // (VZ 는 자기 상한으로 프리런). K4A 는 SDK 가 30 으로 정하므로 비었거나 30 이어야 한다.
        std::optional<double> frame_rate_fps;

        DECLARE_SERIALIZABLE_FIELDS(
            v("source",          o.source);
            v("view",            o.view);
            v("exposure_us",     o.exposure_us);
            v("gain",            o.gain);
            v("roi",             o.roi);
            v("intrinsics_file", o.intrinsics_file);
            v("frame_rate_fps",  o.frame_rate_fps);
        )
    };

    // 캡처를 한 순간으로 묶을 때의 기준과 허용 오차.
    struct sync_config_t
    {
        uint32_t reference_stream_idx{ 0 }; // 다른 스트림들이 시각을 맞추는 기준

        // 기준 캡처와 같은 순간으로 칠 최대 간격. nullopt: 기준 스트림 간격의 절반
        std::optional<pose::millis_f64> max_pair_skew;

        DECLARE_SERIALIZABLE_FIELDS(
            v("reference_stream_idx", o.reference_stream_idx);
            v("max_pair_skew_ms",     o.max_pair_skew);
        )
    };

    // What is printed on the exo, which decides how a frame becomes per-joint measurements.
    enum class marker_kind_t
    {
        apriltag,     // tags carry an id, so a detection names its own joint
        color_marker, // plain coloured discs, placed by their order along the limb
    };

    constexpr std::string_view marker_kind_name(marker_kind_t k) {
        return (k == marker_kind_t::color_marker) ? "color_marker" : "apriltag";
    }

    // nullopt if `name` is neither kind. The names round-trip through `marker_kind_name()`.
    constexpr std::optional<marker_kind_t> marker_kind_from_name(std::string_view name) {
        if (name == marker_kind_name(marker_kind_t::apriltag))     { return marker_kind_t::apriltag; }
        if (name == marker_kind_name(marker_kind_t::color_marker)) { return marker_kind_t::color_marker; }
        return std::nullopt;
    }

    // What a source has to deliver for `k` to be detectable, which is what an open programs into
    // the camera. Colour classification needs the colour; reading a printed pattern takes luminance
    // alone, and asking for one channel there keeps the link bandwidth and the demosaic cost off.
    constexpr hw::frame_format_t marker_frame_format(marker_kind_t k) {
        return (k == marker_kind_t::color_marker) ? hw::frame_format_t::bgr8 : hw::frame_format_t::gray8;
    }

    // What this installation's markers photograph as, measured on site by the debugger's colour
    // panel. Written back into the profile it came from, so what a run detects with and what the
    // file says are the same thing.
    //
    // The blob filters ride with the colour because they are decided in the same sitting and by the
    // same physical facts: how many pixels a marker covers at this distance and frame size, how the
    // light glances off it, how far a swing smears it. `min_score` goes further and is stated
    // against the colour model itself, so it means something different the moment that changes.
    struct color_marker_calibration_t
    {
        pose::color_marker_detector::options_t detector; // blob filters, and the colour model inside
        pose::color_marker_assigner::options_t assigner; // 이 카메라의 원반을 관절로 이름 붙이는 법

        // What the camera delivered while this was measured, which its `cameras` entry does not
        // state: a ROI, a binned mode or another sensor all change how many pixels a marker
        // covers, and the blob gates and the search radius are counted in pixels. Compared at open.
        Eigen::Vector2i frame_resolution{ 0, 0 };

        DECLARE_SERIALIZABLE_FIELDS(
            v("detector",         o.detector);
            v("assigner",         o.assigner);
            v("frame_resolution", o.frame_resolution);
        )
    };

    // The colour-marker path.
    struct color_marker_config_t
    {
        // Per camera; null until someone has measured that one, which is the state a profile is
        // authored in. Nothing is detected without it, and the colour panel is where it comes from.
        std::vector<std::optional<color_marker_calibration_t>> calibration;

        DECLARE_SERIALIZABLE_FIELDS(
            v("calibration", o.calibration);
        )
    };

    // The AprilTag path.
    struct apriltag_config_t
    {
        // Printed black-square edge length [m], which turns a tag quad's pixel size into a metric scale.
        double tag_size_m{ 0.05 };

        pose::tag_detector::options_t detector;

        DECLARE_SERIALIZABLE_FIELDS(
            v("tag_size_m", o.tag_size_m);
            v("detector",   o.detector);
        )
    };

    // Frames -> per-joint measurements. Both marker kinds are kept, so switching loses neither's tuning.
    struct detector_config_t
    {
        // What is printed on the exo, which picks the detection path that runs.
        marker_kind_t kind{ marker_kind_t::apriltag };

        apriltag_config_t apriltag;
        color_marker_config_t color_marker;

        DECLARE_SERIALIZABLE_FIELDS(
            v("kind",         o.kind);
            v("apriltag",     o.apriltag);
            v("color_marker", o.color_marker);
        )
    };

    // Measurements -> joint angles. Both planes are kept, so switching loses neither's tuning.
    // Which one runs follows from the cameras' `view`.
    struct estimator_config_t
    {
        pose::frontal_pose_estimator::options_t frontal;
        pose::sagittal_pose_estimator::options_t sagittal;

        DECLARE_SERIALIZABLE_FIELDS(
            v("frontal",    o.frontal);
            v("sagittal",   o.sagittal);
        )
    };

    // The two halves of the pose pipeline: what finds the markers, and what solves the angles.
    struct pose_config_t
    {
        detector_config_t detector;
        estimator_config_t estimator;

        DECLARE_SERIALIZABLE_FIELDS(
            v("detector",  o.detector);
            v("estimator", o.estimator);
        )
    };

    struct server_config_t
    {
        uint16_t port{ 9002 };

        DECLARE_SERIALIZABLE_FIELDS(
            v("port", o.port);
        )
    };

    struct app_config_t
    {
        // The current config schema version. (YYMMDDRR)
        // It has to be updated whenever any field list nested below this one changes.
        static constexpr int config_version = 26091800;

        server_config_t server;

        // 스트림 순서대로. 비어 있으면 자동으로 열 것이 없다. frontal 은 하나, sagittal 은 왼쪽·오른쪽 하나씩.
        std::vector<camera_config_t> cameras;

        // 라이브 카메라가 둘 이상일 때만 있다. 하나뿐이면 묶을 것이 없고 녹화는 파일이 이미 한 시간축이다.
        std::optional<sync_config_t> sync;

        pose_config_t pose;

        DECLARE_SERIALIZABLE_FIELDS(
            // NOTE: `config_version` MUST be named first, so that a file written for another
            //       schema says so before any key of that schema's shape is read.
            v("config_version", o.config_version);
            v("server",         o.server);
            v("cameras",        o.cameras);
            v("sync",           o.sync);
            v("pose",           o.pose);
        )
    };

    // 엔트리들이 합의한 추정 평면. 모든 엔트리의 `view` 가 같은 평면일 때 그 값이고, 엔트리가 없거나
    // 어긋나면 nullopt. `validate_config()` 를 통과한 config 에서는 엔트리가 있는 한 값이다.
    [[nodiscard]] std::optional<pose::view_plane_t> view_plane_of(std::span<const camera_config_t> cameras);

    // Where the app keeps its own output (`recordings`, `dumps`, `configs`):
    // the nearest such folder at or above the executable, else the executable's own directory.
    // Not config-relative, since these hold what the tool produces, not what an installation names.
    [[nodiscard]] std::filesystem::path project_dir(std::string_view name);

    // What the types alone cannot say: is this a usable installation. A read runs it, and so does
    // an open, so a config assembled in memory is held to what a file is held to.
    [[nodiscard]] bool validate_config(const app_config_t& config, std::string& err);

    // Every key the schema names must be present; a default standing in for an omitted one is the
    // failure this guards against. Keys it does not name are ignored.
    // On failure `err` names the key and `out` is untouched.
    [[nodiscard]] bool load_config(const std::filesystem::path& path, app_config_t& out, std::string& err);

    // Written from the schema, so the file that comes out holds the keys it names and nothing else.
    [[nodiscard]] bool save_config(const app_config_t& config, const std::filesystem::path& path, std::string& err);

    // The JSON text `save_config()` would write. (--dump-config)
    [[nodiscard]] std::string dump_config(const app_config_t& config);

    // Where `--config <name>` points: a bare filename comes from `project_dir("configs")`, the
    // folder a save writes to; a name carrying a folder is that path. Empty when nothing is there.
    [[nodiscard]] std::filesystem::path find_config_file(std::string_view name);

} // namespace app
