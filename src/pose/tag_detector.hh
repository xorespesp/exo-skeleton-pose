#pragma once
#include "hw/calibration.hh" // hw::intrinsic_t

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <vector>

namespace pose
{
    // One pose solution for a tag from orthogonal iteration.
    struct tag_pose_t {
        Eigen::Isometry3d transform{};     // tag -> camera
        double obj_err{ 0.0 };             // object-space error of this solution (lower = better fit)
        std::array<cv::Point2f, 4> axes{}; // pixel-projected {origin, X, Y, Z} of this solution
    };

    struct tag_detection_t {
        int id{ 0 };                          // decoded tag id
        int hamming{ 0 };                     // corrected error bits
        float decision_margin{ 0.0f };        // decode confidence (higher = better)
        cv::Point2f center{};                 // tag center in pixels
        std::array<cv::Point2f, 4> corners{}; // corner pixels, counter-clockwise
        std::optional<tag_pose_t> pose;       // pose the selector chose (tag->camera); empty without intrinsics
    };

    // ---------------------------------------------------------------------------
    // Pose-candidate selector
    // ---------------------------------------------------------------------------
    //
    // Orthogonal iteration can yield two poses (planar ambiguity). The detector applies a selector
    // once while building each detection and keeps only the chosen pose in tag_detection_t::pose.
    // Any callable works (no registration) and may be stateful (keyed on tag id), but must depend
    // only on past detections, as it runs at detection time.
    using tag_pose_candidate_selector_fn = std::function<const tag_pose_t*(int tag_id, std::span<const tag_pose_t> candidates)>;

    namespace selectors
    {
        // Default policy: lowest object-space error (best geometric fit). Stateless.
        [[nodiscard]] inline const tag_pose_t* min_error(int /*tag_id*/, std::span<const tag_pose_t> candidates) noexcept
        {
            if (candidates.empty()) { return nullptr; }
            return &*std::ranges::min_element(candidates, {}, &tag_pose_t::obj_err);
        }
    } // namespace selectors

    // Apriltag detector wrapping the C apriltag library.
    class tag_detector {
    public:
        // TODO: make the tag family selectable (currently fixed to tagStandard41h12).
        struct options_t {
            std::optional<hw::intrinsic_t> intrinsics; // Must be set to estimate pose (and axes); leave empty for 2D-only detection.
            double tag_size_m{ 0.05 };   // black square edge length [m]
            float quad_decimate{ 1.0f }; // 1.0 = full resolution (best corner accuracy)
            float quad_sigma{ 0.0f };    // Gaussian blur sigma for quad detection (0 = none)
            size_t num_threads{ 4 };     // detection worker threads
            bool refine_edges{ true };   // snap quad edges to gradients (better accuracy)
            size_t num_iters{ 50 };      // orthogonal-iteration count (default = 50)
            tag_pose_candidate_selector_fn selector{ selectors::min_error }; // pose-candidate selection policy
        };

        explicit tag_detector(const options_t& opt = {});
        ~tag_detector();

        tag_detector(const tag_detector&) = delete;
        tag_detector& operator=(const tag_detector&) = delete;

        // Accepts BGR / BGRA / grayscale. Fills pose + axes when intrinsics are configured.
        std::vector<tag_detection_t> detect(const cv::Mat& image);

    private:
        struct context_t;

        options_t _opt;
        std::unique_ptr<context_t> _ctx;
    };

    // Draw tag outlines, ids, and 3D axes (when present).
    void draw_tag_detections(
        cv::Mat& bgr, 
        std::span<const tag_detection_t> detections
    );

} // namespace pose
