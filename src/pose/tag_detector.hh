#pragma once
#include "hw/calibration.hh" // hw::intrinsic_t

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <array>
#include <memory>
#include <optional>
#include <span>
#include <vector>

// forward-declaration of C apriltag types (apriltag.h)
struct apriltag_detector;
struct apriltag_family;

namespace pose
{
    struct tag_detection_t {
        int id{ 0 };                                    // decoded tag id
        int hamming{ 0 };                               // corrected error bits
        float decision_margin{ 0.0f };                  // decode confidence (higher = better)
        cv::Point2f center{};                           // tag center in pixels
        std::array<cv::Point2f, 4> corners{};           // corner pixels, counter-clockwise
        std::optional<Eigen::Isometry3d> pose;          // tag -> camera; set when intrinsics are given
        std::optional<std::array<cv::Point2f, 4>> axes; // pixel-projected {origin, X, Y, Z}
    };

    // Apriltag detector wrapping the C apriltag library.
    class tag_detector {
    public:
        // TODO: make the tag family selectable (currently fixed to tagStandard41h12).
        struct options_t {
            std::optional<hw::intrinsic_t> intrinsics; // Must be set to estimate pose (and axes); leave empty for 2D-only detection.
            double tag_size_m{ 0.05 };   // black square edge length [m]
            float quad_decimate{ 1.0f }; // 1.0 = full resolution (best corner accuracy)
            float quad_sigma{ 0.0f };    // Gaussian blur sigma for quad detection (0 = none)
            int nthreads{ 4 };           // detection worker threads
            bool refine_edges{ true };   // snap quad edges to gradients (better accuracy)
        };

        explicit tag_detector(const options_t& opt = {});
        ~tag_detector();

        tag_detector(const tag_detector&) = delete;
        tag_detector& operator=(const tag_detector&) = delete;

        // Accepts BGR / BGRA / grayscale. Fills pose + axes when intrinsics are configured.
        std::vector<tag_detection_t> detect(const cv::Mat& image);

    private:
        options_t _opt;
        std::unique_ptr<::apriltag_detector, void(*)(::apriltag_detector*)> _detector;
        std::unique_ptr<::apriltag_family,   void(*)(::apriltag_family*)> _family;
    };

    // Draw tag outlines, ids, and 3D axes (when present).
    void draw_tag_detections(
        cv::Mat& bgr, 
        std::span<const tag_detection_t> detections
    );

} // namespace pose
