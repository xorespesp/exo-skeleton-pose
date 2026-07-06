#include "tag_detector.hh"

#include <apriltag.h>
#include <apriltag_pose.h>
#include <tagStandard41h12.h>

#include <opencv2/imgproc.hpp>

#include <format>
#include <string>

namespace pose
{
    namespace
    {
        Eigen::Isometry3d to_isometry(const apriltag_pose_t& p)
        {
            // matd_t stores its 3x3 (R) / 3x1 (t) doubles row-major and contiguous.
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            transform.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(p.R->data);
            transform.translation() = Eigen::Map<const Eigen::Vector3d>(p.t->data);
            return transform;
        }

        cv::Point2f project(const hw::intrinsic_t& k, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p_tag)
        {
            const Eigen::Vector3d p_cam = pose * p_tag;
            return {
                static_cast<float>(k.fx * p_cam.x() / p_cam.z() + k.cx),
                static_cast<float>(k.fy * p_cam.y() / p_cam.z() + k.cy)
            };
        }
    } // namespace

    tag_detector::tag_detector(const options_t& opt)
        : _opt{ opt }
        , _detector{ ::apriltag_detector_create(), &::apriltag_detector_destroy }
        , _family{ ::tagStandard41h12_create(), &::tagStandard41h12_destroy }
    {
        ::apriltag_detector_add_family(_detector.get(), _family.get());

        _detector->quad_decimate = _opt.quad_decimate;
        _detector->quad_sigma = _opt.quad_sigma;
        _detector->nthreads = _opt.nthreads;
        _detector->refine_edges = _opt.refine_edges;
    }

    tag_detector::~tag_detector() = default;

    std::vector<tag_detection_t> tag_detector::detect(const cv::Mat& image)
    {
        cv::Mat gray;
        if (image.channels() == 3) { cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY); }
        else if (image.channels() == 4) { cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY); }
        else { gray = image; }
        if (!gray.isContinuous()) { gray = gray.clone(); }

        image_u8_t im{ gray.cols, gray.rows, static_cast<int32_t>(gray.step[0]), gray.data };

        const std::unique_ptr<::zarray_t, void (*)(zarray_t*)> raw{
            ::apriltag_detector_detect(_detector.get(), &im), &::apriltag_detections_destroy
        };

        std::vector<tag_detection_t> detections;
        detections.reserve(::zarray_size(raw.get()));
        for (int i = 0; i < ::zarray_size(raw.get()); ++i) {
            apriltag_detection_t* d = nullptr;
            ::zarray_get(raw.get(), i, &d);

            tag_detection_t& det = detections.emplace_back();
            det.id = d->id;
            det.hamming = d->hamming;
            det.decision_margin = d->decision_margin;
            det.center = { static_cast<float>(d->c[0]), static_cast<float>(d->c[1]) };
            for (int k = 0; k < 4; ++k) {
                det.corners[k] = { static_cast<float>(d->p[k][0]), static_cast<float>(d->p[k][1]) };
            }

            if (!_opt.intrinsics.has_value()) { continue; }

            const hw::intrinsic_t& intr = _opt.intrinsics.value();
            ::apriltag_detection_info_t info{ d, _opt.tag_size_m, intr.fx, intr.fy, intr.cx, intr.cy };
            ::apriltag_pose_t pose{};
            ::estimate_tag_pose(&info, &pose);
            const Eigen::Isometry3d transform = to_isometry(pose);
            ::matd_destroy(pose.R);
            ::matd_destroy(pose.t);

            const double len = _opt.tag_size_m * 0.5;
            det.pose = transform;
            det.axes = std::array<cv::Point2f, 4>{
                project(intr, transform, { 0.0, 0.0, 0.0 }),
                project(intr, transform, { len, 0.0, 0.0 }),
                project(intr, transform, { 0.0, len, 0.0 }),
                project(intr, transform, { 0.0, 0.0, len }),
            };
        }

        return detections;
    }

    void draw_tag_detections(cv::Mat& bgr, std::span<const tag_detection_t> detections)
    {
        for (const auto& d : detections) {
            for (int k = 0; k < 4; ++k) {
                cv::line(bgr, d.corners[k], d.corners[(k + 1) % 4], cv::Scalar{ 0, 255, 0 }, 2);
            }
            cv::circle(bgr, d.corners[0], 4, cv::Scalar{ 0, 0, 255 }, cv::FILLED); // first corner
            cv::putText(bgr, std::format("{}", d.id), d.center,
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar{ 0, 255, 255 }, 2);

            if (d.axes.has_value()) {
                const auto& a = d.axes.value();
                cv::line(bgr, a[0], a[1], cv::Scalar{ 0, 0, 255 }, 2); // X red
                cv::line(bgr, a[0], a[2], cv::Scalar{ 0, 255, 0 }, 2); // Y green
                cv::line(bgr, a[0], a[3], cv::Scalar{ 255, 0, 0 }, 2); // Z blue
            }
            if (d.pose.has_value()) {
                cv::putText(bgr, std::format("{:.2f}m", d.pose->translation().norm()), d.corners[2],
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar{ 255, 255, 255 }, 2);
            }
        }
    }

} // namespace pose
