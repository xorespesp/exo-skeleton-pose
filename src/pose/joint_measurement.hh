#pragma once
#include "joints_def.hh"
#include "view_plane.hh"

#include "hw/timestamp.hh"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace pose
{
    // ---------------------------------------------------------------------------
    // Estimator input: one frame's per-joint measurements
    // ---------------------------------------------------------------------------
    //
    // A measurement names the joint it belongs to, so an estimator never resolves identity itself.
    // Whoever holds the detections does that: 
    // a tag carries an id, a color blob is placed by its position among its neighbours.
    //
    // One shape per estimator dimensionality. Neither carries orientation: every rig rotation this
    // project reports is solved from positions (`joint_state_t::local_anim_rot`).

    // Image-plane measurement, for an estimator that reads angles off pixels (sagittal).
    struct joint_2d_measurement_t
    {
        joint_id_t joint_id{ joint_id_t::pelvis };
        Eigen::Vector2d center_px{ Eigen::Vector2d::Zero() };

        // Metric scale this one measurement supplies:
        // the marker's printed size over its apparent size in pixels.
        // An estimator averages one vote per physical marker and reports positions in
        // approximate meters; joints co-sited on one marker each carry that marker's value, and
        // the vote is keyed by the tag the joint is bound to, so the shared marker still counts
        // once. Empty when the marker's apparent size was not measurable, which leaves the
        // center usable and only withholds a vote on the scale.
        std::optional<double> meters_per_pixel{};
    };

    // Rig-space position measurement [m], for an estimator that works in 3D (frontal).
    struct joint_3d_measurement_t
    {
        joint_id_t joint_id{ joint_id_t::pelvis };
        Eigen::Vector3d position{ Eigen::Vector3d::Zero() };
    };

    // frameset 하나의 측정치. 슬롯 i 는 스트림 i 의 것이다.
    template <typename Measurement>
    struct synced_measurements_t
    {
        struct slot_t
        {
            camera_view_t view{};

            // 트래커가 이 프레임의 측정치를 냈는지. false 면 `measurements` 는 비어 있다.
            // (마커를 못 찾은 프레임은 true 에 빈 벡터)
            bool measured{ false };

            std::vector<Measurement> measurements;

            std::size_t detection_count{ 0 }; // 그 프레임에서 찾은 마커 수
        };

        uint64_t seq{ 0 };           // frameset 순번. 1 부터
        hw::timestamp_t timestamp{}; // frameset 의 시각(가장 이른 캡처)
        std::vector<slot_t> slots;   // 인덱스 = stream_idx
    };

    using synced_2d_measurements_t = synced_measurements_t<joint_2d_measurement_t>;

} // namespace pose
