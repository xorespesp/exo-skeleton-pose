#pragma once
#include "tag_detector.hh" // pose::tag_detection_t

#include <Eigen/Geometry>

#include <array>
#include <optional>
#include <span>
#include <string_view>
#include <cstddef>

namespace pose
{
    // ---------------------------------------------------------------------------
    // Skeleton definition
    // ---------------------------------------------------------------------------

    enum class joint_id_t
    {
        pelvis = 0, r_knee, l_knee, r_ankle, l_ankle, r_foot, l_foot,
        count
    };

    inline constexpr size_t kNumJoints{ static_cast<size_t>(joint_id_t::count) };

    // Static per-joint definition: binds a joint to its tag and its parent.
    struct joint_info_t
    {
        joint_id_t id;
        std::string_view name;
        int tag_id; // tag id bound to this joint
        joint_id_t parent; // parent joint (== id itself for a root)
    };

    // Single source of truth for the rig. Add a joint => add an enum value + one row.
    inline constexpr std::array<joint_info_t, kNumJoints> kJointsInfo{ {
        { joint_id_t::pelvis,  "pelvis",  0, joint_id_t::pelvis  }, // root (self-parent)
        { joint_id_t::r_knee,  "r_knee",  1, joint_id_t::pelvis  },
        { joint_id_t::l_knee,  "l_knee",  2, joint_id_t::pelvis  },
        { joint_id_t::r_ankle, "r_ankle", 3, joint_id_t::r_knee  },
        { joint_id_t::l_ankle, "l_ankle", 4, joint_id_t::l_knee  },
        { joint_id_t::r_foot,  "r_foot",  5, joint_id_t::r_ankle },
        { joint_id_t::l_foot,  "l_foot",  6, joint_id_t::l_ankle },
    } };

    // Table lookups over kJointsInfo.
    constexpr const joint_info_t& joint_info(joint_id_t j) {
        return kJointsInfo[static_cast<size_t>(j)];
    }

    // Reverse lookup: tag id -> joint (linear over kJointsInfo).
    constexpr std::optional<joint_id_t> tag_to_joint(int tag_id) {
        for (const auto& info : kJointsInfo) {
            if (info.tag_id == tag_id) { return info.id; }
        }
        return std::nullopt;
    }

    // NOTE: a root joint is its own parent (parent == id).
    constexpr bool is_root_joint(joint_id_t j) {
        return joint_info(j).parent == j;
    }

    // ---------------------------------------------------------------------------
    // Per-joint result of one estimation step
    // ---------------------------------------------------------------------------
    struct joint_state_t
    {
        std::optional<Eigen::Isometry3d> view_pose; // tag -> camera; set iff the tag was detected this frame
        std::optional<Eigen::Quaterniond> local_rot; // rotation relative to the parent joint's tag
        std::optional<Eigen::Quaterniond> anim_rot; // local_rot relative to the captured rest pose (drives the skeleton)
        Eigen::Vector3d euler_deg{ Eigen::Vector3d::Zero() }; // anim_rot as intrinsic XYZ euler angles [deg]

        bool is_visible() const { return view_pose.has_value(); } // tag detected this frame
    };

    // ---------------------------------------------------------------------------
    // Estimator: tag detections -> per-joint local / animation rotations
    // ---------------------------------------------------------------------------
    //
    // Poses are tag->camera; the camera is not a fixed world frame, so only
    // rotations relative to a parent tag (local_rot = R_parent^-1 * R_child;
    // root: vs camera) or to a captured rest pose (anim_rot = R_rest^-1 * R_local)
    // are meaningful. anim_rot drives the skeleton; the rest pose is captured by
    // calibrate_rest_pose() in any neutral stance (not necessarily a T-pose).
    class pose_estimator
    {
    public:
        struct options_t
        {
            // reserved (e.g. euler decomposition order); currently intrinsic XYZ.
        };

        explicit pose_estimator(const options_t& opt = {});

        // Ingest one frame's detections and recompute every joint state.
        void update(std::span<const tag_detection_t> detections);

        // Latch the current per-joint local_rot as the rest (bind) reference.
        // Returns false if no joint had a computable local_rot this frame.
        bool calibrate_rest_pose();
        void clear_rest_pose();
        bool has_rest_pose() const { return _rest_pose.has_value(); }

        const joint_state_t& get_joint_state(joint_id_t j) const;
        std::span<const joint_state_t> get_joint_states() const { return _joint_states; }

    private:
        options_t _opt;
        std::array<joint_state_t, kNumJoints> _joint_states{};

        // Captured rest pose: outer optional == "calibrated"; 
        // inner per-joint optional == "that joint had a computable local_rot at capture time".
        std::optional<std::array<std::optional<Eigen::Quaterniond>, kNumJoints>> _rest_pose;
    };

} // namespace pose
