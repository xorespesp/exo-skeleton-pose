#include "pose_estimator.hh"

#include <numbers>

namespace pose
{
    namespace
    {
        constexpr double kRadToDeg = 180.0 / std::numbers::pi;

        size_t index_of(joint_id_t j) { return static_cast<size_t>(j); }

        // Orthonormalized rotation part of a tag->camera pose as a quaternion.
        Eigen::Quaterniond rot_of(const Eigen::Isometry3d& pose)
        {
            return Eigen::Quaterniond{ pose.rotation() }.normalized();
        }

        // Rotation of `child` expressed in `parent`'s frame: R = parent^-1 * child.
        Eigen::Quaterniond q_relative(const Eigen::Quaterniond& parent, const Eigen::Quaterniond& child)
        {
            return (parent.conjugate() * child).normalized();
        }

        // Intrinsic XYZ euler decomposition [deg].
        // NOTE: euler angles wrap and hit gimbal singularities; use anim_rot (the
        // quaternion) for anything downstream, and treat these as a readout only.
        Eigen::Vector3d to_euler_deg(const Eigen::Quaterniond& q)
        {
            return q.toRotationMatrix().eulerAngles(0, 1, 2) * kRadToDeg;
        }
    } // namespace

    pose_estimator::pose_estimator(const options_t& opt)
        : _opt{ opt }
    { }

    const joint_state_t& pose_estimator::get_joint_state(joint_id_t j) const
    {
        return _joint_states[index_of(j)];
    }

    void pose_estimator::update(std::span<const tag_detection_t> detections)
    {
        // 1) Reset all states.
        _joint_states = {};

        // 2) Bind detections (with a pose) to joints via the static tag table.
        for (const auto& curr_det : detections)
        {
            if (!curr_det.pose.has_value()) { continue; }
            const auto joint = tag_to_joint(curr_det.id);
            if (!joint.has_value()) { continue; } // tag id not part of the rig

            _joint_states[index_of(joint.value())].view_pose = curr_det.pose.value();
        }

        // 3) Local rotation relative to the parent joint (root: relative to camera).
        // 4) Animation rotation relative to the captured rest pose.
        for (const auto& curr_j_info : kJointsInfo)
        {
            joint_state_t& curr_j_state = _joint_states[index_of(curr_j_info.id)];
            if (!curr_j_state.is_visible()) { continue; }

            const Eigen::Quaterniond r_child = rot_of(curr_j_state.view_pose.value());

            if (is_root_joint(curr_j_info.id)) {
                curr_j_state.local_rot = r_child;
            } else {
                const joint_state_t& parent = _joint_states[index_of(curr_j_info.parent)];
                if (parent.is_visible()) {
                    curr_j_state.local_rot = q_relative(rot_of(parent.view_pose.value()), r_child);
                }
                // parent occluded -> local_rot stays nullopt
            }

            if (!curr_j_state.local_rot.has_value()) { continue; }

            curr_j_state.anim_rot = curr_j_state.local_rot.value(); // no rest captured -> identity rest
            if (_rest_pose.has_value())
            {
                const auto& rest = _rest_pose.value()[index_of(curr_j_info.id)];
                if (rest.has_value()) { curr_j_state.anim_rot = q_relative(rest.value(), curr_j_state.local_rot.value()); }
            }
            curr_j_state.euler_deg = to_euler_deg(curr_j_state.anim_rot.value());
        }
    }

    bool pose_estimator::calibrate_rest_pose()
    {
        std::array<std::optional<Eigen::Quaterniond>, kNumJoints> new_rest_pose{};

        bool any = false;
        for (size_t i = 0; i < kNumJoints; ++i) {
            new_rest_pose[i] = _joint_states[i].local_rot; // nullopt if not computable this frame
            any = any || new_rest_pose[i].has_value();
        }

        if (any) { _rest_pose = new_rest_pose; }
        else { _rest_pose.reset(); }

        return any;
    }

    void pose_estimator::clear_rest_pose()
    {
        _rest_pose.reset();
    }

} // namespace pose
