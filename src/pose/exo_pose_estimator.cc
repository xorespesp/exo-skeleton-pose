#include "exo_pose_estimator.hh"

#include <algorithm>

namespace pose
{
    namespace
    {
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
    } // namespace

    exo_pose_estimator::exo_pose_estimator(const options_t& opt)
        : _opt{ opt }
    {
        for (auto& fs : _filter_states) {
            fs.smoother = make_rotation_filter(_opt.filter);
        }
        _built_kind = _opt.filter.kind;
    }

    const joint_state_t& exo_pose_estimator::get_joint_state(joint_id_t j) const
    {
        return _joint_states[index_of(j)];
    }

    void exo_pose_estimator::update(
        std::span<const tag_detection_t> detections, 
        std::chrono::microseconds timestamp)
    {
        const auto t = timestamp;

        // Pass 0: rebuild / reconfigure kernels.
        // Pick up any edited kernel selection / params before this frame.
        {
            if (_opt.filter.kind != _built_kind) {
                // Algorithm switched: rebuild the kernels (loses filter state; rare, GUI-driven).
                for (auto& fs : _filter_states) {
                    fs.smoother = make_rotation_filter(_opt.filter);
                }
                _built_kind = _opt.filter.kind;
            } else {
                // Same algorithm: push live params (cheap, no state loss).
                for (auto& fs : _filter_states) {
                    fs.smoother->configure(_opt.filter);
                }
            }
        }

        // Reset only the per-frame output; _filter_states (filter/timers) persist.
        _joint_states = {};
        _last_fresh = {};

        // Pass 1: bind detections (with a pose) to joints via the static tag table.
        for (const auto& curr_det : detections)
        {
            if (!curr_det.pose.has_value()) { continue; }
            const auto joint = tag_to_joint(curr_det.id);
            if (!joint.has_value()) { continue; } // tag id not part of the rig

            _joint_states[index_of(joint.value())].view_pose = curr_det.pose.value();
        }

        // Pass 2: smooth+hold each joint's GLOBAL (camera-frame) rotation. (joint order irrelevant)
        // Holding the global rotation lets a child recompute its local_rot from a briefly
        // occluded parent's held rotation, so occlusion doesn't cascade downward.
        for (const auto& curr_j_info : kJointsInfo)
        {
            const size_t idx = index_of(curr_j_info.id);
            joint_filter_state_t& curr_j_fstate = _filter_states[idx];
            joint_state_t& curr_j_state = _joint_states[idx];

            if (curr_j_state.view_pose.has_value()) // fresh detection
            {
                const Eigen::Quaterniond q = rot_of(curr_j_state.view_pose.value());

                // Reseed on cold start, after a long gap, or when smoothing is disabled
                // (disabled -> raw passthrough while keeping the kernel synced for re-enable).
                if (!_opt.enable_smoothing || !curr_j_fstate.last_out.has_value() || (t - curr_j_fstate.last_seen) > _opt.reset_gap) {
                    curr_j_fstate.smoother->reset(q);
                    curr_j_fstate.last_out = q;
                } else {
                    const seconds_f64 dt = std::clamp(seconds_f64{ t - curr_j_fstate.t_prev }, _opt.dt_min, _opt.dt_max);
                    curr_j_fstate.last_out = curr_j_fstate.smoother->filter(q, dt.count()); // hemisphere align + smoothing inside
                }
                curr_j_fstate.t_prev = t;
                curr_j_fstate.last_seen = t;
                curr_j_state.global_rot = curr_j_fstate.last_out;
                _last_fresh[idx] = true;
            }
            else if (curr_j_fstate.last_out.has_value() && (t - curr_j_fstate.last_seen) <= _opt.max_hold) // hold
            {
                curr_j_state.global_rot = curr_j_fstate.last_out; // reuse last smoothed global rotation (kernel/timer untouched)
            }
            // else: lost -> global_rot stays nullopt
        }

        // Pass 3: relative + animation rotations.
        // kJointsInfo is parent-before-child ordered, so a single forward pass suffices.
        for (const auto& curr_j_info : kJointsInfo)
        {
            joint_state_t& curr_j_state = _joint_states[index_of(curr_j_info.id)];
            if (!curr_j_state.global_rot.has_value()) { continue; }

            if (is_root_joint(curr_j_info.id)) {
                curr_j_state.local_rot = curr_j_state.global_rot;
            } else {
                const joint_state_t& parent_j_state = _joint_states[index_of(curr_j_info.parent)];
                if (parent_j_state.global_rot.has_value()) {
                    curr_j_state.local_rot = q_relative(
                        parent_j_state.global_rot.value(), 
                        curr_j_state.global_rot.value()
                    ); // fresh or held
                }
                // parent lost beyond the hold window -> local_rot stays nullopt (cascade, documented)
            }

            if (!curr_j_state.local_rot.has_value()) { continue; }

            curr_j_state.local_anim_rot = curr_j_state.local_rot.value(); // no rest captured == identity rest
            if (_rest_pose.has_value()) {
                const auto& rest = _rest_pose.value()[index_of(curr_j_info.id)];
                if (rest.has_value()) { curr_j_state.local_anim_rot = q_relative(rest.value(), curr_j_state.local_rot.value()); }
            }
        }
    }

    bool exo_pose_estimator::calibrate_rest_pose()
    {
        std::array<std::optional<Eigen::Quaterniond>, kNumJoints> new_rest_pose{};

        bool any = false;
        for (size_t i = 0; i < kNumJoints; ++i) {
            // Only latch freshly detected joints; a held joint's local_rot comes
            // from a frozen global rotation and would capture a stale rest reference.
            if (_last_fresh[i]) { new_rest_pose[i] = _joint_states[i].local_rot; }
            any = any || new_rest_pose[i].has_value();
        }

        if (any) { _rest_pose = new_rest_pose; }
        else { _rest_pose.reset(); }

        return any;
    }

    void exo_pose_estimator::clear_rest_pose()
    {
        _rest_pose.reset();
    }

} // namespace pose
