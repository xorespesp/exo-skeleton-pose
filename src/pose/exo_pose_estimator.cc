#include "exo_pose_estimator.hh"
#include "rotation_constraint.hh"

#include <algorithm>
#include <limits>

namespace pose
{
    namespace
    {
        size_t index_of(joint_id_t jid) { return static_cast<size_t>(jid); }

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

        // Under a swing-twist decomposition about the leg-hinge axis, the twist is the real joint
        // motion and the swing (off-axis part) is pose error: candidates are ranked by least swing
        // and the chosen rotation keeps only its twist about the axis.
        //
        // Returns the index of the candidate closest to a pure leg-hinge rotation (least off-axis swing).
        [[nodiscard]] size_t min_off_axis(std::span<const Eigen::Quaterniond> cands, const Eigen::Vector3d& axis) noexcept
        {
            size_t best = 0;
            double min_swing = std::numeric_limits<double>::max();
            for (size_t i = 0; i < cands.size(); ++i) {
                // Swing angle = how far this candidate departs from a pure rotation about the leg-hinge axis;
                // the planar-ambiguity flip swings off-axis, the correct pose stays near it.
                const double swing = swing_angle(cands[i], axis);
                if (swing < min_swing) { min_swing = swing; best = i; }
            }
            return best;
        }
    } // namespace

    struct exo_pose_estimator::context_t
    {
        // Per-joint state that persists across frames (filter + occlusion timers).
        struct joint_filter_state_t
        {
            std::unique_ptr<rotation_filter_base> smoother; // swappable smoothing kernel
            std::optional<Eigen::Quaterniond> last_out; // last smoothed global rotation (hold output)
            std::chrono::microseconds last_seen{ 0 };   // time of the last fresh detection (hold timer origin)
            std::chrono::microseconds t_prev{ 0 };      // time of the last fresh filter step (dt source)
        };

        // A joint's tag pose candidates for the current frame (reset every update()).
        struct tag_pose_candidates_t
        {
            std::array<Eigen::Isometry3d, 2> transform{}; // tag->camera per candidate (obj_err ascending)
            int n{ 0 };
        };

        rotation_filter_kind built_kind{}; // kernel kind currently instantiated in filter_states
        std::array<joint_filter_state_t, kNumJoints> filter_states{};  // persists across frames
        std::array<joint_state_t, kNumJoints> last_frame_joint_states{}; // per-frame output; reset every update()
        std::array<bool, kNumJoints> last_frame_detection_flags{}; // per-joint fresh-detection flag (gates rest capture)
        std::array<tag_pose_candidates_t, kNumJoints> last_frame_tag_pose_candidates{}; // per-joint tag pose candidates

        // Captured rest pose: outer optional == "calibrated";
        // inner per-joint optional == "that joint had a computable local_rot at capture time".
        std::optional<std::array<std::optional<Eigen::Quaterniond>, kNumJoints>> rest_pose;
    };

    exo_pose_estimator::exo_pose_estimator(const options_t& opt)
        : _opt{ opt }
        , _ctx{ std::make_unique<context_t>() }
    {
        for (auto& fs : _ctx->filter_states) {
            fs.smoother = make_rotation_filter(_opt.filter);
        }
        _ctx->built_kind = _opt.filter.kind;
    }

    exo_pose_estimator::~exo_pose_estimator() = default;

    const joint_state_t& exo_pose_estimator::get_joint_state(joint_id_t j) const
    {
        return _ctx->last_frame_joint_states[index_of(j)];
    }

    std::span<const joint_state_t> exo_pose_estimator::get_joint_states() const
    {
        return _ctx->last_frame_joint_states;
    }

    bool exo_pose_estimator::has_rest_pose() const
    {
        return _ctx->rest_pose.has_value();
    }

    void exo_pose_estimator::update(
        const std::span<const tag_detection_t> tag_detections,
        const std::chrono::microseconds sensor_timestamp)
    {
        const auto t = sensor_timestamp;

        // ----- Pass 0: rebuild / reconfigure kernels -----
        // Pick up any edited kernel selection / params before this frame.
        {
            if (_opt.filter.kind != _ctx->built_kind) {
                // Algorithm switched: rebuild the kernels (loses filter state; rare, GUI-driven).
                for (auto& fs : _ctx->filter_states) {
                    fs.smoother = make_rotation_filter(_opt.filter);
                }
                _ctx->built_kind = _opt.filter.kind;
            } else {
                // Same algorithm: push live params (cheap, no state loss).
                for (auto& fs : _ctx->filter_states) {
                    fs.smoother->configure(_opt.filter);
                }
            }
        }

        // Reset only the per-frame output; filter_states (filter/timers) persist.
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_detection_flags = {};
        _ctx->last_frame_tag_pose_candidates = {};

        // Captured rest reference rotation for joint `jid`, if calibrated and computable.
        const auto rest_rotation_of = [this](joint_id_t jid) -> std::optional<Eigen::Quaterniond> {
            if (_ctx->rest_pose.has_value() && 
                _ctx->rest_pose.value()[index_of(jid)].has_value()) {
                return _ctx->rest_pose.value()[index_of(jid)];
            }
            return std::nullopt;
        };

        // pelvis is a fixed base: once a rest is captured, lock its rotation to that constant.
        const auto is_locked_pelvis_joint = [this, &rest_rotation_of](joint_id_t jid) {
            return _opt.enable_hinge_constraint 
                && is_root_joint(jid) 
                && rest_rotation_of(jid).has_value();
        };

        // ----- Pass 1: bind pose candidates to joints -----
        // Bind each detection's pose candidates to its joint via the static tag table.
        for (const auto& curr_tag_det : tag_detections)
        {
            if (curr_tag_det.num_pose_candidates <= 0) { continue; } // no pose (no intrinsics / undetected)

            const auto curr_tag_joint = tag_to_joint(curr_tag_det.id);
            if (!curr_tag_joint.has_value()) { continue; } // tag id not part of the rig

            auto& curr_j_pose_candidates = _ctx->last_frame_tag_pose_candidates[index_of(curr_tag_joint.value())];
            curr_j_pose_candidates.n = curr_tag_det.num_pose_candidates;
            for (int k = 0; k < curr_tag_det.num_pose_candidates; ++k) {
                curr_j_pose_candidates.transform[k] = curr_tag_det.pose_candidates[k].transform;
            }
        }

        // ----- Pass 2: select candidate + smooth/hold global rotation -----
        // Pick the physically valid candidate, then smooth+hold each joint's GLOBAL (camera-frame)
        // rotation. Parent-before-child order (kJointsInfo) lets candidate selection use the parent's
        // already-resolved rotation. Holding the global rotation lets a child recompute its local_rot
        // from a briefly occluded parent, so occlusion doesn't cascade downward.
        for (const auto& curr_j_info : kJointsInfo)
        {
            const size_t curr_j_idx = index_of(curr_j_info.id);
            auto& curr_j_fstate = _ctx->filter_states[curr_j_idx];
            joint_state_t& curr_j_state = _ctx->last_frame_joint_states[curr_j_idx];
            const auto& curr_j_pose_candidates = _ctx->last_frame_tag_pose_candidates[curr_j_idx];

            // Locked pelvis: constant camera-frame orientation, no smoothing/hold. 
            // Children read this as their parent, so pelvis noise/flips/occlusion never reach them.
            if (is_locked_pelvis_joint(curr_j_info.id)) {
                curr_j_state.global_rot = rest_rotation_of(curr_j_info.id);
                if (curr_j_pose_candidates.n > 0) {
                    curr_j_state.view_pose = curr_j_pose_candidates.transform[0];
                    _ctx->last_frame_detection_flags[curr_j_idx] = true;
                }
                continue;
            }

            if (curr_j_pose_candidates.n <= 0) {
                // No fresh detection: hold the last smoothed rotation within the hold window.
                if (curr_j_fstate.last_out.has_value() && (t - curr_j_fstate.last_seen) <= _opt.max_hold) {
                    curr_j_state.global_rot = curr_j_fstate.last_out;
                }
                continue; // else lost -> global_rot stays nullopt
            }

            const std::optional<Eigen::Quaterniond> curr_j_rest_rot = rest_rotation_of(curr_j_info.id);
            const std::optional<Eigen::Quaterniond> curr_j_parent_global = is_root_joint(curr_j_info.id)
                ? std::nullopt
                : _ctx->last_frame_joint_states[index_of(curr_j_info.parent)].global_rot;

            // Candidate selection: default to the best geometric fit (candidates[0] = min obj_err).
            // With a rest and a resolved parent, prefer the candidate closest to a pure leg-hinge
            // rotation (least off-axis swing), which rejects the planar-ambiguity flip.
            size_t selected_candidate_idx = 0;
            if (_opt.enable_hinge_constraint 
                && curr_j_rest_rot.has_value()
                && (is_root_joint(curr_j_info.id) || curr_j_parent_global.has_value()))
            {
                std::array<Eigen::Quaterniond, 2> local_anim_rot_candidates;
                for (int k = 0; k < curr_j_pose_candidates.n; ++k) {
                    const Eigen::Quaterniond child_abs = rot_of(curr_j_pose_candidates.transform[k]);
                    const Eigen::Quaterniond local = is_root_joint(curr_j_info.id)
                        ? child_abs
                        : q_relative(curr_j_parent_global.value(), child_abs);
                    local_anim_rot_candidates[k] = q_relative(curr_j_rest_rot.value(), local);
                }
                selected_candidate_idx = min_off_axis(
                    std::span<const Eigen::Quaterniond>{ local_anim_rot_candidates.data(), static_cast<size_t>(curr_j_pose_candidates.n) }, 
                    kExoHingeLocalAxis
                );
            }

            curr_j_state.view_pose = curr_j_pose_candidates.transform[selected_candidate_idx]; // reflect the selection in the raw (absolute) view
            Eigen::Quaterniond q = rot_of(curr_j_pose_candidates.transform[selected_candidate_idx]);

            // A quaternion and its negation encode the same rotation (double-cover), so a
            // tag pose can report q on one frame and -q on the next while the joint barely
            // moved. That sign flip would surface as a large jump in the smoother, the
            // time-series plots, and the broadcast stream. Flip the sample into the same
            // hemisphere as the last emitted rotation to keep the stream sign-continuous.
            if (curr_j_fstate.last_out.has_value() && curr_j_fstate.last_out->dot(q) < 0.0) {
                q.coeffs() *= -1.0;
            }

            // Reseed on cold start, after a long gap, or when smoothing is disabled
            // (disabled -> raw passthrough while keeping the kernel synced for re-enable).
            if (!_opt.enable_smoothing || !curr_j_fstate.last_out.has_value() || (t - curr_j_fstate.last_seen) > _opt.reset_gap) {
                curr_j_fstate.smoother->reset(q);
                curr_j_fstate.last_out = q;
            } else {
                const seconds_f64 dt = std::clamp(seconds_f64{ t - curr_j_fstate.t_prev }, _opt.dt_min, _opt.dt_max);
                curr_j_fstate.last_out = curr_j_fstate.smoother->filter(q, dt.count());
            }
            curr_j_fstate.t_prev = t;
            curr_j_fstate.last_seen = t;
            curr_j_state.global_rot = curr_j_fstate.last_out;
            _ctx->last_frame_detection_flags[curr_j_idx] = true;
        } // for

        // ----- Pass 3: relative + animation rotations + leg-hinge constraint -----
        // kJointsInfo is parent-before-child ordered, so a single forward pass suffices.
        for (const auto& curr_j_info : kJointsInfo)
        {
            const size_t curr_j_idx = index_of(curr_j_info.id);
            joint_state_t& curr_j_state = _ctx->last_frame_joint_states[curr_j_idx];
            if (!curr_j_state.global_rot.has_value()) { continue; }

            if (is_locked_pelvis_joint(curr_j_info.id)) {
                curr_j_state.local_rot = curr_j_state.global_rot; // constant camera-frame orientation
                // Locked pelvis is a fixed base: its animation contribution is identity.
                curr_j_state.local_anim_rot = Eigen::Quaterniond::Identity();
                continue;
            }

            if (is_root_joint(curr_j_info.id)) {
                curr_j_state.local_rot = curr_j_state.global_rot;
            } else {
                const joint_state_t& parent_j_state = _ctx->last_frame_joint_states[index_of(curr_j_info.parent)];
                if (parent_j_state.global_rot.has_value()) {
                    curr_j_state.local_rot = q_relative(
                        parent_j_state.global_rot.value(),
                        curr_j_state.global_rot.value()
                    ); // fresh or held
                }
                // parent lost beyond the hold window -> local_rot stays nullopt (cascade, documented)
            }

            if (!curr_j_state.local_rot.has_value()) {
                continue;
            }

            const std::optional<Eigen::Quaterniond> curr_j_rest_rot = rest_rotation_of(curr_j_info.id);
            curr_j_state.local_anim_rot = curr_j_state.local_rot.value(); // no rest captured == identity rest
            if (curr_j_rest_rot.has_value()) {
                curr_j_state.local_anim_rot = q_relative(
                    curr_j_rest_rot.value(), 
                    curr_j_state.local_rot.value()
                );
            }

            // Constrain the joint to its leg-hinge axis: 
            // the joint physically rotates only about this axis, 
            // so keep the rotation about it and drop the off-axis part, which is tag-pose error.
            if (_opt.enable_hinge_constraint && curr_j_rest_rot.has_value() && !is_root_joint(curr_j_info.id)) {
                curr_j_state.local_anim_rot = decompose_swing_twist_about_axis(curr_j_state.local_anim_rot.value(), kExoHingeLocalAxis).twist;
                curr_j_state.local_rot = curr_j_rest_rot.value() * curr_j_state.local_anim_rot.value(); // keep serialized rotations consistent
            }
        } // for
    }

    bool exo_pose_estimator::calibrate_rest_pose()
    {
        std::array<std::optional<Eigen::Quaterniond>, kNumJoints> new_rest_pose{};

        bool any = false;
        for (size_t i = 0; i < kNumJoints; ++i) {
            // Only latch freshly detected joints; a held joint's local_rot comes
            // from a frozen global rotation and would capture a stale rest reference.
            if (_ctx->last_frame_detection_flags[i]) {
                new_rest_pose[i] = _ctx->last_frame_joint_states[i].local_rot;
            }
            any = any || new_rest_pose[i].has_value();
        }

        if (any) { _ctx->rest_pose = new_rest_pose; }
        else { _ctx->rest_pose.reset(); }

        return any;
    }

    void exo_pose_estimator::clear_rest_pose()
    {
        _ctx->rest_pose.reset();
    }

} // namespace pose
