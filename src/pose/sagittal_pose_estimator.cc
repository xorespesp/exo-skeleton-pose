#include "sagittal_pose_estimator.hh"
#include "hinge_angle.hh"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace pose
{
    // ---------------------------------------------------------------------------
    // Angle model: rig-sign geometry, published in the document conventions
    // ---------------------------------------------------------------------------
    //
    // All geometry in this file is measured in ONE sign system, the rig hinge sign:
    //
    //   rig segment angle   A bone's signed turn from the rig's down axis (`kRigDownAxis`) to
    //                       its direction, about the rig's lateral axis (`kRigLateralAxis`).
    //                       A bone hanging straight down reads 0; its far end swung toward the
    //                       exo's back (rig +Z) reads positive, toward its front negative.
    //
    //   rig bend            The bend at a joint: the turn of the bone leaving it away from the
    //                       bone entering it, 0 when the two are collinear. Within one plane the
    //                       angle between two directions is the difference of the angles each
    //                       makes with any common reference, so a bend is a difference of
    //                       adjacent rig segment angles:
    //
    //                         (down -> shin) - (down -> thigh) = (thigh -> shin)
    //
    // With the bones of one leg (thigh = hip->knee, shin = knee->ankle, foot bone = ankle->foot;
    // the hip is co-sited on the pelvis marker):
    //
    //   bend at the hip   = seg(thigh)
    //   bend at the knee  = seg(shin) - seg(thigh)
    //   bend at the ankle = seg(foot) - seg(shin)
    //   foot              = marker end site; no bone leaves it, so no bend
    //
    // The hip's parent bone is the torso, which carries no marker. The exo stands upright on its
    // fixed frame, so the torso runs along the rig's down axis: the reference direction IS the
    // torso segment at 0, and the thigh's segment angle is the hip's bend outright.
    //
    // One sign system for all geometry keeps this file free of per-joint sign choices: every
    // bend is the same subtraction, and `local_anim_rot` is a rest-relative bend put straight
    // onto `kRigLateralAxis` with no sign work at all. The outputs are then produced at the
    // `joint_state_t` boundary:
    //
    //   sagittal_segment_angle / sagittal_clinical_angle / sagittal_included_angle
    //     The biomechanics conventions of docs/joint_angle_convention.md, carried over from the
    //     rig-sign values by the conversion functions beside the rig table (joints_def.hh),
    //     which own every per-joint sign and neutral offset; the included angle is filled from
    //     the clinical one in the same statement block. No captured rest enters: these state the
    //     leg's geometry itself, which is what an external validation compares with the exo's
    //     own joint encoders.
    //
    //   local_anim_rot / sagittal_clinical_angle_delta / sagittal_segment_angle_delta
    //     The change since the captured rest pose,
    //
    //       delta(bone) = wrap_pi(rig_segment_now(bone) - rig_segment_rest(bone))
    //
    //     with the same subtraction cascade on the deltas for the per-joint bends. Zero while
    //     the exo holds the captured pose, which is what drives a rig from its bind pose. The
    //     reference axis cancels in every delta, and so do the conventions' neutral offsets,
    //     leaving only the clinical sign between the delta fields and the rotation.
    //
    // Every angle difference goes through `wrap_pi()`, so no reading jumps by 2*pi.

    namespace
    {
        size_t index_of(joint_id_t jid) { return static_cast<size_t>(jid); }

        // 전제: 다리가 없는 관절은 골반 하나이고, 양쪽 hip 은 골반 태그를 같이 쓴다.
        static_assert([] {
            for (const auto& def : get_joint_defs()) {
                if (def.side == joint_side_t::midline && def.joint_id != get_root_joint()) { return false; }
            }
            const int pelvis_tag = get_joint_def(get_root_joint())->tag_id;
            return get_joint_def(*get_leg_root_joint(joint_side_t::left))->tag_id == pelvis_tag
                && get_joint_def(*get_leg_root_joint(joint_side_t::right))->tag_id == pelvis_tag;
        }(), "the sagittal estimator reads the pelvis off the hips co-sited on its marker");

        // +1 or -1, applied wherever the image plane is carried into rig space.
        double side_sign(joint_side_t leg) {
            return (leg == joint_side_t::right) ? -1.0 : 1.0;
        }

        // A side view's camera frame is not the rig frame: the camera looks along the rig's lateral
        // axis, so what the image plane holds is the rig's sagittal (Y-Z) plane. The exo's left lies
        // at positive rig X, so a camera on that side maps
        //
        //   camera Z -> rig -X   (looking at the exo from its left)
        //   image y  -> rig Y    (down)
        //   image x  -> rig Z    (behind the exo; the remaining right-handed axis)
        //
        // Flexion is therefore a rotation about the rig's lateral axis, and an image-plane point
        // lands on the rig's mid-sagittal plane (X = 0). Viewing from the right mirrors the image,
        // reversing front-to-back and the swing together, which is what `side` (+1 left, -1 right)
        // carries.

        // Image-plane offset as an approximate rig-space position, flat on the sagittal plane.
        Eigen::Vector3d to_rig_space(const Eigen::Vector2d& px, double meters_per_pixel, double side) {
            return Eigen::Vector3d{ 0.0, px.y() * meters_per_pixel, side * px.x() * meters_per_pixel };
        }

        // Direction of the bone from `start` to `end`, lifted from the image plane into rig
        // space: the axis mapping of `to_rig_space()` with the metric scale left out, since a
        // direction has no length to convert. This is what keeps every angle below scale free.
        // Positions and directions sharing one mapping puts the camera-side handedness in a
        // single seam, so the two cannot fall out of step.
        Eigen::Vector3d lift_image_direction(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double side) {
            const Eigen::Vector2d d = end - start;
            return Eigen::Vector3d{ 0.0, d.y(), side * d.x() };
        }

        // Rig-sign segment angle of the bone from `start` to `end`, read off image-plane points
        // (see the angle model atop this file): the signed turn from the rig's down axis to the
        // bone's direction, about the rig's lateral axis, backward swing positive. Empty when an
        // endpoint is missing, and when the two coincide, since a zero-length bone has no
        // direction to measure. Conversion into the published conventions happens where
        // `joint_state_t` is filled.
        std::optional<double> rig_sagittal_segment_angle_from_px(
            const std::optional<Eigen::Vector2d>& start,
            const std::optional<Eigen::Vector2d>& end,
            double side)
        {
            constexpr double kMinBoneLengthSq_px = 1e-6; // coincident-endpoint gate [px^2]

            if (!start.has_value() || !end.has_value()) { return std::nullopt; }
            const Eigen::Vector3d dir = lift_image_direction(start.value(), end.value(), side);
            if (dir.squaredNorm() < kMinBoneLengthSq_px) { return std::nullopt; }
            return hinge_plane_angle(
                pose_estimator_base::kRigDownAxis, dir, pose_estimator_base::kRigLateralAxis);
        }

    } // namespace

    struct sagittal_pose_estimator::context_t
    {
        // Per-joint image-plane filter + occlusion timers, persisting across frames. One Euro per
        // image axis. Smoothing runs on pixels so the angles read off them are smoothed too.
        struct joint_filter_state_t
        {
            std::array<dsp::OneEuroFilter, 2> px_smoother{};
            std::optional<Eigen::Vector2d> last_px_out; // last smoothed point (hold output)
            hw::timestamp_t last_seen{}; // time of the last fresh detection (hold origin)
            hw::timestamp_t last_step_time{}; // time of the last fresh filter step (dt source)
        };

        std::array<joint_filter_state_t, kNumJoints> filter_states{}; // persists across frames
        std::array<joint_state_t, kNumJoints> last_frame_joint_states{}; // per-frame output; reset every update()
        std::array<std::optional<Eigen::Vector2d>, kNumJoints> last_frame_raw_px{}; // per-frame measured points
        std::array<std::optional<Eigen::Vector2d>, kNumJoints> last_frame_px{}; // smoothed + held centers (angle source)
        std::array<bool, kNumJoints> last_frame_detection_flags{}; // per-joint fresh-detection flag

        // One conversion factor per leg, averaged over this frame's measurements and held
        // while none arrive. A per-joint factor would stretch each point away from the pelvis
        // by a different amount, skewing the skeleton's shape; a single one keeps the image geometry
        // and only sets its size.
        double left_meters_per_pixel{ 0.0 };
        double right_meters_per_pixel{ 0.0 };

        // 다리마다 그 카메라가 본 골반 마커의 픽셀(그 다리 hip 의 점).
        std::optional<Eigen::Vector2d> left_pelvis_px;
        std::optional<Eigen::Vector2d> right_pelvis_px;

        // The captured rest (bind) reference. Kept in pixels: that is what the rest bone angles
        // derive from, and it makes them independent of how far the exo stood at capture.
        struct rest_pose_info_t
        {
            // Per-joint image-plane point at capture; empty for a joint that was not measured then.
            // NOTE: 다리 관절의 점은 그 다리를 찍는 카메라의 픽셀이다.
            std::array<std::optional<Eigen::Vector2d>, kNumJoints> joint_px{};
        };
        std::optional<rest_pose_info_t> rest_pose; // empty until calibrated
    };

    sagittal_pose_estimator::sagittal_pose_estimator(const options_t& opt)
        : _opt{ opt }
        , _ctx{ std::make_unique<context_t>() }
    { }

    sagittal_pose_estimator::~sagittal_pose_estimator() = default;

    const joint_state_t& sagittal_pose_estimator::get_joint_state(joint_id_t j) const
    {
        return _ctx->last_frame_joint_states[index_of(j)];
    }

    std::span<const joint_state_t> sagittal_pose_estimator::get_joint_states() const
    {
        return _ctx->last_frame_joint_states;
    }

    bool sagittal_pose_estimator::has_rest_pose() const
    {
        return _ctx->rest_pose.has_value();
    }

    // Converted with the current frame's factor rather than the one at capture, so a rest skeleton
    // drawn next to the measured one shares its size and only their angles can differ.
    std::optional<Eigen::Vector3d> sagittal_pose_estimator::get_rest_position(joint_id_t j) const
    {
        if (!_ctx->rest_pose.has_value()) { return std::nullopt; }

        // 골반은 두 다리가 매달린 원점이다.
        if (j == get_root_joint()) { return Eigen::Vector3d::Zero(); }

        // 그 다리의 rest hip 을 원점으로 잰다.
        const joint_side_t leg = get_joint_side(j).value();
        const auto& px = _ctx->rest_pose->joint_px[index_of(j)];
        const auto& rest_pelvis_px = _ctx->rest_pose->joint_px[index_of(get_leg_root_joint(leg).value())];
        if (!px.has_value() || !rest_pelvis_px.has_value()) { return std::nullopt; }

        const double meters_per_pixel = (leg == joint_side_t::right)
            ? _ctx->right_meters_per_pixel
            : _ctx->left_meters_per_pixel;

        return to_rig_space(
            px.value() - rest_pelvis_px.value(),
            meters_per_pixel,
            side_sign(leg)
        );
    }

    void sagittal_pose_estimator::update(const synced_2d_measurements_t& frameset)
    {
        for (const auto& slot : frameset.slots)
        {
            const auto same_view_slots = std::ranges::count(frameset.slots, slot.view, &synced_2d_measurements_t::slot_t::view);
            if (!is_sagittal(slot.view) || same_view_slots > 1) {
                throw std::invalid_argument(std::format(
                    "sagittal estimator: a slot views the exo from '{}', which is not a side view or is shared with another slot",
                    camera_view_name(slot.view)));
            }
        }

        const auto t = frameset.timestamp;
        const joint_id_t pelvis = get_root_joint();

        // Reset the per-frame output; filter_states persist.
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_raw_px = {};
        _ctx->last_frame_px = {};
        _ctx->last_frame_detection_flags = {};

        // ----- Pass 1: take this frame's image-plane points and metric scale -----
        // Each leg's scale is averaged with one vote per physical marker: the rig sits at one distance
        // from a side camera, so one factor describes it and the spread between markers is
        // measurement noise rather than depth. Joints co-sited on one marker (the pelvis marker
        // carries the pelvis and both hips) arrive as one measurement each, all repeating that
        // marker's scale, so a vote is keyed by the tag the joint is bound to; the shared tag_id
        // in the rig table is the very datum that makes joints co-sited, so the keying follows
        // any change to the co-siting on its own.
        struct scale_vote_t
        {
            double sum{ 0.0 };
            int count{ 0 };
            std::unordered_set<int> voted_tags;

            std::optional<double> get_average() const {
                if (count == 0) { return std::nullopt; }
                return sum / count;
            }
        };
        scale_vote_t left_vote;
        scale_vote_t right_vote;

        // 슬롯마다 그 카메라가 찍는 다리의 관절만 받는다. 골반 태그는 그 다리의 hip 으로 들어온다.
        for (const auto& slot : frameset.slots)
        {
            const joint_side_t leg = viewed_leg_of(slot.view).value();
            scale_vote_t& vote = (leg == joint_side_t::right) ? right_vote : left_vote;

            for (const auto& m : slot.measurements)
            {
                if (get_joint_side(m.joint_id) != leg) { continue; }

                const size_t i = index_of(m.joint_id);
                _ctx->last_frame_raw_px[i] = m.center_px;
                _ctx->last_frame_detection_flags[i] = true;

                const std::optional<joint_definition_t> def = get_joint_def(m.joint_id);
                if (m.meters_per_pixel.has_value()
                    && def.has_value()
                    && vote.voted_tags.insert(def->tag_id).second)
                {
                    vote.sum += m.meters_per_pixel.value();
                    ++vote.count;
                }
            }
        }

        // Nothing measured this frame: keep the last factor so held points stay where they were.
        if (const std::optional<double> average = left_vote.get_average()) { _ctx->left_meters_per_pixel = *average; }
        if (const std::optional<double> average = right_vote.get_average()) { _ctx->right_meters_per_pixel = *average; }

        // ----- Pass 2: smooth + hold each leg joint's image-plane point -----
        for (const auto& def : get_joint_defs())
        {
            if (def.joint_id == pelvis) { continue; } // 골반 태그의 점은 각 카메라의 hip 이 싣는다

            const size_t i = index_of(def.joint_id);
            auto& fs = _ctx->filter_states[i];

            if (_ctx->last_frame_raw_px[i].has_value())
            {
                const Eigen::Vector2d p = _ctx->last_frame_raw_px[i].value();
                for (auto& lpf : fs.px_smoother) {
                    lpf.set_min_cutoff(_opt.position_filter.min_cutoff_hz);
                    lpf.set_beta(_opt.position_filter.beta);
                    lpf.set_derivate_cutoff(_opt.position_filter.dcutoff_hz);
                }

                // Reseed on cold start / long gap / time running backward / smoothing off; else
                // low-pass each image axis. Time runs backward when a frame from before a seek
                // was stepped after the reset that the seek raised.
                if (!_opt.enable_position_smoothing
                    || !fs.last_px_out.has_value()
                    || t < fs.last_seen
                    || (t - fs.last_seen) > _opt.reset_gap)
                {
                    for (auto& lpf : fs.px_smoother) { lpf.reset(); }
                    fs.last_px_out = p;
                }
                else
                {
                    const seconds_f64 dt = std::clamp(seconds_f64{ t - fs.last_step_time }, _opt.dt_min, _opt.dt_max);
                    Eigen::Vector2d out;
                    for (int a = 0; a < 2; ++a) { out[a] = fs.px_smoother[a].filter(p[a], dt.count()); }
                    fs.last_px_out = out;
                }
                fs.last_step_time = t;
                fs.last_seen = t;

                _ctx->last_frame_px[i] = fs.last_px_out;
            }
            else if (fs.last_px_out.has_value() && t >= fs.last_seen && (t - fs.last_seen) <= _opt.max_hold)
            {
                // No fresh detection: hold the point within the window.
                _ctx->last_frame_px[i] = fs.last_px_out;
            }
        }

        // ----- 위치: 다리마다 그 카메라가 본 골반을 원점으로 -----
        // 두 카메라 사이의 변환이 없으므로 한 다리는 그 카메라의 점·스케일·부호로만 잰다.
        for (const joint_side_t leg : { joint_side_t::left, joint_side_t::right })
        {
            const std::optional<Eigen::Vector2d>& hip_px = _ctx->last_frame_px[index_of(get_leg_root_joint(leg).value())];
            std::optional<Eigen::Vector2d>& pelvis_px = (leg == joint_side_t::right) ? _ctx->right_pelvis_px : _ctx->left_pelvis_px;
            if (hip_px.has_value()) { pelvis_px = hip_px; }
        }
        for (const auto& def : get_joint_defs())
        {
            if (def.joint_id == pelvis) { continue; } // 골반은 아래에서 원점으로 둔다

            const bool right_leg = (def.side == joint_side_t::right);
            const std::optional<Eigen::Vector2d>& pelvis_px = right_leg ? _ctx->right_pelvis_px : _ctx->left_pelvis_px;
            if (!pelvis_px.has_value()) { continue; } // 이 카메라가 아직 골반을 못 봤다
            const double meters_per_pixel = right_leg ? _ctx->right_meters_per_pixel : _ctx->left_meters_per_pixel;
            const double side = side_sign(def.side);

            const size_t i = index_of(def.joint_id);
            joint_state_t& st = _ctx->last_frame_joint_states[i];
            if (_ctx->last_frame_raw_px[i].has_value()) {
                st.raw_position = to_rig_space(_ctx->last_frame_raw_px[i].value() - pelvis_px.value(), meters_per_pixel, side);
            }
            if (_ctx->last_frame_px[i].has_value()) {
                st.position = to_rig_space(_ctx->last_frame_px[i].value() - pelvis_px.value(), meters_per_pixel, side);
            }
        }

        // 골반은 원점이다. 어느 hip 이든 새로 잡혔으면 `raw_position`, 붙잡혀 있으면 `position` 이 선다.
        {
            joint_state_t& pelvis_state = _ctx->last_frame_joint_states[index_of(pelvis)];
            for (const joint_side_t leg : { joint_side_t::left, joint_side_t::right })
            {
                const joint_state_t& hip = _ctx->last_frame_joint_states[index_of(get_leg_root_joint(leg).value())];
                if (hip.raw_position.has_value()) { pelvis_state.raw_position = Eigen::Vector3d::Zero(); }
                if (hip.position.has_value()) { pelvis_state.position = Eigen::Vector3d::Zero(); }
            }
        }

        // ----- Pass 3: rig-sign segment angles -> published angles + rest-relative motion -----
        // The model is laid out at the top of this file. Every angle is read off the smoothed +
        // held image points (`last_frame_px`), so the occlusion policy above applies to the
        // angles as well; a direction is scale free, so the metric approximation never enters
        // any of them.

        // The root's bone is the torso, which stands along the reference axis, so its
        // segment angle is 0 by definition, with no measurement involved. It has no parent
        // bone, hence no clinical angle.
        _ctx->last_frame_joint_states[index_of(pelvis)].sagittal_segment_angle = 0.0;

        for (const joint_side_t leg : { joint_side_t::left, joint_side_t::right })
        {
            auto& states = _ctx->last_frame_joint_states;
            const joint_id_t root = get_root_joint();
            const auto at = [](joint_id_t j) { return index_of(j); };

            const std::optional<joint_id_t> hip = get_leg_root_joint(leg);
            const std::optional<joint_id_t> knee =
                hip.has_value() ? get_child_joint(hip.value()) : std::nullopt;
            const std::optional<joint_id_t> ankle =
                knee.has_value() ? get_child_joint(knee.value()) : std::nullopt;
            const std::optional<joint_id_t> foot =
                ankle.has_value() ? get_child_joint(ankle.value()) : std::nullopt;

            if (hip.has_value() && knee.has_value() && ankle.has_value() && foot.has_value())
            {
                const double side = side_sign(leg);
                const auto& px = _ctx->last_frame_px;

                // --- this frame's rig-sign segment angles, one per bone of this leg ---
                // The hip point rides the shared pelvis tag, so the thigh runs from it. Each is
                // empty where an endpoint is missing, and every output below asks for exactly
                // the segments it is defined on, so one lost tag costs only the angles built on
                // it. The foot joint is a marker end site: it lends the foot bone its far
                // endpoint and carries no angles of its own.
                const std::optional<double> rig_sagittal_segment_ang_thigh = rig_sagittal_segment_angle_from_px(px[at(hip.value())], px[at(knee.value())], side);
                const std::optional<double> rig_sagittal_segment_ang_shin  = rig_sagittal_segment_angle_from_px(px[at(knee.value())], px[at(ankle.value())], side);
                const std::optional<double> rig_sagittal_segment_ang_foot  = rig_sagittal_segment_angle_from_px(px[at(ankle.value())], px[at(foot.value())], side);

                // --- published measured angles (no rest pose involved) ---
                // Carried into the document conventions at this boundary; every per-joint sign
                // and neutral offset lives in the joints_def.hh table.
                if (rig_sagittal_segment_ang_thigh.has_value()) {
                    const std::optional<double> hip_clinical =
                        sagittal_clinical_angle_from_rig_bend(hip.value(), rig_sagittal_segment_ang_thigh.value());
                    states[at(hip.value())].sagittal_segment_angle = sagittal_segment_angle_from_rig(rig_sagittal_segment_ang_thigh.value());
                    states[at(hip.value())].sagittal_clinical_angle = hip_clinical;
                    states[at(hip.value())].sagittal_included_angle = hip_clinical.has_value()
                        ? sagittal_included_angle_from_clinical(hip.value(), hip_clinical.value())
                        : std::nullopt;
                }
                if (rig_sagittal_segment_ang_shin.has_value()) {
                    states[at(knee.value())].sagittal_segment_angle = sagittal_segment_angle_from_rig(rig_sagittal_segment_ang_shin.value());
                    if (rig_sagittal_segment_ang_thigh.has_value()) {
                        const std::optional<double> knee_clinical = sagittal_clinical_angle_from_rig_bend(knee.value(),
                            wrap_pi(rig_sagittal_segment_ang_shin.value() - rig_sagittal_segment_ang_thigh.value()));
                        states[at(knee.value())].sagittal_clinical_angle = knee_clinical;
                        states[at(knee.value())].sagittal_included_angle = knee_clinical.has_value()
                            ? sagittal_included_angle_from_clinical(knee.value(), knee_clinical.value())
                            : std::nullopt;
                    }
                }
                if (rig_sagittal_segment_ang_foot.has_value()) {
                    states[at(ankle.value())].sagittal_segment_angle = sagittal_segment_angle_from_rig(rig_sagittal_segment_ang_foot.value());
                    if (rig_sagittal_segment_ang_shin.has_value()) {
                        const std::optional<double> ankle_clinical = sagittal_clinical_angle_from_rig_bend(ankle.value(),
                            wrap_pi(rig_sagittal_segment_ang_foot.value() - rig_sagittal_segment_ang_shin.value()));
                        states[at(ankle.value())].sagittal_clinical_angle = ankle_clinical;
                        states[at(ankle.value())].sagittal_included_angle = ankle_clinical.has_value()
                            ? sagittal_included_angle_from_clinical(ankle.value(), ankle_clinical.value())
                            : std::nullopt;
                    }
                }

                // --- rest-relative motion ---
                // The change of each rig-sign segment angle since the captured rest, then the
                // same subtraction cascade on the changes for the per-joint bends; the reference
                // axis cancels in each change. All three bones are required on both ends: a rig
                // driven by these rotations needs the whole chain to describe one pose.
                if (this->has_rest_pose())
                {
                    const auto& rest_px = _ctx->rest_pose->joint_px;
                    const std::optional<double> rest_rig_sagittal_segment_ang_thigh = rig_sagittal_segment_angle_from_px(rest_px[at(hip.value())], rest_px[at(knee.value())], side);
                    const std::optional<double> rest_rig_sagittal_segment_ang_shin  = rig_sagittal_segment_angle_from_px(rest_px[at(knee.value())], rest_px[at(ankle.value())], side);
                    const std::optional<double> rest_rig_sagittal_segment_ang_foot  = rig_sagittal_segment_angle_from_px(rest_px[at(ankle.value())], rest_px[at(foot.value())], side);

                    if (rig_sagittal_segment_ang_thigh.has_value() && rig_sagittal_segment_ang_shin.has_value() && rig_sagittal_segment_ang_foot.has_value()
                        && rest_rig_sagittal_segment_ang_thigh.has_value() && rest_rig_sagittal_segment_ang_shin.has_value() && rest_rig_sagittal_segment_ang_foot.has_value())
                    {
                        const double rig_sagittal_segment_delta_thigh = wrap_pi(rig_sagittal_segment_ang_thigh.value() - rest_rig_sagittal_segment_ang_thigh.value());
                        const double rig_sagittal_segment_delta_shin  = wrap_pi(rig_sagittal_segment_ang_shin.value()  - rest_rig_sagittal_segment_ang_shin.value());
                        const double rig_sagittal_segment_delta_foot  = wrap_pi(rig_sagittal_segment_ang_foot.value()  - rest_rig_sagittal_segment_ang_foot.value());

                        const double rig_sagittal_bend_delta_hip   = rig_sagittal_segment_delta_thigh;
                        const double rig_sagittal_bend_delta_knee  = wrap_pi(rig_sagittal_segment_delta_shin - rig_sagittal_segment_delta_thigh);
                        const double rig_sagittal_bend_delta_ankle = wrap_pi(rig_sagittal_segment_delta_foot - rig_sagittal_segment_delta_shin);

                        // The rotation is the rig-sign bend delta put straight onto the shared
                        // hinge axis; the delta fields are the same motion in the document
                        // conventions. Pelvis is the fixed base.
                        states[at(root)].local_anim_rot = Eigen::Quaterniond::Identity();
                        states[at(root)].sagittal_segment_angle_delta = 0.0;
                        states[at(hip.value())].local_anim_rot = Eigen::Quaterniond{
                            Eigen::AngleAxisd{ rig_sagittal_bend_delta_hip, pose_estimator_base::kRigLateralAxis } };
                        states[at(knee.value())].local_anim_rot = Eigen::Quaterniond{
                            Eigen::AngleAxisd{ rig_sagittal_bend_delta_knee, pose_estimator_base::kRigLateralAxis } };
                        states[at(ankle.value())].local_anim_rot = Eigen::Quaterniond{
                            Eigen::AngleAxisd{ rig_sagittal_bend_delta_ankle, pose_estimator_base::kRigLateralAxis } };

                        states[at(hip.value())].sagittal_clinical_angle_delta =
                            sagittal_clinical_angle_delta_from_rig_bend_delta(hip.value(), rig_sagittal_bend_delta_hip);
                        states[at(knee.value())].sagittal_clinical_angle_delta =
                            sagittal_clinical_angle_delta_from_rig_bend_delta(knee.value(), rig_sagittal_bend_delta_knee);
                        states[at(ankle.value())].sagittal_clinical_angle_delta =
                            sagittal_clinical_angle_delta_from_rig_bend_delta(ankle.value(), rig_sagittal_bend_delta_ankle);

                        states[at(hip.value())].sagittal_segment_angle_delta = sagittal_segment_angle_from_rig(rig_sagittal_segment_delta_thigh);
                        states[at(knee.value())].sagittal_segment_angle_delta = sagittal_segment_angle_from_rig(rig_sagittal_segment_delta_shin);
                        states[at(ankle.value())].sagittal_segment_angle_delta = sagittal_segment_angle_from_rig(rig_sagittal_segment_delta_foot);
                    }
                }
            }
        }
    }

    bool sagittal_pose_estimator::calibrate_rest_pose()
    {
        // pelvis를 뺀 관절(양쪽 다리)이 모두 새로 잡혀야 한다. (pelvis 태그는 각 카메라의 hip 태그로 대체, 태그 id는 동일)
        const auto is_missing_for_rest = [this](const joint_definition_t& def) {
            if (def.joint_id == get_root_joint()) { return false; }
            const size_t i = index_of(def.joint_id);
            return !_ctx->last_frame_detection_flags[i] || !_ctx->last_frame_raw_px[i].has_value();
        };
        if (std::ranges::any_of(get_joint_defs(), is_missing_for_rest))
        {
            std::string missing_names; // 거부 로그가 대는 이름들
            for (const auto& def : get_joint_defs()) {
                if (!is_missing_for_rest(def)) { continue; }
                if (!missing_names.empty()) { missing_names += ", "; }
                missing_names += def.name;
            }
            spdlog::warn("estimator: rest pose not captured; [{}] not detected in this frame{}"
                , missing_names
                , this->has_rest_pose() ? ", the previous rest pose stays" : "");
            return false; // the previous reference stays
        }

        context_t::rest_pose_info_t new_rest{};
        for (size_t i = 0; i < kNumJoints; ++i) {
            // Only latch freshly detected joints; a held point is a stale reference.
            if (_ctx->last_frame_detection_flags[i]) {
                new_rest.joint_px[i] = _ctx->last_frame_raw_px[i];
            }
        }
        _ctx->rest_pose = new_rest;
        return true;
    }

    void sagittal_pose_estimator::clear_rest_pose()
    {
        _ctx->rest_pose.reset();
    }

    void sagittal_pose_estimator::reset_tracking()
    {
        // Fresh filters/timers and no held points, so the first frame of the next stream reseeds
        // rather than filtering or holding against a prior stream's point and timestamps.
        _ctx->filter_states = {};
        _ctx->last_frame_joint_states = {};
        _ctx->last_frame_raw_px = {};
        _ctx->last_frame_px = {};
        _ctx->last_frame_detection_flags = {};
        _ctx->left_meters_per_pixel = 0.0;
        _ctx->right_meters_per_pixel = 0.0;
        _ctx->left_pelvis_px.reset();
        _ctx->right_pelvis_px.reset();
    }

    void sagittal_pose_estimator::on_frame_geometry_changed()
    {
        // Both the rest reference and the position track are held in image-plane points, 
        // so a moved ROI leaves them describing pixels that are no longer there.
        this->clear_rest_pose();
        this->reset_tracking();
    }

} // namespace pose
