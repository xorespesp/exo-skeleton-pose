#pragma once
#include "joints_def.hh"

#include <cstdint>
#include <optional>
#include <string_view>

namespace pose
{
    // 추정기가 푸는 평면. 세션에 하나다.
    enum class view_plane_t : uint8_t
    {
        frontal,  // camera faces the exo; both legs visible, rig recovered in 3D
        sagittal, // camera at the side; the near leg's flexion lies in the image plane
    };

    constexpr std::string_view view_plane_name(view_plane_t p) {
        return (p == view_plane_t::sagittal) ? "sagittal" : "frontal";
    }

    // 카메라 한 대가 exo 를 보는 자리.
    enum class camera_view_t : uint8_t
    {
        frontal,        // exo 정면에서. 양쪽 다리를 다 본다
        sagittal_left,  // exo의 왼쪽에서. 왼쪽 다리를 찍는다
        sagittal_right, // exo의 오른쪽에서. 오른쪽 다리를 찍는다
    };

    constexpr std::string_view camera_view_name(camera_view_t v) {
        switch (v) {
        case camera_view_t::sagittal_left:  return "sagittal_left";
        case camera_view_t::sagittal_right: return "sagittal_right";
        case camera_view_t::frontal:
        default:                            return "frontal";
        }
    }

    // nullopt if `name` is none of them. The names round-trip through `camera_view_name()`.
    constexpr std::optional<camera_view_t> camera_view_from_name(std::string_view name) {
        if (name == camera_view_name(camera_view_t::frontal))        { return camera_view_t::frontal; }
        if (name == camera_view_name(camera_view_t::sagittal_left))  { return camera_view_t::sagittal_left; }
        if (name == camera_view_name(camera_view_t::sagittal_right)) { return camera_view_t::sagittal_right; }
        return std::nullopt;
    }

    constexpr view_plane_t view_plane_of(camera_view_t v) {
        return (v == camera_view_t::frontal) ? view_plane_t::frontal : view_plane_t::sagittal;
    }

    constexpr bool is_frontal(camera_view_t v)  { return view_plane_of(v) == view_plane_t::frontal; }
    constexpr bool is_sagittal(camera_view_t v) { return view_plane_of(v) == view_plane_t::sagittal; }

    // 이 뷰의 카메라가 찍는 다리. frontal 은 양쪽을 다 보므로 nullopt.
    constexpr std::optional<joint_side_t> viewed_leg_of(camera_view_t v) {
        switch (v) {
        case camera_view_t::sagittal_left:  return joint_side_t::left;
        case camera_view_t::sagittal_right: return joint_side_t::right;
        case camera_view_t::frontal:
        default:                            return std::nullopt;
        }
    }

} // namespace pose
