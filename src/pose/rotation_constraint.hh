#pragma once
#include <Eigen/Geometry>

#include <cmath>

namespace pose
{
    // ---------------------------------------------------------------------------
    // Swing-twist decomposition
    // ---------------------------------------------------------------------------
    //
    // Any rotation q splits uniquely into a "twist" about a chosen unit axis `a` and a
    // "swing" (the remaining rotation, about an axis orthogonal to a), with q = swing * twist.
    // The twist is the projection of q's vector part onto `a`, normalized.
    // Ref: P. Dobrowolski, "Swing-Twist Decomposition in Clifford Algebra", arXiv:1506.05481
    // https://arxiv.org/abs/1506.05481

    struct swing_twist_t
    {
        Eigen::Quaterniond swing; // rotation about an axis orthogonal to `a`
        Eigen::Quaterniond twist; // rotation purely about `a`
    };

    // Swing-twist decomposition of `q` about unit axis `a`, where q = swing * twist.
    [[nodiscard]] inline swing_twist_t decompose_swing_twist_about_axis(const Eigen::Quaterniond& q, const Eigen::Vector3d& a)
    {
        const Eigen::Vector3d p = a * q.vec().dot(a); // project the vector part onto a
        Eigen::Quaterniond twist{ q.w(), p.x(), p.y(), p.z() };
        const double n = twist.norm();
        if (n < 1e-9) { twist = Eigen::Quaterniond::Identity(); } // ~180 deg swing about an axis _|_ a (singular): no twist
        else { twist.coeffs() /= n; }
        return { q * twist.conjugate(), twist };
    }

    // Swing angle [rad] of `q` about unit axis `a`: the magnitude of the rotation left after
    // removing the twist about `a`. 0 == pure rotation about `a`; larger == more off-axis.
    // Uses the robust quaternion angle form theta = 2*atan2(|vec|, |w|).
    // Ref: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    [[nodiscard]] inline double swing_angle(const Eigen::Quaterniond& q, const Eigen::Vector3d& a)
    {
        const Eigen::Quaterniond swing = decompose_swing_twist_about_axis(q, a).swing;
        return 2.0 * std::atan2(swing.vec().norm(), std::abs(swing.w()));
    }

    // Signed twist angle [rad] of `q` about unit axis `a`: theta = 2*atan2(q.vec . a, q.w).
    // Ref: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    [[nodiscard]] inline double twist_angle(const Eigen::Quaterniond& q, const Eigen::Vector3d& a)
    {
        return 2.0 * std::atan2(q.vec().dot(a), q.w());
    }

} // namespace pose
