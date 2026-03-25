#pragma once

#include <array>
#include <cmath>

namespace cpp_control
{
namespace math
{

inline std::array<float, 3> quat_rotate_inverse(const std::array<float, 4>& q,
                                                  const std::array<float, 3>& v)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    // R^T * v
    return {
        (1.f - 2.f * (y * y + z * z)) * v[0] + (2.f * (x * y - w * z)) * v[1] +
            (2.f * (x * z + w * y)) * v[2],
        (2.f * (x * y + w * z)) * v[0] + (1.f - 2.f * (x * x + z * z)) * v[1] +
            (2.f * (y * z - w * x)) * v[2],
        (2.f * (x * z - w * y)) * v[0] + (2.f * (y * z + w * x)) * v[1] +
            (1.f - 2.f * (x * x + y * y)) * v[2]};
}

inline std::array<float, 3> get_projected_gravity(const std::array<float, 4>& quat)
{
    // Must match the exact formula used during policy training (master branch).
    // This is NOT the standard R^T * [0,0,-1]; it uses a specific sign convention
    // from the IsaacLab training environment.
    float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    return {
         2.0f * (-z * x + w * y),
        -2.0f * ( z * y + w * x),
         1.0f - 2.0f * (w * w + z * z)
    };
}

/// Convert quaternion (w,x,y,z) to 6D rotation representation (first two columns of R).
/// Returns [R00, R01, R10, R11, R20, R21] (row-major order).
inline std::array<float, 6> quat_to_rotation_6d(const std::array<float, 4>& q)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    return {
        1.f - 2.f * (y * y + z * z),  // R00
        2.f * (x * y - w * z),         // R01
        2.f * (x * y + w * z),         // R10
        1.f - 2.f * (x * x + z * z),  // R11
        2.f * (x * z - w * y),         // R20
        2.f * (y * z + w * x),         // R21
    };
}

// ── Quaternion arithmetic ─────────────────────────────────────
// Ported from the TextOp tracker task (g1_textop). These follow the wxyz
// convention used throughout IsaacLab / IsaacGym training environments.
// If your task uses a different quaternion order (e.g. xyzw), convert first.

/// Quaternion multiplication (Hamilton product), wxyz convention.
/// Matches IsaacLab's quat_mul: q_result = a * b (rotation b then a).
inline std::array<float, 4> qmul(const std::array<float, 4>& a,
                                  const std::array<float, 4>& b)
{
    // Hamilton product: [w1w2 - v1·v2,  w1*v2 + w2*v1 + v1×v2]
    return {
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    };
}

/// Quaternion inverse (conjugate / norm²), wxyz convention.
/// For unit quaternions this is just the conjugate; the norm² guard handles
/// un-normalized inputs gracefully (clamps near-zero to avoid NaN).
inline std::array<float, 4> qinv(const std::array<float, 4>& q)
{
    float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    float s  = 1.0f / std::max(n2, 1e-9f);
    return {q[0]*s, -q[1]*s, -q[2]*s, -q[3]*s};
}

/// Forward rotation: R(q) * v  (world-frame rotation).
/// This is the complement of quat_rotate_inverse (which applies R^T).
/// Used by the TextOp tracker to transform reference trajectories into
/// the robot's heading frame.
inline std::array<float, 3> quat_rotate(const std::array<float, 4>& q,
                                         const std::array<float, 3>& v)
{
    // Explicit rotation matrix R(q) * v — same matrix elements as
    // quat_rotate_inverse but with the standard (non-transposed) layout.
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float m00 = 1.f - 2.f*(y*y + z*z), m01 = 2.f*(x*y - w*z), m02 = 2.f*(x*z + w*y);
    float m10 = 2.f*(x*y + w*z), m11 = 1.f - 2.f*(x*x + z*z), m12 = 2.f*(y*z - w*x);
    float m20 = 2.f*(x*z - w*y), m21 = 2.f*(y*z + w*x), m22 = 1.f - 2.f*(x*x + y*y);
    return {
        m00*v[0] + m01*v[1] + m02*v[2],
        m10*v[0] + m11*v[1] + m12*v[2],
        m20*v[0] + m21*v[1] + m22*v[2],
    };
}

/// Extract yaw-only quaternion (heading) from a full orientation.
/// Projects the quaternion onto the Z-axis rotation, discarding roll & pitch.
/// Used by the TextOp tracker to align reference motion to the robot's
/// initial heading without tilting the reference frame.
inline std::array<float, 4> heading_quat(const std::array<float, 4>& q)
{
    // atan2 extracts yaw from wxyz quaternion, then rebuild as pure-yaw quat
    float yaw = std::atan2(2.0f * (q[0]*q[3] + q[1]*q[2]),
                           1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]));
    float hy = yaw * 0.5f;
    return {std::cos(hy), 0.0f, 0.0f, std::sin(hy)};
}

// ── Frame math ───────────────────────────────────────────────
// Used by the TextOp tracker to compute relative poses between
// the robot base and reference motion anchors/waypoints.

/// Relative transform: express frame_b in frame_a's coordinate system.
/// Returns {position_b_in_a, orientation_b_in_a}.
/// Equivalent to T_a^{-1} * T_b in SE(3).
inline std::pair<std::array<float, 3>, std::array<float, 4>>
subtract_frames(const std::array<float, 3>& pos_a,
                const std::array<float, 4>& quat_a,
                const std::array<float, 3>& pos_b,
                const std::array<float, 4>& quat_b)
{
    auto q_inv_a = qinv(quat_a);
    auto q_rel   = qmul(q_inv_a, quat_b);
    std::array<float, 3> dp = {pos_b[0] - pos_a[0],
                                pos_b[1] - pos_a[1],
                                pos_b[2] - pos_a[2]};
    auto t_rel = quat_rotate(q_inv_a, dp);
    return {t_rel, q_rel};
}

}  // namespace math
}  // namespace cpp_control
