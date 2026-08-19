#pragma once

/// Where the head camera is pointing, from joints alone.
///
/// Lifted out of the node so `repose_planner_selftest` can assert the ONE number v7 turns
/// on — a nominal stance has waist = 0, so the torso is vertical and the camera
/// recovers its mount angle exactly. That is checkable offline, with no robot
/// and no camera, and it is the whole of the 62.5% -> 95% result.

#include <array>
#include <cmath>

#include "common/math_utils.hpp"
#include "cpp_control/planners/repose/sight.hpp"

namespace cpp_control {
namespace planners {
namespace repose {

using Mat3 = std::array<float, 9>;

/// G1 waist chain, straight off unitree_robots/g1/g1_29dof.xml — the camera is
/// rigid to torso_link, so three hinges are the whole of the kinematics.
inline constexpr float WAIST_ROLL_OFFSET[3] = {-0.0039635f, 0.0f, 0.035f};
inline constexpr float TORSO_OFFSET[3] = {0.0f, 0.0f, 0.019f};

/// Camera mount in torso_link. Defaults are the xml's; config may override.
struct Mount {
  std::array<float, 3> pos{0.05f, 0.0f, 0.4318f};
  std::array<float, 4> quat{0.6533f, 0.2706f, -0.2706f, -0.6533f};  // wxyz
};

inline Mat3 mat_mul(const Mat3& a, const Mat3& b) {
  Mat3 o{};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      o[r * 3 + c] =
          a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
  return o;
}

inline std::array<float, 3> mat_vec(const Mat3& m,
                                    const std::array<float, 3>& v) {
  return {m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
          m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
          m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
}

inline Mat3 rot_x(float a) {
  const float c = std::cos(a), s = std::sin(a);
  return {1, 0, 0, 0, c, -s, 0, s, c};
}
inline Mat3 rot_y(float a) {
  const float c = std::cos(a), s = std::sin(a);
  return {c, 0, s, 0, 1, 0, -s, 0, c};
}
inline Mat3 rot_z(float a) {
  const float c = std::cos(a), s = std::sin(a);
  return {c, -s, 0, s, c, 0, 0, 0, 1};
}

inline Mat3 mat_from_quat(const std::array<float, 4>& q) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
          2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
          2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
}

/// Camera pose in the robot's GRAVITY-ALIGNED base frame: the IMU with its
/// heading removed, then the waist chain, then the fixed mount. Pure
/// kinematics — no odometry anywhere, because the tokenizer never reads a
/// reference position and yaw drift cancels in every difference (docs §1).
///
/// `waist_q` is {yaw, roll, pitch}, in that joint order.
inline CameraPose camera_pose(const std::array<float, 4>& imu_quat,
                              const std::array<float, 3>& waist_q,
                              const Mount& mount) {
  const Mat3 R_grav = mat_from_quat(
      math::qmul(math::qinv(math::heading_quat(imu_quat)), imu_quat));
  const Mat3 R_wy = rot_z(waist_q[0]);
  const Mat3 R_wr = mat_mul(R_wy, rot_x(waist_q[1]));
  const Mat3 R_pt = mat_mul(R_wr, rot_y(waist_q[2]));
  std::array<float, 3> p_pt = mat_vec(
      R_wy, {WAIST_ROLL_OFFSET[0], WAIST_ROLL_OFFSET[1], WAIST_ROLL_OFFSET[2]});
  const auto p_torso =
      mat_vec(R_wr, {TORSO_OFFSET[0], TORSO_OFFSET[1], TORSO_OFFSET[2]});
  for (int i = 0; i < 3; ++i) p_pt[i] += p_torso[i];

  CameraPose out;
  out.R = mat_mul(R_grav, mat_mul(R_pt, mat_from_quat(mount.quat)));
  const auto cam_in_pelvis = mat_vec(R_pt, mount.pos);
  out.t = mat_vec(R_grav, {p_pt[0] + cam_in_pelvis[0], p_pt[1] + cam_in_pelvis[1],
                           p_pt[2] + cam_in_pelvis[2]});
  return out;
}

/// Where the optical axis points, in the gravity-aligned base frame: degrees
/// BELOW horizontal, and degrees off the robot's forward axis. Camera convention
/// is MuJoCo's — +x right, +y up, looks down -z — the same one `build_rays` uses.
struct Gaze {
  float pitch_deg = 0.f;  ///< below horizontal; the mount is 45
  float yaw_deg = 0.f;    ///< off straight ahead
};

inline Gaze gaze_of(const CameraPose& cam) {
  const std::array<float, 3> axis{-cam.R[2], -cam.R[5], -cam.R[8]};  // R * -z
  constexpr float kDeg = 180.0f / static_cast<float>(M_PI);
  return {-std::asin(std::max(-1.0f, std::min(1.0f, axis[2]))) * kDeg,
          std::atan2(axis[1], axis[0]) * kDeg};
}

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
