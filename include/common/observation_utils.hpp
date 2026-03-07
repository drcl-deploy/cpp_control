#pragma once

#include "common/types.hpp"
#include "common/math_utils.hpp"
#include <array>
#include <vector>

namespace cpp_control
{
namespace obs
{

// ── IMU Observations ──────────────────────────────────────────

/// Append angular velocity (gyroscope) to observation vector
inline void append_gyro(std::vector<float>& obs, const RobotState& state)
{
    for (int i = 0; i < 3; ++i)
        obs.push_back(state.imu_gyroscope[i]);
}

/// Append projected gravity to observation vector
inline void append_projected_gravity(std::vector<float>& obs, const RobotState& state)
{
    auto pg = math::get_projected_gravity(state.imu_quaternion);
    for (int i = 0; i < 3; ++i)
        obs.push_back(pg[i]);
}

/// Append both gyro and projected gravity (common IMU obs)
inline void append_imu_obs(std::vector<float>& obs, const RobotState& state)
{
    append_gyro(obs, state);
    append_projected_gravity(obs, state);
}

// ── Command ──────────────────────────────────────────────────

/// Append 3D command velocity to observation vector
inline void append_cmd_vel(std::vector<float>& obs, const std::array<float, 3>& cmd_vel)
{
    for (int i = 0; i < 3; ++i)
        obs.push_back(cmd_vel[i]);
}

/// Append generic 3D command to observation vector
inline void append_cmd(std::vector<float>& obs, const std::array<float, 3>& cmd)
{
    for (int i = 0; i < 3; ++i)
        obs.push_back(cmd[i]);
}

// ── Joint Observations ────────────────────────────────────────

/// Append joint positions relative to default angles
inline void append_joint_pos_rel(std::vector<float>& obs,
                                  const RobotState& state,
                                  const std::vector<float>& default_angles)
{
    int n = static_cast<int>(default_angles.size());
    for (int i = 0; i < n; ++i)
        obs.push_back(state.joint_positions[i] - default_angles[i]);
}

/// Append joint velocities
inline void append_joint_vel(std::vector<float>& obs, const RobotState& state, int n)
{
    for (int i = 0; i < n; ++i)
        obs.push_back(state.joint_velocities[i]);
}

/// Append both joint positions (relative) and velocities
inline void append_joint_obs(std::vector<float>& obs,
                              const RobotState& state,
                              const std::vector<float>& default_angles)
{
    append_joint_pos_rel(obs, state, default_angles);
    append_joint_vel(obs, state, static_cast<int>(default_angles.size()));
}

// ── Action History ────────────────────────────────────────────

/// Append last actions to observation vector
inline void append_last_actions(std::vector<float>& obs,
                                 const std::vector<float>& last_actions,
                                 int num_actions)
{
    for (int i = 0; i < num_actions; ++i)
        obs.push_back(last_actions[i]);
}

/// Append last actions (all)
inline void append_last_actions(std::vector<float>& obs,
                                 const std::vector<float>& last_actions)
{
    for (auto a : last_actions)
        obs.push_back(a);
}

// ── Hand Pose (for manipulation tasks) ────────────────────────

/// Append 3D position
inline void append_position(std::vector<float>& obs, const std::array<float, 3>& pos)
{
    for (int i = 0; i < 3; ++i)
        obs.push_back(pos[i]);
}

/// Append quaternion (w, x, y, z)
inline void append_quaternion(std::vector<float>& obs, const std::array<float, 4>& quat)
{
    for (int i = 0; i < 4; ++i)
        obs.push_back(quat[i]);
}

/// Append pose (position + quaternion)
inline void append_pose(std::vector<float>& obs,
                         const std::array<float, 3>& pos,
                         const std::array<float, 4>& quat)
{
    append_position(obs, pos);
    append_quaternion(obs, quat);
}

// ── Orientation ───────────────────────────────────────────────

/// Append 6D rotation matrix representation (first two columns of R from quaternion)
inline void append_rotation_6d(std::vector<float>& obs, const std::array<float, 4>& quat)
{
    auto r6d = math::quat_to_rotation_6d(quat);
    for (int i = 0; i < 6; ++i)
        obs.push_back(r6d[i]);
}

}  // namespace obs
}  // namespace cpp_control
