#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace cpp_control
{

/// Unified internal robot state — robot nodes populate this from their message callbacks
struct RobotState
{
    std::array<float, 4> imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f};  // w,x,y,z
    std::array<float, 3> imu_gyroscope = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> imu_accelerometer = {0.0f, 0.0f, 0.0f};

    // Base linear velocity in world frame (from odometry / SportModeState).
    // Defaults to zero — only populated when a subscriber provides data.
    std::array<float, 3> base_lin_vel_w = {0.0f, 0.0f, 0.0f};

    std::vector<float> joint_positions;
    std::vector<float> joint_velocities;
    std::vector<float> joint_torques;

    uint32_t tick = 0;
};

/// Single motor command
struct MotorCommand
{
    float q = 0.0f;
    float dq = 0.0f;
    float tau = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
};

/// Full robot command
struct RobotCommand
{
    std::vector<MotorCommand> motor_commands;
};

/// Control mode state machine
enum class ControlMode
{
    ZEROING,
    DAMPING,
    NOMINAL_POSE,
    STANDING_UP,   // smooth interpolation to default_angles, then auto → POLICY
    POLICY
};

}  // namespace cpp_control
