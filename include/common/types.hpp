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
    std::array<float, 3> base_pos_w = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> base_lin_vel_w = {0.0f, 0.0f, 0.0f};

    // Full world-frame base pose and twist. Only a source that genuinely has
    // them sets `base_state_valid`: mj_sim's ground truth (G1State.base_pose /
    // base_twist) or a motion-capture system. An IMU does NOT — it has no
    // position, no heading and no world linear velocity — and neither does
    // SportModeState, which carries no orientation. Tasks whose policy was
    // trained on world-frame observations must check the flag rather than read
    // a plausible-looking identity pose.
    std::array<float, 4> base_quat_w = {1.0f, 0.0f, 0.0f, 0.0f};  // w,x,y,z
    std::array<float, 3> base_ang_vel_w = {0.0f, 0.0f, 0.0f};
    bool base_state_valid = false;

    std::vector<float> joint_positions;
    std::vector<float> joint_velocities;
    std::vector<float> joint_torques;

    // Motor temperature in degrees C, the hotter of each motor's two sensors.
    // Zero on any backend that does not report one — every simulator — and a
    // hardware-only quantity by nature. It is here because it is one of the few
    // things that changes WITHIN a hardware session: a G1 whose leg motors have
    // climbed 30 C over twenty runs is not the plant the first run measured, and
    // without the number that shows up as a policy that "got worse".
    std::vector<float> joint_temperature;

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
    STAND,         // robot-level active stand (g1::SonicStand) — RB, when configured
    POLICY
};

}  // namespace cpp_control
