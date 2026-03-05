#include "cpp_control/tasks/locomotion/base.hpp"
#include <iostream>

namespace cpp_control
{

LocomotionBase::LocomotionBase(const std::string& node_name) : G1BaseNode(node_name)
{
    // Initialize robot (Level 0+1) now that vtable is ready
    init();

    std::string onnx_path = this->declare_parameter("onnx_model_path", "");
    if (onnx_path.empty() && config_)
        onnx_path = config_->onnx_path;

    if (!onnx_path.empty())
    {
        policy_ = std::make_unique<ONNXPolicy>(onnx_path);
        std::cout << "Locomotion policy: " << onnx_path << std::endl;
    }
    else
    {
        std::cerr << "No ONNX model path for locomotion" << std::endl;
    }
}

// ── Observation ───────────────────────────────────────────────

std::vector<float> LocomotionBase::build_observation()
{
    int n = num_motors();
    std::vector<float> obs;
    obs.reserve(3 + 3 + 3 + n + n + n);  // 96

    // Angular velocity (body frame from gyroscope)
    for (int i = 0; i < 3; ++i)
        obs.push_back(robot_state_.imu_gyroscope[i]);
    // print out agular velocity for debugging
    std::cout << "Gyro: [" << robot_state_.imu_gyroscope[0] << ", " << robot_state_.imu_gyroscope[1] << ", " << robot_state_.imu_gyroscope[2] << "]" << std::endl;
    // Projected gravity
    auto pg = math::get_projected_gravity(robot_state_.imu_quaternion);
    for (int i = 0; i < 3; ++i)
        obs.push_back(pg[i]);
    // print out projected gravity for debugging
    std::cout << "Projected Gravity: [" << pg[0] << ", " << pg[1] << ", " << pg[2] << "]" << std::endl;
    // Command velocity
    for (int i = 0; i < 3; ++i)
        obs.push_back(cmd_vel_[i]);
    // print out command velocity for debugging
    std::cout << "Cmd Vel: [" << cmd_vel_[0] << ", " << cmd_vel_[1] << ", " << cmd_vel_[2] << "]" << std::endl;
    // Joint positions relative to default
    for (int i = 0; i < n; ++i)
        obs.push_back(robot_state_.joint_positions[i] - default_angles_[i]);
    // print out joint positions for debugging
    std::string joint_pos_str = "Joint Pos Rel: [";
    for (int i = 0; i < n; ++i)
        joint_pos_str += std::to_string(robot_state_.joint_positions[i] - default_angles_[i]) + (i < n - 1 ? ", " : "]");
    std::cout << joint_pos_str << std::endl;
    // Joint velocities
    for (int i = 0; i < n; ++i)
        obs.push_back(robot_state_.joint_velocities[i]);
    // print out joint velocities for debugging
    std::string joint_vel_str = "Joint Vel: [";
    for (int i = 0; i < n; ++i)
        joint_vel_str += std::to_string(robot_state_.joint_velocities[i]) + (i < n - 1 ? ", " : "]");
    std::cout << joint_vel_str << std::endl;
    // Last action
    for (int i = 0; i < n; ++i)
        obs.push_back(last_actions_[i]);

    // print out last actions for debugging
    std::string last_actions_str = "Last Actions: [";
    for (int i = 0; i < n; ++i)
        last_actions_str += std::to_string(last_actions_[i]) + (i < n - 1 ? ", " : "]");
    std::cout << last_actions_str << std::endl;

    return obs;
}

// ── Policy ────────────────────────────────────────────────────

RobotCommand LocomotionBase::policy_control()
{
    if (!policy_)
        return zeroing_control();

    auto obs = build_observation();
    auto action = policy_->predict(obs);

    int n = num_motors();
    RobotCommand cmd;
    cmd.motor_commands.resize(n);

    for (int i = 0; i < n && i < static_cast<int>(action.size()); ++i)
    {
        actions_[i] = action[i];
        cmd.motor_commands[i].q = default_angles_[i] + action_scale_[i] * action[i];
        cmd.motor_commands[i].kp = kps_[i];
        cmd.motor_commands[i].kd = kds_[i];
    }

    last_actions_ = actions_;
    return cmd;
}

// ── Velocity from Joystick ────────────────────────────────────

void LocomotionBase::on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->axes.size() > joy::XMODE_R1)
    {
        cmd_vel_[0] = static_cast<float>(msg->axes[joy::XMODE_LEFT_JOY_UP_DOWN]) * 0.5f;
        cmd_vel_[1] = static_cast<float>(msg->axes[joy::XMODE_LEFT_JOY_LEFT_RIGHT]) * 0.5f;
        cmd_vel_[2] = static_cast<float>(msg->axes[joy::XMODE_RIGHT_JOY_LEFT_RIGHT]) * 0.5f;
    }
}

// ── Velocity from Gamepad ─────────────────────────────────────

void LocomotionBase::on_gamepad()
{
    cmd_vel_[0] = gamepad_.ly * 0.5f;
    cmd_vel_[1] = gamepad_.lx * 0.5f;
    cmd_vel_[2] = gamepad_.rx * 0.5f;
}

}  // namespace cpp_control
