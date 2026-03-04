#include "cpp_control/tasks/locomanip/base.hpp"

namespace cpp_control
{

LocoManipBase::LocoManipBase(const std::string& node_name) : G1BaseNode(node_name)
{
    // Initialize robot (Level 0+1) now that vtable is ready
    init();

    // ── ONNX policy ──
    std::string onnx_path = this->declare_parameter("onnx_model_path", "");
    if (onnx_path.empty() && config_)
        onnx_path = config_->onnx_path;

    if (!onnx_path.empty())
    {
        policy_ = std::make_unique<ONNXPolicy>(onnx_path);
        RCLCPP_INFO(this->get_logger(), "LocoManip policy: %s", onnx_path.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No ONNX model path for locomanip");
    }

    // ── XR hand-pose subscriptions ──
    std::string left_hand_topic  = "/xr/left_hand_pose";
    std::string right_hand_topic = "/xr/right_hand_pose";

    left_hand_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        left_hand_topic, 10,
        [this](geometry_msgs::msg::Pose::SharedPtr msg) { this->left_hand_callback(msg); });

    right_hand_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        right_hand_topic, 10,
        [this](geometry_msgs::msg::Pose::SharedPtr msg) { this->right_hand_callback(msg); });
}

// ── XR Callbacks ──────────────────────────────────────────────

void LocoManipBase::left_hand_callback(geometry_msgs::msg::Pose::SharedPtr msg)
{
    left_hand_pos_  = {static_cast<float>(msg->position.x),
                       static_cast<float>(msg->position.y),
                       static_cast<float>(msg->position.z)};
    left_hand_quat_ = {static_cast<float>(msg->orientation.w),
                       static_cast<float>(msg->orientation.x),
                       static_cast<float>(msg->orientation.y),
                       static_cast<float>(msg->orientation.z)};
}

void LocoManipBase::right_hand_callback(geometry_msgs::msg::Pose::SharedPtr msg)
{
    right_hand_pos_  = {static_cast<float>(msg->position.x),
                        static_cast<float>(msg->position.y),
                        static_cast<float>(msg->position.z)};
    right_hand_quat_ = {static_cast<float>(msg->orientation.w),
                        static_cast<float>(msg->orientation.x),
                        static_cast<float>(msg->orientation.y),
                        static_cast<float>(msg->orientation.z)};
}

// ── Observation ───────────────────────────────────────────────

std::vector<float> LocoManipBase::build_observation()
{
    int n = num_motors();
    int num_act = config_ ? config_->num_actions : n;
    std::vector<float> obs;
    obs.reserve(config_ ? static_cast<size_t>(config_->num_obs) : 110);

    // Angular velocity (body frame)
    for (int i = 0; i < 3; ++i)
        obs.push_back(robot_state_.imu_gyroscope[i]);

    // Projected gravity
    auto pg = math::get_projected_gravity(robot_state_.imu_quaternion);
    for (int i = 0; i < 3; ++i)
        obs.push_back(pg[i]);

    // Command velocity (locomanip-specific)
    for (int i = 0; i < 3; ++i)
        obs.push_back(locomanip_cmd_vel_[i]);

    // Left hand pose
    for (int i = 0; i < 3; ++i)
        obs.push_back(left_hand_pos_[i]);
    for (int i = 0; i < 4; ++i)
        obs.push_back(left_hand_quat_[i]);

    // Right hand pose
    for (int i = 0; i < 3; ++i)
        obs.push_back(right_hand_pos_[i]);
    for (int i = 0; i < 4; ++i)
        obs.push_back(right_hand_quat_[i]);

    // Joint positions relative to default
    for (int i = 0; i < n; ++i)
        obs.push_back(robot_state_.joint_positions[i] - default_angles_[i]);

    // Joint velocities
    for (int i = 0; i < n; ++i)
        obs.push_back(robot_state_.joint_velocities[i]);

    // Last action
    for (int i = 0; i < num_act; ++i)
        obs.push_back(last_actions_[i]);

    return obs;
}

// ── Policy ────────────────────────────────────────────────────

RobotCommand LocoManipBase::policy_control()
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

// ── Velocity from Joystick (trigger-based) ────────────────────

void LocoManipBase::on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->axes.size() > joy::XMODE_R1)
    {
        locomanip_cmd_vel_[0] =
            static_cast<float>(msg->axes[joy::XMODE_RIGHT_JOY_UP_DOWN]) * 0.5f;
        locomanip_cmd_vel_[1] =
            static_cast<float>(msg->axes[joy::XMODE_LEFT_JOY_LEFT_RIGHT]) * 0.5f;

        float left_trigger  = static_cast<float>(msg->axes[joy::XMODE_L1]);
        float right_trigger = static_cast<float>(msg->axes[joy::XMODE_R1]);
        locomanip_cmd_vel_[2] = left_trigger - right_trigger;
    }
}

// ── Velocity from Gamepad ─────────────────────────────────────

void LocoManipBase::on_gamepad()
{
    locomanip_cmd_vel_[0] = gamepad_.ly * 0.5f;
    locomanip_cmd_vel_[1] = gamepad_.lx * 0.5f;
    locomanip_cmd_vel_[2] = gamepad_.rx * 0.5f;
}

}  // namespace cpp_control
