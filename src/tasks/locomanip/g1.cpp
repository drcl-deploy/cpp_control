#include "cpp_control/tasks/locomanip/g1.hpp"

namespace cpp_control
{

G1LocomanipNode::G1LocomanipNode(const std::string& node_name) : G1Node(node_name)
{
    // Initialize robot (Level 0+1) now that vtable is ready
    init();

    // ── Locomotion policy (stored in base policy_) ───────────────
    std::string loco_onnx = this->declare_parameter("onnx_model_path", "");
    if (loco_onnx.empty() && config_)
        loco_onnx = config_->onnx_path;

    if (!loco_onnx.empty())
    {
        policy_ = std::make_unique<ONNXPolicy>(loco_onnx);
        RCLCPP_INFO(this->get_logger(), "Locomotion policy: %s", loco_onnx.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No ONNX model path for locomotion");
    }

    // ── Locomanip policy ─────────────────────────────────────────
    std::string manip_onnx = this->declare_parameter("locomanip_onnx_path", "");

    if (!manip_onnx.empty())
    {
        locomanip_policy_ = std::make_unique<ONNXPolicy>(manip_onnx);
        RCLCPP_INFO(this->get_logger(), "Locomanip policy: %s", manip_onnx.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No ONNX model path for locomanip");
    }

    // ── XR controller subscriptions ──────────────────────────────
    std::string left_topic = this->declare_parameter("left_xr_topic", "/xr/left_controller");
    std::string right_topic = this->declare_parameter("right_xr_topic", "/xr/right_controller");

    left_xr_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        left_topic, 10,
        [this](geometry_msgs::msg::Pose::SharedPtr msg) { left_xr_callback(msg); });

    right_xr_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        right_topic, 10,
        [this](geometry_msgs::msg::Pose::SharedPtr msg) { right_xr_callback(msg); });

    RCLCPP_INFO(this->get_logger(), "XR subscriptions: left=%s, right=%s",
                left_topic.c_str(), right_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Modes: X=nominal_pose, A=locomanip, LB=locomotion, B=zero, Y=damping");
}

// ── XR Callbacks ─────────────────────────────────────────────

void G1LocomanipNode::left_xr_callback(geometry_msgs::msg::Pose::SharedPtr msg)
{
    left_hand_pos_[0] = static_cast<float>(msg->position.x);
    left_hand_pos_[1] = static_cast<float>(msg->position.y);
    left_hand_pos_[2] = static_cast<float>(msg->position.z) + 0.5f;

    left_hand_quat_[0] = static_cast<float>(msg->orientation.w);
    left_hand_quat_[1] = static_cast<float>(msg->orientation.x);
    left_hand_quat_[2] = static_cast<float>(msg->orientation.y);
    left_hand_quat_[3] = static_cast<float>(msg->orientation.z);
}

void G1LocomanipNode::right_xr_callback(geometry_msgs::msg::Pose::SharedPtr msg)
{
    right_hand_pos_[0] = static_cast<float>(msg->position.x);
    right_hand_pos_[1] = static_cast<float>(msg->position.y);
    right_hand_pos_[2] = static_cast<float>(msg->position.z) + 0.5f;

    right_hand_quat_[0] = static_cast<float>(msg->orientation.w);
    right_hand_quat_[1] = static_cast<float>(msg->orientation.x);
    right_hand_quat_[2] = static_cast<float>(msg->orientation.y);
    right_hand_quat_[3] = static_cast<float>(msg->orientation.z);
}

// ── Observations ─────────────────────────────────────────────

std::vector<float> G1LocomanipNode::build_locomotion_observation()
{
    int n = num_motors();
    std::vector<float> obs;
    obs.reserve(3 + 3 + 3 + n + n + n);  // 96

    obs::append_imu_obs(obs, robot_state_);
    obs::append_cmd_vel(obs, cmd_vel_);
    obs::append_joint_obs(obs, robot_state_, default_angles_);
    obs::append_last_actions(obs, last_actions_, n);

    return obs;
}

std::vector<float> G1LocomanipNode::build_locomanip_observation()
{
    int n = num_motors();
    std::vector<float> obs;
    obs.reserve(3 + 3 + 3 + 3 + 4 + 3 + 4 + n + n + n + 1);  // 111

    obs::append_imu_obs(obs, robot_state_);
    obs::append_cmd_vel(obs, locomanip_cmd_vel_);
    obs::append_pose(obs, left_hand_pos_, left_hand_quat_);
    obs::append_pose(obs, right_hand_pos_, right_hand_quat_);
    obs::append_joint_obs(obs, robot_state_, default_angles_);
    obs::append_last_actions(obs, last_actions_, n);
    obs.push_back(binary_cmd_);

    return obs;
}

// ── Locomotion Policy ────────────────────────────────────────

RobotCommand G1LocomanipNode::policy_control()
{
    if (!policy_)
        return zeroing_control();

    auto obs = build_locomotion_observation();
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

// ── Locomanip Policy ─────────────────────────────────────────

RobotCommand G1LocomanipNode::locomanip_policy_control()
{
    if (!locomanip_policy_)
        return zeroing_control();

    auto obs = build_locomanip_observation();
    auto action = locomanip_policy_->predict(obs);

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

void G1LocomanipNode::on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
{
    // X button → base switches to NOMINAL_POSE; override immediately to locomotion POLICY
    if (just_pressed(msg, joy::XMODE_X) && control_mode_ == ControlMode::NOMINAL_POSE)
    {
        control_mode_ = ControlMode::POLICY;
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        if (policy_)
            policy_->reset_memory();
        RCLCPP_INFO(this->get_logger(), "-> locomotion policy");
    }

    // A button → locomanip policy (overrides base A → POLICY)
    if (just_pressed(msg, joy::XMODE_A))
    {
        control_mode_ = ControlMode::LOCOMANIP_POLICY;
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        if (locomanip_policy_)
            locomanip_policy_->reset_memory();
        RCLCPP_INFO(this->get_logger(), "-> locomanip policy");
    }

    // LB button → locomotion policy
    if (just_pressed(msg, joy::XMODE_LB))
    {
        control_mode_ = ControlMode::POLICY;
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        if (policy_)
            policy_->reset_memory();
        RCLCPP_INFO(this->get_logger(), "-> locomotion policy");
    }

    // vx / vy from left stick
    if (msg->axes.size() > joy::XMODE_LEFT_JOY_LEFT_RIGHT)
    {
        locomanip_cmd_vel_[0] = static_cast<float>(msg->axes[joy::XMODE_LEFT_JOY_UP_DOWN])    * 0.5f;
        locomanip_cmd_vel_[1] = static_cast<float>(msg->axes[joy::XMODE_LEFT_JOY_LEFT_RIGHT]) * 0.5f;
        cmd_vel_[0] = locomanip_cmd_vel_[0];
        cmd_vel_[1] = locomanip_cmd_vel_[1];
    }

    // Yaw rate from right stick (axis 3) for both locomotion and locomanip
    if (msg->axes.size() > joy::XMODE_RIGHT_JOY_LEFT_RIGHT)
    {
        const float yaw = static_cast<float>(msg->axes[joy::XMODE_RIGHT_JOY_LEFT_RIGHT]) * -0.5f;
        cmd_vel_[2] = yaw;
        locomanip_cmd_vel_[2] = yaw;
    }

    // RT (axis 5) rising edge → +0.1 force cmd (clamped to 1.0), 0.5s debounce
    // LT (axis 2) pressed → zero force cmd
    if (msg->axes.size() > joy::XMODE_R1)
    {
        const bool rt_pressed = msg->axes[joy::XMODE_R1] > 0.9f;
        if (rt_pressed && !joy_rt_was_pressed_)
        {
            double now_sec = this->now().seconds();
            if (now_sec - last_force_inc_time_sec_ >= 0.5)
            {
                binary_cmd_ = std::min(binary_cmd_ + 0.1f, 1.0f);
                last_force_inc_time_sec_ = now_sec;
                RCLCPP_INFO(this->get_logger(), "force cmd = %.2f", binary_cmd_);
            }
        }
        joy_rt_was_pressed_ = rt_pressed;

        if (msg->axes[joy::XMODE_L1] > 0.9f && binary_cmd_ != 0.0f)
        {
            binary_cmd_ = 0.0f;
            RCLCPP_INFO(this->get_logger(), "force cmd = 0");
        }
    }
}

// ── Velocity from Gamepad ─────────────────────────────────────

#ifdef HAS_UNITREE_HG
void G1LocomanipNode::on_gamepad()
{
    // Gamepad A → locomanip policy
    if (gamepad_.A.pressed)
    {
        control_mode_ = ControlMode::LOCOMANIP_POLICY;
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        if (locomanip_policy_)
            locomanip_policy_->reset_memory();
        RCLCPP_INFO(this->get_logger(), "-> locomanip policy");
    }

    // D-pad up → +0.1 to force cmd (clamped to 1.0), with 0.5s debounce between presses
    if (gamepad_.up.on_press)
    {
        double now_sec = this->now().seconds();
        if (now_sec - last_force_inc_time_sec_ >= 0.5)
        {
            binary_cmd_ = std::min(binary_cmd_ + 0.1f, 1.0f);
            last_force_inc_time_sec_ = now_sec;
            RCLCPP_INFO(this->get_logger(), "force cmd = %.2f", binary_cmd_);
        }
    }

    // D-pad down → zero force cmd
    if (gamepad_.down.on_press)
    {
        binary_cmd_ = 0.0f;
        RCLCPP_INFO(this->get_logger(), "force cmd = 0");
    }

    // D-pad left → spread hands apart; D-pad right → bring hands in
    // Step per tick at ~500Hz: 0.0002m → ~0.1m/s while held
    constexpr float kYStep = 0.0002f;
    if (gamepad_.left.pressed)
    {
        left_hand_pos_[1]  = std::min(left_hand_pos_[1]  + kYStep,  0.50f);
        right_hand_pos_[1] = std::max(right_hand_pos_[1] - kYStep, -0.50f);
    }
    if (gamepad_.right.pressed)
    {
        left_hand_pos_[1]  = std::max(left_hand_pos_[1]  - kYStep,  0.05f);
        right_hand_pos_[1] = std::min(right_hand_pos_[1] + kYStep, -0.05f);
    }

    // Only update velocity from gamepad when joystick is not active.
    // on_gamepad fires at ~500Hz; if the joystick is connected its idle values (0)
    // would otherwise overwrite whatever the joystick just wrote.
    if (!joy_is_active())
    {
        cmd_vel_[0] = gamepad_.ly * 0.5f;
        cmd_vel_[1] = gamepad_.lx * -0.5f;
        cmd_vel_[2] = gamepad_.rx * -0.5f;
        locomanip_cmd_vel_[0] = cmd_vel_[0];
        locomanip_cmd_vel_[1] = cmd_vel_[1];
        locomanip_cmd_vel_[2] = gamepad_.rx * -0.5f;
    }
}
#endif

}  // namespace cpp_control

// ── Entry point ───────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1LocomanipNode>());
    rclcpp::shutdown();
    return 0;
}
