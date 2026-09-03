#include "cpp_control/base.hpp"

#include <algorithm>
#include <cmath>

namespace cpp_control
{

BaseNode::BaseNode(const std::string& node_name) : rclcpp::Node(node_name)
{
    // Load config path from parameter
    std::string config_path = this->declare_parameter("config_path", "");
    if (!config_path.empty())
    {
        config_ = std::make_unique<Config>(config_path);
    }

    prev_buttons_.resize(12, 0);

    // Joystick subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr msg) { this->joy_callback(msg); });

    // NOTE: init_robot() and num_motors() are pure virtual.
    // Derived class must call init() after its own constructor finishes.
}

void BaseNode::init()
{
    if (init_done_) return;
    init_done_ = true;

    // Let Level 1 set up robot-specific things
    init_robot();

    // Size state and command vectors based on robot
    int n = num_motors();
    robot_state_.joint_positions.resize(n, 0.0f);
    robot_state_.joint_velocities.resize(n, 0.0f);
    robot_state_.joint_torques.resize(n, 0.0f);
    actions_.resize(n, 0.0f);
    last_actions_.resize(n, 0.0f);
    pre_nominal_pos_.resize(n, 0.0f);

    // Start control timer — unless Level 1 is going to drive control_loop() off
    // the robot's state stream instead (config state_decimation), which is the
    // only way to hold the physics-per-control-step ratio against a simulator
    // that does not run in real time.
    double dt = config_ ? config_->control_dt : 0.02;
    state_paced_ = config_ && config_->state_decimation > 0;
    if (!state_paced_)
    {
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt * 1000)),
            [this]() { this->control_loop(); });
    }

    RCLCPP_INFO(this->get_logger(), "BaseNode ready: %d motors, dt=%.3f, paced by %s", n, dt,
                state_paced_ ? "the state stream" : "the wall clock");
}

// ── Control Loop ──────────────────────────────────────────────

void BaseNode::control_loop()
{
    // Nothing to say until the robot has said something. See note_state_received().
    // Cheap and unconditional: under state pacing this is already true by
    // construction, and on hardware the first LowState arrives long before
    // anyone presses anything.
    if (!state_received_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "waiting for the first state message before commanding "
                             "anything (no plant yet?)");
        return;
    }

    // First tick with a real robot behind it, and before anything is published.
    // See on_first_state().
    if (!first_state_handled_)
    {
        first_state_handled_ = true;
        on_first_state();
    }

    RobotCommand cmd;
    switch (control_mode_)
    {
        case ControlMode::ZEROING:
            cmd = zeroing_control();
            break;
        case ControlMode::DAMPING:
            cmd = damping_control();
            break;
        case ControlMode::NOMINAL_POSE:
            cmd = nominal_pose_control();
            break;
        case ControlMode::STANDING_UP:
            cmd = standing_up_control();
            break;
        case ControlMode::STAND:
            cmd = stand_control();
            break;
        case ControlMode::POLICY:
            cmd = policy_control();
            break;
    }
    publish_command(cmd);
}

RobotCommand BaseNode::zeroing_control()
{
    RobotCommand cmd;
    cmd.motor_commands.resize(num_motors());
    return cmd;
}

RobotCommand BaseNode::damping_control()
{
    RobotCommand cmd;
    cmd.motor_commands.resize(num_motors());
    for (auto& mc : cmd.motor_commands)
    {
        mc.kd = 5.0f;
    }
    return cmd;
}

RobotCommand BaseNode::nominal_pose_control()
{
    int n = num_motors();
    RobotCommand cmd;
    cmd.motor_commands.resize(n);
    for (int i = 0; i < n; ++i)
    {
        cmd.motor_commands[i].q =
            (1.0f - alpha_) * pre_nominal_pos_[i] + alpha_ * default_angles_[i];
        cmd.motor_commands[i].kp = kps_[i];
        cmd.motor_commands[i].kd = kds_[i];
    }

    float dt = config_ ? static_cast<float>(config_->control_dt) : 0.02f;
    alpha_ += dt / settle_time_;
    alpha_ = std::min(1.0f, alpha_);

    return cmd;
}

RobotCommand BaseNode::standing_up_control()
{
    // Same interpolation as nominal_pose, but auto-transition to POLICY when done
    auto cmd = nominal_pose_control();

    if (alpha_ >= 1.0f)
    {
        control_mode_ = ControlMode::POLICY;
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        if (policy_)
            policy_->reset_memory();
        RCLCPP_INFO(this->get_logger(), "Stand-up complete -> policy");
    }

    return cmd;
}

RobotCommand BaseNode::policy_control()
{
    // Default: zeroing. Level 2 overrides this.
    return zeroing_control();
}

// ── Joystick ──────────────────────────────────────────────────

void BaseNode::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->buttons.size() <= joy::XMODE_Y)
        return;

    auto pressed = [&](size_t idx) -> bool {
        return msg->buttons[idx] == 1 &&
               (idx < prev_buttons_.size() ? prev_buttons_[idx] == 0 : true);
    };

    if (pressed(joy::XMODE_X))
    {
        control_mode_ = ControlMode::NOMINAL_POSE;
        alpha_ = 0.0f;
        for (int i = 0; i < num_motors(); ++i)
            pre_nominal_pos_[i] = robot_state_.joint_positions[i];
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        RCLCPP_INFO(this->get_logger(), "-> nominal_pose");
    }
    else if (pressed(joy::XMODE_A))
    {
        control_mode_ = ControlMode::POLICY;
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        if (policy_)
            policy_->reset_memory();
        RCLCPP_INFO(this->get_logger(), "-> policy");
    }
    else if (pressed(joy::XMODE_B))
    {
        control_mode_ = ControlMode::ZEROING;
        RCLCPP_INFO(this->get_logger(), "-> zeroing");
    }
    else if (pressed(joy::XMODE_Y))
    {
        control_mode_ = ControlMode::DAMPING;
        RCLCPP_INFO(this->get_logger(), "-> damping");
    }
    else if (msg->buttons.size() > joy::XMODE_R1 && pressed(joy::XMODE_R1) && has_stand())
    {
        engage_stand();
        RCLCPP_INFO(this->get_logger(), "-> stand (robot-level SONIC)");
    }

    // Let Level 2 add task-specific joystick behaviour (velocity, etc.)
    on_joy(msg);

    prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());
}

}  // namespace cpp_control
