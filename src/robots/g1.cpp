#include "cpp_control/robots/g1.hpp"

#include <cstring>

namespace cpp_control
{

G1Node::G1Node(const std::string& node_name) : BaseNode(node_name) {}

void G1Node::init_robot()
{
    // ── Load robot constants from config ──
    if (config_)
    {
        joint_names_ = config_->joint_names;
        default_angles_.resize(G1_NUM_MOTOR);
        kps_.resize(G1_NUM_MOTOR);
        kds_.resize(G1_NUM_MOTOR);
        action_scale_.resize(G1_NUM_MOTOR);
        for (int i = 0; i < G1_NUM_MOTOR; ++i)
        {
            default_angles_[i] = static_cast<float>(config_->default_angles[i]);
            kps_[i] = static_cast<float>(config_->kps[i]);
            kds_[i] = static_cast<float>(config_->kds[i]);
            action_scale_[i] = static_cast<float>(config_->action_scale[i]);
        }
        if (config_->workflow == "drcl_deploy")
            workflow_ = Workflow::DRCL_DEPLOY;
        else
            workflow_ = Workflow::UNITREE;
    }
    else
    {
        // Hardcoded fallback
        joint_names_ = {
            "left_hip_pitch_joint",      "left_hip_roll_joint",
            "left_hip_yaw_joint",        "left_knee_joint",
            "left_ankle_pitch_joint",    "left_ankle_roll_joint",
            "right_hip_pitch_joint",     "right_hip_roll_joint",
            "right_hip_yaw_joint",       "right_knee_joint",
            "right_ankle_pitch_joint",   "right_ankle_roll_joint",
            "waist_yaw_joint",           "waist_roll_joint",
            "waist_pitch_joint",         "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",  "left_shoulder_yaw_joint",
            "left_elbow_joint",          "left_wrist_roll_joint",
            "left_wrist_pitch_joint",    "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint","right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",  "right_elbow_joint",
            "right_wrist_roll_joint",    "right_wrist_pitch_joint",
            "right_wrist_yaw_joint"};
        default_angles_ = {
            -0.1f, 0.f, 0.f, 0.3f, -0.2f, 0.f,
            -0.1f, 0.f, 0.f, 0.3f, -0.2f, 0.f,
            0.f,   0.f, 0.f,
            0.2f,  0.2f,  0.f, 0.6f, 0.f, 0.f, 0.f,
            0.2f, -0.2f,  0.f, 0.6f, 0.f, 0.f, 0.f};
        kps_.assign(G1_NUM_MOTOR, 100.0f);
        kds_.assign(G1_NUM_MOTOR, 3.0f);
        action_scale_.assign(G1_NUM_MOTOR, 0.5f);
    }

    // ── Workflow-based backend init ──
    switch (workflow_)
    {
    case Workflow::DRCL_DEPLOY:
        init_drcl_deploy();
        break;
    case Workflow::UNITREE:
    default:
        init_unitree();
        break;
    }

    RCLCPP_INFO(this->get_logger(), "G1 init_robot: workflow=%s",
                config_ ? config_->workflow.c_str() : "unitree");
}

// ══════════════════════════════════════════════════════════════
//  Unitree HG backend
// ══════════════════════════════════════════════════════════════

void G1Node::init_unitree()
{
    std::string lowcmd_topic = config_ ? config_->lowcmd_topic : "/lowcmd";
    std::string lowstate_topic = config_ ? config_->lowstate_topic : "/lowstate";

    almi_ctrl::init_cmd_hg(low_cmd_hg_, mode_machine_, mode_pr_);

    lowcmd_pub_hg_ = this->create_publisher<unitree_hg::msg::LowCmd>(lowcmd_topic, 10);
    lowstate_sub_hg_ = this->create_subscription<unitree_hg::msg::LowState>(
        lowstate_topic, 10,
        [this](unitree_hg::msg::LowState::SharedPtr msg) { this->subscribe_low_state(msg); });
}

void G1Node::subscribe_low_state(unitree_hg::msg::LowState::SharedPtr msg)
{
    // IMU
    robot_state_.imu_quaternion = {
        msg->imu_state.quaternion[0], msg->imu_state.quaternion[1],
        msg->imu_state.quaternion[2], msg->imu_state.quaternion[3]};
    robot_state_.imu_gyroscope = {
        msg->imu_state.gyroscope[0], msg->imu_state.gyroscope[1],
        msg->imu_state.gyroscope[2]};
    robot_state_.imu_accelerometer = {
        msg->imu_state.accelerometer[0], msg->imu_state.accelerometer[1],
        msg->imu_state.accelerometer[2]};

    // Joints
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        robot_state_.joint_positions[i] = msg->motor_state[i].q;
        robot_state_.joint_velocities[i] = msg->motor_state[i].dq;
        robot_state_.joint_torques[i] = msg->motor_state[i].tau_est;
    }
    robot_state_.tick = msg->tick;

    // Gamepad (mode switching + let Level 2 read velocities)
    handle_gamepad(*msg);
}

void G1Node::handle_gamepad(const unitree_hg::msg::LowState& msg)
{
    memcpy(gamepad_rx_.buff, msg.wireless_remote.data(), 40);
    gamepad_.update(gamepad_rx_.RF_RX);

    // Mode switching only — velocity mapping is task-specific (Level 2)
    if (gamepad_.B.on_press)
    {
        control_mode_ = ControlMode::ZEROING;
        RCLCPP_INFO(this->get_logger(), "[GP] -> zeroing");
    }
    if (gamepad_.Y.on_press)
    {
        control_mode_ = ControlMode::DAMPING;
        RCLCPP_INFO(this->get_logger(), "[GP] -> damping");
    }
    if (gamepad_.X.on_press)
    {
        control_mode_ = ControlMode::NOMINAL_POSE;
        alpha_ = 0.0f;
        for (int i = 0; i < G1_NUM_MOTOR; ++i)
            pre_nominal_pos_[i] = robot_state_.joint_positions[i];
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        RCLCPP_INFO(this->get_logger(), "[GP] -> nominal_pose");
    }
    if (gamepad_.up.on_press || gamepad_.A.on_press)
    {
        control_mode_ = ControlMode::POLICY;
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        if (policy_)
            policy_->reset_memory();
        RCLCPP_INFO(this->get_logger(), "[GP] -> policy");
    }

    on_gamepad();
}

void G1Node::publish_low_cmd(const RobotCommand& cmd)
{
    for (int i = 0; i < G1_NUM_MOTOR && i < static_cast<int>(cmd.motor_commands.size()); ++i)
    {
        low_cmd_hg_.motor_cmd[i].q = cmd.motor_commands[i].q;
        low_cmd_hg_.motor_cmd[i].dq = cmd.motor_commands[i].dq;
        low_cmd_hg_.motor_cmd[i].tau = cmd.motor_commands[i].tau;
        low_cmd_hg_.motor_cmd[i].kp = cmd.motor_commands[i].kp;
        low_cmd_hg_.motor_cmd[i].kd = cmd.motor_commands[i].kd;
    }
    get_crc(low_cmd_hg_);
    lowcmd_pub_hg_->publish(low_cmd_hg_);
}

// ══════════════════════════════════════════════════════════════
//  drcl_deploy backend (G1State / G1Command)
// ══════════════════════════════════════════════════════════════

void G1Node::init_drcl_deploy()
{
    std::string lowcmd_topic = config_ ? config_->lowcmd_topic : "/lowcmd";
    std::string lowstate_topic = config_ ? config_->lowstate_topic : "/lowstate";

    lowcmd_pub_drcl_ = this->create_publisher<messages::msg::G1Command>(lowcmd_topic, 10);
    lowstate_sub_drcl_ = this->create_subscription<messages::msg::G1State>(
        lowstate_topic, 10,
        [this](messages::msg::G1State::SharedPtr msg) { this->subscribe_g1_state(msg); });
}

void G1Node::subscribe_g1_state(messages::msg::G1State::SharedPtr msg)
{
    // IMU (G1State has IMU[1])
    robot_state_.imu_quaternion = {
        msg->imu[0].quaternion[0], msg->imu[0].quaternion[1],
        msg->imu[0].quaternion[2], msg->imu[0].quaternion[3]};
    robot_state_.imu_gyroscope = {
        msg->imu[0].gyroscope[0], msg->imu[0].gyroscope[1],
        msg->imu[0].gyroscope[2]};
    robot_state_.imu_accelerometer = {
        msg->imu[0].accelerometer[0], msg->imu[0].accelerometer[1],
        msg->imu[0].accelerometer[2]};

    // Joints
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        robot_state_.joint_positions[i] = msg->motor_state[i].q;
        robot_state_.joint_velocities[i] = msg->motor_state[i].dq;
        robot_state_.joint_torques[i] = msg->motor_state[i].tauest;
    }


}

void G1Node::publish_g1_command(const RobotCommand& cmd)
{
    messages::msg::G1Command msg;
    for (int i = 0; i < G1_NUM_MOTOR && i < static_cast<int>(cmd.motor_commands.size()); ++i)
    {
        msg.motor_command[i].q = cmd.motor_commands[i].q;
        msg.motor_command[i].dq = cmd.motor_commands[i].dq;
        msg.motor_command[i].tau = cmd.motor_commands[i].tau;
        msg.motor_command[i].kp = cmd.motor_commands[i].kp;
        msg.motor_command[i].kd = cmd.motor_commands[i].kd;
    }
    lowcmd_pub_drcl_->publish(msg);
}

// ══════════════════════════════════════════════════════════════
//  publish_command — dispatches to active backend
// ══════════════════════════════════════════════════════════════

void G1Node::publish_command(const RobotCommand& cmd)
{
    switch (workflow_)
    {
    case Workflow::DRCL_DEPLOY:
        publish_g1_command(cmd);
        break;
    case Workflow::UNITREE:
    default:
        publish_low_cmd(cmd);
        break;
    }
}

}  // namespace cpp_control
