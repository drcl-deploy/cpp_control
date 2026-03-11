#include "cpp_control/robots/mini_pi.hpp"

namespace cpp_control
{

MiniPiNode::MiniPiNode(const std::string& node_name) : BaseNode(node_name) {}

void MiniPiNode::init_robot()
{
    // ── Load robot constants from config ──
    if (config_)
    {
        joint_names_ = config_->joint_names;
        default_angles_.resize(MINI_PI_NUM_MOTOR);
        kps_.resize(MINI_PI_NUM_MOTOR);
        kds_.resize(MINI_PI_NUM_MOTOR);
        action_scale_.resize(MINI_PI_NUM_MOTOR);
        for (int i = 0; i < MINI_PI_NUM_MOTOR; ++i)
        {
            default_angles_[i] = static_cast<float>(config_->default_angles[i]);
            kps_[i] = static_cast<float>(config_->kps[i]);
            kds_[i] = static_cast<float>(config_->kds[i]);
            action_scale_[i] = static_cast<float>(config_->action_scale[i]);
        }
    }
    else
    {
        // Hardcoded fallback (joint order matches JOINT_NAMES_EXPR in constants.py)
        joint_names_ = {
            "l_ankle_roll_joint",  "l_ankle_pitch_joint",
            "l_calf_joint",        "l_thigh_joint",
            "l_hip_roll_joint",    "l_hip_pitch_joint",
            "r_ankle_roll_joint",  "r_ankle_pitch_joint",
            "r_calf_joint",        "r_thigh_joint",
            "r_hip_roll_joint",    "r_hip_pitch_joint"};
        default_angles_ = {
            0.0f, -0.4f, 0.65f, 0.0f, 0.0f, -0.25f,
            0.0f, -0.4f, 0.65f, 0.0f, 0.0f, -0.25f};
        kps_.assign(MINI_PI_NUM_MOTOR, 15.35f);
        kds_.assign(MINI_PI_NUM_MOTOR, 0.98f);
        action_scale_.assign(MINI_PI_NUM_MOTOR, 0.26f);
    }

    // ── drcl_deploy backend ──
    std::string cmd_topic = config_ ? config_->lowcmd_topic : "/robot_command";
    std::string state_topic = config_ ? config_->lowstate_topic : "/robot_state";

    cmd_pub_ = this->create_publisher<messages::msg::MiniPiCommand>(cmd_topic, 10);
    state_sub_ = this->create_subscription<messages::msg::MiniPiState>(
        state_topic, 10,
        [this](messages::msg::MiniPiState::SharedPtr msg) { this->subscribe_mini_pi_state(msg); });

    RCLCPP_INFO(this->get_logger(), "MiniPi init_robot: %d motors", MINI_PI_NUM_MOTOR);
}

// ══════════════════════════════════════════════════════════════
//  State handler
// ══════════════════════════════════════════════════════════════

void MiniPiNode::subscribe_mini_pi_state(messages::msg::MiniPiState::SharedPtr msg)
{
    // IMU
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
    for (int i = 0; i < MINI_PI_NUM_MOTOR; ++i)
    {
        robot_state_.joint_positions[i] = msg->motor_state[i].q;
        robot_state_.joint_velocities[i] = msg->motor_state[i].dq;
        robot_state_.joint_torques[i] = msg->motor_state[i].tauest;
    }
}

// ══════════════════════════════════════════════════════════════
//  Command publisher
// ══════════════════════════════════════════════════════════════

void MiniPiNode::publish_command(const RobotCommand& cmd)
{
    messages::msg::MiniPiCommand msg;
    for (int i = 0; i < MINI_PI_NUM_MOTOR && i < static_cast<int>(cmd.motor_commands.size()); ++i)
    {
        msg.motor_command[i].q = cmd.motor_commands[i].q;
        msg.motor_command[i].dq = cmd.motor_commands[i].dq;
        msg.motor_command[i].tau = cmd.motor_commands[i].tau;
        msg.motor_command[i].kp = cmd.motor_commands[i].kp;
        msg.motor_command[i].kd = cmd.motor_commands[i].kd;
    }
    cmd_pub_->publish(msg);
}

}  // namespace cpp_control
