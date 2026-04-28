#include "cpp_control/robots/hector_v2.hpp"

namespace cpp_control
{

HectorV2Node::HectorV2Node(const std::string& node_name) : BaseNode(node_name) {}

void HectorV2Node::init_robot()
{
    // ── Load robot constants from config ──
    if (config_)
    {
        joint_names_ = config_->joint_names;
        default_angles_.resize(HECTOR_V2_NUM_MOTOR);
        kps_.resize(HECTOR_V2_NUM_MOTOR);
        kds_.resize(HECTOR_V2_NUM_MOTOR);
        action_scale_.resize(HECTOR_V2_NUM_MOTOR);
        for (int i = 0; i < HECTOR_V2_NUM_MOTOR; ++i)
        {
            default_angles_[i] = static_cast<float>(config_->default_angles[i]);
            kps_[i] = static_cast<float>(config_->kps[i]);
            kds_[i] = static_cast<float>(config_->kds[i]);
            action_scale_[i] = static_cast<float>(config_->action_scale[i]);
        }
    }
    else
    {
        // Hardcoded fallback (JOINT_NAMES_EXPR order from constants.py)
        joint_names_ = {
            "l_hip_yaw",   "l_hip_roll",   "l_hip_pitch",   "l_knee",   "l_ankle",
            "r_hip_yaw",   "r_hip_roll",   "r_hip_pitch",   "r_knee",   "r_ankle",
            "l_shoulder_yaw",   "l_shoulder_pitch",   "l_shoulder_roll",   "l_elbow",
            "r_shoulder_yaw",   "r_shoulder_pitch",   "r_shoulder_roll",   "r_elbow"};
        default_angles_ = {
            0.0f,    0.0f,    0.7848f, -1.57f,  0.7848f,
            0.0f,    0.0f,    0.7848f, -1.57f,  0.7848f,
            0.0f,    0.6f,    0.0f,   -1.57f,
            0.0f,    0.6f,    0.0f,   -1.57f};
        // Leg/ankle: A1 gains; knee: A1_KNEE; arm: RS01; elbow: RS01_ELBOW
        kps_ = {31.9775f, 31.9775f, 31.9775f, 127.9101f, 31.9775f,
                31.9775f, 31.9775f, 31.9775f, 127.9101f, 31.9775f,
                11.8559f, 11.8559f, 11.8559f,  26.6757f,
                11.8559f, 11.8559f, 11.8559f,  26.6757f};
        kds_ = {2.0358f, 2.0358f, 2.0358f, 8.1430f, 2.0358f,
                2.0358f, 2.0358f, 2.0358f, 8.1430f, 2.0358f,
                0.7548f, 0.7548f, 0.7548f, 1.6982f,
                0.7548f, 0.7548f, 0.7548f, 1.6982f};
        action_scale_.assign(HECTOR_V2_NUM_MOTOR, 0.5f);
    }

    // ── drcl_deploy backend ──
    std::string cmd_topic   = config_ ? config_->lowcmd_topic   : "/robot_command";
    std::string state_topic = config_ ? config_->lowstate_topic : "/robot_state";

    cmd_pub_   = this->create_publisher<messages::msg::HectorV2Command>(cmd_topic, 10);
    state_sub_ = this->create_subscription<messages::msg::HectorV2State>(
        state_topic, 10,
        [this](messages::msg::HectorV2State::SharedPtr msg) {
            this->subscribe_hector_v2_state(msg);
        });

    RCLCPP_INFO(this->get_logger(), "HectorV2 init_robot: %d motors", HECTOR_V2_NUM_MOTOR);
}

// ══════════════════════════════════════════════════════════════
//  State handler
// ══════════════════════════════════════════════════════════════

void HectorV2Node::subscribe_hector_v2_state(messages::msg::HectorV2State::SharedPtr msg)
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

    // Joints (18 motors)
    for (int i = 0; i < HECTOR_V2_NUM_MOTOR; ++i)
    {
        robot_state_.joint_positions[i]  = msg->motor_state[i].q;
        robot_state_.joint_velocities[i] = msg->motor_state[i].dq;
        robot_state_.joint_torques[i]    = msg->motor_state[i].tauest;
    }
}

// ══════════════════════════════════════════════════════════════
//  Command publisher
// ══════════════════════════════════════════════════════════════

void HectorV2Node::publish_command(const RobotCommand& cmd)
{
    messages::msg::HectorV2Command msg;
    for (int i = 0; i < HECTOR_V2_NUM_MOTOR && i < static_cast<int>(cmd.motor_commands.size()); ++i)
    {
        msg.motor_command[i].q   = cmd.motor_commands[i].q;
        msg.motor_command[i].dq  = cmd.motor_commands[i].dq;
        msg.motor_command[i].tau = cmd.motor_commands[i].tau;
        msg.motor_command[i].kp  = cmd.motor_commands[i].kp;
        msg.motor_command[i].kd  = cmd.motor_commands[i].kd;
    }
    cmd_pub_->publish(msg);
}

}  // namespace cpp_control
