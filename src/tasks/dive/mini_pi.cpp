#include "cpp_control/tasks/dive/mini_pi.hpp"

namespace cpp_control
{

// D-pad axis indices for sensor_msgs/Joy (XMode gamepad layout)
namespace dpad
{
constexpr size_t UP_DOWN    = 7;  // up = +1, down = -1
constexpr size_t LEFT_RIGHT = 6;  // left = +1, right = -1
}  // namespace dpad

MiniPiDiveNode::MiniPiDiveNode(const std::string& node_name) : MiniPiNode(node_name)
{
    init();

    std::string onnx_path = this->declare_parameter("onnx_model_path", "");
    if (onnx_path.empty() && config_)
        onnx_path = config_->onnx_path;

    if (!onnx_path.empty())
    {
        policy_ = std::make_unique<ONNXPolicy>(onnx_path);
        RCLCPP_INFO(this->get_logger(), "Dive policy: %s", onnx_path.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No ONNX model path for dive");
    }

    RCLCPP_INFO(this->get_logger(), "Select flip direction with D-pad before switching to policy mode");
}

// ── Observation ───────────────────────────────────────────────

std::vector<float> MiniPiDiveNode::build_observation()
{
    int n = num_motors();
    std::vector<float> obs;
    obs.reserve(3 + n + n + n + 6 + 3);  // 48

    // [gyro(3), jpos_rel(n), jvel(n), last_act(n), rmat6d(6), cmd(3)]
    obs::append_gyro(obs, robot_state_);
    obs::append_joint_pos_rel(obs, robot_state_, default_angles_);
    obs::append_joint_vel(obs, robot_state_, n);
    obs::append_last_actions(obs, last_actions_);
    obs::append_rotation_6d(obs, robot_state_.imu_quaternion);
    // print the quaternion value
    std::cout << "IMU Quaternion: [" << robot_state_.imu_quaternion[0] << ", "
              << robot_state_.imu_quaternion[1] << ", "
              << robot_state_.imu_quaternion[2] << ", "
              << robot_state_.imu_quaternion[3] << "]" << std::endl;
    obs::append_cmd(obs, cmd_);
    

    return obs;
}

// ── Policy ────────────────────────────────────────────────────

RobotCommand MiniPiDiveNode::policy_control()
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
        constexpr float kActionScale = 0.5f;
        actions_[i] = action[i];
        cmd.motor_commands[i].q = default_angles_[i] + kActionScale * action[i];
        cmd.motor_commands[i].kp = kps_[i];
        cmd.motor_commands[i].kd = kds_[i];
    }

    last_actions_ = actions_;
    return cmd;
}

// ── Flip direction from D-pad ─────────────────────────────────

void MiniPiDiveNode::on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->axes.size() <= dpad::UP_DOWN)
        return;

    float ud = msg->axes[dpad::UP_DOWN];
    float lr = msg->axes[dpad::LEFT_RIGHT];

    if (ud > 0.5f)
    {
        cmd_ = {1.0f, 0.0f, 1.0f};
        RCLCPP_INFO(this->get_logger(), "Front flip selected: cmd = [%.1f, %.1f, %.1f]", cmd_[0], cmd_[1], cmd_[2]);
    }
    else if (ud < -0.5f)
    {
        cmd_ = {-1.0f, 0.0f, 1.0f};
        RCLCPP_INFO(this->get_logger(), "Back flip selected: cmd = [%.1f, %.1f, %.1f]", cmd_[0], cmd_[1], cmd_[2]);
    }
    else if (lr > 0.5f)
    {
        cmd_ = {0.0f, 1.0f, 1.0f};
        RCLCPP_INFO(this->get_logger(), "Left flip selected: cmd = [%.1f, %.1f, %.1f]", cmd_[0], cmd_[1], cmd_[2]);
    }
    else if (lr < -0.5f)
    {
        cmd_ = {0.0f, -1.0f, 1.0f};
        RCLCPP_INFO(this->get_logger(), "Right flip selected: cmd = [%.1f, %.1f, %.1f]", cmd_[0], cmd_[1], cmd_[2]);
    }
    
}

}  // namespace cpp_control

// ── Entry point ───────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::MiniPiDiveNode>());
    rclcpp::shutdown();
    return 0;
}
