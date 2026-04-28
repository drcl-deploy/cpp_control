#include "cpp_control/tasks/locomotion/hector_v2.hpp"

namespace cpp_control
{

HectorV2LocomotionNode::HectorV2LocomotionNode(const std::string& node_name)
    : HectorV2Node(node_name)
{
    init();

    std::string onnx_path = this->declare_parameter("onnx_model_path", "");
    if (onnx_path.empty() && config_)
        onnx_path = config_->onnx_path;

    if (!onnx_path.empty())
    {
        policy_ = std::make_unique<ONNXPolicy>(onnx_path);
        RCLCPP_INFO(this->get_logger(), "Locomotion policy: %s", onnx_path.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No ONNX model path for locomotion");
    }
}

// ── Observation ───────────────────────────────────────────────

std::vector<float> HectorV2LocomotionNode::build_observation()
{
    int n = num_motors();
    std::vector<float> obs;
    obs.reserve(3 + 3 + 3 + n + n + n);  // 63

    obs::append_imu_obs(obs, robot_state_);        // gyro(3) + proj_gravity(3)
    obs::append_cmd_vel(obs, cmd_vel_);             // cmd_vel(3)
    obs::append_joint_obs(obs, robot_state_, default_angles_);  // jpos_rel(18) + jvel(18)
    obs::append_last_actions(obs, last_actions_, n);             // last_actions(18)

    return obs;
}

// ── Policy ────────────────────────────────────────────────────

RobotCommand HectorV2LocomotionNode::policy_control()
{
    if (!policy_)
        return zeroing_control();

    auto obs    = build_observation();
    auto action = policy_->predict(obs);

    int n = num_motors();
    RobotCommand cmd;
    cmd.motor_commands.resize(n);

    for (int i = 0; i < n && i < static_cast<int>(action.size()); ++i)
    {
        actions_[i]                   = action[i];
        cmd.motor_commands[i].q       = default_angles_[i] + action_scale_[i] * action[i];
        cmd.motor_commands[i].kp      = kps_[i];
        cmd.motor_commands[i].kd      = kds_[i];
    }

    last_actions_ = actions_;
    return cmd;
}

// ── Velocity from Joystick ────────────────────────────────────

void HectorV2LocomotionNode::on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->axes.size() > joy::XMODE_R1)
    {
        cmd_vel_[0] = static_cast<float>(msg->axes[joy::XMODE_LEFT_JOY_UP_DOWN])    * 0.5f;
        cmd_vel_[1] = static_cast<float>(msg->axes[joy::XMODE_LEFT_JOY_LEFT_RIGHT]) * 0.5f;
        cmd_vel_[2] = static_cast<float>(msg->axes[joy::XMODE_RIGHT_JOY_LEFT_RIGHT]) * 0.5f;
    }
}

}  // namespace cpp_control

// ── Entry point ───────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::HectorV2LocomotionNode>());
    rclcpp::shutdown();
    return 0;
}
