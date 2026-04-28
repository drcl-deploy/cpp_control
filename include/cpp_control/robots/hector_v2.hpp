#pragma once

#include "cpp_control/base.hpp"

#include <messages/msg/hector_v2_state.hpp>
#include <messages/msg/hector_v2_command.hpp>

namespace cpp_control
{

constexpr int HECTOR_V2_NUM_MOTOR = 18;

/**
 * @brief Level 1: HectorV2 robot-specific base node.
 *
 * Implements drcl_deploy backend (HectorV2State / HectorV2Command).
 * 18 DOF: 5 per leg (hip_yaw, hip_roll, hip_pitch, knee, ankle) x2
 *       + 4 per arm (shoulder_yaw, shoulder_pitch, shoulder_roll, elbow) x2
 *
 * Level 2 tasks override:
 *  - policy_control()  : task-specific inference
 *  - on_joy()          : joystick velocity mapping
 */
class HectorV2Node : public BaseNode
{
public:
    explicit HectorV2Node(const std::string& node_name);
    ~HectorV2Node() override = default;

protected:
    void init_robot() override;
    void publish_command(const RobotCommand& cmd) override;
    int num_motors() const override { return HECTOR_V2_NUM_MOTOR; }

private:
    void subscribe_hector_v2_state(messages::msg::HectorV2State::SharedPtr msg);

    rclcpp::Publisher<messages::msg::HectorV2Command>::SharedPtr cmd_pub_;
    rclcpp::Subscription<messages::msg::HectorV2State>::SharedPtr state_sub_;
};

}  // namespace cpp_control
