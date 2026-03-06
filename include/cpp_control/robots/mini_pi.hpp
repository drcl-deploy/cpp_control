#pragma once

#include "cpp_control/base.hpp"

#include <messages/msg/mini_pi_state.hpp>
#include <messages/msg/mini_pi_command.hpp>

namespace cpp_control
{

constexpr int MINI_PI_NUM_MOTOR = 12;

/**
 * @brief Level 1: MiniPi robot-specific base node.
 *
 * Implements drcl_deploy backend only (MiniPiState / MiniPiCommand).
 *
 * Level 2 tasks override:
 *  - policy_control()       : task-specific inference
 *  - on_joy()               : joystick velocity mapping
 */
class MiniPiNode : public BaseNode
{
public:
    explicit MiniPiNode(const std::string &node_name);
    ~MiniPiNode() override = default;

protected:
    // --- Level 1 implementation of BaseNode interface ---
    void init_robot() override;
    void publish_command(const RobotCommand &cmd) override;
    int num_motors() const override { return MINI_PI_NUM_MOTOR; }

private:
    void subscribe_mini_pi_state(messages::msg::MiniPiState::SharedPtr msg);

    rclcpp::Publisher<messages::msg::MiniPiCommand>::SharedPtr cmd_pub_;
    rclcpp::Subscription<messages::msg::MiniPiState>::SharedPtr state_sub_;
};

}  // namespace cpp_control
