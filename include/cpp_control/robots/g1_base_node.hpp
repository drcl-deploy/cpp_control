#pragma once

#include "cpp_control/base_node.hpp"
#include "common/gamepad.hpp"
#include "common/motor_crc_hg.h"

#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

namespace cpp_control
{

constexpr int G1_NUM_MOTOR = 29;

/**
 * @brief Level 1: G1 robot-specific base node.
 *
 * Implements:
 *  - HG message backend (state parsing + command publishing)
 *  - Unitree gamepad parsing + mode switching via wireless_remote
 *
 * Level 2 tasks override:
 *  - policy_control()       : task-specific inference
 *  - on_joy()               : joystick velocity mapping
 *  - on_gamepad()           : gamepad velocity mapping (called after mode switch)
 */
class G1BaseNode : public BaseNode
{
public:
    explicit G1BaseNode(const std::string& node_name);
    ~G1BaseNode() override = default;

protected:
    // --- Level 1 implementation of BaseNode interface ---
    void init_robot() override;
    void publish_command(const RobotCommand& cmd) override;
    int num_motors() const override { return G1_NUM_MOTOR; }

    // --- Hook for Level 2: read velocities from gamepad_ after mode switching ---
    virtual void on_gamepad() {}

    // --- Gamepad state (readable by Level 2) ---
    unitree::common::Gamepad gamepad_;

private:
    // --- HG message backend ---
    void low_state_handler_hg(unitree_hg::msg::LowState::SharedPtr msg);
    void publish_command_hg(const RobotCommand& cmd);
    void handle_gamepad(const unitree_hg::msg::LowState& msg);

    unitree_hg::msg::LowCmd low_cmd_hg_;
    uint8_t mode_machine_ = 5;
    uint8_t mode_pr_ = 0;

    unitree::common::REMOTE_DATA_RX gamepad_rx_;

    // --- ROS handles ---
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_pub_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
};

}  // namespace cpp_control
