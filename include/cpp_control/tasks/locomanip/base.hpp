#pragma once

#include "cpp_control/robots/g1_base_node.hpp"
#include "common/math_utils.hpp"

#include <geometry_msgs/msg/pose.hpp>

namespace cpp_control
{

/**
 * @brief Task base: Loco-manipulation.
 *
 * Provides:
 *  - XR controller subscriptions for left / right hand targets
 *  - locomanip_cmd_vel_  from gamepad / joystick triggers
 *  - build_observation() : [ang_vel(3), proj_grav(3), cmd_vel(3),
 *                           lhand_pos(3), lhand_quat(4),
 *                           rhand_pos(3), rhand_quat(4),
 *                           jpos_rel(N), jvel(N), last_act(N)]  → 110 for G1
 *  - policy_control()    : obs → predict → action-scaled position command
 *
 * Robot-specific subclass (g1.cpp) can override if needed.
 */
class LocoManipBase : public G1BaseNode
{
public:
    explicit LocoManipBase(const std::string& node_name);
    ~LocoManipBase() override = default;

protected:
    // --- Policy ---
    RobotCommand policy_control() override;
    virtual std::vector<float> build_observation();

    // --- Joystick trigger-based velocity ---
    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;
    void on_gamepad() override;

    // --- Task state ---
    std::array<float, 3> locomanip_cmd_vel_ = {0.0f, 0.0f, 0.0f};

    std::array<float, 3> left_hand_pos_  = {0.0f, 0.0f, 0.0f};
    std::array<float, 4> left_hand_quat_ = {1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> right_hand_pos_ = {0.0f, 0.0f, 0.0f};
    std::array<float, 4> right_hand_quat_= {1.0f, 0.0f, 0.0f, 0.0f};

private:
    // --- XR callbacks ---
    void left_hand_callback(geometry_msgs::msg::Pose::SharedPtr msg);
    void right_hand_callback(geometry_msgs::msg::Pose::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr left_hand_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr right_hand_sub_;
};

}  // namespace cpp_control
