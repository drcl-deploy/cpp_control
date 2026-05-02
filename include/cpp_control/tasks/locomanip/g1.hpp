#pragma once

#include "cpp_control/robots/g1.hpp"
#include "common/observation_utils.hpp"

#include <geometry_msgs/msg/pose.hpp>

namespace cpp_control
{

/**
 * @brief G1 loco-manipulation controller node (dual-policy).
 *
 * Loads both a locomotion and a locomanipulation ONNX policy.
 * Uses POLICY mode for locomotion and LOCOMANIP_POLICY mode for locomanipulation.
 *
 * Locomotion observation (96):
 *  [ang_vel(3), proj_grav(3), cmd_vel(3),
 *   jpos_rel(29), jvel(29), last_act(29)]
 *
 * Locomanip observation (110):
 *  [ang_vel(3), proj_grav(3), cmd_vel(3),
 *   left_hand_pos(3), left_hand_quat(4),
 *   right_hand_pos(3), right_hand_quat(4),
 *   jpos_rel(29), jvel(29), last_act(29)]
 *
 * Subscribes to:
 *  - /xr/left_controller   (geometry_msgs/Pose)
 *  - /xr/right_controller  (geometry_msgs/Pose)
 *
 * Joystick:
 *  A button  → LOCOMANIP_POLICY (overrides base A → POLICY)
 *  LB button → POLICY (locomotion)
 *  left stick  → vx, vy
 *  right stick → yaw rate
 *
 * Gamepad:
 *  up → POLICY (locomotion)
 *  A  → LOCOMANIP_POLICY
 */
class G1LocomanipNode : public G1Node
{
public:
    explicit G1LocomanipNode(const std::string& node_name = "g1_locomanip_controller");
    ~G1LocomanipNode() override = default;

protected:
    // --- Dual-policy ---
    RobotCommand policy_control() override;
    RobotCommand locomanip_policy_control() override;

    std::vector<float> build_locomotion_observation();
    std::vector<float> build_locomanip_observation();

    // --- Velocity from joystick and gamepad ---
    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;
#ifdef HAS_UNITREE_HG
    void on_gamepad() override;
#endif

    std::array<float, 3> cmd_vel_ = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> locomanip_cmd_vel_ = {0.0f, 0.0f, 0.0f};

    // --- End-effector pose from XR controllers ---
    std::array<float, 3> left_hand_pos_  = {0.20f,  0.13f, 0.08f};
    std::array<float, 4> left_hand_quat_ = {1.0f, 0.0f, 0.0f, 0.0f};   // w, x, y, z
    std::array<float, 3> right_hand_pos_  = {0.20f, -0.13f, 0.08f};
    std::array<float, 4> right_hand_quat_ = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z

    // --- Locomanip policy (locomotion policy is in base policy_) ---
    std::unique_ptr<ONNXPolicy> locomanip_policy_;

private:
    void left_xr_callback(geometry_msgs::msg::Pose::SharedPtr msg);
    void right_xr_callback(geometry_msgs::msg::Pose::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr left_xr_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr right_xr_sub_;
};

}  // namespace cpp_control
