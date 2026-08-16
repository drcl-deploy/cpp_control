#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "common/types.hpp"
#include "cpp_control/config_loader.hpp"
#include "cpp_control/onnx_policy.hpp"

namespace cpp_control
{

/// Joystick button/axis indices (X-mode layout)
namespace joy
{
constexpr size_t XMODE_A = 0;
constexpr size_t XMODE_B = 1;
constexpr size_t XMODE_X = 2;
constexpr size_t XMODE_Y = 3;
constexpr size_t XMODE_LEFT_JOY_LEFT_RIGHT = 0;
constexpr size_t XMODE_LEFT_JOY_UP_DOWN = 1;
constexpr size_t XMODE_RIGHT_JOY_LEFT_RIGHT = 3;
constexpr size_t XMODE_RIGHT_JOY_UP_DOWN = 4;
constexpr size_t XMODE_L1 = 4;  // xpad buttons: A0 B1 X2 Y3 LB4 RB5
constexpr size_t XMODE_R1 = 5;
}  // namespace joy

/**
 * @brief Level 0: Robot-agnostic base controller node.
 *
 * Implements:
 *  - Control mode state machine (zeroing → nominal_pose → policy)
 *  - Joystick subscription and mode switching (mode only, no velocity)
 *  - Timer-based control loop
 *  - Smooth stand-up interpolation (nominal_pose)
 *
 * Level 1 (robot) must implement:
 *  - init_robot()          : robot-specific subscribers/publishers
 *  - publish_command()     : convert RobotCommand → robot msg and publish
 *  - num_motors()          : number of actuated joints
 *
 * Level 2 (task) overrides:
 *  - policy_control()      : build obs, run inference, return cmd
 *  - on_joy()              : task-specific joystick behaviour (velocity, etc.)
 */
class BaseNode : public rclcpp::Node
{
public:
    explicit BaseNode(const std::string& node_name);
    virtual ~BaseNode() = default;

protected:
    // --- Call from any derived constructor; runs only once ---
    void init();
    // --- Level 1 must implement ---
    virtual void init_robot() = 0;
    virtual void publish_command(const RobotCommand& cmd) = 0;
    virtual int num_motors() const = 0;

    // --- Level 2 can override ---
    virtual RobotCommand policy_control();
    virtual void on_joy(sensor_msgs::msg::Joy::SharedPtr /*msg*/) {}
    /// Task veto on the A button, checked BEFORE the mode flip: a refusal
    /// leaves the mode untouched, so it can never strand DAMPING/ZEROING
    /// inside POLICY the way an after-the-fact undo would.
    virtual bool allow_policy_entry() { return true; }

    // --- Robot-level stand mode (ControlMode::STAND) ---
    // Level 1 provides the engine; base wires RB to it.
    virtual bool has_stand() const { return false; }
    virtual void engage_stand() {}
    virtual RobotCommand stand_control() { return nominal_pose_control(); }

    // --- Built-in control modes ---
    RobotCommand zeroing_control();
    RobotCommand damping_control();
    RobotCommand nominal_pose_control();
    RobotCommand standing_up_control();  // interpolate → default, then auto-switch to POLICY

    // --- State shared across all levels ---
    RobotState robot_state_;
    std::unique_ptr<Config> config_;
    std::unique_ptr<ONNXPolicy> policy_;

    ControlMode control_mode_ = ControlMode::ZEROING;
    float alpha_ = 0.0f;
    float settle_time_ = 2.0f;

    std::vector<float> actions_;
    std::vector<float> last_actions_;

    // Robot constants (populated by Level 1)
    std::vector<std::string> joint_names_;
    std::vector<float> default_angles_;
    std::vector<float> kps_;
    std::vector<float> kds_;
    std::vector<float> action_scale_;

    // For nominal pose interpolation (protected so Level 1 gamepad can access)
    std::vector<float> pre_nominal_pos_;

private:
    void control_loop();
    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);

    std::vector<int> prev_buttons_;

    bool init_done_ = false;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace cpp_control
