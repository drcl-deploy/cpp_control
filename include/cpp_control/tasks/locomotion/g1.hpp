#pragma once

#include "cpp_control/robots/g1.hpp"
#include "common/observation_utils.hpp"

namespace cpp_control
{

/**
 * @brief G1-specific locomotion node.
 *
 * Provides:
 *  - cmd_vel_ from joystick + gamepad
 *  - build_observation()  : [ang_vel(3), proj_grav(3), cmd_vel(3), jpos_rel(N), jvel(N), last_act(N)]
 *  - policy_control()     : obs → predict → action-scaled position command
 */
class G1LocomotionNode : public G1BaseNode
{
public:
    explicit G1LocomotionNode(const std::string& node_name = "g1_locomotion_controller");
    ~G1LocomotionNode() override = default;

protected:
    // --- Policy ---
    RobotCommand policy_control() override;
    std::vector<float> build_observation();

    // --- Velocity from both joystick and gamepad ---
    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;
    void on_gamepad() override;

    std::array<float, 3> cmd_vel_ = {0.0f, 0.0f, 0.0f};
};

}  // namespace cpp_control
