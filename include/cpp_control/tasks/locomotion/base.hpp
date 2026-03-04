#pragma once

#include "cpp_control/robots/g1_base_node.hpp"
#include "common/math_utils.hpp"

namespace cpp_control
{

/**
 * @brief Task base: SE2 velocity locomotion.
 *
 * Provides:
 *  - cmd_vel_ from joystick + gamepad
 *  - build_observation()  : [ang_vel(3), proj_grav(3), cmd_vel(3), jpos_rel(N), jvel(N), last_act(N)]
 *  - policy_control()     : obs → predict → action-scaled position command
 *
 * Robot-specific subclass (g1.cpp) can override if needed.
 *
 * NOTE: inherits from G1BaseNode for now. When a second robot is added,
 *       this can be templatized or split into a CRTP pattern.
 */
class LocomotionBase : public G1BaseNode
{
public:
    explicit LocomotionBase(const std::string& node_name);
    ~LocomotionBase() override = default;

protected:
    // --- Policy ---
    RobotCommand policy_control() override;
    virtual std::vector<float> build_observation();

    // --- Velocity from both joystick and gamepad ---
    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;
    void on_gamepad() override;

    std::array<float, 3> cmd_vel_ = {0.0f, 0.0f, 0.0f};
};

}  // namespace cpp_control
