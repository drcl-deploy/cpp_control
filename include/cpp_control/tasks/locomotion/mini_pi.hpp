#pragma once

#include "cpp_control/robots/mini_pi.hpp"
#include "common/observation_utils.hpp"

namespace cpp_control
{

/**
 * @brief MiniPi locomotion node.
 *
 * Observation (45):
 *  [ang_vel(3), proj_grav(3), cmd_vel(3), jpos_rel(12), jvel(12), last_act(12)]
 */
class MiniPiLocomotionNode : public MiniPiNode
{
public:
    explicit MiniPiLocomotionNode(const std::string& node_name = "mini_pi_locomotion_controller");
    ~MiniPiLocomotionNode() override = default;

protected:
    RobotCommand policy_control() override;
    std::vector<float> build_observation();

    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;

    std::array<float, 3> cmd_vel_ = {0.0f, 0.0f, 0.0f};
};

}  // namespace cpp_control
