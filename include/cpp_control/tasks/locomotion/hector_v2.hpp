#pragma once

#include "cpp_control/robots/hector_v2.hpp"
#include "common/observation_utils.hpp"

namespace cpp_control
{

/**
 * @brief HectorV2 locomotion node.
 *
 * Observation (63):
 *  [ang_vel(3), proj_grav(3), cmd_vel(3), jpos_rel(18), jvel(18), last_act(18)]
 */
class HectorV2LocomotionNode : public HectorV2Node
{
public:
    explicit HectorV2LocomotionNode(
        const std::string& node_name = "hector_v2_locomotion_controller");
    ~HectorV2LocomotionNode() override = default;

protected:
    RobotCommand policy_control() override;
    std::vector<float> build_observation();

    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;

    std::array<float, 3> cmd_vel_ = {0.0f, 0.0f, 0.0f};
};

}  // namespace cpp_control
