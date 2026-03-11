#pragma once

#include "cpp_control/robots/mini_pi.hpp"
#include "common/observation_utils.hpp"

namespace cpp_control
{

/**
 * @brief MiniPi dive (flip) controller node.
 *
 * Observation (48):
 *  [gyro(3), jpos_rel(12), jvel(12), last_act(12), rmat6d(6), cmd(3)]
 *
 * cmd: [flip_x, flip_y, terrain_height]
 *  front flip  →  [+1,  0, 1]
 *  back  flip  →  [-1,  0, 1]
 *  left  flip  →  [ 0, +1, 1]
 *  right flip  →  [ 0, -1, 1]
 *
 * Action scale: 0.5 (uniform)
 *
 * Joystick (D-pad):
 *  axis 7 (up/down)   : up=+1 → front, down=-1 → back
 *  axis 6 (left/right): left=+1 → left, right=-1 → right
 */
class MiniPiDiveNode : public MiniPiNode
{
public:
    explicit MiniPiDiveNode(const std::string& node_name = "mini_pi_dive_controller");
    ~MiniPiDiveNode() override = default;

protected:
    RobotCommand policy_control() override;
    std::vector<float> build_observation();

    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;

    // [flip_x, flip_y, terrain_height]  (default: front flip)
    std::array<float, 3> cmd_ = {1.0f, 0.0f, 1.0f};

    // Motor-to-action index mapping (loaded from config)
    std::vector<int> motor2action_id_;
};

}  // namespace cpp_control
