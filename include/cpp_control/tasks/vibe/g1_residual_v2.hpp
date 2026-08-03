#pragma once

#include "cpp_control/tasks/vibe/g1_residual.hpp"

namespace cpp_control
{

/**
 * @brief G1 VIBE residual V2 — extended HLC observation space.
 *
 * Inherits everything from G1VibeResidualNode; only overrides build_hlc_observation()
 * to match the v2 training obs layout:
 *
 *   image_features        (embed_dim)  visual backbone
 *   base_ori_mat6d              (6)    root orientation 6D
 *   base_ang_vel                (3)    gyroscope body-frame
 *   joint_pos                  (29)    joint_pos_rel, MJ order
 *   joint_vel                  (29)    MJ order
 *   actions                    (29)    hlc_last_actions, IL order
 *   low_level_actions          (29)    wbc_last_actions, IL order
 *   object_goal_command         (9)    pos[3] + rot_6d[6]
 *   joint_state_command       (290)    future joint pos+vel, IL order
 *   motion_anchor_pos_b        (15)    future anchor pos body-frame
 *   motion_anchor_ori_b        (30)    future anchor ori 6D body-frame
 *                         ──────────
 *                   469 + embed_dim    (default 1621 with embed_dim=1152)
 */
class G1VibeResidualV2Node : public G1VibeResidualNode
{
public:
    explicit G1VibeResidualV2Node(const std::string& node_name = "g1_vibe_residual_v2_controller");
    ~G1VibeResidualV2Node() override = default;

protected:
    std::vector<float> build_hlc_observation() override;
};

}  // namespace cpp_control
