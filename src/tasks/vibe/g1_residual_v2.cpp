#include "cpp_control/tasks/vibe/g1_residual_v2.hpp"

namespace cpp_control
{

// ── Constructor ──────────────────────────────────────────────────

G1VibeResidualV2Node::G1VibeResidualV2Node(const std::string& node_name)
    : G1VibeResidualNode(node_name)
{
    // Override v1 HLC obs size with v2 layout:
    // embed + ori6d(6) + ang_vel(3) + 4×NQ + goal9d(9)
    //       + FUTURE_STEPS × (2×NQ + 3 + 6)
    hlc_num_obs_ = embedding_dim_ + 6 + 3 + 4 * NQ + 9
                 + FUTURE_STEPS * (2 * NQ + 3 + 6);

    RCLCPP_INFO(this->get_logger(), "G1 VIBE Residual V2 node ready (HLC obs=%d, WBC obs=%d)",
                hlc_num_obs_, WBC_NUM_OBS);

    // print out the action scale
    RCLCPP_INFO(this->get_logger(), "HLC action scale: %f", hlc_action_scale_);
}

// ── HLC Observation (v2 layout) ──────────────────────────────────

std::vector<float> G1VibeResidualV2Node::build_hlc_observation()
{
    std::vector<float> obs;
    obs.reserve(hlc_num_obs_);

    // ─ 0. image_features (embed_dim) ────────────────────────────
    obs::append_features(obs, embedding_);

    // ─ 1. base_ori_mat6d (6) ────────────────────────────────────
    {
        auto r6d = math::quat_to_rotation_6d(robot_state_.imu_quaternion);
        for (int i = 0; i < 6; ++i)
            obs.push_back(r6d[i]);
    }

    // ─ 2. base_ang_vel (3) ──────────────────────────────────────
    obs::append_gyro(obs, robot_state_);

    // ─ 3. joint_pos (29) — joint_pos_rel, MJ order ─────────────
    for (int mj = 0; mj < NQ; ++mj)
        obs.push_back(robot_state_.joint_positions[mj] - default_angles_[mj]);

    // ─ 4. joint_vel (29) — MJ order ────────────────────────────
    for (int mj = 0; mj < NQ; ++mj)
        obs.push_back(robot_state_.joint_velocities[mj]);

    // ─ 5. actions (29) — hlc_last_actions, IL order ─────────────
    for (int il = 0; il < NQ; ++il)
        obs.push_back(hlc_last_actions_[il]);

    // ─ 6. low_level_actions (29) — wbc_last_actions, IL order ───
    for (int il = 0; il < NQ; ++il)
        obs.push_back(wbc_last_actions_[il]);

    // ─ 7. object_goal_command (9) — pos[3] + rot_6d[6] ─────────
    for (int i = 0; i < 3; ++i)
        obs.push_back(object_goal_pos_[i]);
    {
        auto goal_r6d = math::quat_to_rotation_6d(object_goal_quat_);
        for (int i = 0; i < 6; ++i)
            obs.push_back(goal_r6d[i]);
    }

    // ─ 8. joint_state_command (290) — future joint pos+vel, IL order
    std::array<float, NQ> row;
    for (int s = 0; s < FUTURE_STEPS; ++s)
    {
        int idx = std::min(mot_t_ + s, mot_T_ - 1);
        mot_.jp_il(idx, row.data());
        obs.insert(obs.end(), row.begin(), row.end());
    }
    for (int s = 0; s < FUTURE_STEPS; ++s)
    {
        int idx = std::min(mot_t_ + s, mot_T_ - 1);
        mot_.jv_il(idx, row.data());
        obs.insert(obs.end(), row.begin(), row.end());
    }

    // ─ 9. motion_anchor_pos_b (15) — future anchor pos, body-frame
    {
        auto robot_pos  = robot_state_.base_pos_w;
        auto robot_quat = robot_state_.imu_quaternion;

        if (!frame_init_)
            setup_init_frame();

        for (int s = 0; s < FUTURE_STEPS; ++s)
        {
            int idx = std::min(mot_t_ + s, mot_T_ - 1);
            auto [ref_pos_r, ref_quat_r] =
                transform_ref_to_robot(mot_.root_pos(idx), mot_.root_quat(idx));
            auto [rel_pos, _] = math::subtract_frames(
                robot_pos, robot_quat, ref_pos_r, ref_quat_r);
            for (int i = 0; i < 3; ++i)
                obs.push_back(rel_pos[i]);
        }
    }

    // ─ 10. motion_anchor_ori_b (30) — future anchor ori 6D, body-frame
    {
        auto robot_quat = robot_state_.imu_quaternion;

        for (int s = 0; s < FUTURE_STEPS; ++s)
        {
            int idx = std::min(mot_t_ + s, mot_T_ - 1);
            auto [ref_pos_r, ref_quat_r] =
                transform_ref_to_robot(mot_.root_pos(idx), mot_.root_quat(idx));
            auto [_, rel_quat] = math::subtract_frames(
                robot_init_pos_, robot_quat, ref_pos_r, ref_quat_r);
            auto r6d = math::quat_to_rotation_6d(rel_quat);
            for (int i = 0; i < 6; ++i)
                obs.push_back(r6d[i]);
        }
    }

    return obs;
}

}  // namespace cpp_control

// ── Entry point ──────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1VibeResidualV2Node>());
    rclcpp::shutdown();
    return 0;
}
