#pragma once

#include "cpp_control/tasks/tracker/g1_textop_tracker.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <array>
#include <vector>

namespace cpp_control
{

/**
 * @brief G1 VIBE residual-action task node.
 *
 * Runs two ONNX policies:
 *   HLC  - high-level controller (visual + proprioceptive -> residual actions)
 *   WBC  - whole-body controller (motion tracking, 431-dim obs, inherited from tracker)
 *
 * Final motor command (mirrors training: default + action_scale * (wbc + hlc_scale * hlc)):
 *   q = default_angles[mj]
 *     + action_scale[mj] * wbc_action[il]
 *     + action_scale[mj] * hlc_action_scale * hlc_action[il]
 *
 * HLC observation layout (v1):
 *   joint_pos_rel              (29)  JOINT_NAMES_EXPR order (= MJ order)
 *   joint_vel                  (29)  MJ order
 *   hlc_last_actions           (29)  IL order
 *   image_features       (embed_dim) from visual backbone (e.g. Theia)
 *   object_goal9d_anchor        (9)  pos[3] + rot_6d[6] in robot frame
 *   robot_ori_mat6d_w           (6)  root orientation in world (6D rotation)
 *   low_level_actions          (29)  WBC last actions, IL order
 *                          ---------
 *                    131 + embed_dim  (default 1283 with embed_dim=1152)
 *
 * WBC observation layout: inherited from G1TextopTrackerNode (431 dims).
 */
class G1VibeResidualNode : public G1TextopTrackerNode
{
public:
    explicit G1VibeResidualNode(const std::string& node_name = "g1_vibe_residual_controller");
    ~G1VibeResidualNode() override = default;

protected:
    // --- Policy ---
    RobotCommand policy_control() override;
    void enter_stand_mode() override;

    // ── HLC (high-level controller) ─────────────────────────────
    std::unique_ptr<ONNXPolicy> hlc_policy_;
    int hlc_num_obs_ = 0;
    virtual std::vector<float> build_hlc_observation();
    std::vector<float> hlc_actions_;
    std::vector<float> hlc_last_actions_;

    // ── WBC action buffer (for HLC+WBC combination) ──────────────
    std::vector<float> wbc_actions_;

    // ── HLC action scale ────────────────────────────────────────
    float hlc_action_scale_ = 0.1f;

    // ── Visual embedding (from external backbone node) ──────────
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr embedding_sub_;
    void on_embedding(std_msgs::msg::Float32MultiArray::SharedPtr msg);
    std::vector<float> embedding_;
    int embedding_dim_ = 1152;
    bool embedding_ready_ = false;

    // ── Object goal (target pose, world frame, wxyz) ────────────
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr object_goal_sub_;
    void on_object_goal(std_msgs::msg::Float32MultiArray::SharedPtr msg);
    std::array<float, 3> object_goal_pos_  = {0.0f, 0.0f, 0.0f};
    std::array<float, 4> object_goal_quat_ = {1.0f, 0.0f, 0.0f, 0.0f};
    bool object_goal_ready_ = false;

    // ── Debug publishers (for viser_ghost viewer) ───────────────
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ghost_motion_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr residual_action_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_anchor_pub_;
};

}  // namespace cpp_control
