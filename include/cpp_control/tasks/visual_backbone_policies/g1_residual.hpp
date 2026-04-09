#pragma once

#include "cpp_control/robots/g1.hpp"
#include "common/math_utils.hpp"
#include "common/observation_utils.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>

#include <array>
#include <vector>

namespace cpp_control
{

/**
 * @brief G1 visual-backbone residual-action task node.
 *
 * Runs two ONNX policies:
 *   HLC  – high-level controller (visual + proprioceptive → residual actions)
 *   WBC  – whole-body controller (motion tracking, same as TextOp, 431-dim obs)
 *
 * Final motor command (mirrors training: default + action_scale * (wbc + hlc_scale * hlc)):
 *   q = default_angles[mj]
 *     + action_scale[mj] * wbc_action[il]
 *     + action_scale[mj] * hlc_action_scale * hlc_action[il]
 *
 * HLC observation layout:
 *   joint_pos_rel              (29)  JOINT_NAMES_EXPR order (= MJ order, preserve_order=True)
 *   joint_vel                  (29)  JOINT_NAMES_EXPR order (= MJ order)
 *   hlc_last_actions           (29)  IL order (action space matches WBC output)
 *   image_features       (embed_dim) from visual backbone (e.g. Theia)
 *   object_goal9d_anchor        (9)  pos[3] + rot_6d[6] in robot frame
 *   robot_ori_mat6d_w           (6)  root orientation in world (6D rotation)
 *                          ─────────
 *                    102 + embed_dim   (default 1254 with embed_dim=1152)
 *
 * WBC observation layout: same as G1TextopNode (431 dims, IL joint order).
 */
class G1ResidualNode : public G1Node
{
public:
    explicit G1ResidualNode(const std::string& node_name = "g1_vbp_residual_controller");
    ~G1ResidualNode() override = default;

protected:
    // --- Policy ---
    RobotCommand policy_control() override;

    // --- Joystick / gamepad (placeholder) ---
    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;
#ifdef HAS_UNITREE_HG
    void on_gamepad() override;
#endif

private:
    // ── Constants ────────────────────────────────────────────────
    static constexpr int FUTURE_STEPS = 5;
    static constexpr int NQ            = 29;
    static constexpr int WBC_NUM_OBS   = 431;

    // ── HLC (high-level controller) ─────────────────────────────
    int hlc_num_obs_ = 0;
    std::vector<float> build_hlc_observation();
    std::vector<float> hlc_actions_;
    std::vector<float> hlc_last_actions_;

    // ── WBC (whole-body controller, mirrors TextOp) ─────────────
    std::unique_ptr<ONNXPolicy> wbc_policy_;
    std::vector<float> build_wbc_observation();
    std::vector<float> wbc_actions_;
    std::vector<float> wbc_last_actions_;

    // ── HLC action scale (scalar, matches training cfg.action_scale) ─
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
    std::array<float, 3> object_goal_pos_  = {0, 0, 0};
    std::array<float, 4> object_goal_quat_ = {1, 0, 0, 0};
    bool object_goal_ready_ = false;

    // ── Motion data (for WBC, same as TextOp) ───────────────────
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motion_sub_;
    void on_motion(std_msgs::msg::Float32MultiArray::SharedPtr msg);

    std::vector<std::vector<float>> mot_joint_pos_;   // [T][NQ] IsaacLab order
    std::vector<std::vector<float>> mot_joint_vel_;   // [T][NQ]
    std::vector<std::array<float, 3>> mot_anchor_pos_;
    std::vector<std::array<float, 4>> mot_anchor_ori_;  // wxyz
    int  mot_T_     = 0;
    int  mot_t_     = 0;
    bool mot_ready_ = false;

    // ── Stand mode (RB) ─────────────────────────────────────────
    bool stand_mode_ = false;
    bool prev_rb_    = false;
    void init_stand_motion();

    // ── Pending motion (staged while standing) ──────────────────
    std::vector<std::vector<float>> pend_joint_pos_;
    std::vector<std::vector<float>> pend_joint_vel_;
    std::vector<std::array<float, 3>> pend_anchor_pos_;
    std::vector<std::array<float, 4>> pend_anchor_ori_;
    int  pend_T_     = 0;
    bool pend_ready_ = false;
    void commit_pending_motion();
    void pad_pending_motion();

    // ── IsaacLab ↔ MuJoCo reindexing ───────────────────────────
    std::array<int, NQ> mj_to_il_{};
    std::array<int, NQ> il_to_mj_{};
    void build_reindex_tables();

    // ── Frame alignment (ref → robot) ───────────────────────────
    bool frame_init_ = false;
    std::array<float, 3> robot_init_pos_  = {0, 0, 0};
    std::array<float, 4> robot_init_hq_   = {1, 0, 0, 0};
    std::array<float, 3> ref_init_pos_    = {0, 0, 0};
    std::array<float, 4> ref_init_hq_     = {1, 0, 0, 0};
    std::array<float, 4> ref2robot_quat_  = {1, 0, 0, 0};

    void setup_init_frame();
    std::pair<std::array<float, 3>, std::array<float, 4>>
    transform_ref_to_robot(const std::array<float, 3>& pos,
                           const std::array<float, 4>& quat) const;
};

}  // namespace cpp_control
