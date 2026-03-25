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
 * @brief G1 whole-body-control (TextOp tracker) task node.
 *
 * Subscribes to motion reference data and tracks it using an ONNX policy
 * trained in IsaacLab (ProjGravObs, 431-dim observation, 29 actions).
 *
 * Observation layout (all in IsaacLab joint order):
 *   command           (290) : future_steps × (joint_pos + joint_vel)
 *   anchor_pos_b      (15) : future_steps × 3
 *   anchor_ori_b      (30) : future_steps × 6  (rotation 6D)
 *   projected_gravity   (3)
 *   base_lin_vel        (3)
 *   base_ang_vel        (3)
 *   joint_pos_rel      (29)
 *   joint_vel_rel      (29)
 *   last_action        (29)
 *                     -----
 *                      431
 */
class G1TextopNode : public G1Node
{
public:
    explicit G1TextopNode(const std::string& node_name = "g1_textop_controller");
    ~G1TextopNode() override = default;

protected:
    // --- Policy ---
    RobotCommand policy_control() override;
    std::vector<float> build_observation();

    // --- Joystick / gamepad (unused for now, placeholder) ---
    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;
#ifdef HAS_UNITREE_HG
    void on_gamepad() override;
#endif

private:
    // ── Constants ────────────────────────────────────────────────
    static constexpr int FUTURE_STEPS = 5;
    static constexpr int NQ            = 29;
    static constexpr int NUM_OBS       = 431;

    // ── Motion data ──────────────────────────────────────────────
    void on_motion(std_msgs::msg::Float32MultiArray::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motion_sub_;

    // Stored in IsaacLab joint order, [T][NQ] / [T][3] / [T][4]
    std::vector<std::vector<float>> mot_joint_pos_;
    std::vector<std::vector<float>> mot_joint_vel_;
    std::vector<std::array<float, 3>> mot_anchor_pos_;
    std::vector<std::array<float, 4>> mot_anchor_ori_;  // wxyz
    int mot_T_     = 0;   // total frames
    int mot_t_     = 0;   // current frame
    bool mot_ready_ = false;

    // ── IsaacLab ↔ MuJoCo reindexing ────────────────────────────
    std::array<int, NQ> mj_to_il_{};  // mj_to_il_[mj] = il
    std::array<int, NQ> il_to_mj_{};  // il_to_mj_[il] = mj
    void build_reindex_tables();

    // ── Frame alignment (ref → robot) ────────────────────────────
    bool frame_init_ = false;
    std::array<float, 3> robot_init_pos_  = {0, 0, 0};
    std::array<float, 4> robot_init_hq_   = {1, 0, 0, 0};  // heading-only quat
    std::array<float, 3> ref_init_pos_    = {0, 0, 0};
    std::array<float, 4> ref_init_hq_     = {1, 0, 0, 0};
    std::array<float, 4> ref2robot_quat_  = {1, 0, 0, 0};

    void setup_init_frame();
    std::pair<std::array<float, 3>, std::array<float, 4>>
    transform_ref_to_robot(const std::array<float, 3>& pos,
                           const std::array<float, 4>& quat) const;
};

}  // namespace cpp_control
