#pragma once

#include "cpp_control/robots/g1.hpp"
#include "common/g1/motion.hpp"
#include "common/math_utils.hpp"
#include "common/observation_utils.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>

#include <array>
#include <vector>

namespace cpp_control
{

/**
 * @brief G1 TextOp tracker base — motion handling, WBC obs, frame alignment.
 *
 * Intermediate class (level 1.5) between G1Node and task nodes that need
 * whole-body motion tracking.  Provides:
 *   - Motion subscription + parsing + padding + commit (g1::Motion)
 *   - WBC observation building (431 dims)
 *   - Frame alignment (ref -> robot)
 *   - Stand mode state machine
 *   - Joystick / gamepad handlers
 *
 * WBC observation layout (all in IsaacLab joint order):
 *   command           (290) : future_steps x (joint_pos + joint_vel)
 *   anchor_pos_b       (15) : future_steps x 3
 *   anchor_ori_b       (30) : future_steps x 6  (rotation 6D)
 *   projected_gravity   (3)
 *   base_lin_vel        (3)
 *   base_ang_vel        (3)
 *   joint_pos_rel      (29)
 *   joint_vel_rel      (29)
 *   last_action        (29)
 *                     -----
 *                      431
 */
class G1TextopTrackerNode : public G1Node
{
public:
    explicit G1TextopTrackerNode(const std::string& node_name);
    ~G1TextopTrackerNode() override = default;

protected:
    // ── Default WBC-only policy control (subclasses override) ────
    RobotCommand policy_control() override;

    // ── Constants ────────────────────────────────────────────────
    static constexpr int FUTURE_STEPS = 5;
    static constexpr int NQ            = 29;
    static constexpr int WBC_NUM_OBS   = 431;

    // ── IsaacLab <-> MuJoCo reindexing (views of common/g1/joint_orders) ──
    std::array<int, NQ> mj_to_il_{};
    std::array<int, NQ> il_to_mj_{};

    // ── WBC policy + observation ────────────────────────────────
    std::unique_ptr<ONNXPolicy> wbc_policy_;
    std::vector<float> build_wbc_observation();
    std::vector<float> wbc_last_actions_;

    // ── Motion (active + pending staged while standing) ─────────
    g1::Motion mot_;   ///< active reference (MJ-canonical; anchor = body 0)
    g1::Motion pend_;
    int  mot_T_     = 0;
    int  mot_t_     = 0;
    bool mot_ready_ = false;
    int  pend_T_     = 0;
    bool pend_ready_ = false;

    // ── Motion handling ─────────────────────────────────────────
    void on_motion(std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void commit_pending_motion();
    virtual void pad_pending_motion();
    void init_stand_motion();

    // ── Stand mode ──────────────────────────────────────────────
    bool stand_mode_ = false;
    void on_stand_engaged() override;
    virtual void enter_stand_mode();

    // ── Frame alignment (ref -> robot) ──────────────────────────
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

    // ── Joystick / gamepad ──────────────────────────────────────
    void on_joy(sensor_msgs::msg::Joy::SharedPtr msg) override;
#ifdef HAS_UNITREE_HG
    void on_gamepad() override;
#endif

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motion_sub_;
};

}  // namespace cpp_control
