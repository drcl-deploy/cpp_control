#include "cpp_control/tasks/tracker/g1_textop_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "common/g1/joint_orders.hpp"

namespace cpp_control
{

// ── Constructor ──────────────────────────────────────────────────

G1TextopTrackerNode::G1TextopTrackerNode(const std::string& node_name)
    : G1Node(node_name)
{
    // init() is idempotent — safe to call here even if a subclass calls it again
    init();

    std::copy(g1::MJ2IL.begin(), g1::MJ2IL.end(), mj_to_il_.begin());
    std::copy(g1::IL2MJ.begin(), g1::IL2MJ.end(), il_to_mj_.begin());
    wbc_last_actions_.assign(NQ, 0.0f);

    // Motion subscription
    std::string motion_topic = this->declare_parameter("motion_topic", "/tracker/motion");
    motion_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        motion_topic, 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) { on_motion(msg); });
    RCLCPP_INFO(this->get_logger(), "Subscribing to motion topic: %s", motion_topic.c_str());

    // Load WBC policy (from explicit param only; subclasses may override)
    std::string wbc_onnx = this->declare_parameter("onnx_model_path", "");
    if (!wbc_onnx.empty())
    {
        wbc_policy_ = std::make_unique<ONNXPolicy>(wbc_onnx);
        RCLCPP_INFO(this->get_logger(), "WBC policy loaded: %s", wbc_onnx.c_str());
    }
}

// ── Motion data callback ─────────────────────────────────────────

void G1TextopTrackerNode::on_motion(std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->layout.dim.size() < 2)
    {
        RCLCPP_WARN(this->get_logger(), "Bad motion layout (need 2 dims)");
        return;
    }

    int T    = static_cast<int>(msg->layout.dim[0].size);
    int cols = static_cast<int>(msg->layout.dim[1].size);
    if (cols != NQ + NQ + 3 + 4)
    {
        RCLCPP_WARN(this->get_logger(), "Bad motion cols: %d (expected %d)", cols, NQ + NQ + 3 + 4);
        return;
    }
    if (static_cast<int>(msg->data.size()) != T * cols)
    {
        RCLCPP_WARN(this->get_logger(), "Motion data size mismatch");
        return;
    }

    pend_ = g1::Motion::from_wire(T, msg->data.data());
    pend_T_ = T;
    pad_pending_motion();
    pend_ready_ = true;

    if (!stand_mode_)
    {
        commit_pending_motion();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),
                     "Motion staged (T=%d). Will commit on A.", pend_T_);
    }
}

// ── Motion buffer helpers ────────────────────────────────────────

void G1TextopTrackerNode::commit_pending_motion()
{
    mot_ = std::move(pend_);
    pend_ = g1::Motion{};
    mot_T_ = mot_.num_frames;
    mot_t_ = 0;
    mot_ready_ = true;
    frame_init_ = false;
    pend_ready_ = false;
    RCLCPP_INFO(this->get_logger(), "Motion committed: T=%d", mot_T_);
}

void G1TextopTrackerNode::pad_pending_motion()
{
    float dt  = config_ ? static_cast<float>(config_->control_dt) : 0.02f;
    float pad = config_ ? static_cast<float>(config_->motion_pad_length) : 0.0f;
    int pad_frames = std::max(0, static_cast<int>(std::round(pad / dt)));
    if (pad_frames == 0 || pend_.num_frames == 0)
        return;

    bool do_pre  = config_ && config_->pre_motion_pad;
    bool do_post = config_ && config_->post_motion_pad;

    // Nominal pose (MJ order — pend_ is MJ-canonical, interpolation is
    // elementwise so the joint ordering cancels out).
    const std::vector<float>& nominal = default_angles_;

    // Pre-pad: linearly interpolate nominal -> motion[0]
    if (do_pre)
    {
        std::vector<float> jp0(pend_.jp(0), pend_.jp(0) + NQ);
        const auto apos0 = pend_.root_pos(0);
        const auto aori0 = pend_.root_quat(0);

        std::vector<float> jp_ins, jv_ins(static_cast<size_t>(pad_frames) * NQ, 0.0f);
        std::vector<float> ap_ins, aq_ins;
        jp_ins.reserve(static_cast<size_t>(pad_frames) * NQ);
        for (int f = 0; f < pad_frames; ++f)
        {
            float a = static_cast<float>(f + 1) / static_cast<float>(pad_frames + 1);
            for (int j = 0; j < NQ; ++j)
                jp_ins.push_back((1.0f - a) * nominal[j] + a * jp0[j]);
            ap_ins.insert(ap_ins.end(), {apos0[0], apos0[1], apos0[2]});
            aq_ins.insert(aq_ins.end(), {aori0[0], aori0[1], aori0[2], aori0[3]});
        }
        pend_.joint_pos.insert(pend_.joint_pos.begin(), jp_ins.begin(), jp_ins.end());
        pend_.joint_vel.insert(pend_.joint_vel.begin(), jv_ins.begin(), jv_ins.end());
        pend_.body_pos_w.insert(pend_.body_pos_w.begin(), ap_ins.begin(), ap_ins.end());
        pend_.body_quat_w.insert(pend_.body_quat_w.begin(), aq_ins.begin(), aq_ins.end());
        pend_.body_lin_vel_w.insert(pend_.body_lin_vel_w.begin(),
                                    static_cast<size_t>(pad_frames) * 3, 0.0f);
        pend_.body_ang_vel_w.insert(pend_.body_ang_vel_w.begin(),
                                    static_cast<size_t>(pad_frames) * 3, 0.0f);
        pend_.num_frames += pad_frames;
    }

    // Post-pad: linearly interpolate motion[-1] -> nominal (joints + anchor)
    // Anchor ramps toward neutral (heading-only quat) so that stand mode sees
    // the same observation as init_stand_motion().
    if (do_post)
    {
        const int last = pend_.num_frames - 1;
        const std::vector<float> jpL(pend_.jp(last), pend_.jp(last) + NQ);
        const auto aposL = pend_.root_pos(last);
        const auto aoriL = pend_.root_quat(last);
        const auto aoriL_yaw = math::heading_quat(aoriL);

        for (int f = 0; f < pad_frames; ++f)
        {
            float a = static_cast<float>(f + 1) / static_cast<float>(pad_frames + 1);
            for (int j = 0; j < NQ; ++j)
                pend_.joint_pos.push_back((1.0f - a) * jpL[j] + a * nominal[j]);
            pend_.joint_vel.insert(pend_.joint_vel.end(), NQ, 0.0f);
            // anchor pos: hold xy, hardcoded stand target height
            pend_.body_pos_w.insert(pend_.body_pos_w.end(), {aposL[0], aposL[1], 0.76f});
            auto aq = math::quat_slerp(aoriL, aoriL_yaw, a);
            pend_.body_quat_w.insert(pend_.body_quat_w.end(), {aq[0], aq[1], aq[2], aq[3]});
            pend_.body_lin_vel_w.insert(pend_.body_lin_vel_w.end(), 3, 0.0f);
            pend_.body_ang_vel_w.insert(pend_.body_ang_vel_w.end(), 3, 0.0f);
        }
        pend_.num_frames += pad_frames;
    }

    pend_T_ = pend_.num_frames;
    RCLCPP_INFO(this->get_logger(), "Motion padded: T=%d (pre=%s, post=%s, d=%.2fs)",
                pend_T_, do_pre ? "on" : "off", do_post ? "on" : "off", pad);
}

void G1TextopTrackerNode::init_stand_motion()
{
    mot_ = g1::Motion::stand(default_angles_);
    mot_T_ = 1;
    mot_t_ = 0;
    mot_ready_ = true;
    frame_init_ = false;
}

// ── Stand mode ───────────────────────────────────────────────────

void G1TextopTrackerNode::enter_stand_mode()
{
    stand_mode_ = true;
    if (!mot_ready_)
        init_stand_motion();
    frame_init_ = false;
    control_mode_ = ControlMode::POLICY;
    std::fill(wbc_last_actions_.begin(), wbc_last_actions_.end(), 0.0f);
    if (wbc_policy_)
        wbc_policy_->reset_memory();
}

// ── Default WBC-only policy control ─────────────────────────────

RobotCommand G1TextopTrackerNode::policy_control()
{
    if (!wbc_policy_ || !mot_ready_)
        return zeroing_control();

    if (stand_mode_)
    {
        mot_t_ = mot_T_ - 1;
        frame_init_ = false;
    }

    auto obs = build_wbc_observation();
    auto action = wbc_policy_->predict(obs);  // IsaacLab order

    int n = num_motors();
    RobotCommand cmd;
    cmd.motor_commands.resize(n);

    for (int il = 0; il < NQ && il < static_cast<int>(action.size()); ++il)
        actions_[il] = action[il];

    for (int mj = 0; mj < n; ++mj)
    {
        int il = mj_to_il_[mj];
        cmd.motor_commands[mj].q =
            default_angles_[mj] + action_scale_[mj] * action[il];
        cmd.motor_commands[mj].kp = kps_[mj];
        cmd.motor_commands[mj].kd = kds_[mj];
    }

    last_actions_ = actions_;
    wbc_last_actions_ = actions_;

    if (!stand_mode_ && mot_t_ < mot_T_ - 1)
        mot_t_++;

    return cmd;
}

// ── WBC Observation (431 dims) ───────────────────────────────────

std::vector<float> G1TextopTrackerNode::build_wbc_observation()
{
    std::vector<float> obs;
    obs.reserve(WBC_NUM_OBS);

    // ─ 0. command (290) : future joint pos + vel in IsaacLab order
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

    auto robot_quat = robot_state_.imu_quaternion;

    if (!frame_init_)
        setup_init_frame();

    // ─ 1. anchor_pos_b (15) : zeroed (no-odom baseline; the hw-proven choice)
    for (int i = 0; i < FUTURE_STEPS * 3; ++i)
        obs.push_back(0.0f);

    // ─ 2. anchor_ori_b (30) : relative orientation as 6D rotation
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

    // ─ 3. projected gravity (3) ──────────────────────────────────
    obs::append_projected_gravity(obs, robot_state_);

    // ─ 4. base_lin_vel (3) : world vel rotated to body frame ────
    //      Mirrors OG textop deployment: quat_rotate_inverse(imu_quat, odom_vel_w)
    {
        auto vel_b = math::quat_rotate_inverse(robot_state_.imu_quaternion,
                                               robot_state_.base_lin_vel_w);
        obs.push_back(vel_b[0]);
        obs.push_back(vel_b[1]);
        obs.push_back(vel_b[2]);
    }

    // ─ 5. base_ang_vel (3) ───────────────────────────────────────
    obs::append_gyro(obs, robot_state_);

    // ─ 6. joint_pos_rel (29) in IsaacLab order ──────────────────
    for (int il = 0; il < NQ; ++il)
    {
        int mj = il_to_mj_[il];
        obs.push_back(robot_state_.joint_positions[mj] - default_angles_[mj]);
    }

    // ─ 7. joint_vel (29) in IsaacLab order ───────────────────────
    for (int il = 0; il < NQ; ++il)
    {
        int mj = il_to_mj_[il];
        obs.push_back(robot_state_.joint_velocities[mj]);
    }

    // ─ 8. last_action (29) — WBC's own last actions, IsaacLab order
    for (int il = 0; il < NQ; ++il)
        obs.push_back(wbc_last_actions_[il]);

    return obs;
}

// ── Frame alignment ──────────────────────────────────────────────

void G1TextopTrackerNode::setup_init_frame()
{
    // Fixed world anchor for the M->W rebase (hw-proven: origin, not odom pos).
    robot_init_pos_ = {0.0f, 0.0f, 0.0f};
    robot_init_hq_  = math::heading_quat(robot_state_.imu_quaternion);

    ref_init_pos_  = mot_.root_pos(mot_t_);
    ref_init_hq_   = math::heading_quat(mot_.root_quat(mot_t_));

    ref2robot_quat_ = math::qmul(robot_init_hq_, math::qinv(ref_init_hq_));
    frame_init_ = true;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Frame aligned (ref -> robot)");
}

std::pair<std::array<float, 3>, std::array<float, 4>>
G1TextopTrackerNode::transform_ref_to_robot(const std::array<float, 3>& pos,
                                             const std::array<float, 4>& quat) const
{
    if (!frame_init_)
        return {pos, quat};

    std::array<float, 3> dp = {pos[0] - ref_init_pos_[0],
                                pos[1] - ref_init_pos_[1],
                                pos[2] - ref_init_pos_[2]};
    auto dp_rot = math::quat_rotate(math::qinv(ref_init_hq_), dp);
    std::array<float, 3> pos_new = {robot_init_pos_[0] + dp_rot[0],
                                     robot_init_pos_[1] + dp_rot[1],
                                     robot_init_pos_[2] + dp_rot[2]};

    auto quat_new = math::qmul(ref2robot_quat_, quat);

    return {pos_new, quat_new};
}

// ── Joystick / Gamepad ───────────────────────────────────────────

void G1TextopTrackerNode::on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
{
    constexpr size_t RB = 5;  // Xbox X-mode right bumper

    bool rb = (msg->buttons.size() > RB) && (msg->buttons[RB] == 1);
    if (rb && !prev_rb_)
    {
        enter_stand_mode();
        RCLCPP_INFO(this->get_logger(), "-> stand (WBC @ settle)");
    }
    prev_rb_ = rb;

    // A exits stand mode
    if (msg->buttons.size() > joy::XMODE_A && msg->buttons[joy::XMODE_A] == 1 && stand_mode_)
    {
        stand_mode_ = false;
        if (pend_ready_)
            commit_pending_motion();
        else
        {
            mot_t_ = 0;
            frame_init_ = false;
        }
    }
}

#ifdef HAS_UNITREE_HG
void G1TextopTrackerNode::on_gamepad()
{
    if (gamepad_.R1.on_press)
    {
        enter_stand_mode();
        RCLCPP_INFO(this->get_logger(), "[GP] -> stand (WBC @ default pose)");
    }

    // A exits stand mode
    if (gamepad_.A.on_press && stand_mode_)
    {
        stand_mode_ = false;
        if (pend_ready_)
            commit_pending_motion();
        else
        {
            mot_t_ = 0;
            frame_init_ = false;
        }
    }
}
#endif

}  // namespace cpp_control

// ── Entry point (standalone WBC tracker) ─────────────────────────

#ifndef CPP_CONTROL_TEXTOP_TRACKER_LIB
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1TextopTrackerNode>(
        "g1_textop_controller"));
    rclcpp::shutdown();
    return 0;
}
#endif
