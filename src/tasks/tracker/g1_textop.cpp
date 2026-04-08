#include "cpp_control/tasks/tracker/g1_textop.hpp"

#include <algorithm>
#include <cmath>

namespace cpp_control
{

// ── IsaacLab joint name order (used during training) ────────────
static const std::vector<std::string> IL_JOINTS = {
    "left_hip_pitch_joint",     "right_hip_pitch_joint",     "waist_yaw_joint",
    "left_hip_roll_joint",      "right_hip_roll_joint",      "waist_roll_joint",
    "left_hip_yaw_joint",       "right_hip_yaw_joint",       "waist_pitch_joint",
    "left_knee_joint",          "right_knee_joint",
    "left_shoulder_pitch_joint","right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",   "right_ankle_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint",    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",  "right_shoulder_yaw_joint",
    "left_elbow_joint",         "right_elbow_joint",
    "left_wrist_roll_joint",    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",   "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",     "right_wrist_yaw_joint",
};

// MuJoCo joint name order (hardware / config order)
static const std::vector<std::string> MJ_JOINTS = {
    "left_hip_pitch_joint",     "left_hip_roll_joint",       "left_hip_yaw_joint",
    "left_knee_joint",          "left_ankle_pitch_joint",    "left_ankle_roll_joint",
    "right_hip_pitch_joint",    "right_hip_roll_joint",      "right_hip_yaw_joint",
    "right_knee_joint",         "right_ankle_pitch_joint",   "right_ankle_roll_joint",
    "waist_yaw_joint",          "waist_roll_joint",          "waist_pitch_joint",
    "left_shoulder_pitch_joint","left_shoulder_roll_joint",   "left_shoulder_yaw_joint",
    "left_elbow_joint",         "left_wrist_roll_joint",     "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint","right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint",        "right_wrist_roll_joint",    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
};

// ── Constructor ──────────────────────────────────────────────────

G1TextopNode::G1TextopNode(const std::string& node_name) : G1Node(node_name)
{
    // Level 0+1 init (must be called after vtable is ready)
    init();

    // Build reindex tables
    build_reindex_tables();

    // Load ONNX policy
    std::string onnx_path = this->declare_parameter("onnx_model_path", "");
    if (onnx_path.empty() && config_)
        onnx_path = config_->onnx_path;

    if (!onnx_path.empty())
    {
        policy_ = std::make_unique<ONNXPolicy>(onnx_path);
        RCLCPP_INFO(this->get_logger(), "TextOp policy loaded: %s", onnx_path.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No ONNX model path for textop");
    }

    // Subscribe to motion data (packed Float32MultiArray on /tracker/motion)
    std::string motion_topic = this->declare_parameter("motion_topic", "/tracker/motion");
    motion_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        motion_topic, 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) { on_motion(msg); });
    RCLCPP_INFO(this->get_logger(), "Subscribing to motion topic: %s", motion_topic.c_str());
}

// ── Reindex tables ───────────────────────────────────────────────

void G1TextopNode::build_reindex_tables()
{
    for (int mj = 0; mj < NQ; ++mj)
    {
        auto it = std::find(IL_JOINTS.begin(), IL_JOINTS.end(), MJ_JOINTS[mj]);
        mj_to_il_[mj] = static_cast<int>(std::distance(IL_JOINTS.begin(), it));
    }
    for (int il = 0; il < NQ; ++il)
    {
        auto it = std::find(MJ_JOINTS.begin(), MJ_JOINTS.end(), IL_JOINTS[il]);
        il_to_mj_[il] = static_cast<int>(std::distance(MJ_JOINTS.begin(), it));
    }
}

// ── Motion data callback ─────────────────────────────────────────

void G1TextopNode::on_motion(std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // Expect layout: dim[0] = T, dim[1] = 65  (29+29+3+4)
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

    // Parse into pending buffers (all in IsaacLab joint order)
    pend_joint_pos_.resize(T);
    pend_joint_vel_.resize(T);
    pend_anchor_pos_.resize(T);
    pend_anchor_ori_.resize(T);

    for (int t = 0; t < T; ++t)
    {
        int off = t * cols;
        pend_joint_pos_[t].assign(msg->data.begin() + off,
                                  msg->data.begin() + off + NQ);
        off += NQ;
        pend_joint_vel_[t].assign(msg->data.begin() + off,
                                  msg->data.begin() + off + NQ);
        off += NQ;
        pend_anchor_pos_[t] = {msg->data[off], msg->data[off + 1], msg->data[off + 2]};
        off += 3;
        pend_anchor_ori_[t] = {msg->data[off], msg->data[off + 1],
                               msg->data[off + 2], msg->data[off + 3]};
    }

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

// ── Observation ──────────────────────────────────────────────────

std::vector<float> G1TextopNode::build_observation()
{
    std::vector<float> obs;
    obs.reserve(NUM_OBS);

    // ─ 0. command (290) : future joint pos + vel in IsaacLab order ─
    for (int s = 0; s < FUTURE_STEPS; ++s)
    {
        int idx = std::min(mot_t_ + s, mot_T_ - 1);
        for (int j = 0; j < NQ; ++j)
            obs.push_back(mot_joint_pos_[idx][j]);
    }
    for (int s = 0; s < FUTURE_STEPS; ++s)
    {
        int idx = std::min(mot_t_ + s, mot_T_ - 1);
        for (int j = 0; j < NQ; ++j)
            obs.push_back(mot_joint_vel_[idx][j]);
    }

    // ─ 1. anchor_pos_b (15) : zeroed (matches reference deployment) ─
    for (int i = 0; i < FUTURE_STEPS * 3; ++i)
        obs.push_back(0.0f);

    // ─ 2. anchor_ori_b (30) : relative orientation as 6D rotation ──
    auto robot_quat = robot_state_.imu_quaternion;  // wxyz

    if (!frame_init_)
        setup_init_frame();

    for (int s = 0; s < FUTURE_STEPS; ++s)
    {
        int idx = std::min(mot_t_ + s, mot_T_ - 1);
        auto [ref_pos_r, ref_quat_r] =
            transform_ref_to_robot(mot_anchor_pos_[idx], mot_anchor_ori_[idx]);

        // Relative orientation: robot body → reference
        auto [_, rel_quat] = math::subtract_frames(
            robot_init_pos_, robot_quat, ref_pos_r, ref_quat_r);

        auto r6d = math::quat_to_rotation_6d(rel_quat);
        for (int i = 0; i < 6; ++i)
            obs.push_back(r6d[i]);
    }

    // ─ 3. projected gravity (3) ─────────────────────────────────
    obs::append_projected_gravity(obs, robot_state_);

    // ─ 4. base_lin_vel (3) : zeroed (no odom available) ─────────
    obs.push_back(0.0f);
    obs.push_back(0.0f);
    obs.push_back(0.0f);

    // ─ 5. base_ang_vel (3) ──────────────────────────────────────
    obs::append_gyro(obs, robot_state_);

    // ─ 6. joint_pos_rel (29) in IsaacLab order ─────────────────
    for (int il = 0; il < NQ; ++il)
    {
        int mj = il_to_mj_[il];
        obs.push_back(robot_state_.joint_positions[mj] - default_angles_[mj]);
    }

    // ─ 7. joint_vel (29) in IsaacLab order ──────────────────────
    for (int il = 0; il < NQ; ++il)
    {
        int mj = il_to_mj_[il];
        obs.push_back(robot_state_.joint_velocities[mj]);
    }

    // ─ 8. last_action (29) in IsaacLab order ────────────────────
    obs::append_last_actions(obs, last_actions_, NQ);

    return obs;
}

// ── Policy control ───────────────────────────────────────────────

RobotCommand G1TextopNode::policy_control()
{
    if (!policy_ || !mot_ready_)
        return zeroing_control();

    // Stand mode: lock to the settle frame and continuously re-anchor
    // the reference frame so the policy always sees near-identity
    // relative orientation (prevents yaw drift / limit cycle).
    if (stand_mode_)
    {
        mot_t_ = mot_T_ - 1;
        frame_init_ = false;
    }

    auto obs = build_observation();
    auto action = policy_->predict(obs);  // IsaacLab order

    int n = num_motors();
    RobotCommand cmd;
    cmd.motor_commands.resize(n);

    // Store actions in IsaacLab order (for next observation's last_action)
    for (int il = 0; il < NQ && il < static_cast<int>(action.size()); ++il)
        actions_[il] = action[il];

    // Convert IsaacLab-order actions → MuJoCo-order motor commands
    for (int mj = 0; mj < n; ++mj)
    {
        int il = mj_to_il_[mj];
        cmd.motor_commands[mj].q =
            default_angles_[mj] + action_scale_[mj] * action[il];
        cmd.motor_commands[mj].kp = kps_[mj];
        cmd.motor_commands[mj].kd = kds_[mj];
    }

    last_actions_ = actions_;

    // Advance motion time (clamps at settle frame; skip if standing)
    if (!stand_mode_ && mot_t_ < mot_T_ - 1)
        mot_t_++;

    return cmd;
}

// ── Motion buffer helpers ────────────────────────────────────────

void G1TextopNode::commit_pending_motion()
{
    mot_joint_pos_ = std::move(pend_joint_pos_);
    mot_joint_vel_ = std::move(pend_joint_vel_);
    mot_anchor_pos_ = std::move(pend_anchor_pos_);
    mot_anchor_ori_ = std::move(pend_anchor_ori_);
    mot_T_ = pend_T_;
    mot_t_ = 0;
    mot_ready_ = true;
    frame_init_ = false;
    pend_ready_ = false;
    RCLCPP_INFO(this->get_logger(), "Motion committed: T=%d", mot_T_);
}

void G1TextopNode::pad_pending_motion()
{
    float dt  = config_ ? static_cast<float>(config_->control_dt) : 0.02f;
    float pad = config_ ? static_cast<float>(config_->motion_pad_length) : 0.0f;
    int pad_frames = std::max(0, static_cast<int>(std::round(pad / dt)));
    if (pad_frames == 0)
        return;

    // Nominal pose in IsaacLab joint order
    std::vector<float> nominal_il(NQ);
    for (int il = 0; il < NQ; ++il)
        nominal_il[il] = default_angles_[il_to_mj_[il]];

    bool do_pre  = config_ && config_->pre_motion_pad;
    bool do_post = config_ && config_->post_motion_pad;

    // Pre-pad: linearly interpolate nominal → motion[0]
    if (do_pre && !pend_joint_pos_.empty())
    {
        const auto& first_pos  = pend_joint_pos_.front();
        const auto& first_apos = pend_anchor_pos_.front();
        const auto& first_aori = pend_anchor_ori_.front();

        std::vector<std::vector<float>>    pre_pos(pad_frames);
        std::vector<std::vector<float>>    pre_vel(pad_frames, std::vector<float>(NQ, 0.0f));
        std::vector<std::array<float, 3>>  pre_apos(pad_frames, first_apos);
        std::vector<std::array<float, 4>>  pre_aori(pad_frames, first_aori);

        for (int f = 0; f < pad_frames; ++f)
        {
            float a = static_cast<float>(f + 1) / static_cast<float>(pad_frames + 1);
            pre_pos[f].resize(NQ);
            for (int j = 0; j < NQ; ++j)
                pre_pos[f][j] = (1.0f - a) * nominal_il[j] + a * first_pos[j];
        }

        pend_joint_pos_.insert(pend_joint_pos_.begin(), pre_pos.begin(), pre_pos.end());
        pend_joint_vel_.insert(pend_joint_vel_.begin(), pre_vel.begin(), pre_vel.end());
        pend_anchor_pos_.insert(pend_anchor_pos_.begin(), pre_apos.begin(), pre_apos.end());
        pend_anchor_ori_.insert(pend_anchor_ori_.begin(), pre_aori.begin(), pre_aori.end());
    }

    // Post-pad: linearly interpolate motion[-1] → nominal (joints + anchor)
    // Anchor ramps toward neutral (origin + identity quat) so that stand
    // mode sees the same observation as init_stand_motion().
    if (do_post && !pend_joint_pos_.empty())
    {
        const auto last_pos  = pend_joint_pos_.back();   // copy (back shifts as we push)
        const auto last_apos = pend_anchor_pos_.back();
        const auto last_aori = pend_anchor_ori_.back();
        constexpr std::array<float, 3> zero_pos = {0.0f, 0.0f, 0.0f};
        constexpr std::array<float, 4> identity_ori = {1.0f, 0.0f, 0.0f, 0.0f};

        for (int f = 0; f < pad_frames; ++f)
        {
            float a = static_cast<float>(f + 1) / static_cast<float>(pad_frames + 1);

            std::vector<float> pos(NQ);
            for (int j = 0; j < NQ; ++j)
                pos[j] = (1.0f - a) * last_pos[j] + a * nominal_il[j];

            std::array<float, 3> apos;
            for (int i = 0; i < 3; ++i)
                apos[i] = (1.0f - a) * last_apos[i] + a * zero_pos[i];

            std::array<float, 4> aori;
            float norm = 0.0f;
            for (int i = 0; i < 4; ++i)
            {
                aori[i] = (1.0f - a) * last_aori[i] + a * identity_ori[i];
                norm += aori[i] * aori[i];
            }
            norm = std::sqrt(norm);
            for (int i = 0; i < 4; ++i)
                aori[i] /= norm;

            pend_joint_pos_.push_back(pos);
            pend_joint_vel_.push_back(std::vector<float>(NQ, 0.0f));
            pend_anchor_pos_.push_back(apos);
            pend_anchor_ori_.push_back(aori);
        }
    }

    pend_T_ = static_cast<int>(pend_joint_pos_.size());
    RCLCPP_INFO(this->get_logger(), "Motion padded: T=%d (pre=%s, post=%s, d=%.2fs)",
                pend_T_, do_pre ? "on" : "off", do_post ? "on" : "off", pad);
}

void G1TextopNode::init_stand_motion()
{
    // Synthesise a single settle frame so the policy can run without
    // having received any motion data yet.
    std::vector<float> nominal_il(NQ);
    for (int il = 0; il < NQ; ++il)
        nominal_il[il] = default_angles_[il_to_mj_[il]];

    mot_joint_pos_ = {nominal_il};
    mot_joint_vel_ = {std::vector<float>(NQ, 0.0f)};
    mot_anchor_pos_ = {{0.0f, 0.0f, 0.0f}};
    mot_anchor_ori_ = {{1.0f, 0.0f, 0.0f, 0.0f}};  // identity wxyz
    mot_T_ = 1;
    mot_t_ = 0;
    mot_ready_ = true;
    frame_init_ = false;
}

// ── Joystick / Gamepad ───────────────────────────────────────────

void G1TextopNode::on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
{
    constexpr size_t RB = 5;  // Xbox X-mode right bumper (button index)

    bool rb = (msg->buttons.size() > RB) && (msg->buttons[RB] == 1);
    if (rb && !prev_rb_)
    {
        stand_mode_ = true;
        if (!mot_ready_)
            init_stand_motion();
        frame_init_ = false;
        control_mode_ = ControlMode::POLICY;
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        if (policy_)
            policy_->reset_memory();
        RCLCPP_INFO(this->get_logger(), "-> stand (policy @ settle)");
    }
    prev_rb_ = rb;

    // A exits stand mode (base already switches to normal POLICY)
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
void G1TextopNode::on_gamepad()
{
    if (gamepad_.R1.on_press)
    {
        stand_mode_ = true;
        if (!mot_ready_)
            init_stand_motion();
        frame_init_ = false;
        control_mode_ = ControlMode::POLICY;
        std::fill(actions_.begin(), actions_.end(), 0.0f);
        std::fill(last_actions_.begin(), last_actions_.end(), 0.0f);
        if (policy_)
            policy_->reset_memory();
        RCLCPP_INFO(this->get_logger(), "[GP] -> stand (policy @ default pose)");
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

// ── Frame alignment ──────────────────────────────────────────────

void G1TextopNode::setup_init_frame()
{
    robot_init_pos_ = {0.0f, 0.0f, 0.0f};  // no odom, assume origin
    robot_init_hq_  = math::heading_quat(robot_state_.imu_quaternion);

    ref_init_pos_  = mot_anchor_pos_[mot_t_];
    ref_init_hq_   = math::heading_quat(mot_anchor_ori_[mot_t_]);

    ref2robot_quat_ = math::qmul(robot_init_hq_, math::qinv(ref_init_hq_));
    frame_init_ = true;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Frame aligned (ref → robot)");
}

std::pair<std::array<float, 3>, std::array<float, 4>>
G1TextopNode::transform_ref_to_robot(const std::array<float, 3>& pos,
                                     const std::array<float, 4>& quat) const
{
    if (!frame_init_)
        return {pos, quat};

    // Position: rotate (pos - ref_init_pos) by ref_init_heading^-1, add robot_init_pos
    std::array<float, 3> dp = {pos[0] - ref_init_pos_[0],
                                pos[1] - ref_init_pos_[1],
                                pos[2] - ref_init_pos_[2]};
    auto dp_rot = math::quat_rotate(math::qinv(ref_init_hq_), dp);
    std::array<float, 3> pos_new = {robot_init_pos_[0] + dp_rot[0],
                                     robot_init_pos_[1] + dp_rot[1],
                                     robot_init_pos_[2] + dp_rot[2]};

    // Orientation: ref2robot * ref_quat
    auto quat_new = math::qmul(ref2robot_quat_, quat);

    return {pos_new, quat_new};
}


}  // namespace cpp_control

// ── Entry point ──────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1TextopNode>());
    rclcpp::shutdown();
    return 0;
}
