#include "cpp_control/tasks/tracker/g1_textop.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

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

    // Parse into storage (all in IsaacLab joint order)
    mot_joint_pos_.resize(T);
    mot_joint_vel_.resize(T);
    mot_anchor_pos_.resize(T);
    mot_anchor_ori_.resize(T);

    for (int t = 0; t < T; ++t)
    {
        int off = t * cols;
        mot_joint_pos_[t].assign(msg->data.begin() + off,
                                 msg->data.begin() + off + NQ);
        off += NQ;
        mot_joint_vel_[t].assign(msg->data.begin() + off,
                                 msg->data.begin() + off + NQ);
        off += NQ;
        mot_anchor_pos_[t] = {msg->data[off], msg->data[off + 1], msg->data[off + 2]};
        off += 3;
        mot_anchor_ori_[t] = {msg->data[off], msg->data[off + 1],
                              msg->data[off + 2], msg->data[off + 3]};
    }

    mot_T_ = T;
    mot_t_ = 0;
    mot_ready_ = true;
    frame_init_ = false;  // re-align on new motion

    RCLCPP_INFO(this->get_logger(), "Received motion: T=%d", T);
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
        auto [_, rel_quat] = subtract_frames(
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

    // Advance motion time
    if (mot_t_ < mot_T_ - 1)
        mot_t_++;

    return cmd;
}

// ── Joystick / Gamepad (placeholder) ─────────────────────────────

void G1TextopNode::on_joy(sensor_msgs::msg::Joy::SharedPtr /*msg*/)
{
    // No velocity command needed for motion tracking
}

#ifdef HAS_UNITREE_HG
void G1TextopNode::on_gamepad()
{
    // No velocity command needed for motion tracking
}
#endif

// ── Frame alignment ──────────────────────────────────────────────

void G1TextopNode::setup_init_frame()
{
    robot_init_pos_ = {0.0f, 0.0f, 0.0f};  // no odom, assume origin
    robot_init_hq_  = heading_quat(robot_state_.imu_quaternion);

    ref_init_pos_  = mot_anchor_pos_[0];
    ref_init_hq_   = heading_quat(mot_anchor_ori_[0]);

    ref2robot_quat_ = qmul(robot_init_hq_, qinv(ref_init_hq_));
    frame_init_ = true;

    RCLCPP_INFO(this->get_logger(), "Frame aligned (ref → robot)");
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
    auto dp_rot = qapply(qinv(ref_init_hq_), dp);
    std::array<float, 3> pos_new = {robot_init_pos_[0] + dp_rot[0],
                                     robot_init_pos_[1] + dp_rot[1],
                                     robot_init_pos_[2] + dp_rot[2]};

    // Orientation: ref2robot * ref_quat
    auto quat_new = qmul(ref2robot_quat_, quat);

    return {pos_new, quat_new};
}

// ── Quaternion math ──────────────────────────────────────────────

std::array<float, 4> G1TextopNode::qmul(const std::array<float, 4>& a,
                                         const std::array<float, 4>& b)
{
    return {
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    };
}

std::array<float, 4> G1TextopNode::qinv(const std::array<float, 4>& q)
{
    float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    float s  = 1.0f / std::max(n2, 1e-9f);
    return {q[0]*s, -q[1]*s, -q[2]*s, -q[3]*s};
}

std::array<float, 3> G1TextopNode::qapply(const std::array<float, 4>& q,
                                           const std::array<float, 3>& v)
{
    // R(q) * v  using quaternion-based rotation
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float m00 = 1.f - 2.f*(y*y + z*z), m01 = 2.f*(x*y - w*z), m02 = 2.f*(x*z + w*y);
    float m10 = 2.f*(x*y + w*z), m11 = 1.f - 2.f*(x*x + z*z), m12 = 2.f*(y*z - w*x);
    float m20 = 2.f*(x*z - w*y), m21 = 2.f*(y*z + w*x), m22 = 1.f - 2.f*(x*x + y*y);
    return {
        m00*v[0] + m01*v[1] + m02*v[2],
        m10*v[0] + m11*v[1] + m12*v[2],
        m20*v[0] + m21*v[1] + m22*v[2],
    };
}

std::array<float, 4> G1TextopNode::heading_quat(const std::array<float, 4>& q)
{
    float yaw = std::atan2(2.0f * (q[0]*q[3] + q[1]*q[2]),
                           1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]));
    float hy = yaw * 0.5f;
    return {std::cos(hy), 0.0f, 0.0f, std::sin(hy)};
}

std::pair<std::array<float, 3>, std::array<float, 4>>
G1TextopNode::subtract_frames(const std::array<float, 3>& pos_a,
                              const std::array<float, 4>& quat_a,
                              const std::array<float, 3>& pos_b,
                              const std::array<float, 4>& quat_b)
{
    auto q_inv_a = qinv(quat_a);
    auto q_rel   = qmul(q_inv_a, quat_b);
    std::array<float, 3> dp = {pos_b[0] - pos_a[0],
                                pos_b[1] - pos_a[1],
                                pos_b[2] - pos_a[2]};
    auto t_rel = qapply(q_inv_a, dp);
    return {t_rel, q_rel};
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
