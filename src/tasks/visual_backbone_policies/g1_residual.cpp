#include "cpp_control/tasks/visual_backbone_policies/g1_residual.hpp"

#include <yaml-cpp/yaml.h>

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

G1ResidualNode::G1ResidualNode(const std::string& node_name) : G1Node(node_name)
{
    // Level 0+1 init (must be called after vtable is ready)
    init();

    // Build reindex tables
    build_reindex_tables();

    // ── Parse extra config fields directly from YAML ──────────
    std::string config_path = this->get_parameter("config_path").as_string();
    YAML::Node yaml;
    if (!config_path.empty())
        yaml = YAML::LoadFile(config_path);

    // Embedding dimension
    if (yaml["embedding_dim"])
        embedding_dim_ = yaml["embedding_dim"].as<int>();
    embedding_.resize(embedding_dim_, 0.0f);

    // Residual action scale (MuJoCo order)
    if (yaml["residual_action_scale"])
    {
        auto ras = yaml["residual_action_scale"].as<std::vector<double>>();
        residual_action_scale_.resize(ras.size());
        for (size_t i = 0; i < ras.size(); ++i)
            residual_action_scale_[i] = static_cast<float>(ras[i]);
    }
    else
    {
        residual_action_scale_.assign(NQ, 0.1f);
    }

    // HLC observation size
    hlc_num_obs_ = 3 * NQ + embedding_dim_ + 9 + 6;  // joint_pos + joint_vel + actions + embed + goal9d + ori6d
    if (yaml["hlc_num_obs"])
        hlc_num_obs_ = yaml["hlc_num_obs"].as<int>();

    // ── Load HLC policy (primary ONNX) ────────────────────────
    std::string hlc_onnx = this->declare_parameter("onnx_model_path", "");
    if (hlc_onnx.empty() && config_)
        hlc_onnx = config_->onnx_path;

    if (!hlc_onnx.empty())
    {
        policy_ = std::make_unique<ONNXPolicy>(hlc_onnx);
        RCLCPP_INFO(this->get_logger(), "HLC policy loaded: %s", hlc_onnx.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No HLC ONNX model path");
    }

    // ── Load WBC policy (secondary ONNX, same as textop) ──────
    std::string wbc_onnx = this->declare_parameter("wbc_onnx_model_path", "");
    if (wbc_onnx.empty() && yaml["wbc_onnx_path"])
    {
        // Resolve relative path against models/ directory
        wbc_onnx = yaml["wbc_onnx_path"].as<std::string>();
    }

    if (!wbc_onnx.empty())
    {
        wbc_policy_ = std::make_unique<ONNXPolicy>(wbc_onnx);
        RCLCPP_INFO(this->get_logger(), "WBC policy loaded: %s", wbc_onnx.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No WBC ONNX model path");
    }

    // ── Initialise action buffers ─────────────────────────────
    hlc_actions_.assign(NQ, 0.0f);
    hlc_last_actions_.assign(NQ, 0.0f);
    wbc_actions_.assign(NQ, 0.0f);
    wbc_last_actions_.assign(NQ, 0.0f);

    // ── Subscribe to visual embedding ─────────────────────────
    std::string embedding_topic = this->declare_parameter("embedding_topic", "/theia_embedding");
    embedding_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        embedding_topic, 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) { on_embedding(msg); });
    RCLCPP_INFO(this->get_logger(), "Subscribing to embedding: %s (dim=%d)",
                embedding_topic.c_str(), embedding_dim_);

    // ── Subscribe to object goal ──────────────────────────────
    std::string goal_topic = this->declare_parameter("object_goal_topic", "/object_goal");
    object_goal_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        goal_topic, 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) { on_object_goal(msg); });
    RCLCPP_INFO(this->get_logger(), "Subscribing to object goal: %s", goal_topic.c_str());

    // ── Subscribe to motion data (for WBC) ────────────────────
    std::string motion_topic = this->declare_parameter("motion_topic", "/tracker/motion");
    motion_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        motion_topic, 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) { on_motion(msg); });
    RCLCPP_INFO(this->get_logger(), "Subscribing to motion: %s", motion_topic.c_str());

    RCLCPP_INFO(this->get_logger(), "G1 VBP Residual node ready  (HLC obs=%d, WBC obs=%d)",
                hlc_num_obs_, WBC_NUM_OBS);
}

// ── Reindex tables ───────────────────────────────────────────────

void G1ResidualNode::build_reindex_tables()
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

// ── Embedding callback ───────────────────────────────────────────

void G1ResidualNode::on_embedding(std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    int n = static_cast<int>(msg->data.size());
    if (n != embedding_dim_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Embedding size mismatch: got %d, expected %d", n, embedding_dim_);
        return;
    }
    std::copy(msg->data.begin(), msg->data.end(), embedding_.begin());
    embedding_ready_ = true;
}

// ── Object goal callback ─────────────────────────────────────────

void G1ResidualNode::on_object_goal(std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // Expect 7 floats: pos[3] + quat_wxyz[4]
    if (msg->data.size() < 7)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Object goal needs 7 floats (pos3 + quat_wxyz4), got %zu",
                              msg->data.size());
        return;
    }
    object_goal_pos_  = {msg->data[0], msg->data[1], msg->data[2]};
    object_goal_quat_ = {msg->data[3], msg->data[4], msg->data[5], msg->data[6]};
    object_goal_ready_ = true;
}

// ── Motion data callback (identical to TextOp) ───────────────────

void G1ResidualNode::on_motion(std_msgs::msg::Float32MultiArray::SharedPtr msg)
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

    // Append settle frame (nominal pose, zero vel, last anchor)
    {
        std::vector<float> nominal_il(NQ);
        for (int il = 0; il < NQ; ++il)
            nominal_il[il] = default_angles_[il_to_mj_[il]];

        mot_joint_pos_.push_back(nominal_il);
        mot_joint_vel_.push_back(std::vector<float>(NQ, 0.0f));
        mot_anchor_pos_.push_back(mot_anchor_pos_.back());
        mot_anchor_ori_.push_back(mot_anchor_ori_.back());
        T += 1;
    }

    mot_T_ = T;
    mot_t_ = 0;
    mot_ready_ = true;
    frame_init_ = false;

    RCLCPP_INFO(this->get_logger(), "Received motion: T=%d (incl. settle frame)", T);
}

// ── HLC Observation ──────────────────────────────────────────────

std::vector<float> G1ResidualNode::build_hlc_observation()
{
    std::vector<float> obs;
    obs.reserve(hlc_num_obs_);

    // ─ 0. joint_pos_rel (29) in JOINT_NAMES_EXPR order (= MJ order)
    //      HLC was trained with preserve_order=True + JOINT_NAMES_EXPR
    for (int mj = 0; mj < NQ; ++mj)
        obs.push_back(robot_state_.joint_positions[mj] - default_angles_[mj]);

    // ─ 1. joint_vel (29) in JOINT_NAMES_EXPR order (= MJ order) ─
    for (int mj = 0; mj < NQ; ++mj)
        obs.push_back(robot_state_.joint_velocities[mj]);

    // ─ 2. hlc_last_actions (29) in IsaacLab order ───────────────
    for (int il = 0; il < NQ; ++il)
        obs.push_back(hlc_last_actions_[il]);

    // ─ 3. image_features (embedding_dim) ─────────────────────────
    obs::append_features(obs, embedding_);

    // ─ 4. object_goal9d_anchor (9) — goal relative to robot ─────
    //      pos[3] + rot_6d[6]  in robot body frame
    obs::append_pose9d_body_relative(
        obs,
        robot_init_pos_, robot_state_.imu_quaternion,
        object_goal_pos_, object_goal_quat_);

    // ─ 5. robot_ori_mat6d_w (6) — root orientation in world ─────
    obs::append_rotation_6d(obs, robot_state_.imu_quaternion);

    return obs;
}

// ── WBC Observation (same as TextOp, 431 dims) ──────────────────

std::vector<float> G1ResidualNode::build_wbc_observation()
{
    std::vector<float> obs;
    obs.reserve(WBC_NUM_OBS);

    // ─ 0. command (290) : future joint pos + vel in IsaacLab order
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

    // ─ 1. anchor_pos_b (15) : zeroed (no odom) ──────────────────
    for (int i = 0; i < FUTURE_STEPS * 3; ++i)
        obs.push_back(0.0f);

    // ─ 2. anchor_ori_b (30) : relative orientation as 6D rotation
    auto robot_quat = robot_state_.imu_quaternion;

    if (!frame_init_)
        setup_init_frame();

    for (int s = 0; s < FUTURE_STEPS; ++s)
    {
        int idx = std::min(mot_t_ + s, mot_T_ - 1);
        auto [ref_pos_r, ref_quat_r] =
            transform_ref_to_robot(mot_anchor_pos_[idx], mot_anchor_ori_[idx]);

        auto [_, rel_quat] = math::subtract_frames(
            robot_init_pos_, robot_quat, ref_pos_r, ref_quat_r);

        auto r6d = math::quat_to_rotation_6d(rel_quat);
        for (int i = 0; i < 6; ++i)
            obs.push_back(r6d[i]);
    }

    // ─ 3. projected gravity (3) ──────────────────────────────────
    obs::append_projected_gravity(obs, robot_state_);

    // ─ 4. base_lin_vel (3) : zeroed ──────────────────────────────
    obs.push_back(0.0f);
    obs.push_back(0.0f);
    obs.push_back(0.0f);

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

// ── Policy control ───────────────────────────────────────────────

RobotCommand G1ResidualNode::policy_control()
{
    if (!wbc_policy_ || !mot_ready_)
        return zeroing_control();

    // --- WBC inference ---
    auto wbc_obs = build_wbc_observation();
    auto wbc_action = wbc_policy_->predict(wbc_obs);  // IsaacLab order

    for (int il = 0; il < NQ && il < static_cast<int>(wbc_action.size()); ++il)
        wbc_actions_[il] = wbc_action[il];

    // --- HLC inference (requires embedding) ---
    bool run_hlc = (policy_ != nullptr) && embedding_ready_;
    if (run_hlc)
    {
        auto hlc_obs = build_hlc_observation();
        auto hlc_action = policy_->predict(hlc_obs);  // IsaacLab order

        for (int il = 0; il < NQ && il < static_cast<int>(hlc_action.size()); ++il)
            hlc_actions_[il] = hlc_action[il];
    }

    // --- Combine actions → motor commands ---
    int n = num_motors();
    RobotCommand cmd;
    cmd.motor_commands.resize(n);

    for (int mj = 0; mj < n; ++mj)
    {
        int il = mj_to_il_[mj];

        // WBC target
        float q_target = default_angles_[mj] + action_scale_[mj] * wbc_actions_[il];

        // Add HLC residual
        if (run_hlc)
            q_target += residual_action_scale_[mj] * hlc_actions_[il];

        cmd.motor_commands[mj].q  = q_target;
        cmd.motor_commands[mj].kp = kps_[mj];
        cmd.motor_commands[mj].kd = kds_[mj];
    }

    // Update last actions for next observation
    wbc_last_actions_ = wbc_actions_;
    hlc_last_actions_ = hlc_actions_;

    // Advance motion time (clamps at settle frame)
    if (mot_t_ < mot_T_ - 1)
        mot_t_++;

    return cmd;
}

// ── Joystick / Gamepad (placeholder) ─────────────────────────────

void G1ResidualNode::on_joy(sensor_msgs::msg::Joy::SharedPtr /*msg*/)
{
    // No velocity command needed — behaviour driven by vision + motion
}

#ifdef HAS_UNITREE_HG
void G1ResidualNode::on_gamepad()
{
    // No velocity command needed
}
#endif

// ── Frame alignment (same as TextOp) ─────────────────────────────

void G1ResidualNode::setup_init_frame()
{
    robot_init_pos_ = {0.0f, 0.0f, 0.0f};
    robot_init_hq_  = math::heading_quat(robot_state_.imu_quaternion);

    ref_init_pos_  = mot_anchor_pos_[0];
    ref_init_hq_   = math::heading_quat(mot_anchor_ori_[0]);

    ref2robot_quat_ = math::qmul(robot_init_hq_, math::qinv(ref_init_hq_));
    frame_init_ = true;

    RCLCPP_INFO(this->get_logger(), "Frame aligned (ref → robot)");
}

std::pair<std::array<float, 3>, std::array<float, 4>>
G1ResidualNode::transform_ref_to_robot(const std::array<float, 3>& pos,
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

}  // namespace cpp_control

// ── Entry point ──────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1ResidualNode>());
    rclcpp::shutdown();
    return 0;
}
