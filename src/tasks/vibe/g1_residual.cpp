#include "cpp_control/tasks/vibe/g1_residual.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

namespace cpp_control
{

// ── Constructor ──────────────────────────────────────────────────

G1VibeResidualNode::G1VibeResidualNode(const std::string& node_name)
    : G1TextopTrackerNode(node_name)
{
    // init() already called by G1TextopTrackerNode (idempotent)

    // ── Parse extra config fields directly from YAML ──────────
    std::string config_path = this->get_parameter("config_path").as_string();
    YAML::Node yaml;
    if (!config_path.empty())
        yaml = YAML::LoadFile(config_path);

    // Embedding dimension
    if (yaml["embedding_dim"])
        embedding_dim_ = yaml["embedding_dim"].as<int>();
    embedding_.resize(embedding_dim_, 0.0f);

    // HLC action scale (scalar, matches training cfg.action_scale)
    if (yaml["hlc_action_scale"])
        hlc_action_scale_ = yaml["hlc_action_scale"].as<float>();

    // HLC observation size
    // joint_pos + joint_vel + hlc_last_actions + embed + goal9d + ori6d + wbc_last_actions
    // hlc_num_obs_ = 4 * NQ + embedding_dim_ + 9 + 6;
    hlc_num_obs_ = 3 * NQ + embedding_dim_ + 9 + 6;  // joint_pos + joint_vel + actions + embed + goal9d + ori6d
    if (yaml["hlc_num_obs"])
        hlc_num_obs_ = yaml["hlc_num_obs"].as<int>();

    // ── Load HLC policy ────────────────────────────────────────
    std::string hlc_onnx = this->declare_parameter("hlc_onnx_model_path", "");
    if (hlc_onnx.empty() && config_)
        hlc_onnx = config_->hlc_onnx_path;

    if (!hlc_onnx.empty())
    {
        hlc_policy_ = std::make_unique<ONNXPolicy>(hlc_onnx);
        RCLCPP_INFO(this->get_logger(), "HLC policy loaded: %s", hlc_onnx.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No HLC ONNX model path");
    }

    // ── Override WBC policy if a separate path is given ──────
    std::string wbc_onnx = this->declare_parameter("wbc_onnx_model_path", "");
    if (wbc_onnx.empty() && yaml["wbc_onnx_path"])
        wbc_onnx = yaml["wbc_onnx_path"].as<std::string>();

    if (!wbc_onnx.empty())
    {
        wbc_policy_ = std::make_unique<ONNXPolicy>(wbc_onnx);
        RCLCPP_INFO(this->get_logger(), "WBC policy overridden: %s", wbc_onnx.c_str());
    }

    // ── Initialise action buffers ─────────────────────────────
    hlc_actions_.assign(NQ, 0.0f);
    hlc_last_actions_.assign(NQ, 0.0f);
    wbc_actions_.assign(NQ, 0.0f);

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

    // Motion subscription already set up by G1TextopTrackerNode

    // ── Debug publishers (viser_ghost viewer) ─────────────────
    ghost_motion_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/g1_residual/ghost_motion_state", 10);
    residual_action_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/g1_residual/hlc_debug/residual_action", 10);
    goal_pose_anchor_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/g1_residual/hlc_debug/goal_pose_anchor", 10);

    RCLCPP_INFO(this->get_logger(), "G1 VIBE Residual node ready  (HLC obs=%d, WBC obs=%d)",
                hlc_num_obs_, WBC_NUM_OBS);
}

// ── Stand mode ───────────────────────────────────────────────────

void G1VibeResidualNode::enter_stand_mode()
{
    G1TextopTrackerNode::enter_stand_mode();
    std::fill(hlc_actions_.begin(), hlc_actions_.end(), 0.0f);
    std::fill(hlc_last_actions_.begin(), hlc_last_actions_.end(), 0.0f);
    std::fill(wbc_actions_.begin(), wbc_actions_.end(), 0.0f);
    if (hlc_policy_)
        hlc_policy_->reset_memory();
}

// ── Embedding callback ───────────────────────────────────────────

void G1VibeResidualNode::on_embedding(std_msgs::msg::Float32MultiArray::SharedPtr msg)
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

void G1VibeResidualNode::on_object_goal(std_msgs::msg::Float32MultiArray::SharedPtr msg)
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
    RCLCPP_INFO(this->get_logger(),
                "Object goal received: pos=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f]",
                msg->data[0], msg->data[1], msg->data[2],
                msg->data[3], msg->data[4], msg->data[5], msg->data[6]);
}

// ── HLC Observation ──────────────────────────────────────────────

std::vector<float> G1VibeResidualNode::build_hlc_observation()
{
    std::vector<float> obs;
    obs.reserve(hlc_num_obs_);

    // ─ 0. joint_pos_rel (29) in JOINT_NAMES_EXPR order (= MJ order)
    //      Verified: JOINT_NAMES_EXPR == MJ_JOINTS (preserve_order=True)
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

    // ─ 4. object_goal9d_command (9) — goal in world frame
    //      pos[3] + rot_6d[6]. Mirrors IsaacLab's object_goal9d_command
    //      (no anchoring); /object_goal is published in the same world frame.
    for (int i = 0; i < 3; ++i)
        obs.push_back(object_goal_pos_[i]);
    {
        auto goal_r6d = math::quat_to_rotation_6d(object_goal_quat_);
        for (int i = 0; i < 6; ++i)
            obs.push_back(goal_r6d[i]);
    }

    // ─ 5. robot_ori_mat6d_w (6) — root orientation in world ─────
    {
        auto root_r6d = math::quat_to_rotation_6d(robot_state_.imu_quaternion);
        for (int i = 0; i < 6; ++i)
            obs.push_back(root_r6d[i]);
    }

    // ─ 6. low_level_actions (29) — WBC's last action, IsaacLab order
    //      Mirrors training's `last_low_level_action`, which is the action
    //      fed into WBC's process_actions (i.e. wbc_raw + hlc_scale*hlc_raw).
    for (int il = 0; il < NQ; ++il)
        obs.push_back(wbc_last_actions_[il]);

    return obs;
}

// ── Policy control ───────────────────────────────────────────────

RobotCommand G1VibeResidualNode::policy_control()
{
    if (!wbc_policy_ || !mot_ready_)
        return zeroing_control();

    // Stand mode: lock to settle frame, continuously re-anchor
    if (stand_mode_)
    {
        mot_t_ = mot_T_ - 1;
        frame_init_ = false;
    }

    // --- WBC inference ---
    auto wbc_obs = build_wbc_observation();
    auto wbc_action = wbc_policy_->predict(wbc_obs);  // IsaacLab order

    for (int il = 0; il < NQ && il < static_cast<int>(wbc_action.size()); ++il)
        wbc_actions_[il] = wbc_action[il];

    // --- HLC inference (only in motion tracking mode) ---
    bool run_hlc = !stand_mode_ && (hlc_policy_ != nullptr);
    if (run_hlc)
    {
        auto hlc_obs = build_hlc_observation();
        auto hlc_action = hlc_policy_->predict(hlc_obs);  // IsaacLab order

        for (int il = 0; il < NQ && il < static_cast<int>(hlc_action.size()); ++il)
            hlc_actions_[il] = hlc_action[il];
    }

    // --- Combine actions -> motor commands ---
    int n = num_motors();
    RobotCommand cmd;
    cmd.motor_commands.resize(n);

    for (int mj = 0; mj < n; ++mj)
    {
        int il = mj_to_il_[mj];

        // WBC target + HLC residual (mirrors training chain:
        //   joint = default + action_scale * (wbc_raw + hlc_scale * hlc_raw))
        float q_target = default_angles_[mj] + action_scale_[mj] * wbc_actions_[il];

        if (run_hlc)
            q_target += action_scale_[mj] * hlc_action_scale_ * hlc_actions_[il];

        cmd.motor_commands[mj].q  = q_target;
        cmd.motor_commands[mj].kp = kps_[mj];
        cmd.motor_commands[mj].kd = kds_[mj];
    }

    // Update last actions for next observation
    // WBC last_actions must reflect the combined action (WBC + residual),
    // matching training where process_actions receives the sum.
    for (int il = 0; il < NQ; ++il)
        wbc_last_actions_[il] = wbc_actions_[il] + hlc_action_scale_ * hlc_actions_[il];
    hlc_last_actions_ = hlc_actions_;

    // ── Debug: ghost motion state (for viser viewer) ─────────────
    if (mot_ready_ && !stand_mode_ && ghost_motion_pub_)
    {
        std_msgs::msg::Float32MultiArray gmsg;
        gmsg.data.reserve(NQ + 3 + 4);
        std::array<float, NQ> jp;  // IL order
        mot_.jp_il(mot_t_, jp.data());
        const auto ap = mot_.root_pos(mot_t_);
        const auto aq = mot_.root_quat(mot_t_);
        gmsg.data.insert(gmsg.data.end(), jp.begin(), jp.end());
        gmsg.data.insert(gmsg.data.end(), {ap[0], ap[1], ap[2]});
        gmsg.data.insert(gmsg.data.end(), {aq[0], aq[1], aq[2], aq[3]});
        ghost_motion_pub_->publish(gmsg);
    }

    // ── Debug: residual action (MJ order, units of q - default_angles) ──
    if (run_hlc && residual_action_pub_)
    {
        std_msgs::msg::Float32MultiArray rmsg;
        rmsg.data.reserve(NQ);
        for (int mj = 0; mj < NQ; ++mj)
            rmsg.data.push_back(action_scale_[mj] * hlc_action_scale_
                                * hlc_actions_[mj_to_il_[mj]]);
        residual_action_pub_->publish(rmsg);
    }

    // ── Debug: object goal in anchor frame ──────────────────────
    if (run_hlc && goal_pose_anchor_pub_)
    {
        const auto& iq = robot_state_.imu_quaternion;
        const auto& rp = robot_state_.base_pos_w;
        std::array<float, 3> dp = {
                                    object_goal_pos_[0] - rp[0],
                                    object_goal_pos_[1] - rp[1],
                                    object_goal_pos_[2] - rp[2]};
        auto pos_b  = math::quat_rotate_inverse(iq, dp);
        auto quat_b = math::qmul(math::qinv(iq), object_goal_quat_);

        geometry_msgs::msg::PoseStamped pmsg;
        pmsg.header.stamp = this->get_clock()->now();
        pmsg.header.frame_id = "anchor";
        pmsg.pose.position.x = pos_b[0];
        pmsg.pose.position.y = pos_b[1];
        pmsg.pose.position.z = pos_b[2];
        // ROS quat is xyzw; ours is wxyz
        pmsg.pose.orientation.w = quat_b[0];
        pmsg.pose.orientation.x = quat_b[1];
        pmsg.pose.orientation.y = quat_b[2];
        pmsg.pose.orientation.z = quat_b[3];
        goal_pose_anchor_pub_->publish(pmsg);
    }

    // Advance motion time (clamps at settle frame; skip if standing)
    if (!stand_mode_ && mot_t_ < mot_T_ - 1)
        mot_t_++;

    // Auto-switch to stand mode once playback hits the last frame.
    // Mirrors RB/R1: drops residual, locks WBC at settle, awaits next motion.
    if (!stand_mode_ && mot_ready_ && mot_t_ >= mot_T_ - 1)
    {
        enter_stand_mode();
        RCLCPP_INFO(this->get_logger(), "-> stand (motion complete, T=%d)", mot_T_);
    }

    return cmd;
}

}  // namespace cpp_control

// ── Entry point (excluded when compiled as library for v2) ───────

#ifndef CPP_CONTROL_VIBE_RESIDUAL_LIB
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1VibeResidualNode>());
    rclcpp::shutdown();
    return 0;
}
#endif
