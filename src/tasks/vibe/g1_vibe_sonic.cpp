#include "cpp_control/tasks/vibe/g1_vibe_sonic.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "common/math_utils.hpp"

namespace cpp_control
{

// ── Constructor ──────────────────────────────────────────────────

G1VibeSonicNode::G1VibeSonicNode(const std::string& node_name)
    : G1SonicNode(node_name, /*bind_now=*/false), last_token_time_(0, 0, RCL_ROS_TIME)
{
    const std::string tokens_topic = this->declare_parameter("tokens_topic", "/enc/tokens");
    const std::string attn_topic =
        this->declare_parameter("attention_topic", "/vibe/sonic/attention_mask");
    goal_color_ = static_cast<int>(this->declare_parameter("goal_color", 0));
    stale_ticks_ = static_cast<int>(this->declare_parameter("token_stale_ticks", 5));
    prep_rate_ = this->declare_parameter("prep_rate", 1.5);
    prep_min_s_ = this->declare_parameter("prep_min_s", 0.5);
    prep_max_s_ = this->declare_parameter("prep_max_s", 2.0);
    prep_joints_name_ = this->declare_parameter("prep_joints", "arms");
    if (prep_joints_name_ == "arms")
        prep_joints_ = g1::ARM_JOINT_INDICES;
    else if (prep_joints_name_ == "arms_waist")
    {
        prep_joints_ = g1::WAIST_JOINT_INDICES;
        prep_joints_.insert(prep_joints_.end(), g1::ARM_JOINT_INDICES.begin(),
                            g1::ARM_JOINT_INDICES.end());
    }
    else if (prep_joints_name_ == "all")
    {
        prep_joints_.resize(G1_NUM_MOTOR);
        std::iota(prep_joints_.begin(), prep_joints_.end(), 0);
    }
    else
        throw std::runtime_error("g1_vibe_sonic: prep_joints must be arms | arms_waist | all, "
                                 "got '" + prep_joints_name_ + "'");

    // kv port shape (P, D) is the manifest's truth about the trained encoder
    const auto* kv = manifest_.find_input("kv_tokens__img_tokens");
    if (!kv || kv->shape.size() != 2)
        throw std::runtime_error("g1_vibe_sonic: manifest has no (P, D) kv_tokens__img_tokens "
                                 "port — not an extractor export?");
    token_rows_ = static_cast<int>(kv->shape[0]);
    token_dim_ = static_cast<int>(kv->shape[1]);
    kv_buf_.assign(static_cast<size_t>(token_rows_) * token_dim_, 0.0f);
    const auto* q_cls = manifest_.find_input("q_cls");
    cls_buf_.assign(q_cls ? q_cls->dim() : 0, 0.0f);

    // q_* query ports in manifest order == the graph's attn row order
    for (const auto& port : manifest_.inputs)
        if (port.name.rfind("q_", 0) == 0)
            query_names_.push_back(port.name);
    for (const auto& out : manifest_.outputs)
        if (out == "attn")
            attn_name_ = out;

    // the contact command is a clip channel — its width is the graph, not a param
    const auto* aug = manifest_.find_input("augmentation");
    if (aug)
        for (const auto& t : aug->terms)
            if (t.name == "bodywise_contact_cmd" && t.dim != g1::NUM_CONTACT_BODIES)
                throw std::runtime_error(
                    "g1_vibe_sonic: bodywise_contact_cmd is " + std::to_string(t.dim) +
                    " wide, the contact graph has " + std::to_string(g1::NUM_CONTACT_BODIES) +
                    " nodes — motion.hpp CONTACT_GRAPH_BODIES is stale vs orcs");

    bind_ports();  // virtual make_binding resolves here, after full construction

    tokens_sub_ = this->create_subscription<vision_encoders::msg::ImageTokens>(
        tokens_topic, rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](vision_encoders::msg::ImageTokens::SharedPtr msg) { on_tokens(msg); });
    goal_color_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/vibe/sonic/goal_color", 10, [this](std_msgs::msg::Int32::SharedPtr msg) {
            goal_color_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "goal_color -> %d", goal_color_);
        });
    if (!attn_name_.empty())
        attn_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(attn_topic, 10);

    std::ostringstream queries;
    for (const auto& q : query_names_)
        queries << q << ' ';
    RCLCPP_INFO(this->get_logger(),
                "g1_vibe_sonic ready: %s | kv (%d, %d) <- %s | queries: %s| goal_color=%d | "
                "attn -> %s",
                manifest_.model_class.c_str(), token_rows_, token_dim_, tokens_topic.c_str(),
                queries.str().c_str(), goal_color_,
                attn_name_.empty() ? "(absent)" : attn_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
                "prep gate ON: RB stand -> L1 ramp onto the clip's first frame "
                "(%s, %zu joints, %.1f rad/s, %.1f-%.1f s) -> A. A without a prep is refused.",
                prep_joints_name_.c_str(), prep_joints_.size(), prep_rate_, prep_min_s_,
                prep_max_s_);
}

// ── Extractor-era port writers ───────────────────────────────────

G1SonicNode::Binding G1VibeSonicNode::make_binding(float* dst, const deploy::PortSpec& port,
                                                    const deploy::TermSpec& spec)
{
    // q_proprio: current values, no history (orcs proprio_terms)
    if (port.name == "q_proprio")
    {
        if (spec.name == "projected_gravity")
            return {dst, &spec, [this](float* v) {
                        auto g = math::get_projected_gravity(robot_state_.imu_quaternion);
                        v[0] = g[0]; v[1] = g[1]; v[2] = g[2];
                    }};
        // no base_lin_vel: orcs proprio_terms dropped it (no state estimator on hw)
        if (spec.name == "base_ang_vel")
            return {dst, &spec, [this](float* v) {
                        const auto& g = robot_state_.imu_gyroscope;
                        v[0] = g[0]; v[1] = g[1]; v[2] = g[2];
                    }};
        if (spec.name == "joint_pos")
            return {dst, &spec, [this](float* v) {
                        for (int i = 0; i < G1_NUM_MOTOR; ++i)
                            v[i] = robot_state_.joint_positions[i] - default_angles_[i];
                    }};
        if (spec.name == "joint_vel")
            return {dst, &spec, [this](float* v) {
                        for (int i = 0; i < G1_NUM_MOTOR; ++i)
                            v[i] = robot_state_.joint_velocities[i];
                    }};
    }

    // q_task_cmd: goal up-face color one-hot (vibe.repose object_goal_color)
    if (port.name == "q_task_cmd" && spec.name == "object_goal_color")
        return {dst, &spec, [this, dim = spec.dim](float* v) {
                    std::fill(v, v + dim, 0.0f);
                    if (goal_color_ >= 0 && goal_color_ < dim)
                        v[goal_color_] = 1.0f;
                }};

    // augmentation: sys1 command stream (orcs robot_motion_cmd_terms)
    if (port.name == "augmentation")
    {
        if (spec.name == "bodywise_contact_cmd")
            return {dst, &spec, [this](float* v) {
                        std::memcpy(v, active_motion_->contact(active_clock_->frame()),
                                    g1::NUM_CONTACT_BODIES * sizeof(float));
                    }};
        if (spec.name == "robot_root_lin_vel_cmd")
            return {dst, &spec, [this](float* v) {
                        auto t = active_motion_->root_lin_vel_b(active_clock_->frame());
                        v[0] = t[0]; v[1] = t[1]; v[2] = t[2];
                    }};
        if (spec.name == "robot_root_ang_vel_cmd")
            return {dst, &spec, [this](float* v) {
                        auto t = active_motion_->root_ang_vel_b(active_clock_->frame());
                        v[0] = t[0]; v[1] = t[1]; v[2] = t[2];
                    }};
    }

    // vision ports: memcpy of the latest /enc/tokens (validated in on_tokens)
    if (port.name == "kv_tokens__img_tokens" && spec.name == "img_tokens")
        return {dst, &spec, [this](float* v) {
                    std::memcpy(v, kv_buf_.data(), kv_buf_.size() * sizeof(float));
                }};
    if (port.name == "q_cls" && spec.name == "img_cls")
        return {dst, &spec, [this](float* v) {
                    std::memcpy(v, cls_buf_.data(), cls_buf_.size() * sizeof(float));
                }};

    return G1SonicNode::make_binding(dst, port, spec);
}

// ── Prep: stand, driving a lead-in clip onto the clip's first frame ──
//
// SONIC balances throughout — only the reference it tracks moves. Open-loop PD
// to an arbitrary pose would not stabilise on hardware; this walks the same
// policy there. Only prep_joints_ ramp, so under the `arms` default the arm
// reference is continuous at A while legs/waist step by the clip's frame-0
// delta — deliberate: see docs/trackers/custom_sonic.md#which-joints-ramp.

void G1VibeSonicNode::enter_prep()
{
    if (control_mode_ != ControlMode::POLICY || !stand_mode_)
    {
        RCLCPP_WARN(this->get_logger(), "L1 refused — prep runs inside stand: press RB first");
        return;
    }
    if (pend_ready_)
        commit_pending_motion();
    if (!motion_)
    {
        RCLCPP_WARN(this->get_logger(), "L1: no motion loaded (publish-motion <npz>)");
        return;
    }

    // Start from the reference SONIC is holding, not the measured pose: it is
    // the only pose the policy is currently being asked for, so a re-press of
    // L1 mid-ramp continues from where the ramp got to instead of snapping back.
    const int cur = std::clamp(stand_clock_->frame(), 0, stand_motion_->num_frames - 1);
    const std::vector<float> from(stand_motion_->jp(cur), stand_motion_->jp(cur) + G1_NUM_MOTOR);
    const int f = std::clamp(start_frame_, 0, motion_->num_frames - 1);
    const float* f0 = motion_->jp(f);

    // Only prep_joints_ chase the clip; the rest hold nominal, so the pose SONIC
    // must balance on until A is a stance it can hold, not a dynamic keyframe.
    std::vector<float> to = default_angles_;
    for (int i : prep_joints_)
        to[i] = f0[i];

    const auto worst_of = [&](const float* a, const float* b) {
        int idx = 0;
        float max_d = 0.0f;
        for (int i = 0; i < G1_NUM_MOTOR; ++i)
            if (const float d = std::fabs(a[i] - b[i]); d > max_d)
            {
                max_d = d;
                idx = i;
            }
        return std::pair<int, float>{idx, max_d};
    };
    const auto name = [&](int i) {
        return i < static_cast<int>(joint_names_.size()) ? joint_names_[i].c_str() : "?";
    };
    const auto [worst, max_d] = worst_of(to.data(), from.data());   // what the ramp covers
    const auto [held, step] = worst_of(f0, to.data());              // what A still steps by
    const double secs = std::clamp(max_d / prep_rate_, prep_min_s_, prep_max_s_);
    const float fps = motion_->fps;
    const int T = static_cast<int>(std::lround(secs * fps));

    stand_motion_ = std::make_unique<g1::Motion>(g1::Motion::lead_in(from, to, T, fps));
    stand_clock_ = std::make_unique<g1::MotionClock>(*stand_motion_, 0);
    prep_active_ = true;
    prepped_ = false;
    enter_stand();  // stand_mode_, pending_engage_ (active_* still point at the old clip)
    RCLCPP_INFO(this->get_logger(),
                "prep -> frame %d [%s]: ramp max |dq| %.2f rad (%s), %.2f s / %d frames · "
                "handover step %.2f rad (%s)",
                f, prep_joints_name_.c_str(), max_d, name(worst), secs, stand_motion_->num_frames,
                step, step > 0.0f ? name(held) : "none — reference is continuous");
}

void G1VibeSonicNode::restore_stand_reference()
{
    if (!prep_active_)
        return;
    make_stand_motion();
    pending_engage_ = true;  // active_* must never outlive the lead-in they point at
    prep_active_ = false;
    prepped_ = false;
}

void G1VibeSonicNode::on_button_a()
{
    if (pend_ready_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "A refused — a clip was staged after the prep, press L1 again");
        return;
    }
    if (!prepped_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "A refused — RB to stand, then L1 to ramp onto frame %d",
                             start_frame_);
        return;
    }
    restore_stand_reference();   // a later RB must mean nominal again
    G1SonicNode::on_button_a();  // leave stand, engage the clip at t=0
}

// ── Engage ───────────────────────────────────────────────────────

void G1VibeSonicNode::engage_reset()
{
    G1SonicNode::engage_reset();

    // The contact command rides the clip, so a clip without one silently feeds
    // the adapter zeros. Say what is actually about to be commanded.
    const auto& c = active_motion_->bodywise_contact;
    const int T = active_motion_->num_frames;
    int active_frames = 0;
    std::vector<int> per_body(g1::NUM_CONTACT_BODIES, 0);
    for (int f = 0; f < T; ++f)
    {
        int hits = 0;
        for (int k = 0; k < g1::NUM_CONTACT_BODIES; ++k)
        {
            if (c[static_cast<size_t>(f) * g1::NUM_CONTACT_BODIES + k] == 0.0f)
                continue;
            ++hits;
            ++per_body[k];
        }
        active_frames += hits > 0;
    }
    std::ostringstream bodies;
    for (int k = 0; k < g1::NUM_CONTACT_BODIES; ++k)
        if (per_body[k])
            bodies << g1::CONTACT_GRAPH_BODIES[k] << '(' << per_body[k] << ") ";

    if (!active_motion_->has_contact)
        RCLCPP_WARN(this->get_logger(),
                    "contact cmd: clip carries NO contact schedule — zeros to the adapter "
                    "(npz needs a sibling contact_matrix.npz; wire needs %d cols)",
                    g1::WIRE_COLS_FULL);
    else
        RCLCPP_INFO(this->get_logger(), "contact cmd: %d/%d frames | %s", active_frames, T,
                    active_frames ? bodies.str().c_str() : "(never in contact)");
}

// ── Tokens ───────────────────────────────────────────────────────

void G1VibeSonicNode::on_tokens(vision_encoders::msg::ImageTokens::SharedPtr msg)
{
    if (!tokens_seen_)
    {
        // The wired encoder must be the one the policy was trained on.
        const int P = msg->grid_h * msg->grid_w, D = msg->dim;
        if (P != token_rows_ || D != token_dim_)
            throw std::runtime_error(
                "g1_vibe_sonic: encoder '" + msg->header.frame_id + "' emits (" +
                std::to_string(P) + ", " + std::to_string(D) + ") tokens, policy wants (" +
                std::to_string(token_rows_) + ", " + std::to_string(token_dim_) + ")");
        if (!cls_buf_.empty() && msg->cls.empty())
            throw std::runtime_error("g1_vibe_sonic: policy has a q_cls port but encoder '" +
                                     msg->header.frame_id + "' emits no CLS token");
        tokens_seen_ = true;
        RCLCPP_INFO(this->get_logger(), "tokens flowing: %s (%u x %u, dim %u)",
                    msg->header.frame_id.c_str(), msg->grid_h, msg->grid_w, msg->dim);
    }

    std::memcpy(kv_buf_.data(), msg->patches.data(), kv_buf_.size() * sizeof(float));
    if (!cls_buf_.empty())
        std::memcpy(cls_buf_.data(), msg->cls.data(), cls_buf_.size() * sizeof(float));
    last_token_time_ = this->now();
}

// ── Control ──────────────────────────────────────────────────────

RobotCommand G1VibeSonicNode::policy_control()
{
    const double dt = config_ ? config_->control_dt : 0.02;
    if (!tokens_seen_)
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "no tokens yet — kv/cls ports run on zeros");
    else if ((this->now() - last_token_time_).seconds() > stale_ticks_ * dt)
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "tokens stale (>%.0f ms) — encoder alive?", stale_ticks_ * dt * 1e3);

    auto cmd = G1SonicNode::policy_control();
    publish_attn();

    if (prep_active_ && !prepped_ && stand_clock_->finished())
    {
        prepped_ = true;
        const float* target = stand_motion_->jp(stand_motion_->num_frames - 1);
        float err = 0.0f;
        for (int i = 0; i < G1_NUM_MOTOR; ++i)
            err = std::max(err, std::fabs(robot_state_.joint_positions[i] - target[i]));
        RCLCPP_INFO(this->get_logger(), "prepped on frame %d (posture err %.3f rad) — A to run",
                    start_frame_, err);
    }
    return cmd;
}

void G1VibeSonicNode::publish_attn()
{
    if (!attn_pub_)
        return;
    const float* attn = session_->output(attn_name_);
    const int n = static_cast<int>(session_->output_dim(attn_name_));
    const int rows = n / token_rows_;

    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(2);
    std::ostringstream label;  // row order == q_* port order in the manifest
    for (size_t i = 0; i < query_names_.size(); ++i)
        label << (i ? "|" : "") << query_names_[i];
    msg.layout.dim[0].label = label.str();
    msg.layout.dim[0].size = rows;
    msg.layout.dim[0].stride = n;
    msg.layout.dim[1].label = "patch";
    msg.layout.dim[1].size = token_rows_;
    msg.layout.dim[1].stride = token_rows_;
    msg.data.assign(attn, attn + n);
    attn_pub_->publish(msg);
}

// ── Joystick / Gamepad: L1 preps, RB restores the nominal stand ──

void G1VibeSonicNode::on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
{
    const auto down = [&](size_t i) { return msg->buttons.size() > i && msg->buttons[i] == 1; };
    const bool rb = down(joy::XMODE_R1), l1 = down(joy::XMODE_L1);
    if (rb && !prev_rb_)
        restore_stand_reference();  // before the base engages stand on the lead-in
    if (l1 && !prev_l1_)
        enter_prep();
    prev_rb_ = rb;
    prev_l1_ = l1;

    G1SonicNode::on_joy(msg);  // RB -> stand, A -> on_button_a (gated above)

    if (control_mode_ != ControlMode::POLICY)
        restore_stand_reference();  // X / B / Y left policy — the prep is void
}

#ifdef HAS_UNITREE_HG
void G1VibeSonicNode::on_gamepad()
{
    if (gamepad_.R1.on_press)
        restore_stand_reference();
    if (gamepad_.L1.on_press)
        enter_prep();

    G1SonicNode::on_gamepad();

    if (control_mode_ != ControlMode::POLICY)
        restore_stand_reference();
}
#endif

}  // namespace cpp_control

// ── Entry point ──────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1VibeSonicNode>());
    rclcpp::shutdown();
    return 0;
}
