#include "cpp_control/tasks/vibe/g1_adapt_sonic.hpp"

#include <cstring>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include "common/math_utils.hpp"

namespace cpp_control
{

// ── Constructor ──────────────────────────────────────────────────

G1AdaptSonicNode::G1AdaptSonicNode(const std::string& node_name)
    : G1SonicNode(node_name, /*bind_now=*/false), last_token_time_(0, 0, RCL_ROS_TIME)
{
    const std::string tokens_topic = this->declare_parameter("tokens_topic", "/enc/tokens");
    const std::string attn_topic =
        this->declare_parameter("attention_topic", "/vibe/adapt_sonic/attention_mask");
    goal_color_ = static_cast<int>(this->declare_parameter("goal_color", 0));
    stale_ticks_ = static_cast<int>(this->declare_parameter("token_stale_ticks", 5));

    // kv port shape (P, D) is the manifest's truth about the trained encoder
    const auto* kv = manifest_.find_input("kv_tokens__img_tokens");
    if (!kv || kv->shape.size() != 2)
        throw std::runtime_error("g1_adapt_sonic: manifest has no (P, D) kv_tokens__img_tokens "
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

    const auto* aug = manifest_.find_input("augmentation");
    if (aug)
        for (const auto& t : aug->terms)
            if (t.name == "bodywise_contact_cmd")
                contact_cmd_.assign(t.dim, 0.0f);  // no object in the loop yet

    bind_ports();  // virtual make_binding resolves here, after full construction

    tokens_sub_ = this->create_subscription<vision_encoders::msg::ImageTokens>(
        tokens_topic, rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](vision_encoders::msg::ImageTokens::SharedPtr msg) { on_tokens(msg); });
    goal_color_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/vibe/adapt_sonic/goal_color", 10, [this](std_msgs::msg::Int32::SharedPtr msg) {
            goal_color_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "goal_color -> %d", goal_color_);
        });
    if (!attn_name_.empty())
        attn_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(attn_topic, 10);

    std::ostringstream queries;
    for (const auto& q : query_names_)
        queries << q << ' ';
    RCLCPP_INFO(this->get_logger(),
                "g1_adapt_sonic ready: %s | kv (%d, %d) <- %s | queries: %s| goal_color=%d | "
                "attn -> %s",
                manifest_.model_class.c_str(), token_rows_, token_dim_, tokens_topic.c_str(),
                queries.str().c_str(), goal_color_,
                attn_name_.empty() ? "(absent)" : attn_topic.c_str());
}

// ── Extractor-era port writers ───────────────────────────────────

G1SonicNode::Binding G1AdaptSonicNode::make_binding(float* dst, const deploy::PortSpec& port,
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
        if (spec.name == "base_lin_vel")
            // odometry world vel -> base frame (textop hw-proven path)
            return {dst, &spec, [this](float* v) {
                        auto b = math::quat_rotate_inverse(robot_state_.imu_quaternion,
                                                           robot_state_.base_lin_vel_w);
                        v[0] = b[0]; v[1] = b[1]; v[2] = b[2];
                    }};
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
                        std::memcpy(v, contact_cmd_.data(), contact_cmd_.size() * sizeof(float));
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

// ── Tokens ───────────────────────────────────────────────────────

void G1AdaptSonicNode::on_tokens(vision_encoders::msg::ImageTokens::SharedPtr msg)
{
    if (!tokens_seen_)
    {
        // The wired encoder must be the one the policy was trained on.
        const int P = msg->grid_h * msg->grid_w, D = msg->dim;
        if (P != token_rows_ || D != token_dim_)
            throw std::runtime_error(
                "g1_adapt_sonic: encoder '" + msg->header.frame_id + "' emits (" +
                std::to_string(P) + ", " + std::to_string(D) + ") tokens, policy wants (" +
                std::to_string(token_rows_) + ", " + std::to_string(token_dim_) + ")");
        if (!cls_buf_.empty() && msg->cls.empty())
            throw std::runtime_error("g1_adapt_sonic: policy has a q_cls port but encoder '" +
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

RobotCommand G1AdaptSonicNode::policy_control()
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
    return cmd;
}

void G1AdaptSonicNode::publish_attn()
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

}  // namespace cpp_control

// ── Entry point ──────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1AdaptSonicNode>());
    rclcpp::shutdown();
    return 0;
}
