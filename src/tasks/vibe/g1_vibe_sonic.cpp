#include "cpp_control/tasks/vibe/g1_vibe_sonic.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include "common/math_utils.hpp"

namespace cpp_control {

namespace {

bool has_group(const deploy::PortSpec& port, const std::string& name) {
  return std::find(port.groups.begin(), port.groups.end(), name) !=
         port.groups.end();
}

}  // namespace

// ── Constructor ──────────────────────────────────────────────────

G1VibeSonicNode::G1VibeSonicNode(const std::string& node_name)
    : G1SonicNode(node_name, /*bind_now=*/false),
      last_token_time_(0, 0, RCL_ROS_TIME) {
  task_profile_ = vibe::profile_from_task_id(manifest_.task_id);
  vibe::validate_contract(manifest_, task_profile_);
  const std::string expected_family =
      this->declare_parameter("expected_task_family", "");
  if (!expected_family.empty() && expected_family != task_profile_.family)
    throw std::runtime_error("g1_vibe: launch expects task family '" +
                             expected_family + "', artifact is '" +
                             task_profile_.family + "'");
  if (!task_profile_.requires_motion && motion_) {
    RCLCPP_WARN(this->get_logger(),
                "%s ignores motion_path — using the nominal stand reference",
                task_profile_.family.c_str());
    motion_.reset();
    clock_.reset();
    stand_mode_ = true;
    active_motion_ = stand_motion_.get();
    active_clock_ = stand_clock_.get();
  }

  const std::string tokens_topic =
      this->declare_parameter("tokens_topic", "/enc/tokens");
  const std::string attn_topic =
      this->declare_parameter("attention_topic", "/vibe/sonic/attention_mask");
  encoder_tag_ = this->declare_parameter("encoder_tag", "theia-tiny");
  goal_color_ = static_cast<int>(this->declare_parameter("goal_color", 0));
  if (task_profile_.goal == vibe::GoalKind::COLOR &&
      (goal_color_ < 0 || goal_color_ >= vibe::NUM_CUBE_COLORS))
    throw std::runtime_error("g1_vibe: goal_color must be in [0, 5]");
  // Under a planner the reference stream never steps: the planner rate-limits its own
  // lead-in onto every clip, which is the same mechanism the L1 prep provides
  // for a human. Gating on a button here would just deadlock the loop.
  if (planner_ && task_profile_.requires_prep) {
    task_profile_.requires_prep = false;
    RCLCPP_INFO(this->get_logger(),
                "the planner mode: prep gate OFF — the planner ramps onto each clip");
  }
  stale_ticks_ =
      static_cast<int>(this->declare_parameter("token_stale_ticks", 5));

  // kv port shape (P, D) is the manifest's truth about the trained encoder
  const auto* kv = manifest_.find_group("kv_tokens");
  if (!kv || kv->shape.size() != 2)
    throw std::runtime_error(
        "g1_vibe_sonic: manifest has no (P, D) kv_tokens__img_tokens "
        "port — not an extractor export?");
  token_rows_ = static_cast<int>(kv->shape[0]);
  token_dim_ = static_cast<int>(kv->shape[1]);
  kv_buf_.assign(static_cast<size_t>(token_rows_) * token_dim_, 0.0f);
  const auto* q_cls = manifest_.find_group("q_cls");
  cls_buf_.assign(q_cls ? q_cls->dim() : 0, 0.0f);

  // q_* query ports in manifest order == the graph's attn row order
  query_names_ = vibe::query_groups(manifest_);
  for (const auto& out : manifest_.outputs)
    if (out == "attn") attn_name_ = out;

  // the contact command is a clip channel — its width is the graph, not a param
  const auto* aug = manifest_.find_input("augmentation");
  if (aug)
    for (const auto& t : aug->terms) {
      has_contact_command_ |= t.name == "bodywise_contact_cmd";
      if (t.name == "bodywise_contact_cmd" && t.dim != g1::NUM_CONTACT_BODIES)
        throw std::runtime_error(
            "g1_vibe_sonic: bodywise_contact_cmd is " + std::to_string(t.dim) +
            " wide, the contact graph has " +
            std::to_string(g1::NUM_CONTACT_BODIES) +
            " nodes — motion.hpp CONTACT_GRAPH_BODIES is stale vs orcs");
    }

  bind_ports();  // virtual make_binding resolves here, after full construction

  tokens_sub_ = this->create_subscription<vision_encoders::msg::ImageTokens>(
      tokens_topic, rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](vision_encoders::msg::ImageTokens::SharedPtr msg) {
        on_tokens(msg);
      });
  if (task_profile_.goal == vibe::GoalKind::COLOR)
    goal_color_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/vibe/sonic/goal_color", 10,
        [this](std_msgs::msg::Int32::SharedPtr msg) {
          if (msg->data < 0 || msg->data >= vibe::NUM_CUBE_COLORS) {
            RCLCPP_WARN(this->get_logger(), "ignoring invalid goal_color %d",
                        msg->data);
            return;
          }
          goal_color_ = msg->data;
          RCLCPP_INFO(this->get_logger(), "goal_color -> %d (%s)", goal_color_,
                      vibe::cube_color_name(goal_color_));
        });
  if (!attn_name_.empty()) {
    const size_t expected =
        query_names_.size() * static_cast<size_t>(token_rows_);
    if (session_->output_dim(attn_name_) != expected)
      throw std::runtime_error(
          "g1_vibe: attn output size " +
          std::to_string(session_->output_dim(attn_name_)) +
          " != queries*patches " + std::to_string(expected));
    attn_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        attn_topic, 10);
  }

  if (planner_) {
    planner_heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/vibe/planner/heartbeat",
        rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](std_msgs::msg::Empty::SharedPtr) {
          last_planner_ = std::chrono::steady_clock::now();
          planner_seen_ = true;
        });
    planner_watchdog_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() {
          if (planner_seen_ && !planner_fresh()) {
            RCLCPP_ERROR(this->get_logger(),
                         "planner heartbeat lost (>1 s); shutting down tracker");
            rclcpp::shutdown();
          }
        });
  }

  std::ostringstream queries;
  for (const auto& q : query_names_) queries << q << ' ';
  RCLCPP_INFO(this->get_logger(),
              "g1_vibe ready: %s [%s] | kv (%d, %d) <- %s (%s) | queries: %s| "
              "attn -> %s",
              manifest_.task_id.c_str(), task_profile_.family.c_str(),
              token_rows_, token_dim_, tokens_topic.c_str(),
              encoder_tag_.c_str(), queries.str().c_str(),
              attn_name_.empty() ? "(absent)" : attn_topic.c_str());
  if (planner_) {
    RCLCPP_INFO(this->get_logger(),
                "the planner safety gate: RB locks nominal stand; A arms rollout");
  } else if (task_profile_.requires_prep) {
    RCLCPP_INFO(this->get_logger(),
                "prep gate ON: RB stand -> L1 ramp onto the clip's first frame "
                "(%s, %zu joints, %.1f rad/s, %.1f-%.1f s) -> A.",
                prep_joints_name_.c_str(), prep_joints_.size(), prep_rate_,
                prep_min_s_, prep_max_s_);
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "stand-reactive task: A or RB engages the nominal SONIC reference; "
        "no motion/prep stream is used");
  }
}

// ── Extractor-era port writers ───────────────────────────────────

G1SonicNode::Binding G1VibeSonicNode::make_binding(
    float* dst, const deploy::PortSpec& port, const deploy::TermSpec& spec) {
  // q_proprio: current values, no history (orcs proprio_terms)
  if (has_group(port, "q_proprio")) {
    if (spec.name == "projected_gravity")
      return {dst, &spec, [this](float* v) {
                auto g =
                    math::get_projected_gravity(robot_state_.imu_quaternion);
                v[0] = g[0];
                v[1] = g[1];
                v[2] = g[2];
              }};
    // no base_lin_vel: orcs proprio_terms dropped it (no state estimator on hw)
    if (spec.name == "base_ang_vel")
      return {dst, &spec, [this](float* v) {
                const auto& g = robot_state_.imu_gyroscope;
                v[0] = g[0];
                v[1] = g[1];
                v[2] = g[2];
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

  // Repose task command: goal up-face color one-hot.
  if (has_group(port, "q_task_cmd") && spec.name == "object_goal_color")
    return {dst, &spec, [this, dim = spec.dim](float* v) {
              std::fill(v, v + dim, 0.0f);
              if (goal_color_ >= 0 && goal_color_ < dim) v[goal_color_] = 1.0f;
            }};

  // UOLM task command: canonical clip-final object pose in the ENV frame.
  // The goal is query-only in training, so heading-rebasing it here would
  // change the learned contract rather than close a control frame.
  if (has_group(port, "q_task_cmd") && spec.name == "object_goal_ori")
    return {dst, &spec, [this](float* v) {
              if (!motion_ || !motion_->has_object_goal) {
                std::fill(v, v + 6, 0.0f);
                return;
              }
              const auto r6d =
                  math::quat_to_rotation_6d(motion_->object_goal_quat);
              std::copy(r6d.begin(), r6d.end(), v);
            }};
  if (has_group(port, "q_task_cmd") && spec.name == "object_goal_pos")
    return {dst, &spec, [this](float* v) {
              if (!motion_ || !motion_->has_object_goal) {
                std::fill(v, v + 3, 0.0f);
                return;
              }
              std::copy(motion_->object_goal_pos.begin(),
                        motion_->object_goal_pos.end(), v);
            }};

  // augmentation: the planner command stream (orcs robot_motion_cmd_terms)
  if (has_group(port, "augmentation")) {
    if (spec.name == "bodywise_contact_cmd")
      return {dst, &spec, [this](float* v) {
                std::memcpy(v, active_motion_->contact(active_clock_->frame()),
                            g1::NUM_CONTACT_BODIES * sizeof(float));
              }};
    if (spec.name == "robot_root_lin_vel_cmd")
      return {dst, &spec, [this](float* v) {
                auto t = active_motion_->root_lin_vel_b(active_clock_->frame());
                v[0] = t[0];
                v[1] = t[1];
                v[2] = t[2];
              }};
    if (spec.name == "robot_root_ang_vel_cmd")
      return {dst, &spec, [this](float* v) {
                auto t = active_motion_->root_ang_vel_b(active_clock_->frame());
                v[0] = t[0];
                v[1] = t[1];
                // The joystick rotates the synthetic stand reference itself;
                // report the matching twist to the adapter instead of lying
                // that this rolling reference is static.
                v[2] = stand_yaw_reference_active() ? stand_yaw_rate() : t[2];
              }};
  }

  // vision ports: memcpy of the latest /enc/tokens (validated in on_tokens)
  if (has_group(port, "kv_tokens") && spec.name == "img_tokens")
    return {dst, &spec, [this](float* v) {
              std::memcpy(v, kv_buf_.data(), kv_buf_.size() * sizeof(float));
            }};
  if (has_group(port, "q_cls") && spec.name == "img_cls")
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

bool G1VibeSonicNode::motion_contract_ready() const {
  if (!motion_) return false;
  if (task_profile_.requires_twist && !motion_->has_twist) return false;
  if (task_profile_.goal == vibe::GoalKind::OBJECT_POSE &&
      !motion_->has_object_goal)
    return false;
  return true;
}

bool G1VibeSonicNode::tokens_fresh() const {
  if (!tokens_seen_) return false;
  const double dt = config_ ? config_->control_dt : 0.02;
  return (this->now() - last_token_time_).seconds() <= stale_ticks_ * dt;
}

bool G1VibeSonicNode::allow_policy_entry() {
  if (!tokens_fresh()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "policy entry refused — waiting for fresh %s tokens",
                         encoder_tag_.c_str());
    return false;
  }
  return G1SonicNode::allow_policy_entry();
}

bool G1VibeSonicNode::planner_fresh() const {
  return planner_seen_ &&
         std::chrono::steady_clock::now() - last_planner_ <
             std::chrono::seconds(1);
}

void G1VibeSonicNode::on_button_a() {
  if (calibration_lock_) {
    enter_stand();
    RCLCPP_INFO(this->get_logger(),
                "calibration lock: A keeps the nominal stand reference");
    return;
  }
  // The planner supplies its own rate-limited lead-in. Here A is the explicit rollout
  // arm, not the open-loop Repose prep/track transition below.
  if (planner_) {
    if (!planner_fresh()) {
      RCLCPP_WARN(this->get_logger(), "A refused: waiting for planner status");
      return;
    }
    if (!allow_policy_entry()) return;
    G1SonicNode::on_button_a();
    return;
  }
  if (task_profile_.stand_reactive) {
    if (pend_ready_) {
      RCLCPP_WARN(
          this->get_logger(),
          "Dodge ignores staged motions — running the nominal stand reference");
      pend_motion_.reset();
      pend_ready_ = false;
    }
    stand_mode_ = true;
    pending_engage_ = true;
    return;
  }
  G1SonicNode::on_button_a();
}

// ── Engage ───────────────────────────────────────────────────────

void G1VibeSonicNode::engage_reset(bool reset_history) {
  G1SonicNode::engage_reset(reset_history);

  // The contact report is a commit-time announcement for a human. A planner
  // commits every couple of seconds, so say it only on a cold engage.
  if (!has_contact_command_ || !reset_history) return;

  // The contact command rides the clip, so a clip without one silently feeds
  // the adapter zeros. Say what is actually about to be commanded.
  const auto& c = active_motion_->bodywise_contact;
  const int T = active_motion_->num_frames;
  int active_frames = 0;
  std::vector<int> per_body(g1::NUM_CONTACT_BODIES, 0);
  for (int f = 0; f < T; ++f) {
    int hits = 0;
    for (int k = 0; k < g1::NUM_CONTACT_BODIES; ++k) {
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
    RCLCPP_WARN(
        this->get_logger(),
        "contact cmd: clip carries NO contact schedule — zeros to the adapter "
        "(npz needs a sibling contact_matrix.npz; wire needs %d cols)",
        g1::WIRE_COLS_FULL);
  else
    RCLCPP_INFO(this->get_logger(), "contact cmd: %d/%d frames | %s",
                active_frames, T,
                active_frames ? bodies.str().c_str() : "(never in contact)");
}

// ── Tokens ───────────────────────────────────────────────────────

void G1VibeSonicNode::on_tokens(
    vision_encoders::msg::ImageTokens::SharedPtr msg) {
  const int P = msg->grid_h * msg->grid_w;
  const bool shape_ok = P == token_rows_ && msg->dim == token_dim_;
  const bool sizes_ok = msg->patches.size() == kv_buf_.size() &&
                        msg->cls.size() == cls_buf_.size();
  const bool tag_ok =
      encoder_tag_.empty() || msg->header.frame_id == encoder_tag_;
  const bool finite = std::all_of(msg->patches.begin(), msg->patches.end(),
                                  [](float v) { return std::isfinite(v); }) &&
                      std::all_of(msg->cls.begin(), msg->cls.end(),
                                  [](float v) { return std::isfinite(v); });
  if (!shape_ok || !sizes_ok || !tag_ok || !finite) {
    const std::string why =
        "encoder '" + msg->header.frame_id +
        "' message mismatch: got tokens (" + std::to_string(P) + ", " +
        std::to_string(msg->dim) +
        "), patches=" + std::to_string(msg->patches.size()) +
        ", cls=" + std::to_string(msg->cls.size()) + "; want tag='" +
        encoder_tag_ + "', tokens (" + std::to_string(token_rows_) + ", " +
        std::to_string(token_dim_) +
        "), patches=" + std::to_string(kv_buf_.size()) +
        ", cls=" + std::to_string(cls_buf_.size()) +
        (finite ? "" : ", finite values");
    if (!tokens_seen_) throw std::runtime_error("g1_vibe: " + why);
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s",
                          why.c_str());
    return;
  }

  if (!tokens_seen_) {
    tokens_seen_ = true;
    RCLCPP_INFO(this->get_logger(), "tokens flowing: %s (%u x %u, dim %u)",
                msg->header.frame_id.c_str(), msg->grid_h, msg->grid_w,
                msg->dim);
  }

  std::memcpy(kv_buf_.data(), msg->patches.data(),
              kv_buf_.size() * sizeof(float));
  if (!cls_buf_.empty())
    std::memcpy(cls_buf_.data(), msg->cls.data(),
                cls_buf_.size() * sizeof(float));
  last_token_time_ = this->now();
}

// ── Control ──────────────────────────────────────────────────────

RobotCommand G1VibeSonicNode::policy_control() {
  const double dt = config_ ? config_->control_dt : 0.02;
  if (!tokens_seen_)
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "no tokens yet — kv/cls ports run on zeros");
  else if ((this->now() - last_token_time_).seconds() > stale_ticks_ * dt)
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "STALE VISION TOKENS (>%.0f ms) — encoder/camera "
                         "stream stalled; press Y for damping if needed",
                         stale_ticks_ * dt * 1e3);

  auto cmd = G1SonicNode::policy_control();
  publish_attn();

  return cmd;
}

void G1VibeSonicNode::publish_attn() {
  if (!attn_pub_) return;
  const float* attn = session_->output(attn_name_);
  const int n = static_cast<int>(session_->output_dim(attn_name_));
  const int rows = n / token_rows_;
  if (rows != static_cast<int>(query_names_.size()))
    throw std::runtime_error(
        "g1_vibe: attention query count changed after startup");

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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cpp_control::G1VibeSonicNode>());
  rclcpp::shutdown();
  return 0;
}
