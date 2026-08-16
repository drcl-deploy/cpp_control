#include "cpp_control/tasks/tracker/g1_sonic.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "common/g1/joint_orders.hpp"
#include "common/math_utils.hpp"

namespace cpp_control {

// ── Constructor ──────────────────────────────────────────────────

G1SonicNode::G1SonicNode(const std::string& node_name)
    : G1SonicNode(node_name, /*bind_now=*/true) {}

G1SonicNode::G1SonicNode(const std::string& node_name, bool bind_now)
    : G1Node(node_name), last_policy_tick_(0, 0, RCL_ROS_TIME) {
  construct(bind_now);
}

void G1SonicNode::construct(bool bind_now) {
  const std::string onnx_path = this->declare_parameter("onnx_path", "");
  std::string manifest_path = this->declare_parameter("manifest_path", "");
  const std::string motion_path = this->declare_parameter("motion_path", "");
  future_steps_ = this->declare_parameter("future_steps", 10);
  frame_skip_ = this->declare_parameter("frame_skip", 5);
  anchor_body_ = this->declare_parameter("anchor_body_index", 0);
  cmd_frame_skip_ = this->declare_parameter("cmd_frame_skip", 1);
  start_frame_ = this->declare_parameter("motion_start_frame", 0);
  const bool il_ordered = this->declare_parameter("il_ordered", false);
  const std::string motion_topic =
      this->declare_parameter("motion_topic", "/tracker/motion");
  const std::string reference_topic =
      this->declare_parameter("reference_topic", "/tracker/reference");
  sys1_ = this->declare_parameter("sys1", false);

  if (onnx_path.empty())
    throw std::runtime_error("g1_sonic: onnx_path is required");
  if (manifest_path.empty()) {
    // <model>.onnx → <model>.manifest.json (the exporter writes them side by
    // side)
    manifest_path =
        onnx_path.substr(0, onnx_path.rfind(".onnx")) + ".manifest.json";
  }

  manifest_ = deploy::DeployManifest::load(manifest_path);
  if (!motion_path.empty()) {
    motion_ = std::make_unique<g1::Motion>(g1::Motion::from_npz(
        motion_path, il_ordered ? g1::MJ2IL : std::vector<int>{}));
    clock_ = std::make_unique<g1::MotionClock>(*motion_, anchor_body_);
  }
  // No clip -> boot into stand; motions arrive over motion_topic and A starts
  // them.
  stand_mode_ = (motion_ == nullptr);
  motion_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      motion_topic, 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        on_motion(msg);
      });
  reference_sub_ = this->create_subscription<cpp_control::msg::MotionReference>(
      reference_topic, 10,
      [this](cpp_control::msg::MotionReference::SharedPtr msg) {
        on_reference(msg);
      });
  if (sys1_) {
    status_pub_ = this->create_publisher<cpp_control::msg::Sys0Status>(
        this->declare_parameter("status_topic", "/vibe/sys0/status"),
        rclcpp::QoS(1).best_effort().durability_volatile());
    // Its own timer, not the control tick: the planner has to hear "not
    // accepting" too, and that is exactly when policy_control is not running.
    const auto hz = this->declare_parameter("status_rate_hz", 20.0);
    status_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / hz)),
        [this]() { publish_sys0_status(); });
  }

  init();
  apply_manifest_action_meta();
  make_stand_motion();
  active_motion_ = motion_.get();
  active_clock_ = clock_.get();

  if (config_ && std::abs(config_->control_dt - manifest_.step_dt) > 1e-6)
    RCLCPP_WARN(
        this->get_logger(),
        "control_dt %.4f != manifest step_dt %.4f — timer runs at control_dt",
        config_->control_dt, manifest_.step_dt);

  policy_actions_.assign(G1_NUM_MOTOR, 0.0f);
  session_ = std::make_unique<deploy::OnnxSession>(onnx_path);
  if (std::find(manifest_.outputs.begin(), manifest_.outputs.end(),
                "actions") == manifest_.outputs.end())
    throw std::runtime_error("g1_sonic: manifest has no 'actions' output");
  output_name_ = "actions";
  if (session_->output_dim(output_name_) != G1_NUM_MOTOR)
    throw std::runtime_error("g1_sonic: actions output is not 29 floats");
  if (!bind_now) return;  // subclass finishes: extra state, then bind_ports()
  bind_ports();

  if (motion_)
    RCLCPP_INFO(this->get_logger(),
                "g1_sonic ready: %s (%s) | %zu input ports | motion %d frames "
                "@ %.0f fps%s",
                onnx_path.c_str(), manifest_.model_class.c_str(),
                manifest_.inputs.size(), motion_->num_frames,
                static_cast<double>(motion_->fps),
                il_ordered ? " (IL->MJ remapped)" : "");
  else
    RCLCPP_INFO(
        this->get_logger(),
        "g1_sonic ready: %s (%s) | %zu input ports | no clip — stand until a "
        "motion arrives on %s (publish-motion <npz>)",
        onnx_path.c_str(), manifest_.model_class.c_str(),
        manifest_.inputs.size(), motion_topic.c_str());
}

// ── Streamed motion (textop wire: [jp29 | jv29 | apos3 | aquat4], IL order) ──

void G1SonicNode::on_motion(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (typed_reference_seen_) {
    if (!legacy_ignore_logged_) {
      RCLCPP_INFO(this->get_logger(),
                  "typed reference active — ignoring legacy motion messages");
      legacy_ignore_logged_ = true;
    }
    return;
  }
  const int cols = msg->layout.dim.size() < 2
                       ? 0
                       : static_cast<int>(msg->layout.dim[1].size);
  if ((cols != g1::WIRE_COLS_MIN && cols != g1::WIRE_COLS_FULL) ||
      static_cast<int>(msg->data.size()) !=
          static_cast<int>(msg->layout.dim[0].size) * cols) {
    RCLCPP_WARN(this->get_logger(),
                "bad motion wire (need [T, %d] or [T, %d], got cols=%d)",
                g1::WIRE_COLS_MIN, g1::WIRE_COLS_FULL, cols);
    return;
  }
  const int T = static_cast<int>(msg->layout.dim[0].size);
  pend_motion_ = std::make_unique<g1::Motion>(
      g1::Motion::from_wire(T, msg->data.data(), cols));
  pend_ready_ = true;
  RCLCPP_INFO(this->get_logger(),
              "motion staged (T=%d @ %.0f fps, %d cols%s) — A to start", T,
              static_cast<double>(pend_motion_->fps), cols,
              pend_motion_->has_contact ? "" : " — NO twist/contact cmds");
}

void G1SonicNode::on_reference(
    cpp_control::msg::MotionReference::SharedPtr msg) {
  using Msg = cpp_control::msg::MotionReference;
  if (msg->schema_version > Msg::SCHEMA_VERSION || msg->frames == 0 ||
      msg->cols != static_cast<uint32_t>(g1::WIRE_COLS_FULL) ||
      !std::isfinite(msg->fps) || msg->fps <= 0.0f ||
      msg->data.size() != static_cast<size_t>(msg->frames) * msg->cols) {
    RCLCPP_WARN(this->get_logger(),
                "bad MotionReference v%u id='%s' (need v%u, frames>0, cols=%d, "
                "finite fps, exact data size)",
                msg->schema_version, msg->reference_id.c_str(),
                Msg::SCHEMA_VERSION, g1::WIRE_COLS_FULL);
    return;
  }
  if (!std::isfinite(msg->entry_yaw_offset)) {
    RCLCPP_WARN(this->get_logger(),
                "MotionReference '%s' has a non-finite entry_yaw_offset",
                msg->reference_id.c_str());
    return;
  }
  if (!std::all_of(msg->data.begin(), msg->data.end(),
                   [](float v) { return std::isfinite(v); })) {
    RCLCPP_WARN(this->get_logger(),
                "MotionReference '%s' contains non-finite data",
                msg->reference_id.c_str());
    return;
  }

  auto motion = std::make_unique<g1::Motion>(g1::Motion::from_wire(
      static_cast<int>(msg->frames), msg->data.data(),
      static_cast<int>(msg->cols), msg->has_twist, msg->has_contact, msg->fps));
  if (msg->has_object_goal) {
    const bool finite =
        std::all_of(msg->object_goal_pos.begin(), msg->object_goal_pos.end(),
                    [](float v) { return std::isfinite(v); }) &&
        std::all_of(msg->object_goal_quat_wxyz.begin(),
                    msg->object_goal_quat_wxyz.end(),
                    [](float v) { return std::isfinite(v); });
    float norm2 = 0.0f;
    for (float v : msg->object_goal_quat_wxyz) norm2 += v * v;
    if (!finite || norm2 < std::numeric_limits<float>::epsilon()) {
      RCLCPP_WARN(this->get_logger(),
                  "MotionReference '%s' has an invalid object goal",
                  msg->reference_id.c_str());
      return;
    }
    const float inv_norm = 1.0f / std::sqrt(norm2);
    std::copy(msg->object_goal_pos.begin(), msg->object_goal_pos.end(),
              motion->object_goal_pos.begin());
    for (size_t i = 0; i < motion->object_goal_quat.size(); ++i)
      motion->object_goal_quat[i] = msg->object_goal_quat_wxyz[i] * inv_norm;
    motion->has_object_goal = true;
  }

  typed_reference_seen_ = true;
  pend_motion_ = std::move(motion);
  pend_ready_ = true;
  pend_entry_yaw_ = msg->entry_yaw_offset;
  pend_reference_id_ = msg->reference_id;

  // Under a planner the reference IS the command: staging it behind a button
  // would stall the loop it closes. The FSM still owns entry into POLICY, so a
  // reference arriving anywhere else waits exactly as it does for a human.
  if (sys1_ && control_mode_ == ControlMode::POLICY) {
    stand_mode_ = false;  // before the commit, so it rebinds active_* to the clip
    commit_pending_motion();
    pending_engage_ = true;
    return;
  }
  RCLCPP_INFO(
      this->get_logger(),
      "reference staged: '%s' (T=%u @ %.0f fps, twist=%s contact=%s goal=%s) — "
      "A to start",
      msg->reference_id.c_str(), msg->frames, static_cast<double>(msg->fps),
      msg->has_twist ? "yes" : "no", msg->has_contact ? "yes" : "no",
      msg->has_object_goal ? "yes" : "no");
}

void G1SonicNode::commit_pending_motion() {
  motion_ = std::move(pend_motion_);
  pend_ready_ = false;
  // Wire motions carry the anchor body only; clamp the anchor index.
  int anchor = anchor_body_;
  if (anchor >= motion_->num_bodies) {
    RCLCPP_WARN(this->get_logger(),
                "anchor_body %d > wire bodies %d — using body 0", anchor_body_,
                motion_->num_bodies);
    anchor = 0;
  }
  clock_ = std::make_unique<g1::MotionClock>(*motion_, anchor);
  rebind_active();  // the two lines above freed what active_* may have named
  start_frame_ = 0;  // streamed clips always start at their first frame
  entry_yaw_ = pend_entry_yaw_;
  reference_id_ = pend_reference_id_;
  pend_entry_yaw_ = 0.0f;
  if (sys1_) return;  // a planner commits every couple of seconds; stay quiet
  RCLCPP_INFO(this->get_logger(), "motion committed: %d frames @ %.0f fps",
              motion_->num_frames, static_cast<double>(motion_->fps));
}

void G1SonicNode::on_button_a() {
  if (pend_ready_) commit_pending_motion();
  if (!motion_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "no motion loaded — staying in stand (publish-motion <npz>)");
    return;
  }
  stand_mode_ = false;
  pending_engage_ = true;
}

// ── Stand reference (textop enter_stand_mode equivalent) ─────────

void G1SonicNode::make_stand_motion() {
  stand_motion_ = std::make_unique<g1::Motion>(
      g1::Motion::stand(default_angles_, motion_ ? motion_->fps : 50.0f));
  stand_clock_ = std::make_unique<g1::MotionClock>(*stand_motion_, 0);
  rebind_active();
}

void G1SonicNode::enter_stand() {
  stand_mode_ = true;
  pending_engage_ = true;
  control_mode_ = ControlMode::POLICY;
}

// ── Manifest is the authority on action metadata ─────────────────

void G1SonicNode::apply_manifest_action_meta() {
  const auto& a = manifest_.action;
  if (static_cast<int>(a.joint_names.size()) != G1_NUM_MOTOR)
    throw std::runtime_error("g1_sonic: manifest has " +
                             std::to_string(a.joint_names.size()) +
                             " joints, expected 29");
  // The checkpoint's joint order must equal the hardware motor order
  // (G1 29-dof: unitree_hg index == mjlab MJ index).
  for (int i = 0; i < G1_NUM_MOTOR; ++i)
    if (!joint_names_.empty() && joint_names_[i] != a.joint_names[i])
      throw std::runtime_error("g1_sonic: joint order mismatch at " +
                               std::to_string(i) + ": config '" +
                               joint_names_[i] + "' vs manifest '" +
                               a.joint_names[i] + "'");
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    default_angles_[i] = a.default_joint_pos[i];
    kps_[i] = a.stiffness[i];
    kds_[i] = a.damping[i];
    action_scale_[i] = a.scale[i];
  }
}

// ── Port binding ─────────────────────────────────────────────────

void G1SonicNode::bind_ports() {
  for (const auto& port : manifest_.inputs) {
    if (!session_->has_input(port.name))
      throw std::runtime_error("g1_sonic: manifest port '" + port.name +
                               "' not found in the ONNX graph");
    float* buf = session_->input(port.name);
    if (static_cast<int>(session_->input_dim(port.name)) != port.dim())
      throw std::runtime_error("g1_sonic: port '" + port.name +
                               "' dim mismatch: graph " +
                               std::to_string(session_->input_dim(port.name)) +
                               " vs manifest " + std::to_string(port.dim()));
    for (const auto& term : port.terms) {
      if (term.offset + term.dim > port.dim())
        throw std::runtime_error("g1_sonic: term '" + term.name +
                                 "' overruns port '" + port.name + "'");
      bindings_.push_back(make_binding(buf + term.offset, port, term));
    }
  }
}

G1SonicNode::Binding G1SonicNode::make_binding(float* dst,
                                               const deploy::PortSpec& port,
                                               const deploy::TermSpec& spec) {
  auto history_binding = [&](int width,
                             std::function<void(float*)> compute) -> Binding {
    if (spec.history <= 0 || spec.dim != width * spec.history)
      throw std::runtime_error("g1_sonic: term '" + spec.name + "' dim " +
                               std::to_string(spec.dim) + " != width " +
                               std::to_string(width) + " x history " +
                               std::to_string(spec.history));
    auto hist = std::make_unique<obs::HistoryTerm>(width, spec.history);
    obs::HistoryTerm* h = hist.get();
    histories_.push_back(std::move(hist));
    history_updates_.push_back([h, compute, width]() {
      // scratch on the stack is fine: widths are tiny (3 or 29)
      float value[64];
      compute(value);
      h->push(value);
    });
    return {dst, &spec, [h](float* out) { h->write(out); }};
  };

  // ── proprio terms (mocke/sonic/profile.py policy_obs_terms) ──
  if (spec.name == "base_ang_vel")
    return history_binding(3, [this](float* v) {
      const auto& g = robot_state_.imu_gyroscope;
      v[0] = g[0];
      v[1] = g[1];
      v[2] = g[2];
    });
  if (spec.name == "joint_pos")
    return history_binding(G1_NUM_MOTOR, [this](float* v) {
      for (int i = 0; i < G1_NUM_MOTOR; ++i)
        v[i] = robot_state_.joint_positions[i] - default_angles_[i];
    });
  if (spec.name == "joint_vel")
    return history_binding(G1_NUM_MOTOR, [this](float* v) {
      for (int i = 0; i < G1_NUM_MOTOR; ++i)
        v[i] = robot_state_.joint_velocities[i];
    });
  if (spec.name == "actions")
    return history_binding(G1_NUM_MOTOR, [this](float* v) {
      std::memcpy(v, policy_actions_.data(), G1_NUM_MOTOR * sizeof(float));
    });
  if (spec.name == "gravity_dir")
    return history_binding(3, [this](float* v) {
      auto g = math::get_projected_gravity(robot_state_.imu_quaternion);
      v[0] = g[0];
      v[1] = g[1];
      v[2] = g[2];
    });

  // ── tokenizer term (mocke/sonic/mdp/observations.py sonic_g1_tokenizer) ──
  if (spec.name == "g1_tokenizer") {
    const int expected = future_steps_ * (2 * G1_NUM_MOTOR + 6);
    if (spec.dim != expected)
      throw std::runtime_error("g1_sonic: tokenizer dim " +
                               std::to_string(spec.dim) +
                               " != F*(2J+6) = " + std::to_string(expected));
    tokenizer_flat_.assign(2 * future_steps_ * G1_NUM_MOTOR, 0.0f);
    return {dst, &spec, [this](float* out) { fill_tokenizer(out); }};
  }

  // ── adapter stream (mocke env_cfg adapter=True: FutureMotionCommand.command)
  // ── [jp(f0..fF-1).flat | jv(f0..fF-1).flat] at cmd_frame_skip; F from the
  // dim.
  if (spec.name == "motion_cmd") {
    const int J = G1_NUM_MOTOR;
    if (spec.dim % (2 * J) != 0)
      throw std::runtime_error("g1_sonic: motion_cmd dim " +
                               std::to_string(spec.dim) +
                               " not divisible by 2*J");
    const int F = spec.dim / (2 * J);
    return {dst, &spec, [this, F, J](float* out) {
              for (int s = 0; s < F; ++s) {
                const int f = active_clock_->future_frame(s * cmd_frame_skip_);
                std::memcpy(out + s * J, active_motion_->jp(f),
                            J * sizeof(float));
                std::memcpy(out + (F + s) * J, active_motion_->jv(f),
                            J * sizeof(float));
              }
            }};
  }

  throw std::runtime_error(
      "g1_sonic: no writer for term '" + spec.name + "' in port '" + port.name +
      "' — known: base_ang_vel joint_pos joint_vel actions gravity_dir "
      "g1_tokenizer motion_cmd");
}

// ── Tokenizer (verbatim sonic_g1_tokenizer op-chain) ─────────────
//
// flat = [jp(f0..fF) | jv(f0..fF)] reinterpreted as (F, 2J) rows; per-frame
// output row = [flat_row(2J) | 6D of quat_inv(robot_pelvis) * ref_pelvis(f)].

void G1SonicNode::fill_tokenizer(float* dst) {
  const int F = future_steps_, J = active_motion_->num_joints,
            skip = frame_skip_;

  for (int s = 0; s < F; ++s) {
    const int f = active_clock_->future_frame(s * skip);
    std::memcpy(&tokenizer_flat_[s * J], active_motion_->jp(f),
                J * sizeof(float));
    std::memcpy(&tokenizer_flat_[(F + s) * J], active_motion_->jv(f),
                J * sizeof(float));
  }

  const auto& robot_quat = robot_state_.imu_quaternion;
  const int row = 2 * J + 6;
  for (int s = 0; s < F; ++s) {
    std::memcpy(dst + s * row, &tokenizer_flat_[s * 2 * J],
                2 * J * sizeof(float));
    const int f = active_clock_->future_frame(s * skip);
    auto rot_dif =
        math::qmul(math::qinv(robot_quat), active_clock_->aligned_root_quat(f));
    auto r6d = math::quat_to_rotation_6d(rot_dif);
    std::memcpy(dst + s * row + 2 * J, r6d.data(), 6 * sizeof(float));
  }
}

// ── Engage / reset ───────────────────────────────────────────────

// Two owning pairs, one pair of raw views. A mode switch only changes WHICH
// pair is read and is safe to defer to the next policy tick; replacing an owner
// is not, because it frees the object a view may still name — and the sys1
// status timer runs outside that tick. So every replacement calls this.
void G1SonicNode::rebind_active() {
  active_motion_ = stand_mode_ ? stand_motion_.get() : motion_.get();
  active_clock_ = stand_mode_ ? stand_clock_.get() : clock_.get();
}

void G1SonicNode::engage_reset(bool reset_history) {
  rebind_active();
  active_clock_->engage(robot_state_.imu_quaternion,
                        stand_mode_ ? 0 : start_frame_,
                        stand_mode_ ? 0.0f : entry_yaw_);
  // A soft re-engage keeps the histories: the policy's proprio terms carry 10
  // steps and a planner swaps references far faster than that refills.
  if (reset_history) {
    for (auto& h : histories_) h->reset();
    std::fill(policy_actions_.begin(), policy_actions_.end(), 0.0f);
  }
  motion_end_logged_ = false;
  if (!reset_history) return;
  if (stand_mode_)
    RCLCPP_INFO(this->get_logger(),
                "sonic engaged: STAND (nominal-pose reference)");
  else
    RCLCPP_INFO(this->get_logger(),
                "sonic engaged: track from frame %d, heading aligned%s",
                start_frame_,
                entry_yaw_ != 0.0f ? " + planner yaw" : "");
}

void G1SonicNode::publish_sys0_status() {
  if (!status_pub_ || !active_motion_ || !active_clock_) return;
  cpp_control::msg::Sys0Status s;
  s.control_mode = static_cast<uint8_t>(control_mode_);
  s.stand = stand_mode_;
  s.accepting = control_mode_ == ControlMode::POLICY;
  s.reference_id = stand_mode_ ? "" : reference_id_;
  s.frame = static_cast<uint32_t>(std::max(0, active_clock_->frame()));
  s.frames = static_cast<uint32_t>(std::max(0, active_motion_->num_frames));
  s.finished = active_clock_->finished();
  s.default_joint_pos = default_angles_;  // manifest-sourced; sys1's v7 stand
  status_pub_->publish(s);
}

// ── Control ──────────────────────────────────────────────────────

RobotCommand G1SonicNode::policy_control() {
  const double dt = config_ ? config_->control_dt : 0.02;

  // Fresh engage = first policy tick after any other mode (detected via the
  // gap in policy_control call times) OR an explicit stand<->track switch
  // (pending_engage_, since those never leave POLICY).
  const auto now = this->now();
  const bool from_elsewhere = (now - last_policy_tick_).seconds() > 5.0 * dt;
  if (pending_engage_ || from_elsewhere) {
    // Arriving from another control mode is a cold start; a reference swap
    // inside POLICY is not — under sys1 that happens every couple of seconds.
    engage_reset(/*reset_history=*/from_elsewhere || !sys1_);
    pending_engage_ = false;
  }
  last_policy_tick_ = now;

  for (auto& update : history_updates_) update();
  for (auto& b : bindings_) b.write(b.dst);

  session_->run();
  const float* act = session_->output(output_name_);
  std::memcpy(policy_actions_.data(), act, G1_NUM_MOTOR * sizeof(float));
  std::copy(policy_actions_.begin(), policy_actions_.end(), actions_.begin());
  std::copy(policy_actions_.begin(), policy_actions_.end(),
            last_actions_.begin());

  RobotCommand cmd;
  cmd.motor_commands.resize(G1_NUM_MOTOR);
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    auto& mc = cmd.motor_commands[i];
    mc.q = default_angles_[i] + action_scale_[i] * policy_actions_[i];
    mc.kp = kps_[i];
    mc.kd = kds_[i];
  }

  active_clock_->step(dt);
  if (!stand_mode_ && active_clock_->finished() && !motion_end_logged_) {
    motion_end_logged_ = true;
    RCLCPP_INFO(this->get_logger(), "motion finished — holding last frame");
  }

  return cmd;
}

// ── Joystick / Gamepad (textop parity: RB/R1 = stand, A = track) ─

void G1SonicNode::on_joy(sensor_msgs::msg::Joy::SharedPtr msg) {
  const bool rb =
      msg->buttons.size() > joy::XMODE_R1 && msg->buttons[joy::XMODE_R1] == 1;
  if (rb && !prev_rb_joy_) {
    enter_stand();
    RCLCPP_INFO(this->get_logger(), "-> stand (SONIC @ nominal)");
  }
  prev_rb_joy_ = rb;

  // A (base already switched to POLICY): commit any staged motion, then
  // leave stand and (re)start it.
  if (msg->buttons.size() > joy::XMODE_A && msg->buttons[joy::XMODE_A] == 1)
    on_button_a();
}

#ifdef HAS_UNITREE_HG
void G1SonicNode::on_gamepad() {
  if (gamepad_.R1.on_press) {
    enter_stand();
    RCLCPP_INFO(this->get_logger(), "[GP] -> stand (SONIC @ nominal)");
  }
  if (gamepad_.A.on_press) on_button_a();
}
#endif

}  // namespace cpp_control

// ── Entry point ──────────────────────────────────────────────────

#ifndef CPP_CONTROL_SONIC_LIB
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cpp_control::G1SonicNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
