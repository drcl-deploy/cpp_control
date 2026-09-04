#include "cpp_control/tasks/tracker/g1_sonic.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "common/asset_path.hpp"
#include "common/g1/joint_orders.hpp"
#include "common/math_utils.hpp"

namespace cpp_control {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

double yaw_of(const std::array<float, 4>& q) {
  return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                    1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

g1::Motion slice_motion(const g1::Motion& source, int first, int last) {
  first = std::clamp(first, 0, source.num_frames - 1);
  last = std::clamp(last, first, source.num_frames - 1);
  const int count = last - first + 1;
  g1::Motion out = source;
  out.num_frames = count;
  auto slice = [first, count](const std::vector<float>& values, int width) {
    const auto begin = values.begin() + static_cast<size_t>(first) * width;
    return std::vector<float>(begin,
                              begin + static_cast<size_t>(count) * width);
  };
  out.joint_pos = slice(source.joint_pos, source.num_joints);
  out.joint_vel = slice(source.joint_vel, source.num_joints);
  out.body_pos_w = slice(source.body_pos_w, source.num_bodies * 3);
  out.body_quat_w = slice(source.body_quat_w, source.num_bodies * 4);
  out.body_lin_vel_w = slice(source.body_lin_vel_w, source.num_bodies * 3);
  out.body_ang_vel_w = slice(source.body_ang_vel_w, source.num_bodies * 3);
  out.bodywise_contact = slice(source.bodywise_contact, g1::NUM_CONTACT_BODIES);
  return out;
}

std::unique_ptr<g1::Motion> joint_transition(const float* from,
                                             const std::vector<float>& to,
                                             int frames, float fps,
                                             const std::array<float, 4>& quat) {
  const std::vector<float> begin(from, from + to.size());
  auto motion =
      std::make_unique<g1::Motion>(g1::Motion::lead_in(begin, to, frames, fps));
  for (int frame = 0; frame < motion->num_frames; ++frame)
    std::copy(quat.begin(), quat.end(),
              motion->body_quat_w.begin() + static_cast<size_t>(frame) * 4);
  return motion;
}

}  // namespace

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
  planner_ = this->declare_parameter("planner", false);
  calibration_lock_ = this->declare_parameter("calibration_lock", false);

  g1::StandYawConfig stand_yaw_config;
  stand_yaw_config.enabled = this->declare_parameter("stand_yaw_teleop", false);
  stand_yaw_config.max_rate =
      this->declare_parameter("stand_yaw_rate_deg_s", 45.0) * kDegToRad;
  stand_yaw_config.max_accel =
      this->declare_parameter("stand_yaw_accel_deg_s2", 180.0) * kDegToRad;
  stand_yaw_config.deadband =
      this->declare_parameter("stand_yaw_deadband", 0.15);
  stand_yaw_config.input_timeout =
      this->declare_parameter("stand_yaw_timeout_s", 0.25);
  stand_yaw_settle_rate_ =
      this->declare_parameter("stand_yaw_settle_rate_deg_s", 2.0) * kDegToRad;
  stand_yaw_settle_gyro_ =
      this->declare_parameter("stand_yaw_settle_gyro_deg_s", 5.0) * kDegToRad;
  stand_yaw_settle_error_ =
      this->declare_parameter("stand_yaw_settle_error_deg", 5.0) * kDegToRad;
  if (stand_yaw_settle_rate_ <= 0.0 || stand_yaw_settle_gyro_ <= 0.0 ||
      stand_yaw_settle_error_ <= 0.0)
    throw std::runtime_error("g1_sonic: stand-yaw settle limits must be > 0");
  stand_yaw_.configure(stand_yaw_config);

  manual_walk_enabled_ = this->declare_parameter("stand_walk_teleop", false);
  const std::string manual_walk_path =
      this->declare_parameter("stand_walk_motion", "");
  manual_walk_distance_m_ =
      this->declare_parameter("stand_walk_distance_m", 0.32);
  manual_walk_lead_s_ = this->declare_parameter("stand_walk_lead_s", 0.35);
  manual_walk_exit_s_ = this->declare_parameter("stand_walk_exit_s", 0.45);
  manual_walk_pause_s_ = this->declare_parameter("stand_walk_pause_s", 0.35);
  manual_walk_deadband_ = this->declare_parameter("stand_walk_deadband", 0.20);
  manual_walk_timeout_ = this->declare_parameter("stand_walk_timeout_s", 0.25);
  if (manual_walk_enabled_ && !planner_)
    throw std::runtime_error(
        "g1_sonic: stand-walk teleop is restricted to planner stand mode");
  if (manual_walk_enabled_ && manual_walk_path.empty())
    throw std::runtime_error(
        "g1_sonic: stand_walk_motion is required when teleop is enabled");
  if (manual_walk_distance_m_ <= 0.0 || manual_walk_lead_s_ <= 0.0 ||
      manual_walk_exit_s_ <= 0.0 || manual_walk_pause_s_ < 0.0 ||
      manual_walk_deadband_ < 0.0 || manual_walk_deadband_ >= 1.0 ||
      manual_walk_timeout_ <= 0.0)
    throw std::runtime_error("g1_sonic: invalid stand-walk configuration");

  if (onnx_path.empty())
    throw std::runtime_error("g1_sonic: onnx_path is required");
  if (manifest_path.empty()) {
    // <model>.onnx → <model>.manifest.json (the exporter writes them side by
    // side)
    manifest_path =
        onnx_path.substr(0, onnx_path.rfind(".onnx")) + ".manifest.json";
  }

  manifest_ = deploy::DeployManifest::load(manifest_path);
  if (calibration_lock_ && !motion_path.empty())
    throw std::runtime_error(
        "g1_sonic: calibration_lock requires an empty motion_path");
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
  if (planner_ || calibration_lock_) {
    status_pub_ = this->create_publisher<cpp_control::msg::ControllerStatus>(
        this->declare_parameter("status_topic", "/vibe/controller/status"),
        rclcpp::QoS(1).best_effort().durability_volatile());
    // Its own timer, not the control tick: the planner has to hear "not
    // accepting" too, and that is exactly when policy_control is not running.
    const auto hz = this->declare_parameter("status_rate_hz", 20.0);
    status_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / hz)),
        [this]() { publish_controller_status(); });
  }

  init();
  apply_manifest_action_meta();
  make_stand_motion();
  if (manual_walk_enabled_)
    make_manual_walk_motion(asset_path(manual_walk_path));
  active_motion_ = motion_.get();
  active_clock_ = clock_.get();

  if (config_ && std::abs(config_->control_dt - manifest_.step_dt) > 1e-6)
    RCLCPP_WARN(
        this->get_logger(),
        "control_dt %.4f != manifest step_dt %.4f — timer runs at control_dt",
        config_->control_dt, manifest_.step_dt);
  if (stand_yaw_.enabled())
    RCLCPP_INFO(this->get_logger(),
                "stand yaw teleop ON: right-stick X, %.1f deg/s, %.1f "
                "deg/s^2, deadband %.2f, watchdog %.2f s",
                stand_yaw_config.max_rate / kDegToRad,
                stand_yaw_config.max_accel / kDegToRad,
                stand_yaw_config.deadband, stand_yaw_config.input_timeout);
  if (manual_walk_enabled_)
    RCLCPP_INFO(this->get_logger(),
                "stand walk teleop ON: left-stick forward, %.2f m bouts, "
                "%.2f s lead / %.2f s exit / %.2f s repeat pause",
                manual_walk_distance_m_, manual_walk_lead_s_,
                manual_walk_exit_s_, manual_walk_pause_s_);

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
  if (calibration_lock_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "calibration lock: ignoring streamed motion");
    return;
  }
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
  if (calibration_lock_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "calibration lock: ignoring MotionReference '%s'",
                         msg->reference_id.c_str());
    return;
  }
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

  // Once A has armed the planner, the reference IS the command: staging every
  // update behind another button would stall the closed loop. RB clears the
  // latch, so a reference racing with the operator's stop request can only be
  // staged.
  if (planner_ && planner_active_ && control_mode_ == ControlMode::POLICY) {
    stand_mode_ =
        false;  // before the commit, so it rebinds active_* to the clip
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
  rebind_active();   // the two lines above freed what active_* may have named
  start_frame_ = 0;  // streamed clips always start at their first frame
  entry_yaw_ = pend_entry_yaw_;
  reference_id_ = pend_reference_id_;
  pend_entry_yaw_ = 0.0f;
  if (planner_)
    return;  // a planner commits every couple of seconds; stay quiet
  RCLCPP_INFO(this->get_logger(), "motion committed: %d frames @ %.0f fps",
              motion_->num_frames, static_cast<double>(motion_->fps));
}

void G1SonicNode::on_button_a() {
  // Also gate direct task-handler calls; BaseNode performs the same check
  // before changing modes, but ROS /joy dispatch reaches this method too.
  if (!allow_policy_entry()) return;
  if (calibration_lock_) {
    enter_stand();
    RCLCPP_INFO(this->get_logger(),
                "calibration lock: A keeps the nominal stand reference");
    return;
  }
  if (planner_) {
    if (!planner_active_) {
      // Never commit a reference staged before this arm edge. The planner sees
      // accepting=true next and publishes a fresh episode from nominal stand.
      pend_motion_.reset();
      pend_ready_ = false;
      pend_reference_id_.clear();
      pend_entry_yaw_ = 0.0f;
      planner_active_ = true;
      RCLCPP_INFO(
          this->get_logger(),
          "the planner rollout ARMED — holding stand for first fresh plan");
    }
    return;
  }
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

void G1SonicNode::make_manual_walk_motion(const std::string& path) {
  const g1::Motion source =
      g1::Motion::from_npz(path, g1::MJ2IL, /*load_companions=*/false);
  if (source.num_joints != G1_NUM_MOTOR || source.num_bodies < 1 ||
      source.num_frames < 3 || !source.has_twist)
    throw std::runtime_error(
        "g1_sonic: stand walk needs a >=3-frame 29-joint motion with twist");

  std::vector<float> speed(source.num_frames, 0.0f);
  for (int frame = 1; frame < source.num_frames; ++frame) {
    const auto a = source.root_pos(frame - 1);
    const auto b = source.root_pos(frame);
    speed[frame] = std::hypot(b[0] - a[0], b[1] - a[1]) * source.fps;
  }
  constexpr int kRadius = 4;  // same 9-frame moving mean as ApproachSource
  std::vector<float> smooth(source.num_frames, 0.0f);
  for (int frame = 0; frame < source.num_frames; ++frame) {
    float sum = 0.0f;
    for (int k = -kRadius; k <= kRadius; ++k)
      if (frame + k >= 0 && frame + k < source.num_frames)
        sum += speed[frame + k];
    smooth[frame] = sum / (2 * kRadius + 1);
  }
  int onset = -1;
  for (int frame = 0; frame < source.num_frames; ++frame)
    if (smooth[frame] >= 0.05f) {
      onset = frame;
      break;
    }
  if (onset < 0)
    throw std::runtime_error("g1_sonic: stand walk motion never moves");

  const auto p0 = source.root_pos(onset);
  int cut = onset + 1;
  for (; cut + 1 < source.num_frames; ++cut) {
    const auto p = source.root_pos(cut);
    if (std::hypot(p[0] - p0[0], p[1] - p0[1]) >= manual_walk_distance_m_)
      break;
  }
  if (cut + 1 >= source.num_frames)
    throw std::runtime_error(
        "g1_sonic: stand walk motion is shorter than stand_walk_distance_m");

  manual_walk_motion_ =
      std::make_unique<g1::Motion>(slice_motion(source, onset, cut));
  // This reference has no cube. Mark the zero-filled contact matrix as an
  // intentional command, not a missing companion file.
  manual_walk_motion_->has_contact = true;
  const auto q0 = manual_walk_motion_->root_quat(0);
  const auto q1 =
      manual_walk_motion_->root_quat(manual_walk_motion_->num_frames - 1);
  manual_walk_enter_motion_ = joint_transition(
      default_angles_.data(),
      std::vector<float>(manual_walk_motion_->jp(0),
                         manual_walk_motion_->jp(0) + G1_NUM_MOTOR),
      std::max(2,
               static_cast<int>(std::lround(manual_walk_lead_s_ * source.fps))),
      source.fps, q0);
  manual_walk_exit_motion_ = joint_transition(
      manual_walk_motion_->jp(manual_walk_motion_->num_frames - 1),
      default_angles_,
      std::max(2,
               static_cast<int>(std::lround(manual_walk_exit_s_ * source.fps))),
      source.fps, q1);
  const auto p1 = source.root_pos(cut);
  const float covered = std::hypot(p1[0] - p0[0], p1[1] - p0[1]);
  RCLCPP_INFO(this->get_logger(),
              "stand walk source: frames %d..%d, %.3f m in %.2f s from %s",
              onset, cut, covered, (cut - onset) / source.fps, path.c_str());
}

bool G1SonicNode::manual_walk_input_fresh(double now) const {
  return manual_walk_enabled_ && manual_walk_have_input_ &&
         std::isfinite(now) && now >= manual_walk_last_input_ &&
         now - manual_walk_last_input_ <= manual_walk_timeout_;
}

void G1SonicNode::start_manual_walk_stage(ManualWalkStage stage) {
  manual_walk_stage_ = stage;
  const g1::Motion* motion = nullptr;
  const char* label = "stand";
  if (stage == ManualWalkStage::ENTER) {
    motion = manual_walk_enter_motion_.get();
    label = "lead-in";
  } else if (stage == ManualWalkStage::WALK) {
    motion = manual_walk_motion_.get();
    label = "walk";
  } else if (stage == ManualWalkStage::EXIT) {
    motion = manual_walk_exit_motion_.get();
    label = "lead-out";
  }
  if (motion)
    manual_walk_clock_ = std::make_unique<g1::MotionClock>(*motion, 0);
  else
    manual_walk_clock_.reset();
  rebind_active();
  pending_engage_ = true;
  motion_end_logged_ = false;
  RCLCPP_INFO(this->get_logger(), "stand walk -> %s", label);
}

void G1SonicNode::update_manual_walk(double now) {
  manual_walk_wants_ = manual_walk_input_fresh(now) && manual_walk_input_ > 0.0;
  if (manual_walk_require_release_ && !manual_walk_wants_)
    manual_walk_require_release_ = false;
  const bool owns_stand = manual_walk_enabled_ && !calibration_lock_ &&
                          stand_mode_ && !planner_active_ &&
                          control_mode_ == ControlMode::POLICY;
  if (!owns_stand) {
    if (manual_walk_busy()) {
      manual_walk_stage_ = ManualWalkStage::IDLE;
      manual_walk_clock_.reset();
      rebind_active();
    }
    manual_walk_wants_ = false;
    return;
  }

  if (manual_walk_stage_ == ManualWalkStage::IDLE) {
    // Left and right stick acts are deliberately serial in v9. Wait for the
    // yaw reference to stop before committing the walking anchor.
    if (manual_walk_wants_ && !manual_walk_require_release_ &&
        stand_yaw_.centered(now) &&
        std::abs(stand_yaw_.rate()) <= stand_yaw_settle_rate_)
      start_manual_walk_stage(ManualWalkStage::ENTER);
    return;
  }
  if (manual_walk_stage_ == ManualWalkStage::PAUSE) {
    if (now < manual_walk_pause_until_) return;
    if (manual_walk_wants_)
      start_manual_walk_stage(ManualWalkStage::ENTER);
    else
      start_manual_walk_stage(ManualWalkStage::IDLE);
    return;
  }
  if (!manual_walk_clock_ || !manual_walk_clock_->finished()) return;
  if (manual_walk_stage_ == ManualWalkStage::ENTER)
    // Starting the bout is the commitment boundary. Even if the operator
    // releases during the lead-in, finish the recorded stride before exiting;
    // jumping directly to an exit built from the stride's final pose would be
    // both discontinuous and contrary to the finite-bout contract.
    start_manual_walk_stage(ManualWalkStage::WALK);
  else if (manual_walk_stage_ == ManualWalkStage::WALK)
    start_manual_walk_stage(ManualWalkStage::EXIT);
  else if (manual_walk_stage_ == ManualWalkStage::EXIT) {
    manual_walk_pause_until_ = now + manual_walk_pause_s_;
    start_manual_walk_stage(ManualWalkStage::PAUSE);
  }
}

void G1SonicNode::enter_stand() {
  if (planner_ && planner_active_) {
    RCLCPP_INFO(this->get_logger(),
                "the planner rollout DISARMED — nominal stand locked");
  }
  planner_active_ = false;
  manual_walk_stage_ = ManualWalkStage::IDLE;
  manual_walk_clock_.reset();
  manual_walk_wants_ = false;
  manual_walk_require_release_ = true;
  stand_yaw_.reset();
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
    const double seconds_ahead =
        static_cast<double>(s * skip) / active_motion_->fps;
    auto rot_dif = math::qmul(math::qinv(robot_quat),
                              reference_root_quat(f, seconds_ahead));
    auto r6d = math::quat_to_rotation_6d(rot_dif);
    std::memcpy(dst + s * row + 2 * J, r6d.data(), 6 * sizeof(float));
  }
}

std::array<float, 4> G1SonicNode::reference_root_quat(
    int frame, double seconds_ahead) const {
  const auto base = active_clock_->aligned_root_quat(frame);
  if (!stand_yaw_reference_active()) return base;
  // Keep the integrator unwrapped so a held stick can rotate indefinitely;
  // reduce only the trigonometric argument used to build the quaternion.
  const double yaw =
      std::remainder(stand_yaw_.future_angle(seconds_ahead), 2.0 * kPi);
  const float half = static_cast<float>(0.5 * yaw);
  return math::qmul({std::cos(half), 0.0f, 0.0f, std::sin(half)}, base);
}

bool G1SonicNode::stand_yaw_reference_active() const {
  return stand_yaw_.enabled() && !calibration_lock_ && stand_mode_ &&
         !manual_walk_busy() && control_mode_ == ControlMode::POLICY;
}

bool G1SonicNode::stand_yaw_drive_active() const {
  return stand_yaw_reference_active() && !planner_active_ &&
         !manual_walk_wants_;
}

float G1SonicNode::stand_yaw_rate() const {
  return stand_yaw_reference_active() ? static_cast<float>(stand_yaw_.rate())
                                      : 0.0f;
}

// ── Engage / reset ───────────────────────────────────────────────

// Two owning pairs, one pair of raw views. A mode switch only changes WHICH
// pair is read and is safe to defer to the next policy tick; replacing an owner
// is not, because it frees the object a view may still name — and the planner
// status timer runs outside that tick. So every replacement calls this.
void G1SonicNode::rebind_active() {
  if (stand_mode_ && manual_walk_busy() &&
      manual_walk_stage_ != ManualWalkStage::PAUSE) {
    if (manual_walk_stage_ == ManualWalkStage::ENTER)
      active_motion_ = manual_walk_enter_motion_.get();
    else if (manual_walk_stage_ == ManualWalkStage::WALK)
      active_motion_ = manual_walk_motion_.get();
    else
      active_motion_ = manual_walk_exit_motion_.get();
    active_clock_ = manual_walk_clock_.get();
    return;
  }
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
                start_frame_, entry_yaw_ != 0.0f ? " + planner yaw" : "");
}

void G1SonicNode::publish_controller_status() {
  if (!status_pub_ || !active_motion_ || !active_clock_) return;
  cpp_control::msg::ControllerStatus s;
  s.control_mode = static_cast<uint8_t>(control_mode_);
  s.stand = stand_mode_;
  // For the planner, accepting means the operator explicitly armed autonomous
  // references with A. RB remains in POLICY to run SONIC stand, but is inert
  // from the planner's point of view.
  s.accepting =
      control_mode_ == ControlMode::POLICY && (!planner_ || planner_active_);
  s.calibration_lock = calibration_lock_;
  s.reference_id = stand_mode_ ? "" : reference_id_;
  s.frame = static_cast<uint32_t>(std::max(0, active_clock_->frame()));
  s.frames = static_cast<uint32_t>(std::max(0, active_motion_->num_frames));
  s.finished = active_clock_->finished();
  s.manual_walk_stage = static_cast<uint8_t>(manual_walk_stage_);
  s.manual_walk_requested = manual_walk_wants_;
  s.default_joint_pos =
      default_angles_;  // manifest-sourced; the planner's v7 stand
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
  // Any excursion through nominal/zeroing/damping invalidates the old stand
  // anchor. Re-enter at the measured heading with no latent stick/rate state.
  if (from_elsewhere && stand_mode_) stand_yaw_.reset();
  update_manual_walk(now.seconds());
  if (pending_engage_ || from_elsewhere) {
    // Arriving from another control mode is a cold start; a reference swap
    // inside POLICY is not — under the planner that happens every couple of
    // seconds.
    engage_reset(/*reset_history=*/from_elsewhere || !planner_);
    pending_engage_ = false;
  }
  last_policy_tick_ = now;

  if (stand_yaw_reference_active()) {
    stand_yaw_.step(now.seconds(), dt, stand_yaw_drive_active());
    if (std::abs(stand_yaw_.target_rate()) > 0.0 ||
        std::abs(stand_yaw_.rate()) > stand_yaw_settle_rate_)
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "stand yaw: stick=%+.2f target=%+.1f rate=%+.1f angle=%+.1f deg",
          stand_yaw_.input(), stand_yaw_.target_rate() / kDegToRad,
          stand_yaw_.rate() / kDegToRad, stand_yaw_.angle() / kDegToRad);
  } else {
    stand_yaw_.reset();
  }

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

void G1SonicNode::on_joy_input(sensor_msgs::msg::Joy::SharedPtr msg) {
  const double now = this->now().seconds();
  const float yaw = msg->axes.size() > joy::XMODE_RIGHT_JOY_LEFT_RIGHT
                        ? msg->axes[joy::XMODE_RIGHT_JOY_LEFT_RIGHT]
                        : 0.0f;
  stand_yaw_.set_input(yaw, now);
  float forward = msg->axes.size() > joy::XMODE_LEFT_JOY_UP_DOWN
                      ? msg->axes[joy::XMODE_LEFT_JOY_UP_DOWN]
                      : 0.0f;
  if (!std::isfinite(forward)) forward = 0.0f;
  forward = std::clamp(forward, 0.0f, 1.0f);  // no reverse in v9
  manual_walk_input_ =
      forward <= manual_walk_deadband_
          ? 0.0
          : (forward - manual_walk_deadband_) / (1.0 - manual_walk_deadband_);
  manual_walk_last_input_ = now;
  manual_walk_have_input_ = true;
}

bool G1SonicNode::allow_policy_entry() {
  // The ordinary A transition (nominal -> policy) stays unchanged. This gate
  // matters only when A arms a planner from an already-running SONIC stand.
  const double now = this->now().seconds();
  const bool walk_centered =
      !manual_walk_input_fresh(now) || manual_walk_input_ == 0.0;
  if (manual_walk_enabled_ &&
      (!walk_centered || manual_walk_busy() || manual_walk_require_release_)) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "A refused — release left stick and let stand walk return to nominal");
    return false;
  }
  if (!stand_yaw_reference_active()) return true;

  const auto desired =
      reference_root_quat(active_clock_->frame(), /*seconds_ahead=*/0.0);
  const auto heading_error =
      math::qmul(math::qinv(math::heading_quat(robot_state_.imu_quaternion)),
                 math::heading_quat(desired));
  const double error = yaw_of(heading_error);
  const bool ready =
      stand_yaw_.centered(now) &&
      std::abs(stand_yaw_.rate()) <= stand_yaw_settle_rate_ &&
      std::abs(robot_state_.imu_gyroscope[2]) <= stand_yaw_settle_gyro_ &&
      std::abs(error) <= stand_yaw_settle_error_;
  if (!ready)
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "A refused — center right stick and let stand yaw settle "
        "(ref rate %.1f, gyro %.1f, heading err %.1f deg)",
        stand_yaw_.rate() / kDegToRad,
        robot_state_.imu_gyroscope[2] / kDegToRad, error / kDegToRad);
  return ready;
}

void G1SonicNode::on_joy(sensor_msgs::msg::Joy::SharedPtr msg) {
  const bool rb =
      msg->buttons.size() > joy::XMODE_R1 && msg->buttons[joy::XMODE_R1] == 1;
  const bool a =
      msg->buttons.size() > joy::XMODE_A && msg->buttons[joy::XMODE_A] == 1;
  if (rb && !prev_rb_joy_) {
    enter_stand();
    RCLCPP_INFO(this->get_logger(), "-> stand (SONIC @ nominal)");
  }
  prev_rb_joy_ = rb;

  // A (base already switched to POLICY): commit any staged motion, then
  // leave stand and (re)start it.
  if (a && !prev_a_joy_) on_button_a();
  prev_a_joy_ = a;
}

#ifdef HAS_UNITREE_HG
void G1SonicNode::on_gamepad_input() {
  const double now = this->now().seconds();
  stand_yaw_.set_input(gamepad_.rx, now);
  const float forward =
      std::isfinite(gamepad_.ly) ? std::clamp(gamepad_.ly, 0.0f, 1.0f) : 0.0f;
  manual_walk_input_ =
      forward <= manual_walk_deadband_
          ? 0.0
          : (forward - manual_walk_deadband_) / (1.0 - manual_walk_deadband_);
  manual_walk_last_input_ = now;
  manual_walk_have_input_ = true;
}

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
