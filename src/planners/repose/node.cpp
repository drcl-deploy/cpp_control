/**
 * Repose planner — level 3, above the controller. Camera in, MotionReference
 * out.
 *
 * Soft real-time and a separate process on purpose: it must never sit in the
 * 50 Hz loop, and the controller has to stay runnable without it (open-loop
 * rollouts are the default). It owns no clock — the act is timed off
 * ControllerStatus, so what the robot is ACTUALLY playing is the only thing
 * that advances the plan.
 *
 * OBSERVE every tick the camera is quiet, PLAN every tick for free, ACT only
 * when the controller says it finished. That split is the whole node:
 *
 *     SENSE    lowstate fresh                          ->  belief << look()
 *              new frame AND |omega_cam| < omega_still
 *     GUARD    controller accepting? nominal stance known?
 *     PLAN     candidate = decide(belief)                   PURE, ~50 us
 *     ACT      controller.finished                       ->  commit(candidate)
 *
 * SENSE sits ABOVE the guard on purpose: the planner reads and publishes its
 * belief whenever lowstate is alive, controller or not, so the console is a
 * live view of the perception stack before anything is armed. Arming clears the
 * belief, so an idle read can never become an episode's evidence.
 *
 *   ros2 launch cpp_control g1_repose_planner.launch.py
 *   ros2 run cpp_control repose_console.py        # set the target colour live
 *
 * Design + the findings that shrank it: docs/planners/repose/planner.md
 */

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cpp_control/msg/controller_status.hpp>
#include <cpp_control/msg/motion_reference.hpp>
#include <cpp_control/msg/planner_status.hpp>
#include <cstring>
#include <memory>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <thread>
#include <unitree_hg/msg/low_state.hpp>
#include <vector>
#include <vision_encoders/frame_wire.hpp>

#include "common/asset_path.hpp"
#include "common/g1/joint_orders.hpp"
#include "common/g1/motion.hpp"
#include "common/math_utils.hpp"
#include "cpp_control/planners/repose/approach.hpp"
#include "cpp_control/planners/repose/belief.hpp"
#include "cpp_control/planners/repose/clips.hpp"
#include "cpp_control/planners/repose/kinematics.hpp"
#include "cpp_control/planners/repose/sight.hpp"
#include "cpp_control/planners/repose/table.hpp"
#include "cpp_control/planners/repose/writer.hpp"
#include "cpp_control/tasks/vibe/task_profile.hpp"

namespace cpp_control {
namespace planners {
namespace repose {

namespace wire = vision_encoders::wire;

namespace {

std::array<float, 3> yaml_vec3(const YAML::Node& n,
                               const std::array<float, 3>& fallback) {
  if (!n || n.size() != 3) return fallback;
  return {n[0].as<float>(), n[1].as<float>(), n[2].as<float>()};
}

float wrap(float a) {
  constexpr float kPi = 3.14159265358979323846f;
  while (a > kPi) a -= 2.0f * kPi;
  while (a < -kPi) a += 2.0f * kPi;
  return a;
}

}  // namespace

class ReposeNode : public rclcpp::Node {
 public:
  ReposeNode() : Node("g1_repose_planner_node") {
    const auto config_path = this->declare_parameter("config_path", "");
    if (config_path.empty())
      throw std::runtime_error("repose planner: config_path is required");
    load_config(config_path);
    approach_only_ = this->declare_parameter("approach_only", false);
    if (approach_only_ && !cfg_.approach_enabled)
      throw std::runtime_error(
          "repose planner: approach_only needs an approach-enabled config");
    // The experiment environment owns the camera address — setup.sh picks it,
    // the launch passes the same `camera_ip` to the encoder and to here, so the
    // two halves of one run cannot look at different cameras. Empty keeps the
    // yaml's value, which is what a standalone `ros2 run` gets.
    const auto camera_host = this->declare_parameter("camera_host", "");
    if (!camera_host.empty()) camera_ip_ = camera_host;

    // The goal colour conditions the POLICY's one-hot and steers the PLANNER's
    // ladder, and the two must not boot disagreeing: the topic keeps them in
    // step afterwards, but nothing did until the first message.
    // g1_repose_planner hands its `goal_color` here as well as to the
    // controller, so one launch argument sets both. Out of range (the default)
    // keeps the yaml, which is what a standalone `ros2 run` gets.
    const auto target =
        static_cast<int>(this->declare_parameter("target_color", -1));
    if (target >= 0 && target < vibe::NUM_CUBE_COLORS)
      clips_->set_target_color(target);

    reference_pub_ = this->create_publisher<msg::MotionReference>(
        this->declare_parameter("reference_topic", "/tracker/reference"), 10);
    status_pub_ = this->create_publisher<msg::PlannerStatus>(
        this->declare_parameter("planner_status_topic", "/vibe/planner/status"),
        10);
    controller_sub_ = this->create_subscription<msg::ControllerStatus>(
        this->declare_parameter("controller_status_topic",
                                "/vibe/controller/status"),
        rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](msg::ControllerStatus::SharedPtr m) {
          controller_ = *m;
          controller_seen_ = true;
          adopt_nominal_stand(m->default_joint_pos);
        });
    lowstate_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
        this->declare_parameter("lowstate_topic", "/lowstate"), 10,
        [this](unitree_hg::msg::LowState::SharedPtr m) { on_lowstate(m); });
    goal_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        this->declare_parameter("goal_color_topic", "/vibe/sonic/goal_color"),
        10, [this](std_msgs::msg::Int32::SharedPtr m) { on_goal_color(m); });

    running_.store(true);
    tcp_thread_ = std::thread(&ReposeNode::tcp_loop, this);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / rate_hz_)),
        [this]() { tick(); });

    const std::string ramp =
        cfg_.enter_yaw_rate_deg > 0.0f
            ? " | ENTER ramp " +
                  std::to_string(static_cast<int>(cfg_.enter_yaw_rate_deg)) +
                  " deg/s"
            : "";
    const std::string approach =
        cfg_.approach_enabled
            ? " | APPROACH " +
                  (cfg_.approach_anisotropic
                       ? "|f|>" +
                             std::to_string(cfg_.approach_enter_forward_m) +
                             " or |l|>" +
                             std::to_string(cfg_.approach_enter_lateral_m)
                       : ">" + std::to_string(cfg_.approach_enter_m)) +
                  " m, window >=" + std::to_string(cfg_.approach_min_window_m) +
                  " m" + (approach_only_ ? " ONLY" : "")
            : "";
    RCLCPP_INFO(
        this->get_logger(),
        "the planner %s ready: %zu clips (F%zu B%zu L%zu R%zu) | pattern %s | "
        "target %s | belief +%d/-%d of %d reads under %.2f rad/s%s%s | "
        "camera %s:%u @ %.0f Hz",
        version_.c_str(), table_.rows().size(), table_.pool('F').size(),
        table_.pool('B').size(), table_.pool('L').size(),
        table_.pool('R').size(), cfg_.pattern.c_str(),
        vibe::cube_color_name(clips_->target_color()), cfg_.belief_min_votes,
        cfg_.belief_scan_after_reads, cfg_.belief_window, cfg_.omega_still,
        ramp.c_str(), approach.c_str(), camera_ip_.c_str(), camera_port_,
        rate_hz_);
  }

  ~ReposeNode() override {
    running_.store(false);
    if (tcp_thread_.joinable()) tcp_thread_.join();
  }

 private:
  // ── Config ─────────────────────────────────────────────────────

  void load_config(const std::string& path) {
    const YAML::Node root = YAML::LoadFile(path);
    version_ = root["version"] ? root["version"].as<std::string>() : "v7";
    cfg_ = Cfg::from_yaml(root);

    // Every path below is relative to $VIBE_ASSET_ROOT unless it is absolute
    // (common/asset_path.hpp), so this yaml is one string on both boxes.
    table_ = ClipTable::load(asset_path(root["clips"].as<std::string>()));
    const auto frames = root["frames"];
    const auto source = frames["source"].as<std::string>();
    const auto frames_path = asset_path(frames["path"].as<std::string>());
    if (source == "retargeted")
      table_.load_frames_retargeted(
          asset_path(frames["library"].as<std::string>()), frames_path);
    else if (source == "baked")
      table_.load_frames_baked(frames_path);
    else
      throw std::runtime_error(
          "repose planner: frames.source must be retargeted | baked");

    const auto cam = root["camera"];
    camera_ip_ = cam["host"] ? cam["host"].as<std::string>() : "127.0.0.1";
    camera_port_ = cam["port"] ? cam["port"].as<uint16_t>() : 5555;
    rate_hz_ = root["rate_hz"] ? root["rate_hz"].as<double>() : 20.0;
    stale_s_ = root["stale_s"] ? root["stale_s"].as<double>() : 1.0;

    const auto mount = root["mount"];
    mount_.pos = yaml_vec3(mount ? mount["xyz"] : YAML::Node(), Mount{}.pos);
    if (const auto q = mount ? mount["quat"] : YAML::Node(); q && q.size() == 4)
      mount_.quat = {q[0].as<float>(), q[1].as<float>(), q[2].as<float>(),
                     q[3].as<float>()};

    const int target =
        root["target_color"] ? root["target_color"].as<int>() : 4;
    clips_ = std::make_unique<Clips>(table_, cfg_, target);
    eye_ = std::make_unique<CubeSight>(cfg_, table_.half_extent());
    belief_ = std::make_unique<Belief>(cfg_);
    if (cfg_.approach_enabled)
      approach_ = std::make_unique<ApproachSource>(
          asset_path(cfg_.approach_motion), cfg_);
    writer_ = std::make_unique<ReferenceWriter>(table_, cfg_, approach_.get());

    for (int i = 0; i < 3; ++i) {
      static const char* kWaist[3] = {"waist_yaw_joint", "waist_roll_joint",
                                      "waist_pitch_joint"};
      const auto it =
          std::find(g1::MJ_JOINTS.begin(), g1::MJ_JOINTS.end(), kWaist[i]);
      if (it == g1::MJ_JOINTS.end())
        throw std::runtime_error(
            "repose planner: G1 joint table has no waist chain");
      waist_[i] = static_cast<int>(it - g1::MJ_JOINTS.begin());
    }
    live_.joint_pos_il.assign(g1::NUM_JOINTS, 0.0f);
    live_.joint_vel_il.assign(g1::NUM_JOINTS, 0.0f);
  }

  // ── Robot state ────────────────────────────────────────────────

  void on_lowstate(unitree_hg::msg::LowState::SharedPtr m) {
    imu_quat_ = {m->imu_state.quaternion[0], m->imu_state.quaternion[1],
                 m->imu_state.quaternion[2], m->imu_state.quaternion[3]};
    for (int j = 0; j < g1::NUM_JOINTS; ++j) {
      const int mj = g1::IL2MJ[j];  // wire rows are IL-ordered
      live_.joint_pos_il[j] = m->motor_state[mj].q;
      live_.joint_vel_il[j] = m->motor_state[mj].dq;
    }
    // HOW FAST THE CAMERA IS SWINGING, which is the only thing that decides
    // whether a frame is evidence or blur. The camera is rigid to torso_link,
    // so the base's gyro plus the waist chain's rates bound it — no FK, and
    // both numbers were already on this message.
    const auto& g = m->imu_state.gyroscope;
    float w = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    for (int i = 0; i < 3; ++i) {
      waist_q_[i] = m->motor_state[waist_[i]].q;
      w += std::fabs(m->motor_state[waist_[i]].dq);
    }
    omega_cam_ = w;
    last_state_ = this->now();
  }

  CameraPose camera_pose() const {
    return planners::repose::camera_pose(imu_quat_, waist_q_, mount_);
  }

  // ── The v7 still pose ──────────────────────────────────────────
  //
  // Taken from the controller rather than parsed here: `default_joint_pos` is
  // the manifest's, which is the pose the controller actually holds in
  // NOMINAL_POSE and the same array mjlab FKs. One source, so the stance the
  // planner commands and the stance the robot stands in cannot disagree.
  // Arrives on the first ControllerStatus, which the planner already waits for.

  void adopt_nominal_stand(const std::vector<float>& jp_mj) {
    if (stand_ready_ || !cfg_.nominal_stand) return;
    if (jp_mj.size() != static_cast<size_t>(g1::NUM_JOINTS)) return;
    bool any = false;
    for (float v : jp_mj) any = any || v != 0.0f;
    if (!any) return;  // manifest not loaded yet — all-zero is not a stance

    table_.set_nominal_stand(jp_mj);
    stand_ready_ = true;

    // Log the gaze, because the gaze IS the change. A deployed robot whose
    // nominal stance carries a stooped waist gets v6's band back, silently.
    std::array<float, 3> waist{};
    for (int i = 0; i < 3; ++i) waist[i] = jp_mj[waist_[i]];
    const Gaze g = gaze_of(
        planners::repose::camera_pose({1.f, 0.f, 0.f, 0.f}, waist, mount_));
    RCLCPP_INFO(this->get_logger(),
                "v7 nominal stand: waist %.1f/%.1f/%.1f deg -> camera %.1f deg "
                "down, %+.1f off-axis (mount is 45.0)",
                waist[0] * 57.2958f, waist[1] * 57.2958f, waist[2] * 57.2958f,
                g.pitch_deg, g.yaw_deg);
    if (std::fabs(g.pitch_deg - 45.0f) > 5.0f)
      RCLCPP_WARN(this->get_logger(),
                  "nominal stance does not recover the mount angle — the whole "
                  "v7 result is that waist ~ 0 makes the torso vertical");
  }

  void on_goal_color(std_msgs::msg::Int32::SharedPtr m) {
    if (m->data < 0 || m->data >= vibe::NUM_CUBE_COLORS) {
      RCLCPP_WARN(this->get_logger(), "ignoring invalid goal_color %d",
                  m->data);
      return;
    }
    // Re-picking the colour already selected is the operator saying "go again":
    // it is the only way out of a LATCHED done, and it aborts a trial that went
    // wrong. Same key, new episode, either way.
    if (m->data == clips_->target_color()) {
      clips_->reset();
      belief_->clear();
      reset_approach();
      RCLCPP_INFO(this->get_logger(), "re-armed on %s — episode %d",
                  vibe::cube_color_name(m->data), clips_->episode());
      return;
    }
    clips_->set_target_color(m->data);
    belief_->clear();
    reset_approach();
    RCLCPP_INFO(this->get_logger(), "target -> %d (%s), episode %d", m->data,
                vibe::cube_color_name(m->data), clips_->episode());
  }

  // ── Camera wire ────────────────────────────────────────────────

  static bool recv_exact(int fd, uint8_t* buf, size_t n) {
    size_t got = 0;
    while (got < n) {
      const ssize_t r = ::recv(fd, buf + got, n - got, 0);
      if (r <= 0) return false;
      got += static_cast<size_t>(r);
    }
    return true;
  }

  void tcp_loop() {
    std::vector<uint8_t> payload;
    while (running_.load()) {
      const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
      sockaddr_in addr{};
      addr.sin_family = AF_INET;
      addr.sin_port = htons(camera_port_);
      inet_pton(AF_INET, camera_ip_.c_str(), &addr.sin_addr);
      if (fd < 0 ||
          ::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "repose planner: cannot reach %s:%u, retrying",
                    camera_ip_.c_str(), camera_port_);
        if (fd >= 0) ::close(fd);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "repose planner: camera %s:%u connected",
                  camera_ip_.c_str(), camera_port_);
      while (running_.load()) {
        uint32_t len = 0;
        if (!recv_exact(fd, reinterpret_cast<uint8_t*>(&len), 4)) break;
        payload.resize(len);
        if (!recv_exact(fd, payload.data(), len)) break;
        decode(payload);
      }
      ::close(fd);
    }
  }

  void decode(const std::vector<uint8_t>& payload) {
    wire::Frame f;
    if (!wire::parse(payload, f)) return;
    const wire::Plane* colour = f.find(wire::Kind::COLOR);
    const wire::Plane* depth = f.find(wire::Kind::DEPTH);
    if (!colour) return;
    if (!depth) {  // colour-only frames are the majority under --depth-hz
      if (!depth_warned_ && ++colour_only_ > 200) {
        depth_warned_ = true;
        RCLCPP_ERROR(this->get_logger(),
                     "repose planner: 200 frames with no depth plane — start "
                     "the streamer "
                     "with --depth (or set camera_depth: 1 in the sim config)");
      }
      return;
    }
    if (!depth->has_intrinsics()) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "repose planner: depth plane carries no intrinsics");
      return;
    }

    const cv::Mat encoded(1, static_cast<int>(colour->nbytes), CV_8UC1,
                          const_cast<uint8_t*>(colour->data));
    cv::Mat bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (bgr.empty()) return;

    cv::Mat metres;
    if (depth->fmt == wire::Fmt::RAW_U16_MM) {
      const cv::Mat mm(depth->h, depth->w, CV_16UC1,
                       const_cast<uint8_t*>(depth->data));
      mm.convertTo(metres, CV_32FC1, 1e-3);
    } else {
      const cv::Mat enc(1, static_cast<int>(depth->nbytes), CV_8UC1,
                        const_cast<uint8_t*>(depth->data));
      cv::Mat raw = cv::imdecode(enc, cv::IMREAD_ANYDEPTH);
      if (raw.empty()) return;
      raw.convertTo(metres, CV_32FC1, 1e-3);
    }

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      bgr_ = std::move(bgr);
      depth_ = std::move(metres);
      intr_ = {depth->fx, depth->fy, depth->cx, depth->cy};
    }
    // Atomics, because these two are the only fields the ROS thread reads
    // WITHOUT the lock — the freshness log and the "is this frame new" test.
    last_frame_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
    frame_seq_.fetch_add(1, std::memory_order_release);
  }

  // ── OBSERVE ────────────────────────────────────────────────────

  /// One read, if there is a new frame AND the camera was holding still for it.
  /// The old "take exactly the last 2*read_tail frames" was a proxy for this;
  /// measuring the camera's own rate is both simpler and the true condition, so
  /// a SCAN's sweep rejects itself and a SETTLE's whole hold is readable.
  ///
  /// A CLIP/APPROACH is excluded outright rather than left to the gate. A clip
  /// has quiet moments, but it spends them in an arbitrary exit pose whose
  /// waist aims the head wherever the recording left it — that is precisely the
  /// v6 gaze the nominal stance exists to escape, so those reads are quiet and
  /// still wrong. The mandatory settle after every clip is what this produces:
  /// a clip ends with an empty belief, and an empty belief plans the stance.
  void observe() {
    if (plan_.mode == Mode::CLIP || plan_.mode == Mode::APPROACH) return;
    const uint64_t seq = frame_seq_.load(std::memory_order_acquire);
    if (seq == seen_seq_) return;  // no new frame; the camera is slower than us
    seen_seq_ = seq;
    ++frames_offered_;
    if (omega_cam_ > cfg_.omega_still) return;  // moving: this frame is blur

    cv::Mat bgr, depth;
    Intrinsics intr;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      bgr = bgr_;
      depth = depth_;
      intr = intr_;
    }
    if (bgr.empty() || depth.empty()) return;

    const auto t0 = std::chrono::steady_clock::now();
    belief_->push((*eye_)(bgr, depth, intr, camera_pose()));
    observe_ms_ = std::chrono::duration<float, std::milli>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
    ++frames_read_;
  }

  // ── The loop ───────────────────────────────────────────────────

  void tick() {
    // ── SENSE ──────────────────────────────────────────────
    //
    // The controller is NOT a gate on LOOKING. A planner that may not act can
    // still read the cube, and the console is the only window onto what the
    // perception stack actually sees — which is what a mismeasured palette gets
    // wrong silently, and the one thing worth watching before arming.
    //
    // lowstate STAYS a gate: `camera_pose()` is FK off the IMU and the waist
    // chain, so without it every geometric test would run against a pose the
    // node invented, and the belief would be confident about it.
    //
    // Arming still clears the belief, so nothing read here can reach an
    // episode: what is published below is a view, never evidence.
    const bool state_fresh = fresh(last_state_);
    const bool stance_known = stand_ready_ || !cfg_.nominal_stand;
    if (state_fresh) observe();

    // ── GUARD ──────────────────────────────────────────────
    if (!controller_seen_ || !controller_.accepting) {
      if (armed_)
        RCLCPP_INFO(
            this->get_logger(),
            "repose planner: controller left POLICY — idle, still observing");
      armed_ = false;
      in_enter_ =
          false;  // a ramp is only valid against the yaw it was frozen at
      reset_approach();
      // decide() is PURE, so an idle planner can still show what it WOULD do.
      // Only once the stance is known, or the preview would be solved against
      // the library's stand frame — v6's gaze, and a quietly wrong answer.
      if (state_fresh && stance_known)
        candidate_ = admit_approach(clips_->decide(*belief_));
      publish_status();
      return;
    }
    // The camera is deliberately NOT a gate. Losing it starves the belief, and
    // a starved belief plans the nominal stance — so a camera that dies mid
    // clip returns the robot to its feet instead of freezing it in the exit
    // pose of a half-finished turn.
    if (!state_fresh || !stance_known) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "repose planner: waiting for %s%s", state_fresh ? "" : "lowstate ",
          stance_known ? "" : "the controller's nominal stance");
      publish_status();
      return;
    }
    if (!camera_fresh())
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "repose planner: no camera frames — standing by");

    if (!armed_) {  // fresh episode: never plan the new one from the old cube
      armed_ = true;
      in_enter_ = false;
      clips_->reset();
      belief_->clear();
      reset_approach();
      RCLCPP_INFO(
          this->get_logger(), "repose planner: armed, target %s, episode %d",
          vibe::cube_color_name(clips_->target_color()), clips_->episode());
      candidate_ = clips_->decide(*belief_);
      commit(candidate_);
      publish_status();
      return;
    }

    clips_->observe(*belief_);
    Plan raw_candidate = clips_->decide(*belief_);
    raw_candidate = hold_candidate_identity(raw_candidate);
    raw_candidate = hold_approach_turn_identity(raw_candidate);
    update_approach_feedback(raw_candidate);
    candidate_ = shape_scan(admit_approach(raw_candidate));

    if (controller_.reference_id != reference_id_) {
      // the controller commits on arrival, so this is a dropped message, not a
      // race.
      if (++waiting_ > 3) {
        RCLCPP_WARN(this->get_logger(),
                    "repose planner: '%s' never engaged — resending",
                    reference_id_.c_str());
        republish();
        waiting_ = 0;  // one resend per 4 ticks, not one per tick forever
      }
    } else {
      waiting_ = 0;
      // The ramp landed: its clip goes out next, not whatever the planner would
      // pick now. `candidate_` is a preview between acts, and the belief has
      // been cleared since the read that chose this clip.
      if (controller_.finished) {
        const bool keep_parked = cfg_.approach_latch_blocked &&
                                 terminal_approach_hold(plan_) &&
                                 terminal_approach_hold(candidate_) &&
                                 plan_.label == candidate_.label;
        // V9.3's negative horizon can be longer than one hardware settle's
        // quiet tail. Recommitting the identical zero-yaw stance would clear
        // its belief and make eight accepted reads impossible to accumulate.
        // Hold the already-finished reference instead; positive evidence still
        // changes `candidate_` immediately on its third agreeing pose.
        const bool keep_collecting =
            cfg_.belief_scan_after_reads > cfg_.belief_min_votes &&
            plan_.mode == Mode::SETTLE && plan_.label == "settle" &&
            candidate_.mode == Mode::SETTLE && candidate_.label == "settle";
        if (!keep_parked && !keep_collecting)
          in_enter_ ? commit_clip(pending_) : commit(candidate_);
      }
    }
    publish_status();
  }

  bool fresh(const rclcpp::Time& t) const {
    return t.nanoseconds() > 0 && (this->now() - t).seconds() < stale_s_;
  }

  /// The robot's heading off the pelvis IMU. Yaw is the only channel of it the
  /// seam ever needs, and the only one the hardware gives without odometry.
  float robot_yaw() const {
    return std::atan2(
        2.0f * (imu_quat_[0] * imu_quat_[3] + imu_quat_[1] * imu_quat_[2]),
        1.0f -
            2.0f * (imu_quat_[2] * imu_quat_[2] + imu_quat_[3] * imu_quat_[3]));
  }

  bool camera_fresh() const {
    const int64_t ns = last_frame_ns_.load(std::memory_order_relaxed);
    return ns > 0 &&
           (this->now() - rclcpp::Time(ns, RCL_ROS_TIME)).seconds() < stale_s_;
  }

  void reset_approach() {
    approach_attempts_ = 0;
    approach_no_progress_ = 0;
    approach_rung_ = -1;
    awaiting_approach_measurement_ = false;
    approach_before_m_ = 0.0f;
    approach_progress_m_ = 0.0f;
    approach_last_window_ = -1;
    approach_measure_anchor_ = Plan{};
    approach_measure_target_yaw_ = 0.0f;
    clear_candidate_lock();
    reset_scan();
    clear_approach_turn();
  }

  void clear_candidate_lock() {
    candidate_locked_ = false;
    candidate_lock_rung_ = -1;
    candidate_anchor_ = Plan{};
    candidate_target_yaw_ = 0.0f;
  }

  /// A square pose is reported modulo 90 degrees. Near the +/-45 degree fold,
  /// two equally valid frames can therefore change `phi` by 90 degrees. A
  /// fresh global arg-min then changes row/symmetry and makes the desired
  /// stance jump even though neither robot nor cube did. Lock the first
  /// selected row and its absolute entry heading for this decision interval;
  /// `retarget_heading_locked` changes the symmetry index when the fold moves.
  Plan hold_candidate_identity(const Plan& raw) {
    if (!cfg_.lock_candidate_identity) return raw;
    if (clips_->done() ||
        (candidate_locked_ && clips_->rung() != candidate_lock_rung_))
      clear_candidate_lock();
    if (approach_turn_locked_) return raw;
    // A lost/partial sight ends this decision interval. Keeping the identity
    // across a scan would resurrect a stale stance after the robot or cube had
    // moved; reacquire from the full table when a placed cube returns.
    if (raw.mode != Mode::CLIP) {
      clear_candidate_lock();
      return raw;
    }
    if (!candidate_locked_) {
      candidate_locked_ = true;
      candidate_lock_rung_ = clips_->rung();
      candidate_anchor_ = raw;
      candidate_target_yaw_ = wrap(robot_yaw() + raw.entry_yaw);
      return raw;
    }
    return clips_->retarget_heading_locked(candidate_anchor_, belief_->pose(),
                                           robot_yaw(), candidate_target_yaw_);
  }

  void clear_approach_turn() {
    approach_turn_locked_ = false;
    approach_turn_attempts_ = 0;
    approach_turn_rung_ = -1;
    approach_turn_anchor_ = Plan{};
    approach_turn_target_yaw_ = 0.0f;
  }

  /// A robot-frame turn must not trigger a new global arg-min over clip rows
  /// and cube symmetries. That makes the target itself discontinuous: the
  /// newly cheapest entry can lie on the other side of the robot, even though
  /// the cube did not move. Keep only the chosen identity across this turn and
  /// re-evaluate all of its geometry from the newest pose.
  Plan hold_approach_turn_identity(const Plan& raw) {
    if (!approach_turn_locked_) return raw;
    if (clips_->done() || clips_->rung() != approach_turn_rung_) {
      clear_approach_turn();
      return raw;
    }
    // A CLIP answer proves that this belief has a valid, placed cube. Blind or
    // partial observations keep their normal SETTLE/SCAN recovery; the lock is
    // merely waiting and never substitutes a stale pose.
    if (raw.mode != Mode::CLIP) return raw;
    return cfg_.lock_candidate_identity
               ? clips_->retarget_heading_locked(approach_turn_anchor_,
                                                 belief_->pose(), robot_yaw(),
                                                 approach_turn_target_yaw_)
               : clips_->retarget(approach_turn_anchor_, belief_->pose());
  }

  /// Evaluate one completed leg only after SETTLE has rebuilt a valid pose.
  /// There is deliberately no odometry estimate: this is the observed entry
  /// residual before minus after, which is the outcome the walk exists to buy.
  void update_approach_feedback(const Plan& raw) {
    if (raw.mode != Mode::CLIP) return;
    if (approach_rung_ >= 0 && clips_->rung() != approach_rung_)
      reset_approach();
    if (awaiting_approach_measurement_) {
      const Plan measured = clips_->retarget_heading_locked(
          approach_measure_anchor_, belief_->pose(), robot_yaw(),
          approach_measure_target_yaw_);
      approach_progress_m_ = approach_before_m_ - measured.entry_translation;
      awaiting_approach_measurement_ = false;
      if (approach_progress_m_ + 1e-6f < cfg_.approach_min_progress_m)
        ++approach_no_progress_;
      else
        approach_no_progress_ = 0;
      RCLCPP_INFO(this->get_logger(),
                  "approach measured: %.3f -> %.3f m (progress %+.3f, "
                  "attempt %d/%d)",
                  approach_before_m_, measured.entry_translation,
                  approach_progress_m_, approach_attempts_,
                  cfg_.approach_max_attempts);
    }
    if (approach_ready(raw, cfg_)) {
      // Inside the proven clip-correction band. This also makes an
      // approach-only drag test re-arm automatically once the robot arrives.
      approach_attempts_ = 0;
      approach_no_progress_ = 0;
      approach_rung_ = clips_->rung();
      approach_last_window_ = -1;
    }
  }

  bool terminal_approach_hold(const Plan& p) const {
    return p.mode == Mode::SETTLE && (p.label == "approach blocked" ||
                                      p.label == "approach turn blocked" ||
                                      p.label == "reposition required");
  }

  Plan parked_candidate(const Plan& raw, const char* label) const {
    Plan p = clips_->still(0.0f);
    p.cost = raw.cost;
    p.entry_translation = raw.entry_translation;
    p.entry_bearing = raw.entry_bearing;
    p.entry_forward = raw.entry_forward;
    p.entry_lateral = raw.entry_lateral;
    p.label = label;
    return p;
  }

  /// Insert APPROACH around the unchanged retrieval algorithm. This function
  /// does not mutate the clip ladder or freeze the candidate: after walking,
  /// the mandatory settle builds a new belief and retrieves again.
  Plan admit_approach(const Plan& raw) const {
    if (raw.mode != Mode::CLIP) return raw;
    if (!cfg_.approach_enabled)
      return approach_only_ ? parked_candidate(raw, "approach off") : raw;
    if (approach_ready(raw, cfg_))
      return approach_only_ ? parked_candidate(raw, "approach ready") : raw;
    if (approach_attempts_ >= cfg_.approach_max_attempts ||
        approach_no_progress_ >= cfg_.approach_no_progress_limit)
      return parked_candidate(raw, "approach blocked");

    // The source has no reverse gait. Once a stance outside the direct-clip
    // band lies behind the robot, walking forward can only make it worse.
    if (!approach_forward_reachable(raw, cfg_))
      return parked_candidate(raw, "reposition required");

    const float turn_max =
        cfg_.approach_turn_max_deg * 3.14159265358979323846f / 180.0f;
    if (std::fabs(raw.entry_bearing) > turn_max) {
      if (approach_turn_attempts_ >= cfg_.approach_turn_max_attempts)
        return parked_candidate(raw, "approach turn blocked");
      Plan p = clips_->still(
          std::clamp(raw.entry_bearing,
                     -cfg_.scan_sweep_deg * 3.14159265358979323846f / 180.0f,
                     cfg_.scan_sweep_deg * 3.14159265358979323846f / 180.0f));
      p.cost = raw.cost;
      p.entry_translation = raw.entry_translation;
      p.entry_bearing = raw.entry_bearing;
      p.entry_forward = raw.entry_forward;
      p.entry_lateral = raw.entry_lateral;
      p.entry_yaw = raw.entry_yaw;
      p.matched_entry_yaw = raw.entry_yaw;
      p.row = raw.row;
      p.sym = raw.sym;
      p.delta = raw.delta;
      p.approach_turn = true;
      p.label = "turn->approach";
      return p;
    }
    const int minimum_window = cfg_.approach_escalate_window &&
                                       approach_no_progress_ > 0 &&
                                       approach_last_window_ >= 0
                                   ? approach_last_window_ + 1
                                   : 0;
    return approach_->plan(raw, minimum_window);
  }

  void reset_scan() {
    search_scan_active_ = false;
    search_scan_phase_ = 0;
    search_scan_sign_ = 1.0f;
    search_scan_rung_ = -1;
  }

  /// Turn a blind search into a non-cancelling sequence. Relative headings
  /// +90,-180,-90 visit left, right, then back instead of oscillating between
  /// the same two headings. A colour-bearing partial top is not blind: keep its
  /// directed reframe, and make it smaller when the partial geometry is close.
  Plan shape_scan(Plan raw) {
    if (raw.mode != Mode::SCAN || raw.approach_turn) {
      if (raw.mode == Mode::CLIP || raw.mode == Mode::APPROACH ||
          (cfg_.search_left_first && raw.approach_turn))
        reset_scan();
      return raw;
    }
    if (!cfg_.stateful_scan) return raw;
    if (belief_->valid()) {
      reset_scan();
      const Sight& partial = belief_->newest();
      if (partial.range_m() > 0.0f &&
          partial.range_m() < cfg_.near_reframe_range_m) {
        const float limit =
            cfg_.near_reframe_deg * 3.14159265358979323846f / 180.0f;
        raw.yaw_offset = std::clamp(raw.yaw_offset, -limit, limit);
        if (std::fabs(raw.yaw_offset) < 0.25f * limit)
          raw.yaw_offset = std::copysign(0.25f * limit, raw.yaw_offset);
        raw.label = "near reframe";
      }
      return raw;
    }
    if (!search_scan_active_ || search_scan_rung_ != clips_->rung()) {
      search_scan_active_ = true;
      search_scan_phase_ = 0;
      search_scan_rung_ = clips_->rung();
      // Positive base yaw is physically left / counter-clockwise. V9.1 fixes
      // that as the first global-search leg; v9 retains its hint-selected sign.
      search_scan_sign_ = cfg_.search_left_first
                              ? 1.0f
                              : (raw.yaw_offset < 0.0f ? -1.0f : 1.0f);
    }
    static constexpr float kScale[3] = {1.0f, -2.0f, -1.0f};
    const float sweep = cfg_.scan_sweep_deg * 3.14159265358979323846f / 180.0f;
    raw.yaw_offset = search_scan_sign_ * kScale[search_scan_phase_] * sweep;
    raw.search_scan = true;
    raw.search_phase = search_scan_phase_;
    raw.label = "search " + std::to_string(search_scan_phase_ + 1) + "/3";
    return raw;
  }

  /// v7.1 only. A clip is entered through a ramp, and the ramp is its OWN act:
  /// the controller engages it, plays it, reports finished, and only THEN does
  /// the clip go out — re-engaged on the pose the ramp actually reached.
  ///
  /// This is not cosmetic. `MotionClock::engage` stamps the reference's world
  /// anchor from row 0 and never re-measures it, so every frame between that
  /// stamp and the clip's first frame is dead reckoning. v7's lead-in was ~0.5
  /// s of it; a 1.2 s ramp is 5x, and the drift lands as a cube miss. Python
  /// does not have this problem because its warp is cube-absolute and re-solved
  /// at the clip's own commit (vibe docs/sys1_v7.md §7.1, `_maybe_enter`); here
  /// the equivalent is to re-engage, and to carry the heading across the seam
  /// as an absolute target rather than a residual that goes stale.
  bool ramping(const Plan& p) const {
    return cfg_.enter_yaw_rate_deg > 0.0f &&
           (p.mode == Mode::CLIP || p.mode == Mode::APPROACH);
  }

  void commit(const Plan& plan) {
    if (ramping(plan)) return commit_enter(plan);
    stage_ = ReferenceWriter::Stage::FULL;
    live_.held_row.clear();
    plan_ = plan;
    clips_->commit(plan_);  // burn the clip, count the roll
    if (plan_.search_scan) {
      search_scan_phase_ = plan_.search_phase + 1;
      if (search_scan_phase_ >= 3) {
        search_scan_phase_ = 0;
        search_scan_sign_ =
            cfg_.search_left_each_cycle ? 1.0f : -search_scan_sign_;
      }
    }
    if (plan_.approach_turn) {
      if (!approach_turn_locked_) {
        approach_turn_locked_ = true;
        approach_turn_anchor_ = plan_;
        approach_turn_anchor_.mode = Mode::CLIP;
        approach_turn_rung_ = clips_->rung();
        approach_turn_target_yaw_ = wrap(robot_yaw() + plan_.entry_yaw);
        RCLCPP_INFO(this->get_logger(),
                    "approach turn locked: row %d sym %d at rung %d", plan_.row,
                    plan_.sym, approach_turn_rung_);
      }
      ++approach_turn_attempts_;
      clear_candidate_lock();
      RCLCPP_INFO(this->get_logger(),
                  "approach turn commit: entry %.3f m at %+.1f deg "
                  "(attempt %d/%d)",
                  plan_.entry_translation, plan_.entry_bearing * 57.2958f,
                  approach_turn_attempts_, cfg_.approach_turn_max_attempts);
    } else if (approach_only_ && plan_.label == "approach ready") {
      // The unit-test launch parks here. Once aligned, release the old identity
      // so dragging the cube starts a completely fresh approach trial.
      clear_approach_turn();
    } else if (plan_.mode == Mode::APPROACH) {
      // A walk is intentionally followed by a global fresh retrieval. The
      // identity lock crosses only the in-place turn, never locomotion.
      clear_approach_turn();
      approach_before_m_ = plan_.entry_translation;
      approach_measure_anchor_ = plan_;
      approach_measure_target_yaw_ =
          wrap(robot_yaw() + plan_.matched_entry_yaw);
      awaiting_approach_measurement_ = true;
      approach_rung_ = clips_->rung();
      approach_last_window_ = plan_.approach_window;
      ++approach_attempts_;
      clear_candidate_lock();
      RCLCPP_INFO(this->get_logger(),
                  "approach commit: entry %.3f m at %+.1f deg, ask %.3f m, "
                  "window %.3f m (attempt %d/%d)",
                  plan_.entry_translation, plan_.entry_bearing * 57.2958f,
                  plan_.approach_requested, plan_.approach_covered,
                  approach_attempts_, cfg_.approach_max_attempts);
    } else if (plan_.mode == Mode::CLIP) {
      clear_candidate_lock();
      reset_approach();
    }
    reference_id_ = "the planner-" + std::to_string(++seq_);
    waiting_ = 0;
    committed_ = true;
    // A new act invalidates the evidence gathered under the old one: the next
    // decision must be made from reads taken while THIS reference was playing.
    belief_->clear();
    publish_reference(writer_->build(plan_, live_, stage_));
  }

  void commit_enter(const Plan& clip) {
    // Freeze the heading the ramp is paying toward while the robot is still
    // quiescent — the ONE number that must survive the ramp. `entry_yaw` is a
    // base-frame residual, so it goes stale the instant the robot turns; the
    // absolute target it implies does not.
    enter_target_yaw_ = wrap(robot_yaw() + clip.entry_yaw);
    enter_matched_target_yaw_ = wrap(robot_yaw() + clip.matched_entry_yaw);
    // Start the ramp from what the controller is being TOLD, not from where it
    // is.
    if (const float* held = writer_->final_row())
      live_.held_row.assign(held, held + writer_->cols());
    else
      live_.held_row.clear();

    pending_ = clip;
    plan_ = clip;  // same row and sym; the writer emits the ramp, not the clip
    plan_.label = "->" + clip.label;
    stage_ = ReferenceWriter::Stage::ENTER;
    in_enter_ = true;
    reference_id_ = "the planner-" + std::to_string(++seq_);
    waiting_ = 0;
    committed_ = true;
    belief_->clear();  // the ramp decides nothing, and reads nothing
    publish_reference(writer_->build(plan_, live_, stage_));
  }

  void commit_clip(Plan clip) {
    // What the ramp FAILED to pay, measured. The controller tracks a swept
    // heading with a lag, so this is small but never zero, and assuming zero is
    // exactly the error the split exists to remove.
    clip.entry_yaw = wrap(enter_target_yaw_ - robot_yaw());
    stage_ = ReferenceWriter::Stage::CLIP;
    in_enter_ = false;
    live_.held_row.clear();
    plan_ = clip;
    clips_->commit(plan_);  // APPROACH is ignored; a CLIP burns exactly once
    if (plan_.mode == Mode::APPROACH) {
      clear_approach_turn();
      approach_before_m_ = plan_.entry_translation;
      approach_measure_anchor_ = plan_;
      approach_measure_target_yaw_ = enter_matched_target_yaw_;
      awaiting_approach_measurement_ = true;
      approach_rung_ = clips_->rung();
      approach_last_window_ = plan_.approach_window;
      ++approach_attempts_;
      clear_candidate_lock();
      RCLCPP_INFO(this->get_logger(),
                  "approach commit after ramp: entry %.3f m at %+.1f deg, "
                  "ask %.3f m, window %.3f m (attempt %d/%d)",
                  plan_.entry_translation, plan_.entry_bearing * 57.2958f,
                  plan_.approach_requested, plan_.approach_covered,
                  approach_attempts_, cfg_.approach_max_attempts);
    } else {
      clear_candidate_lock();
      reset_approach();
    }
    reference_id_ = "the planner-" + std::to_string(++seq_);
    waiting_ = 0;
    committed_ = true;
    belief_->clear();
    publish_reference(writer_->build(plan_, live_, stage_));
  }

  void republish() { publish_reference(writer_->build(plan_, live_, stage_)); }

  void publish_reference(const std::vector<float>& rows) {
    msg::MotionReference m;
    m.schema_version = msg::MotionReference::SCHEMA_VERSION;
    m.reference_id = reference_id_;
    m.frames = static_cast<uint32_t>(writer_->frames());
    m.cols = static_cast<uint32_t>(writer_->cols());
    m.fps = table_.fps();
    // Both are source values, not fallbacks: a clip carries its recorded twist
    // and contact schedule, and a stand truthfully commands zero of each.
    m.has_twist = true;
    m.has_contact = true;
    m.has_object_goal = false;
    // v7.1 ramps the heading inside the rows, and engage() would compose the
    // two: the residual is claimed exactly once, by whoever carried it.
    const bool heading_act =
        plan_.mode == Mode::CLIP || plan_.mode == Mode::APPROACH;
    m.entry_yaw_offset =
        heading_act && !writer_->ramped() ? plan_.entry_yaw : 0.0f;
    m.mode = static_cast<uint8_t>(plan_.mode);
    m.data = rows;
    reference_pub_->publish(m);
  }

  /// Every tick, not every commit. The belief moves between commits and so does
  /// the candidate plan, so a once-per-second status froze the console on stale
  /// numbers and gave a bag one sample per act to reason about.
  void publish_status() {
    msg::PlannerStatus m;
    m.mode = static_cast<uint8_t>(plan_.mode);
    m.label = plan_.label;
    m.cost = plan_.cost;
    m.entry_yaw = plan_.entry_yaw;
    m.entry_translation_m = plan_.entry_translation;
    m.entry_bearing_rad = plan_.entry_bearing;
    m.entry_forward_m = plan_.entry_forward;
    m.entry_lateral_m = plan_.entry_lateral;
    m.approach_requested_m = plan_.approach_requested;
    m.approach_covered_m = plan_.approach_covered;
    m.frames = writer_->frames();
    m.lead_in_frames = writer_->lead_in_frames();
    m.committed = committed_;
    committed_ = false;

    m.cand_mode = static_cast<uint8_t>(candidate_.mode);
    m.cand_label = candidate_.label;
    m.cand_cost = candidate_.cost;
    m.cand_entry_translation_m = candidate_.entry_translation;
    m.cand_entry_bearing_rad = candidate_.entry_bearing;
    m.cand_entry_forward_m = candidate_.entry_forward;
    m.cand_entry_lateral_m = candidate_.entry_lateral;
    m.candidate_locked = candidate_locked_ || approach_turn_locked_;
    const Plan& lock =
        approach_turn_locked_ ? approach_turn_anchor_ : candidate_anchor_;
    m.candidate_lock_row = m.candidate_locked ? lock.row : -1;
    m.candidate_lock_sym = m.candidate_locked ? lock.sym : -1;
    m.search_phase = plan_.search_phase;
    m.approach_progress_m = approach_progress_m_;
    m.approach_attempts = approach_attempts_;
    m.approach_no_progress = approach_no_progress_;
    m.approach_only = approach_only_;
    m.approach_turn_locked = approach_turn_locked_;
    m.approach_turn_attempts = approach_turn_attempts_;
    m.approach_turn_row =
        approach_turn_locked_ ? approach_turn_anchor_.row : -1;
    m.approach_turn_sym =
        approach_turn_locked_ ? approach_turn_anchor_.sym : -1;

    m.delta = static_cast<uint8_t>(clips_->delta());
    m.rung = clips_->rung();
    m.tips = clips_->tips();
    m.stall = clips_->stall();
    m.burned = clips_->burned();
    m.rolls = clips_->rolls();
    m.episode = clips_->episode();
    m.target_color = clips_->target_color();
    m.done = clips_->done();

    m.n_reads = belief_->n_reads();
    m.n_votes = belief_->n_votes();
    m.omega_cam = omega_cam_;
    m.accept_rate = frames_offered_
                        ? static_cast<float>(frames_read_) / frames_offered_
                        : 0.0f;
    m.color = belief_->color();
    m.sees = belief_->valid();
    m.pose_ok = belief_->pose_ok();
    if (belief_->pose_ok()) {
      const Sight& s = belief_->pose();
      m.range_m = s.range_m();
      m.bearing_rad = s.bearing();
      m.phi_rad = s.phi;
      m.n_px = s.n_px;
    }
    m.reason = belief_->empty() ? "blind" : belief_->newest().reason;
    m.version = version_;
    m.observe_ms = observe_ms_;
    status_pub_->publish(m);
  }

  // config + artifacts
  Cfg cfg_;
  std::string version_;
  ClipTable table_;
  std::unique_ptr<Clips> clips_;
  std::unique_ptr<CubeSight> eye_;
  std::unique_ptr<Belief> belief_;
  std::unique_ptr<ApproachSource> approach_;
  std::unique_ptr<ReferenceWriter> writer_;
  std::string camera_ip_;
  uint16_t camera_port_ = 5555;
  double rate_hz_ = 20.0, stale_s_ = 1.0;
  Mount mount_;
  bool stand_ready_ =
      false;  ///< v7: the nominal stance has arrived from the controller
  std::array<int, 3> waist_{};

  // live state
  LiveState live_;
  std::array<float, 4> imu_quat_{1.f, 0.f, 0.f, 0.f};
  std::array<float, 3> waist_q_{};
  float omega_cam_ = 0.f;  ///< rad/s; the quiescence gate reads this
  rclcpp::Time last_state_{0, 0, RCL_ROS_TIME};
  msg::ControllerStatus controller_;
  bool controller_seen_ = false;

  // camera thread
  std::thread tcp_thread_;
  std::atomic<bool> running_{false};
  std::mutex frame_mutex_;
  cv::Mat bgr_, depth_;
  Intrinsics intr_;
  std::atomic<uint64_t> frame_seq_{0};
  std::atomic<int64_t> last_frame_ns_{0};
  uint64_t seen_seq_ = 0;
  int colour_only_ = 0;
  bool depth_warned_ = false;

  // plan state
  Plan plan_, candidate_;
  Plan pending_;  ///< v7.1: the clip the running ENTER ramp leads into
  ReferenceWriter::Stage stage_ = ReferenceWriter::Stage::FULL;
  bool in_enter_ = false;
  float enter_target_yaw_ = 0.f;  ///< absolute heading, frozen at the ramp
  float enter_matched_target_yaw_ =
      0.f;  ///< manipulation stance retained while an APPROACH ramp turns
  std::string reference_id_;
  uint64_t seq_ = 0;
  int waiting_ = 0;
  uint64_t frames_offered_ = 0, frames_read_ = 0;
  bool armed_ = false, committed_ = false;
  bool approach_only_ = false;
  int approach_attempts_ = 0, approach_no_progress_ = 0;
  int approach_last_window_ = -1;
  int approach_rung_ = -1;
  Plan candidate_anchor_;
  bool candidate_locked_ = false;
  int candidate_lock_rung_ = -1;
  float candidate_target_yaw_ = 0.0f;
  Plan approach_turn_anchor_;
  bool approach_turn_locked_ = false;
  int approach_turn_attempts_ = 0, approach_turn_rung_ = -1;
  float approach_turn_target_yaw_ = 0.0f;
  bool awaiting_approach_measurement_ = false;
  Plan approach_measure_anchor_;
  float approach_measure_target_yaw_ = 0.0f;
  float approach_before_m_ = 0.0f, approach_progress_m_ = 0.0f;
  bool search_scan_active_ = false;
  int search_scan_phase_ = 0, search_scan_rung_ = -1;
  float search_scan_sign_ = 1.0f;
  float observe_ms_ = 0.f;

  rclcpp::Publisher<msg::MotionReference>::SharedPtr reference_pub_;
  rclcpp::Publisher<msg::PlannerStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<msg::ControllerStatus>::SharedPtr controller_sub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cpp_control::planners::repose::ReposeNode>());
  rclcpp::shutdown();
  return 0;
}
