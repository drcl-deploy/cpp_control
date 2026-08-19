/**
 * sys1 — the planner above sys0. Camera in, MotionReference out.
 *
 * Soft real-time and a separate process on purpose: it must never sit in the
 * 50 Hz loop, and sys0 has to stay runnable without it (open-loop rollouts are
 * the default). It owns no clock — the act is timed off Sys0Status, so what the
 * robot is ACTUALLY playing is the only thing that advances the plan.
 *
 * OBSERVE every tick the camera is quiet, PLAN every tick for free, ACT only
 * when sys0 says it finished. That split is the whole node:
 *
 *     GUARD    sys0 accepting? lowstate fresh? nominal stance known?
 *     OBSERVE  new frame AND |omega_cam| < omega_still  ->  belief << look()
 *     PLAN     candidate = decide(belief)                   PURE, ~50 us
 *     ACT      sys0.finished                            ->  commit(candidate)
 *
 *   ros2 launch cpp_control g1_sys1_repose.launch.py
 *   ros2 run cpp_control sys1_console.py        # set the target colour live
 *
 * Design + the findings that shrank it: docs/vibe/sys1/planner.md
 */

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <cpp_control/msg/motion_reference.hpp>
#include <cpp_control/msg/sys0_status.hpp>
#include <cpp_control/msg/sys1_status.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <vision_encoders/frame_wire.hpp>

#include "common/g1/joint_orders.hpp"
#include "common/g1/motion.hpp"
#include "common/math_utils.hpp"
#include "cpp_control/tasks/vibe/task_profile.hpp"
#include "sys1/belief.hpp"
#include "sys1/clips.hpp"
#include "sys1/kinematics.hpp"
#include "sys1/sight.hpp"
#include "sys1/table.hpp"
#include "sys1/writer.hpp"

namespace cpp_control {
namespace sys1 {

namespace wire = vision_encoders::wire;

namespace {

std::array<float, 3> yaml_vec3(const YAML::Node& n,
                               const std::array<float, 3>& fallback) {
  if (!n || n.size() != 3) return fallback;
  return {n[0].as<float>(), n[1].as<float>(), n[2].as<float>()};
}

}  // namespace

class Sys1Node : public rclcpp::Node {
 public:
  Sys1Node() : Node("g1_sys1_repose_node") {
    const auto config_path = this->declare_parameter("config_path", "");
    if (config_path.empty())
      throw std::runtime_error("sys1: config_path is required");
    load_config(config_path);
    // The experiment environment owns the camera address — setup.sh picks it,
    // the launch passes the same `camera_ip` to the encoder and to here, so the
    // two halves of one run cannot look at different cameras. Empty keeps the
    // yaml's value, which is what a standalone `ros2 run` gets.
    const auto camera_host = this->declare_parameter("camera_host", "");
    if (!camera_host.empty()) camera_ip_ = camera_host;

    // The goal colour conditions the POLICY's one-hot and steers the PLANNER's
    // ladder, and the two must not boot disagreeing: the topic keeps them in
    // step afterwards, but nothing did until the first message. g1_sys1_repose
    // hands its `goal_color` here as well as to sys0, so one launch argument
    // sets both. Out of range (the default) keeps the yaml, which is what a
    // standalone `ros2 run` gets.
    const auto target = static_cast<int>(
        this->declare_parameter("target_color", -1));
    if (target >= 0 && target < vibe::NUM_CUBE_COLORS)
      clips_->set_target_color(target);

    reference_pub_ = this->create_publisher<msg::MotionReference>(
        this->declare_parameter("reference_topic", "/tracker/reference"), 10);
    status_pub_ = this->create_publisher<msg::Sys1Status>(
        this->declare_parameter("sys1_status_topic", "/vibe/sys1/status"), 10);
    sys0_sub_ = this->create_subscription<msg::Sys0Status>(
        this->declare_parameter("sys0_status_topic", "/vibe/sys0/status"),
        rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](msg::Sys0Status::SharedPtr m) {
          sys0_ = *m;
          sys0_seen_ = true;
          adopt_nominal_stand(m->default_joint_pos);
        });
    lowstate_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
        this->declare_parameter("lowstate_topic", "/lowstate"), 10,
        [this](unitree_hg::msg::LowState::SharedPtr m) { on_lowstate(m); });
    goal_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        this->declare_parameter("goal_color_topic", "/vibe/sonic/goal_color"),
        10, [this](std_msgs::msg::Int32::SharedPtr m) { on_goal_color(m); });

    running_.store(true);
    tcp_thread_ = std::thread(&Sys1Node::tcp_loop, this);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / rate_hz_)),
        [this]() { tick(); });

    RCLCPP_INFO(this->get_logger(),
                "sys1 %s ready: %zu clips (F%zu B%zu L%zu R%zu) | pattern %s | "
                "target %s | belief %d/%d reads under %.2f rad/s | camera "
                "%s:%u @ %.0f Hz",
                version_.c_str(), table_.rows().size(), table_.pool('F').size(),
                table_.pool('B').size(), table_.pool('L').size(),
                table_.pool('R').size(), cfg_.pattern.c_str(),
                vibe::cube_color_name(clips_->target_color()),
                cfg_.belief_min_votes, cfg_.belief_window, cfg_.omega_still,
                camera_ip_.c_str(), camera_port_, rate_hz_);
  }

  ~Sys1Node() override {
    running_.store(false);
    if (tcp_thread_.joinable()) tcp_thread_.join();
  }

 private:
  // ── Config ─────────────────────────────────────────────────────

  void load_config(const std::string& path) {
    const YAML::Node root = YAML::LoadFile(path);
    version_ = root["version"] ? root["version"].as<std::string>() : "v7";
    cfg_ = Cfg::preset(version_);
    if (const auto k = root["knobs"]) {
      auto f = [&](const char* n, float& v) { if (k[n]) v = k[n].as<float>(); };
      auto i = [&](const char* n, int& v) { if (k[n]) v = k[n].as<int>(); };
      auto b = [&](const char* n, bool& v) { if (k[n]) v = k[n].as<bool>(); };
      if (k["pattern"]) cfg_.pattern = k["pattern"].as<std::string>();
      b("nominal_stand", cfg_.nominal_stand);  // v6 + this IS v7; A/B in one edit
      i("retry_limit", cfg_.retry_limit);
      i("settle_steps", cfg_.settle_steps);
      i("scan_steps", cfg_.scan_steps);
      i("hold_tail", cfg_.hold_tail);
      i("belief_window", cfg_.belief_window);
      i("belief_min_votes", cfg_.belief_min_votes);
      f("omega_still", cfg_.omega_still);
      i("blend_frames", cfg_.blend_frames);
      i("proc_width", cfg_.proc_width);
      i("min_value", cfg_.min_value);
      f("min_area_frac", cfg_.min_area_frac);
      f("min_visible", cfg_.min_visible);
      f("min_visible_color", cfg_.min_visible_color);
      f("min_rel_sat", cfg_.min_rel_sat);
      f("up_dot_min", cfg_.up_dot_min);
      f("arm_radius", cfg_.arm_radius);
      f("horizon_gain", cfg_.horizon_gain);
      f("stance_band_m", cfg_.stance_band_m);
      f("scan_sweep_deg", cfg_.scan_sweep_deg);
      f("lead_in_rate", cfg_.lead_in_rate);
      f("lead_in_min_s", cfg_.lead_in_min_s);
      f("lead_in_max_s", cfg_.lead_in_max_s);
    }
    cfg_.validate();

    table_ = ClipTable::load(root["clips"].as<std::string>());
    const auto frames = root["frames"];
    const auto source = frames["source"].as<std::string>();
    const auto frames_path = frames["path"].as<std::string>();
    if (source == "retargeted")
      table_.load_frames_retargeted(frames["library"].as<std::string>(),
                                    frames_path);
    else if (source == "baked")
      table_.load_frames_baked(frames_path);
    else
      throw std::runtime_error("sys1: frames.source must be retargeted | baked");

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

    const int target = root["target_color"] ? root["target_color"].as<int>() : 4;
    clips_ = std::make_unique<Sys1Clips>(table_, cfg_, target);
    eye_ = std::make_unique<CubeSight>(cfg_, PALETTE_SIM, table_.half_extent());
    belief_ = std::make_unique<Belief>(cfg_);
    writer_ = std::make_unique<ReferenceWriter>(table_, cfg_);

    for (int i = 0; i < 3; ++i) {
      static const char* kWaist[3] = {"waist_yaw_joint", "waist_roll_joint",
                                      "waist_pitch_joint"};
      const auto it = std::find(g1::MJ_JOINTS.begin(), g1::MJ_JOINTS.end(),
                                kWaist[i]);
      if (it == g1::MJ_JOINTS.end())
        throw std::runtime_error("sys1: G1 joint table has no waist chain");
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
    return sys1::camera_pose(imu_quat_, waist_q_, mount_);
  }

  // ── The v7 still pose ──────────────────────────────────────────
  //
  // Taken from sys0 rather than parsed here: `default_joint_pos` is the
  // manifest's, which is the pose the controller actually holds in
  // NOMINAL_POSE and the same array mjlab FKs. One source, so the stance the
  // planner commands and the stance the robot stands in cannot disagree.
  // Arrives on the first Sys0Status, which the planner already waits for.

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
    const Gaze g = gaze_of(sys1::camera_pose({1.f, 0.f, 0.f, 0.f}, waist, mount_));
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
      RCLCPP_WARN(this->get_logger(), "ignoring invalid goal_color %d", m->data);
      return;
    }
    // Re-picking the colour already selected is the operator saying "go again":
    // it is the only way out of a LATCHED done, and it aborts a trial that went
    // wrong. Same key, new episode, either way.
    if (m->data == clips_->target_color()) {
      clips_->reset();
      belief_->clear();
      RCLCPP_INFO(this->get_logger(), "re-armed on %s — episode %d",
                  vibe::cube_color_name(m->data), clips_->episode());
      return;
    }
    clips_->set_target_color(m->data);
    belief_->clear();
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
      if (fd < 0 || ::connect(fd, reinterpret_cast<sockaddr*>(&addr),
                              sizeof(addr)) < 0) {
        RCLCPP_WARN(this->get_logger(), "sys1: cannot reach %s:%u, retrying",
                    camera_ip_.c_str(), camera_port_);
        if (fd >= 0) ::close(fd);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "sys1: camera %s:%u connected",
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
                     "sys1: 200 frames with no depth plane — start the streamer "
                     "with --depth (or set camera_depth: 1 in the sim config)");
      }
      return;
    }
    if (!depth->has_intrinsics()) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "sys1: depth plane carries no intrinsics");
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
  /// A CLIP is excluded outright rather than left to the gate. A clip has quiet
  /// moments, but it spends them in an arbitrary exit pose whose waist aims the
  /// head wherever the recording left it — that is precisely the v6 gaze the
  /// nominal stance exists to escape, so those reads are quiet and still wrong.
  /// The mandatory settle after every clip is what this produces: a clip ends
  /// with an empty belief, and an empty belief plans the stance.
  void observe() {
    if (plan_.mode == Mode::CLIP) return;
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
                      std::chrono::steady_clock::now() - t0).count();
    ++frames_read_;
  }

  // ── The loop ───────────────────────────────────────────────────

  void tick() {
    if (!sys0_seen_ || !sys0_.accepting) {
      if (armed_) RCLCPP_INFO(this->get_logger(), "sys1: sys0 left POLICY — idle");
      armed_ = false;
      return;
    }
    // The camera is deliberately NOT a gate. Losing it starves the belief, and
    // a starved belief plans the nominal stance — so a camera that dies mid
    // clip returns the robot to its feet instead of freezing it in the exit
    // pose of a half-finished turn.
    if (!fresh(last_state_) || (cfg_.nominal_stand && !stand_ready_)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "sys1: waiting for %s%s",
                           fresh(last_state_) ? "" : "lowstate ",
                           (!cfg_.nominal_stand || stand_ready_)
                               ? ""
                               : "sys0's nominal stance");
      return;
    }
    if (!camera_fresh())
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "sys1: no camera frames — standing by");

    if (!armed_) {  // fresh episode: never plan the new one from the old cube
      armed_ = true;
      clips_->reset();
      belief_->clear();
      RCLCPP_INFO(this->get_logger(), "sys1: armed, target %s, episode %d",
                  vibe::cube_color_name(clips_->target_color()),
                  clips_->episode());
      candidate_ = clips_->decide(*belief_);
      commit(candidate_);
      publish_status();
      return;
    }

    observe();
    clips_->observe(*belief_);
    candidate_ = clips_->decide(*belief_);

    if (sys0_.reference_id != reference_id_) {
      // sys0 commits on arrival, so this is a dropped message, not a race.
      if (++waiting_ > 3) {
        RCLCPP_WARN(this->get_logger(), "sys1: '%s' never engaged — resending",
                    reference_id_.c_str());
        republish();
        waiting_ = 0;  // one resend per 4 ticks, not one per tick forever
      }
    } else {
      waiting_ = 0;
      if (sys0_.finished) commit(candidate_);
    }
    publish_status();
  }

  bool fresh(const rclcpp::Time& t) const {
    return t.nanoseconds() > 0 && (this->now() - t).seconds() < stale_s_;
  }

  bool camera_fresh() const {
    const int64_t ns = last_frame_ns_.load(std::memory_order_relaxed);
    return ns > 0 &&
           (this->now() - rclcpp::Time(ns, RCL_ROS_TIME)).seconds() < stale_s_;
  }

  void commit(const Plan& plan) {
    plan_ = plan;
    clips_->commit(plan_);  // burn the clip, count the roll
    reference_id_ = "sys1-" + std::to_string(++seq_);
    waiting_ = 0;
    committed_ = true;
    // A new act invalidates the evidence gathered under the old one: the next
    // decision must be made from reads taken while THIS reference was playing.
    belief_->clear();
    publish_reference(writer_->build(plan_, live_));
  }

  void republish() { publish_reference(writer_->build(plan_, live_)); }

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
    m.entry_yaw_offset = plan_.entry_yaw;
    m.mode = static_cast<uint8_t>(plan_.mode);
    m.data = rows;
    reference_pub_->publish(m);
  }

  /// Every tick, not every commit. The belief moves between commits and so does
  /// the candidate plan, so a once-per-second status froze the console on stale
  /// numbers and gave a bag one sample per act to reason about.
  void publish_status() {
    msg::Sys1Status m;
    m.mode = static_cast<uint8_t>(plan_.mode);
    m.label = plan_.label;
    m.cost = plan_.cost;
    m.entry_yaw = plan_.entry_yaw;
    m.frames = writer_->frames();
    m.lead_in_frames = writer_->lead_in_frames();
    m.committed = committed_;
    committed_ = false;

    m.cand_mode = static_cast<uint8_t>(candidate_.mode);
    m.cand_label = candidate_.label;
    m.cand_cost = candidate_.cost;

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
  std::unique_ptr<Sys1Clips> clips_;
  std::unique_ptr<CubeSight> eye_;
  std::unique_ptr<Belief> belief_;
  std::unique_ptr<ReferenceWriter> writer_;
  std::string camera_ip_;
  uint16_t camera_port_ = 5555;
  double rate_hz_ = 20.0, stale_s_ = 1.0;
  Mount mount_;
  bool stand_ready_ = false;  ///< v7: the nominal stance has arrived from sys0
  std::array<int, 3> waist_{};

  // live state
  LiveState live_;
  std::array<float, 4> imu_quat_{1.f, 0.f, 0.f, 0.f};
  std::array<float, 3> waist_q_{};
  float omega_cam_ = 0.f;  ///< rad/s; the quiescence gate reads this
  rclcpp::Time last_state_{0, 0, RCL_ROS_TIME};
  msg::Sys0Status sys0_;
  bool sys0_seen_ = false;

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
  std::string reference_id_;
  uint64_t seq_ = 0;
  int waiting_ = 0;
  uint64_t frames_offered_ = 0, frames_read_ = 0;
  bool armed_ = false, committed_ = false;
  float observe_ms_ = 0.f;

  rclcpp::Publisher<msg::MotionReference>::SharedPtr reference_pub_;
  rclcpp::Publisher<msg::Sys1Status>::SharedPtr status_pub_;
  rclcpp::Subscription<msg::Sys0Status>::SharedPtr sys0_sub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sys1
}  // namespace cpp_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cpp_control::sys1::Sys1Node>());
  rclcpp::shutdown();
  return 0;
}
