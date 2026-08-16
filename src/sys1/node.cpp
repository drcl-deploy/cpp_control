/**
 * sys1 — the planner above sys0. Camera in, MotionReference out.
 *
 * Soft real-time and a separate process on purpose: it must never sit in the
 * 50 Hz loop, and sys0 has to stay runnable without it (open-loop rollouts are
 * the default). It owns no clock — every mode is timed off Sys0Status, so what
 * the robot is ACTUALLY playing is the only thing that advances the plan.
 *
 *   ros2 launch cpp_control g1_sys1_repose.launch.py
 *   ros2 run cpp_control sys1_console.py        # set the target colour live
 *
 * Design + the four findings that shrank it: docs/sys1.md
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
#include "sys1/clips.hpp"
#include "sys1/kinematics.hpp"
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
                "target %s | camera %s:%u @ %.0f Hz",
                version_.c_str(), table_.rows().size(), table_.pool('F').size(),
                table_.pool('B').size(), table_.pool('L').size(),
                table_.pool('R').size(), cfg_.pattern.c_str(),
                vibe::cube_color_name(clips_->target_color()),
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
    version_ = root["version"] ? root["version"].as<std::string>() : "v6";
    cfg_ = Cfg::preset(version_);
    if (const auto k = root["knobs"]) {
      auto f = [&](const char* n, float& v) { if (k[n]) v = k[n].as<float>(); };
      auto i = [&](const char* n, int& v) { if (k[n]) v = k[n].as<int>(); };
      auto b = [&](const char* n, bool& v) { if (k[n]) v = k[n].as<bool>(); };
      if (k["pattern"]) cfg_.pattern = k["pattern"].as<std::string>();
      b("nominal_stand", cfg_.nominal_stand);  // v6 + this IS v7; A/B in one edit
      i("retry_limit", cfg_.retry_limit);
      i("stall_limit", cfg_.stall_limit);
      i("settle_steps", cfg_.settle_steps);
      i("scan_steps", cfg_.scan_steps);
      i("read_tail", cfg_.read_tail);
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
    for (int i = 0; i < 3; ++i) waist_q_[i] = m->motor_state[waist_[i]].q;
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
    if (m->data == clips_->target_color()) return;
    clips_->set_target_color(m->data);
    RCLCPP_INFO(this->get_logger(), "target -> %d (%s)", m->data,
                vibe::cube_color_name(m->data));
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

    std::lock_guard<std::mutex> lock(frame_mutex_);
    bgr_ = std::move(bgr);
    depth_ = std::move(metres);
    intr_ = {depth->fx, depth->fy, depth->cx, depth->cy};
    last_frame_ = this->now();
  }

  // ── The loop ───────────────────────────────────────────────────

  bool fresh(const rclcpp::Time& t) const {
    return t.nanoseconds() > 0 && (this->now() - t).seconds() < stale_s_;
  }

  void tick() {
    if (!sys0_seen_ || !sys0_.accepting) {
      if (armed_) RCLCPP_INFO(this->get_logger(), "sys1: sys0 left POLICY — idle");
      armed_ = false;
      return;
    }
    if (!fresh(last_state_) || !fresh(last_frame_) ||
        (cfg_.nominal_stand && !stand_ready_)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "sys1: waiting for %s%s%s",
                           fresh(last_state_) ? "" : "lowstate ",
                           fresh(last_frame_) ? "" : "camera frames ",
                           (!cfg_.nominal_stand || stand_ready_)
                               ? ""
                               : "sys0's nominal stance");
      return;
    }

    if (!armed_) {  // fresh episode: never plan the new one from the old cube
      armed_ = true;
      clips_->reset();
      RCLCPP_INFO(this->get_logger(), "sys1: armed, target %s",
                  vibe::cube_color_name(clips_->target_color()));
      commit(clips_->still(Mode::SETTLE, 0.0f));
      return;
    }

    if (sys0_.reference_id != reference_id_) {
      // sys0 commits on arrival, so this is a dropped message, not a race.
      if (++waiting_ > 3) {
        RCLCPP_WARN(this->get_logger(), "sys1: '%s' never engaged — resending",
                    reference_id_.c_str());
        republish();
      }
      return;
    }
    waiting_ = 0;

    if (plan_.mode == Mode::CLIP) {  // eyes shut, pure replay
      if (sys0_.finished) commit(clips_->still(Mode::SETTLE, 0.0f));
      return;
    }
    // The sweep finishes 2 read-tails before the end, so these are exactly the
    // frames with the camera stationary at nominal pitch.
    if (sys0_.frame + 2 * cfg_.read_tail >= sys0_.frames) look();
    if (sys0_.finished) commit(clips_->decide());
  }

  void look() {
    cv::Mat bgr, depth;
    Intrinsics intr;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      bgr = bgr_;
      depth = depth_;
      intr = intr_;
    }
    const auto t0 = std::chrono::steady_clock::now();
    clips_->look(bgr, depth, intr, camera_pose());
    observe_ms_ = std::chrono::duration<float, std::milli>(
                      std::chrono::steady_clock::now() - t0).count();
    ++looks_;
  }

  void commit(const Plan& plan) {
    plan_ = plan;
    reference_id_ = "sys1-" + std::to_string(++seq_);
    waiting_ = 0;
    looks_ = 0;
    publish_reference(writer_->build(plan_, live_));
    publish_status();
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

  void publish_status() {
    const Sight& s = clips_->sight();
    msg::Sys1Status m;
    m.mode = static_cast<uint8_t>(plan_.mode);
    m.label = plan_.label;
    m.delta = static_cast<uint8_t>(clips_->delta());
    m.rung = clips_->rung();
    m.tips = clips_->tips();
    m.stall = clips_->stall();
    m.burned = clips_->burned();
    m.target_color = clips_->target_color();
    m.done = clips_->done();
    m.sees = s.color_ok;
    m.pose_ok = s.ok;
    m.color = s.color;
    m.reason = s.reason;
    m.range_m = s.range_m();
    m.bearing_rad = s.bearing();
    m.phi_rad = s.phi;
    m.n_px = s.n_px;
    m.cost = plan_.cost;
    m.entry_yaw = plan_.entry_yaw;
    m.frames = writer_->frames();
    m.lead_in_frames = writer_->lead_in_frames();
    m.version = version_;
    m.observe_ms = observe_ms_;
    status_pub_->publish(m);
  }

  // config + artifacts
  Cfg cfg_;
  std::string version_;
  ClipTable table_;
  std::unique_ptr<Sys1Clips> clips_;
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
  rclcpp::Time last_state_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_frame_{0, 0, RCL_ROS_TIME};
  msg::Sys0Status sys0_;
  bool sys0_seen_ = false;

  // camera thread
  std::thread tcp_thread_;
  std::atomic<bool> running_{false};
  std::mutex frame_mutex_;
  cv::Mat bgr_, depth_;
  Intrinsics intr_;
  int colour_only_ = 0;
  bool depth_warned_ = false;

  // plan state
  Plan plan_;
  std::string reference_id_;
  uint64_t seq_ = 0;
  int waiting_ = 0, looks_ = 0;
  bool armed_ = false;
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
