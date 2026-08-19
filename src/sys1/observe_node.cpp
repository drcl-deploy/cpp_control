/**
 * Hardware/sim acquisition source for calibrating sys1's observation block.
 *
 * Camera TCP in; synchronized ROS RGB-D + production CubeSight diagnostics
 * out. It never publishes a robot command. The launch wrapper puts sys0 under
 * calibration_lock, and every observation says whether that lock is actually
 * engaged so the offboard labeler cannot count unsafe frames.
 */

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <cpp_control/msg/sys0_status.hpp>
#include <cpp_control/msg/sys1_observation.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <vision_encoders/frame_wire.hpp>

#include "common/gamepad.hpp"
#include "common/g1/joint_orders.hpp"
#include "sys1/cfg.hpp"
#include "sys1/kinematics.hpp"
#include "sys1/sight.hpp"
#include "sys1/table.hpp"

namespace cpp_control {
namespace sys1 {

namespace wire = vision_encoders::wire;

namespace {

std::array<float, 3> yaml_vec3(const YAML::Node& n,
                               const std::array<float, 3>& fallback) {
  if (!n || n.size() != 3) return fallback;
  return {n[0].as<float>(), n[1].as<float>(), n[2].as<float>()};
}

rclcpp::QoS image_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

}  // namespace

class Sys1ObserveNode final : public rclcpp::Node {
 public:
  Sys1ObserveNode() : Node("g1_sys1_observe_node") {
    const std::string config_path =
        this->declare_parameter("config_path", "");
    if (config_path.empty())
      throw std::runtime_error("sys1 observe: config_path is required");
    load_config(config_path);

    const std::string camera_host =
        this->declare_parameter("camera_host", "");
    if (!camera_host.empty()) camera_ip_ = camera_host;

    color_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        this->declare_parameter("color_topic",
                                "/vibe/sys1/observe/color/compressed"),
        image_qos());
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        this->declare_parameter("depth_topic", "/vibe/sys1/observe/depth"),
        image_qos());
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        this->declare_parameter("camera_info_topic",
                                "/vibe/sys1/observe/camera_info"),
        image_qos());
    labels_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        this->declare_parameter("labels_topic", "/vibe/sys1/observe/labels"),
        image_qos());
    observation_pub_ = this->create_publisher<msg::Sys1Observation>(
        this->declare_parameter("observation_topic",
                                "/vibe/sys1/observe/observation"),
        image_qos());
    click_pub_ = this->create_publisher<std_msgs::msg::UInt64>(
        this->declare_parameter("click_topic",
                                "/vibe/sys1/calibration/click"),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

    lowstate_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
        this->declare_parameter("lowstate_topic", "/lowstate"), 10,
        [this](unitree_hg::msg::LowState::SharedPtr m) { on_lowstate(*m); });
    sys0_sub_ = this->create_subscription<msg::Sys0Status>(
        this->declare_parameter("sys0_status_topic", "/vibe/sys0/status"),
        image_qos(),
        [this](msg::Sys0Status::SharedPtr m) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          sys0_ = *m;
          sys0_seen_ = true;
          last_sys0_ = this->now();
        });

    running_.store(true);
    tcp_thread_ = std::thread(&Sys1ObserveNode::tcp_loop, this);
    RCLCPP_INFO(this->get_logger(),
                "sys1 observe ready: camera %s:%u, publish every paired RGB-D "
                "record (source depth rate owns the clock)",
                camera_ip_.c_str(), camera_port_);
  }

  ~Sys1ObserveNode() override {
    running_.store(false);
    const int fd = socket_fd_.load();
    if (fd >= 0) ::shutdown(fd, SHUT_RDWR);
    if (tcp_thread_.joinable()) tcp_thread_.join();
  }

 private:
  void load_config(const std::string& path) {
    const YAML::Node root = YAML::LoadFile(path);
    const std::string version =
        root["version"] ? root["version"].as<std::string>() : "v7";
    cfg_ = Cfg::preset(version);
    if (const auto k = root["knobs"]) {
      auto f = [&](const char* n, float& v) {
        if (k[n]) v = k[n].as<float>();
      };
      auto i = [&](const char* n, int& v) {
        if (k[n]) v = k[n].as<int>();
      };
      i("proc_width", cfg_.proc_width);
      i("min_value", cfg_.min_value);
      f("min_area_frac", cfg_.min_area_frac);
      f("min_visible", cfg_.min_visible);
      f("min_visible_color", cfg_.min_visible_color);
      f("min_rel_sat", cfg_.min_rel_sat);
      f("up_dot_min", cfg_.up_dot_min);
    }
    cfg_.validate();

    const ClipTable table = ClipTable::load(root["clips"].as<std::string>());
    eye_ = std::make_unique<CubeSight>(cfg_, PALETTE_SIM, table.half_extent());

    const auto cam = root["camera"];
    camera_ip_ = cam["host"] ? cam["host"].as<std::string>() : "127.0.0.1";
    camera_port_ = cam["port"] ? cam["port"].as<uint16_t>() : 5555;
    stale_s_ = root["stale_s"] ? root["stale_s"].as<double>() : 1.0;

    const auto mount = root["mount"];
    mount_.pos = yaml_vec3(mount ? mount["xyz"] : YAML::Node(), Mount{}.pos);
    if (const auto q = mount ? mount["quat"] : YAML::Node();
        q && q.size() == 4)
      mount_.quat = {q[0].as<float>(), q[1].as<float>(), q[2].as<float>(),
                     q[3].as<float>()};

    for (int i = 0; i < 3; ++i) {
      static const char* names[3] = {"waist_yaw_joint", "waist_roll_joint",
                                     "waist_pitch_joint"};
      const auto it = std::find(g1::MJ_JOINTS.begin(), g1::MJ_JOINTS.end(),
                                names[i]);
      if (it == g1::MJ_JOINTS.end())
        throw std::runtime_error("sys1 observe: G1 joint table has no waist");
      waist_[i] = static_cast<int>(it - g1::MJ_JOINTS.begin());
    }
  }

  void on_lowstate(const unitree_hg::msg::LowState& m) {
    std::memcpy(gamepad_rx_.buff, m.wireless_remote.data(),
                sizeof(gamepad_rx_.buff));
    gamepad_.update(gamepad_rx_.RF_RX);
    if (gamepad_.L2.on_press) {
      std_msgs::msg::UInt64 click;
      click.data = ++click_sequence_;
      click_pub_->publish(click);
      RCLCPP_INFO(this->get_logger(),
                  "calibration LT click %lu: select next paired RGB-D frame",
                  click.data);
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    imu_quat_ = {m.imu_state.quaternion[0], m.imu_state.quaternion[1],
                 m.imu_state.quaternion[2], m.imu_state.quaternion[3]};
    for (int i = 0; i < 3; ++i) waist_q_[i] = m.motor_state[waist_[i]].q;
    state_seen_ = true;
    last_state_ = this->now();
  }

  static bool recv_exact(int fd, uint8_t* data, size_t size) {
    size_t received = 0;
    while (received < size) {
      const ssize_t n = ::recv(fd, data + received, size - received, 0);
      if (n <= 0) return false;
      received += static_cast<size_t>(n);
    }
    return true;
  }

  void tcp_loop() {
    std::vector<uint8_t> payload;
    while (running_.load()) {
      const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
      socket_fd_.store(fd);
      sockaddr_in address{};
      address.sin_family = AF_INET;
      address.sin_port = htons(camera_port_);
      ::inet_pton(AF_INET, camera_ip_.c_str(), &address.sin_addr);
      if (fd < 0 ||
          ::connect(fd, reinterpret_cast<sockaddr*>(&address),
                    sizeof(address)) < 0) {
        if (fd >= 0) ::close(fd);
        socket_fd_.store(-1);
        if (!running_.load()) break;
        RCLCPP_WARN(this->get_logger(),
                    "sys1 observe: cannot reach %s:%u, retrying",
                    camera_ip_.c_str(), camera_port_);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "sys1 observe: camera connected");
      while (running_.load()) {
        uint32_t size = 0;
        if (!recv_exact(fd, reinterpret_cast<uint8_t*>(&size), sizeof(size)))
          break;
        if (size == 0 || size > 16 * 1024 * 1024) break;
        payload.resize(size);
        if (!recv_exact(fd, payload.data(), payload.size())) break;
        process(payload);
      }
      ::close(fd);
      socket_fd_.store(-1);
    }
  }

  void process(const std::vector<uint8_t>& payload) {
    wire::Frame frame;
    if (!wire::parse(payload, frame)) return;
    const wire::Plane* color = frame.find(wire::Kind::COLOR);
    const wire::Plane* depth = frame.find(wire::Kind::DEPTH);
    if (!color || !depth || !depth->has_intrinsics()) return;

    const cv::Mat color_bytes(1, static_cast<int>(color->nbytes), CV_8UC1,
                              const_cast<uint8_t*>(color->data));
    const cv::Mat bgr = cv::imdecode(color_bytes, cv::IMREAD_COLOR);
    if (bgr.empty()) return;

    cv::Mat depth_mm;
    if (depth->fmt == wire::Fmt::RAW_U16_MM) {
      depth_mm = cv::Mat(depth->h, depth->w, CV_16UC1,
                         const_cast<uint8_t*>(depth->data))
                     .clone();
    } else {
      const cv::Mat encoded(1, static_cast<int>(depth->nbytes), CV_8UC1,
                            const_cast<uint8_t*>(depth->data));
      depth_mm = cv::imdecode(encoded, cv::IMREAD_ANYDEPTH);
    }
    if (depth_mm.empty() || depth_mm.type() != CV_16UC1) return;
    cv::Mat depth_m;
    depth_mm.convertTo(depth_m, CV_32FC1, 1e-3);

    const rclcpp::Time stamp = this->now();
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = "sys1_camera";

    sensor_msgs::msg::CompressedImage color_msg;
    color_msg.header = header;
    color_msg.format = color->fmt == wire::Fmt::PNG ? "png" : "jpeg";
    color_msg.data.assign(color->data, color->data + color->nbytes);
    color_pub_->publish(color_msg);

    sensor_msgs::msg::Image depth_msg;
    depth_msg.header = header;
    depth_msg.height = depth->h;
    depth_msg.width = depth->w;
    depth_msg.encoding = "16UC1";
    depth_msg.is_bigendian = false;
    depth_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(
        depth->w * sizeof(uint16_t));
    depth_msg.data.assign(depth_mm.datastart, depth_mm.dataend);
    depth_pub_->publish(depth_msg);

    sensor_msgs::msg::CameraInfo info;
    info.header = header;
    info.height = depth->h;
    info.width = depth->w;
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    info.k = {depth->fx, 0.0, depth->cx, 0.0, depth->fy, depth->cy,
              0.0,       0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {depth->fx, 0.0, depth->cx, 0.0, 0.0, depth->fy,
              depth->cy, 0.0, 0.0,       0.0,       1.0, 0.0};
    camera_info_pub_->publish(info);

    std::array<float, 4> imu{};
    std::array<float, 3> waist{};
    bool state_ready = false, stand_locked = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const rclcpp::Time now = this->now();
      state_ready = state_seen_ && (now - last_state_).seconds() < stale_s_;
      const bool sys0_fresh =
          sys0_seen_ && (now - last_sys0_).seconds() < stale_s_;
      stand_locked = sys0_fresh && sys0_.calibration_lock &&
                     sys0_.accepting && sys0_.stand;
      imu = imu_quat_;
      waist = waist_q_;
    }

    msg::Sys1Observation observation;
    observation.header = header;
    observation.sequence = ++sequence_;
    observation.state_ready = state_ready;
    observation.stand_locked = stand_locked;
    observation.color = -1;
    observation.reason = state_ready ? "unprocessed" : "no_state";
    cv::Mat labels_for_publish(depth_mm.rows, depth_mm.cols, CV_16SC1,
                               cv::Scalar(-1));

    if (state_ready) {
      const CameraPose camera = camera_pose(imu, waist, mount_);
      const Sight sight = (*eye_)(
          bgr, depth_m, {depth->fx, depth->fy, depth->cx, depth->cy}, camera);
      observation.color_ok = sight.color_ok;
      observation.pose_ok = sight.ok;
      observation.color = sight.color;
      observation.reason = sight.reason;
      observation.position = sight.pos;
      observation.phi_rad = sight.phi;
      observation.n_px = sight.n_px;
      observation.range_m = sight.range_m();
      observation.bearing_rad = sight.bearing();
      observation.camera_rotation = camera.R;
      observation.camera_translation = camera.t;

      for (const Candidate& candidate : eye_->last()) {
        observation.candidate_color.push_back(candidate.color);
        observation.candidate_pixels.push_back(candidate.n_px);
        observation.candidate_up_dot.push_back(candidate.up_dot);
        observation.candidate_visible.push_back(candidate.vis);
        observation.candidate_big.push_back(candidate.big);
        observation.candidate_center_x.push_back(candidate.cx);
        observation.candidate_center_y.push_back(candidate.cy);
      }

      labels_for_publish = eye_->labels().clone();
    }

    // Publish an all-background label plane even before LowState is ready.
    // That keeps the five calibration topics pairable and lets the offboard
    // UI explain the interlock instead of appearing to have no camera.
    sensor_msgs::msg::Image label_msg;
    label_msg.header = header;
    label_msg.height = labels_for_publish.rows;
    label_msg.width = labels_for_publish.cols;
    label_msg.encoding = "mono8";
    label_msg.is_bigendian = false;
    label_msg.step = labels_for_publish.cols;
    label_msg.data.resize(static_cast<size_t>(labels_for_publish.rows) *
                          labels_for_publish.cols);
    for (int i = 0; i < labels_for_publish.rows * labels_for_publish.cols; ++i) {
      const int value = labels_for_publish.ptr<int16_t>()[i];
      label_msg.data[static_cast<size_t>(i)] =
          value < 0 ? 255 : static_cast<uint8_t>(value);
    }
    labels_pub_->publish(label_msg);
    observation_pub_->publish(observation);
  }

  Cfg cfg_;
  std::unique_ptr<CubeSight> eye_;
  Mount mount_;
  std::array<int, 3> waist_{};
  std::string camera_ip_ = "127.0.0.1";
  uint16_t camera_port_ = 5555;
  double stale_s_ = 1.0;

  std::mutex state_mutex_;
  std::array<float, 4> imu_quat_{1.0f, 0.0f, 0.0f, 0.0f};
  std::array<float, 3> waist_q_{};
  bool state_seen_ = false, sys0_seen_ = false;
  rclcpp::Time last_state_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sys0_{0, 0, RCL_ROS_TIME};
  msg::Sys0Status sys0_;

  std::atomic<bool> running_{false};
  std::atomic<int> socket_fd_{-1};
  std::thread tcp_thread_;
  uint64_t sequence_ = 0;
  uint64_t click_sequence_ = 0;
  unitree::common::REMOTE_DATA_RX gamepad_rx_{};
  unitree::common::Gamepad gamepad_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_, labels_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<msg::Sys1Observation>::SharedPtr observation_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr click_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<msg::Sys0Status>::SharedPtr sys0_sub_;
};

}  // namespace sys1
}  // namespace cpp_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<cpp_control::sys1::Sys1ObserveNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("g1_sys1_observe_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
