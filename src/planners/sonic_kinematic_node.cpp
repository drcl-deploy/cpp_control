#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unitree_hg/msg/low_state.hpp>
#include <utility>

#include "common/gamepad.hpp"
#include "cpp_control/msg/controller_status.hpp"
#include "cpp_control/msg/motion_reference.hpp"
#include "cpp_control/planners/sonic_kinematic.hpp"

namespace cpp_control {
namespace planners {

namespace {

constexpr float kPi = 3.14159265358979323846f;

float deadband(float value, float zone) {
  if (std::fabs(value) <= zone) return 0.0f;
  return std::copysign((std::fabs(value) - zone) / (1.0f - zone), value);
}

float wrap_angle(float value) {
  while (value > kPi) value -= 2.0f * kPi;
  while (value < -kPi) value += 2.0f * kPi;
  return value;
}

float axis(const sensor_msgs::msg::Joy& joy, int index) {
  return index >= 0 && index < static_cast<int>(joy.axes.size())
             ? joy.axes[index]
             : 0.0f;
}

bool button(const sensor_msgs::msg::Joy& joy, int index) {
  return index >= 0 && index < static_cast<int>(joy.buttons.size()) &&
         joy.buttons[index] != 0;
}

}  // namespace

class SonicKinematicNode : public rclcpp::Node {
 public:
  SonicKinematicNode() : Node("g1_sonic_kinematic_planner_node") {
    const std::string model_path = this->declare_parameter("model_path", "");
    if (model_path.empty())
      throw std::runtime_error(
          "sonic kinematic planner: model_path is required");

    input_source_ = this->declare_parameter("input_source", "unitree");
    if (input_source_ != "unitree" && input_source_ != "joy")
      throw std::runtime_error(
          "sonic kinematic planner: input_source must be unitree or joy");
    rate_hz_ = this->declare_parameter("rate_hz", 10.0);
    stale_s_ = this->declare_parameter("stale_s", 0.5);
    dead_zone_ = this->declare_parameter("dead_zone", 0.12);
    yaw_rate_ = this->declare_parameter("yaw_rate", 0.9);
    reverse_speed_max_ = this->declare_parameter("reverse_speed_max", 0.4);
    blend_frames_ = this->declare_parameter("blend_frames", 8);
    terminal_ = this->declare_parameter("terminal", true);
    joy_left_y_axis_ = this->declare_parameter("joy_left_y_axis", 1);
    joy_right_x_axis_ = this->declare_parameter("joy_right_x_axis", 3);
    joy_dpad_x_axis_ = this->declare_parameter("joy_dpad_x_axis", 6);
    joy_lt_axis_ = this->declare_parameter("joy_lt_axis", 2);
    joy_lt_button_ = this->declare_parameter("joy_lt_button", -1);
    joy_lt_pressed_below_ =
        this->declare_parameter("joy_lt_pressed_below", -0.5);

    if (rate_hz_ <= 0.0 || stale_s_ <= 0.0 || dead_zone_ < 0.0 ||
        dead_zone_ >= 1.0 || blend_frames_ < 1)
      throw std::runtime_error(
          "sonic kinematic planner: invalid timing/input knob");

    planner_ = std::make_unique<SonicKinematicPlanner>(
        model_path, this->declare_parameter("default_height", 0.788740),
        this->declare_parameter("lookahead_frames", 2),
        this->declare_parameter<int64_t>("random_seed", 1234));

    selected_mode_ =
        std::clamp(static_cast<int>(this->declare_parameter("initial_mode", 2)),
                   0, SonicKinematicPlanner::kNumModes - 1);
    active_mode_ = 0;

    reference_pub_ = this->create_publisher<msg::MotionReference>(
        this->declare_parameter("reference_topic", "/tracker/reference"), 10);
    controller_sub_ = this->create_subscription<msg::ControllerStatus>(
        this->declare_parameter("controller_status_topic",
                                "/vibe/controller/status"),
        rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](msg::ControllerStatus::SharedPtr status) {
          controller_ = *status;
          controller_seen_ = true;
          last_controller_ = this->now();
        });
    lowstate_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
        this->declare_parameter("lowstate_topic", "/lowstate"), 10,
        [this](unitree_hg::msg::LowState::SharedPtr state) {
          on_lowstate(*state);
        });
    if (input_source_ == "joy") {
      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
          this->declare_parameter("joy_topic", "/joy"), 10,
          [this](sensor_msgs::msg::Joy::SharedPtr joy) { on_joy(*joy); });
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz_));
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
    last_tick_ = this->now();

    RCLCPP_INFO(
        this->get_logger(),
        "SONIC kinematic planner ready: %s | input=%s | D-pad Left/Right "
        "browse, LT commit | left Y drive, right X face | no X/Y/A/B/RB",
        model_path.c_str(), input_source_.c_str());
    log_selection();
  }

  ~SonicKinematicNode() override {
    if (terminal_) std::cout << '\n';
  }

 private:
  void on_lowstate(const unitree_hg::msg::LowState& state) {
    for (int joint = 0; joint < g1::NUM_JOINTS; ++joint)
      joints_mj_[joint] = state.motor_state[joint].q;
    state_seen_ = true;
    last_state_ = this->now();

    if (input_source_ != "unitree") return;
    std::memcpy(gamepad_rx_.buff, state.wireless_remote.data(),
                sizeof(gamepad_rx_.buff));
    gamepad_.update(gamepad_rx_.RF_RX);
    forward_axis_ = gamepad_.ly;
    yaw_axis_ = gamepad_.rx;
    if (gamepad_.left.on_press) browse(-1);
    if (gamepad_.right.on_press) browse(1);
    if (gamepad_.L2.on_press) commit_mode();
    input_seen_ = true;
    last_input_ = last_state_;
  }

  void on_joy(const sensor_msgs::msg::Joy& joy) {
    forward_axis_ = axis(joy, joy_left_y_axis_);
    yaw_axis_ = axis(joy, joy_right_x_axis_);
    const float dpad = axis(joy, joy_dpad_x_axis_);
    const bool left = dpad > 0.5f;
    const bool right = dpad < -0.5f;
    if (left && !joy_left_) browse(-1);
    if (right && !joy_right_) browse(1);
    joy_left_ = left;
    joy_right_ = right;

    const bool lt = joy_lt_button_ >= 0
                        ? button(joy, joy_lt_button_)
                        : axis(joy, joy_lt_axis_) < joy_lt_pressed_below_;
    if (lt && !joy_lt_) commit_mode();
    joy_lt_ = lt;
    input_seen_ = true;
    last_input_ = this->now();
  }

  void browse(int delta) {
    selected_mode_ =
        (selected_mode_ + delta + SonicKinematicPlanner::kNumModes) %
        SonicKinematicPlanner::kNumModes;
    log_selection();
  }

  void commit_mode() {
    active_mode_ = selected_mode_;
    command_dirty_ = true;
    RCLCPP_INFO(this->get_logger(), "LT COMMIT -> [%02d] %s", active_mode_,
                SonicKinematicPlanner::mode_name(active_mode_));
  }

  void log_selection() {
    RCLCPP_INFO(this->get_logger(),
                "menu [%02d] %-20s | active [%02d] %-20s | LT commits",
                selected_mode_,
                SonicKinematicPlanner::mode_name(selected_mode_), active_mode_,
                SonicKinematicPlanner::mode_name(active_mode_));
  }

  static std::pair<float, float> speed_range(int mode) {
    if (mode == 1) return {0.10f, 0.55f};
    if (mode == 2) return {0.20f, 1.20f};
    if (mode == 3) return {1.20f, 3.00f};
    if (mode == 8 || mode == 14) return {0.10f, 0.55f};
    if (mode == 10) return {0.15f, 0.80f};
    return {0.15f, 0.90f};
  }

  static float mode_height(int mode) {
    switch (mode) {
      case 4:
        return 0.65f;
      case 5:
      case 6:
        return 0.38f;
      case 7:
        return 0.22f;
      case 8:
        return 0.42f;
      case 14:
        return 0.32f;
      default:
        return -1.0f;
    }
  }

  SonicKinematicCommand command() const {
    SonicKinematicCommand command;
    command.mode = active_mode_;
    command.height = mode_height(active_mode_);
    const float forward =
        deadband(forward_axis_, static_cast<float>(dead_zone_));
    const float c = std::cos(facing_yaw_), s = std::sin(facing_yaw_);
    command.facing_direction = {c, s, 0.0f};

    if (SonicKinematicPlanner::is_locomotion_mode(active_mode_)) {
      if (forward == 0.0f) {
        // Plain walk/run modes have a clean learned idle. Character and crawl
        // modes retain their committed identity and simply receive zero speed.
        if (active_mode_ >= 1 && active_mode_ <= 3) command.mode = 0;
        command.target_speed = command.mode == 0 ? -1.0f : 0.0f;
      } else {
        const auto range = speed_range(active_mode_);
        float speed =
            range.first + std::fabs(forward) * (range.second - range.first);
        if (forward < 0.0f)
          speed = std::min(speed, static_cast<float>(reverse_speed_max_));
        command.target_speed = speed;
        const float sign = forward > 0.0f ? 1.0f : -1.0f;
        command.movement_direction = {sign * c, sign * s, 0.0f};
      }
    } else if (active_mode_ == 17) {
      // Forward Jump is a committed one-shot, not a hidden stick mode.
      command.movement_direction = command.facing_direction;
    } else if (active_mode_ == 11 || active_mode_ == 12 || active_mode_ == 15 ||
               active_mode_ == 16) {
      // Punches are conditioned on the current facing direction in the
      // reference deployment even while translational speed is zero.
      command.target_speed = 0.0f;
      command.movement_direction = command.facing_direction;
    }
    return command;
  }

  static bool changed(const SonicKinematicCommand& a,
                      const SonicKinematicCommand& b) {
    if (a.mode != b.mode ||
        std::fabs(a.target_speed - b.target_speed) > 0.04f ||
        std::fabs(a.height - b.height) > 0.01f)
      return true;
    for (int i = 0; i < 3; ++i)
      if (std::fabs(a.movement_direction[i] - b.movement_direction[i]) >
              0.05f ||
          std::fabs(a.facing_direction[i] - b.facing_direction[i]) > 0.025f)
        return true;
    return false;
  }

  bool stale(const rclcpp::Time& stamp, const rclcpp::Time& now) const {
    return stamp.nanoseconds() == 0 || (now - stamp).seconds() > stale_s_;
  }

  void reset_episode() {
    current_motion_.reset();
    reference_id_.clear();
    planner_initialized_ = false;
    stale_idle_sent_ = false;
    command_dirty_ = true;
    last_command_valid_ = false;
  }

  void tick() {
    const rclcpp::Time now = this->now();
    const double dt = std::clamp((now - last_tick_).seconds(), 0.0, 0.25);
    last_tick_ = now;
    facing_yaw_ = wrap_angle(
        facing_yaw_ - deadband(yaw_axis_, static_cast<float>(dead_zone_)) *
                          static_cast<float>(yaw_rate_ * dt));

    const bool fresh = state_seen_ && input_seen_ && !stale(last_state_, now) &&
                       !stale(last_input_, now);
    const bool controller_fresh =
        controller_seen_ && !stale(last_controller_, now);
    if (!controller_fresh || !controller_.accepting) {
      if (was_accepting_) reset_episode();
      was_accepting_ = false;
      dashboard(
          !controller_fresh ? "WAIT STATUS" : (fresh ? "WAIT A" : "WAIT INPUT"),
          command());
      return;
    }
    if (!was_accepting_) {
      reset_episode();
      was_accepting_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "controller armed: starting a fresh kinematic episode");
    }
    if (!fresh) {
      if (current_motion_ && !stale_idle_sent_) {
        SonicKinematicCommand idle;
        idle.facing_direction = {std::cos(facing_yaw_), std::sin(facing_yaw_),
                                 0.0f};
        generate_and_publish(idle, now);
        stale_idle_sent_ = true;
      }
      dashboard("STALE -> IDLE", command());
      return;
    }
    stale_idle_sent_ = false;

    if (!planner_initialized_) {
      planner_->initialize(joints_mj_);
      planner_initialized_ = true;
    }

    const SonicKinematicCommand next = command();
    const bool accepted =
        !reference_id_.empty() && controller_.reference_id == reference_id_;
    if (!reference_id_.empty() && !accepted &&
        (now - last_publish_).seconds() < 1.0) {
      dashboard("HANDOFF", next);
      return;
    }

    const bool moving = SonicKinematicPlanner::is_locomotion_mode(next.mode) &&
                        next.target_speed > 0.0f;
    const double refresh_s = next.mode == 3 ? 0.25 : 0.45;
    const bool due = moving && (now - last_publish_).seconds() >= refresh_s;
    if (command_dirty_ || !last_command_valid_ ||
        changed(next, last_command_) || due)
      generate_and_publish(next, now);
    dashboard("RUN", next);
  }

  void generate_and_publish(const SonicKinematicCommand& command,
                            const rclcpp::Time& now) {
    const bool controller_has_current =
        current_motion_ && controller_.reference_id == reference_id_;
    const g1::Motion* previous =
        controller_has_current ? current_motion_.get() : nullptr;
    const int current_frame =
        previous ? static_cast<int>(controller_.frame) : 0;
    try {
      const auto result = planner_->plan(command, previous, current_frame);
      g1::Motion motion = result.motion;
      if (previous) {
        const int inference_frames = static_cast<int>(
            std::lround(result.inference_ms * previous->fps / 1000.0));
        const int handoff = current_frame + std::max(1, inference_frames);
        motion = SonicKinematicPlanner::splice(
            *previous, handoff, result.motion, result.generation_frame,
            blend_frames_);
      }

      msg::MotionReference reference;
      reference.schema_version = msg::MotionReference::SCHEMA_VERSION;
      reference.reference_id = "sonic-kinematic-" + std::to_string(++sequence_);
      reference.frames = static_cast<uint32_t>(motion.num_frames);
      reference.cols = static_cast<uint32_t>(g1::WIRE_COLS_FULL);
      reference.fps = motion.fps;
      reference.has_twist = true;
      reference.has_contact = true;
      reference.has_object_goal = false;
      reference.entry_yaw_offset = 0.0f;
      reference.mode = static_cast<uint8_t>(command.mode);
      reference.data = SonicKinematicPlanner::to_wire_rows(motion);
      reference_pub_->publish(reference);

      current_motion_ = std::make_unique<g1::Motion>(std::move(motion));
      reference_id_ = reference.reference_id;
      last_publish_ = now;
      last_command_ = command;
      last_command_valid_ = true;
      command_dirty_ = false;
      inference_ms_ = result.inference_ms;
    } catch (const std::exception& error) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "SONIC planner inference failed: %s", error.what());
    }
  }

  void dashboard(const char* state,
                 const SonicKinematicCommand& effective) const {
    if (!terminal_) return;
    std::ostringstream line;
    line << '\r' << "\033[2KSONIC | " << std::left << std::setw(13) << state
         << " | active " << std::right << std::setw(2) << std::setfill('0')
         << active_mode_ << ' ' << std::setfill(' ') << std::left
         << std::setw(20) << SonicKinematicPlanner::mode_name(active_mode_)
         << " | menu " << std::right << std::setw(2) << std::setfill('0')
         << selected_mode_ << ' ' << std::setfill(' ') << std::left
         << std::setw(20) << SonicKinematicPlanner::mode_name(selected_mode_)
         << (selected_mode_ == active_mode_ ? " " : " *") << " | eff "
         << std::right << std::setw(2) << std::setfill('0') << effective.mode
         << ' ' << std::setfill(' ') << std::left << std::setw(14)
         << SonicKinematicPlanner::mode_name(effective.mode) << " | cmd "
         << std::fixed << std::setprecision(2) << forward_axis_ << " yaw "
         << std::setw(6) << facing_yaw_ << " | model " << std::setw(6)
         << std::setprecision(1) << inference_ms_ << " ms | ref "
         << (reference_id_.empty() ? "-" : reference_id_);
    std::cout << line.str() << std::flush;
  }

  std::unique_ptr<SonicKinematicPlanner> planner_;
  std::unique_ptr<g1::Motion> current_motion_;
  rclcpp::Publisher<msg::MotionReference>::SharedPtr reference_pub_;
  rclcpp::Subscription<msg::ControllerStatus>::SharedPtr controller_sub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  msg::ControllerStatus controller_;
  std::array<float, g1::NUM_JOINTS> joints_mj_{};
  unitree::common::REMOTE_DATA_RX gamepad_rx_{};
  unitree::common::Gamepad gamepad_;
  std::string input_source_;
  std::string reference_id_;
  rclcpp::Time last_state_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_input_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_controller_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};

  SonicKinematicCommand last_command_;
  int selected_mode_ = 2;
  int active_mode_ = 0;
  uint64_t sequence_ = 0;
  float forward_axis_ = 0.0f;
  float yaw_axis_ = 0.0f;
  float facing_yaw_ = 0.0f;
  double inference_ms_ = 0.0;
  double rate_hz_ = 10.0;
  double stale_s_ = 0.5;
  double dead_zone_ = 0.12;
  double yaw_rate_ = 0.9;
  double reverse_speed_max_ = 0.4;
  int blend_frames_ = 8;
  int joy_left_y_axis_ = 1;
  int joy_right_x_axis_ = 3;
  int joy_dpad_x_axis_ = 6;
  int joy_lt_axis_ = 2;
  int joy_lt_button_ = -1;
  double joy_lt_pressed_below_ = -0.5;
  bool terminal_ = true;
  bool state_seen_ = false;
  bool input_seen_ = false;
  bool controller_seen_ = false;
  bool was_accepting_ = false;
  bool planner_initialized_ = false;
  bool stale_idle_sent_ = false;
  bool command_dirty_ = true;
  bool last_command_valid_ = false;
  bool joy_left_ = false;
  bool joy_right_ = false;
  bool joy_lt_ = false;
};

}  // namespace planners
}  // namespace cpp_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  int status = EXIT_SUCCESS;
  try {
    rclcpp::spin(std::make_shared<cpp_control::planners::SonicKinematicNode>());
  } catch (const std::exception& error) {
    std::cerr << "g1_sonic_kinematic_planner_node: " << error.what() << '\n';
    status = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return status;
}
