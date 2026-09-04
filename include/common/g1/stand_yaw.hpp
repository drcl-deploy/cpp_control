#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cpp_control {
namespace g1 {

/// ROS-free state machine for a rate-controlled stand heading reference.
///
/// The angle is deliberately unwrapped: holding the stick may turn through any
/// number of revolutions. Only the quaternion writer wraps it implicitly via
/// sin/cos. A stale input or a disabled drive slews the rate safely to zero.
struct StandYawConfig {
  bool enabled = false;
  double max_rate = 0.7853981633974483;  // 45 deg/s
  double max_accel = 3.141592653589793;  // 180 deg/s^2
  double deadband = 0.15;
  double input_timeout = 0.25;
};

class StandYaw {
 public:
  StandYaw() = default;
  explicit StandYaw(const StandYawConfig& config) { configure(config); }

  void configure(const StandYawConfig& config) {
    if (!std::isfinite(config.max_rate) || config.max_rate <= 0.0 ||
        !std::isfinite(config.max_accel) || config.max_accel <= 0.0 ||
        !std::isfinite(config.deadband) || config.deadband < 0.0 ||
        config.deadband >= 1.0 || !std::isfinite(config.input_timeout) ||
        config.input_timeout <= 0.0)
      throw std::invalid_argument("invalid stand-yaw configuration");
    config_ = config;
    reset();
  }

  void reset() {
    angle_ = 0.0;
    rate_ = 0.0;
    target_rate_ = 0.0;
    input_ = 0.0;
    last_input_ = 0.0;
    have_input_ = false;
  }

  void set_input(double raw_axis, double now) {
    if (!std::isfinite(raw_axis)) raw_axis = 0.0;
    raw_axis = std::clamp(raw_axis, -1.0, 1.0);
    const double magnitude = std::abs(raw_axis);
    input_ = magnitude <= config_.deadband
                 ? 0.0
                 : std::copysign((magnitude - config_.deadband) /
                                     (1.0 - config_.deadband),
                                 raw_axis);
    last_input_ = now;
    have_input_ = std::isfinite(now);
  }

  void step(double now, double dt, bool drive) {
    if (!std::isfinite(dt) || dt <= 0.0) return;
    target_rate_ = drive && fresh(now) ? input_ * config_.max_rate : 0.0;
    const double previous = rate_;
    const double max_delta = config_.max_accel * dt;
    rate_ += std::clamp(target_rate_ - rate_, -max_delta, max_delta);
    angle_ += 0.5 * (previous + rate_) * dt;
  }

  /// Predict the unwrapped heading through the same acceleration limiter used
  /// by step(). This gives SONIC a smooth future window, including braking.
  double future_angle(double seconds) const {
    if (!std::isfinite(seconds) || seconds <= 0.0) return angle_;
    const double delta = target_rate_ - rate_;
    const double hit = std::abs(delta) / config_.max_accel;
    const double accel = std::copysign(config_.max_accel, delta);
    if (hit >= seconds)
      return angle_ + rate_ * seconds + 0.5 * accel * seconds * seconds;
    return angle_ + rate_ * hit + 0.5 * accel * hit * hit +
           target_rate_ * (seconds - hit);
  }

  bool fresh(double now) const {
    return config_.enabled && have_input_ && std::isfinite(now) &&
           now >= last_input_ && now - last_input_ <= config_.input_timeout;
  }

  bool centered(double now) const { return !fresh(now) || input_ == 0.0; }
  bool enabled() const { return config_.enabled; }
  double angle() const { return angle_; }
  double rate() const { return rate_; }
  double target_rate() const { return target_rate_; }
  double input() const { return input_; }

 private:
  StandYawConfig config_;
  double angle_ = 0.0;
  double rate_ = 0.0;
  double target_rate_ = 0.0;
  double input_ = 0.0;
  double last_input_ = 0.0;
  bool have_input_ = false;
};

}  // namespace g1
}  // namespace cpp_control
