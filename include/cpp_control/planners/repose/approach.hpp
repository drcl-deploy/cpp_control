#pragma once

/// A robot-anchored walk vocabulary for Repose v8.5.
///
/// The source is the same recorded walk used by the Python OGMP oracle. It is
/// cut into distance-indexed windows at load time; a request selects a real
/// window without scaling its positions or velocities. MotionClock supplies
/// the live robot anchor when the reference is engaged.

#include <string>
#include <vector>

#include "common/g1/motion.hpp"
#include "cpp_control/planners/repose/cfg.hpp"
#include "cpp_control/planners/repose/clips.hpp"

namespace cpp_control {
namespace planners {
namespace repose {

struct ApproachWindow {
  int length = 0;      ///< source-frame intervals after onset
  float distance = 0;  ///< source root displacement, metres
  /// Net displacement direction relative to the source root heading at onset.
  float travel_from_root = 0;
};

class ApproachSource {
 public:
  ApproachSource(const std::string& motion_path, const Cfg& cfg);

  /// Turn a winning clip candidate into an APPROACH plan. `entry_translation`
  /// and `entry_bearing` are measured in the live robot base frame.
  Plan plan(const Plan& candidate) const;

  const float* span(int window) const;
  int frames(int window) const;
  int cols() const { return g1::WIRE_COLS_FULL; }
  float fps() const { return motion_.fps; }
  int onset() const { return onset_; }
  const std::vector<ApproachWindow>& windows() const { return windows_; }

 private:
  g1::Motion motion_;
  Cfg cfg_;
  int onset_ = -1;
  std::vector<ApproachWindow> windows_;
  std::vector<float> wire_;
};

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
