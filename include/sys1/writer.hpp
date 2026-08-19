#pragma once

/// Plan -> the reference rows sys0 plays. The whole seam, and it is small:
/// the tokenizer reads joint_pos, joint_vel and root ORIENTATION only, so the
/// sim's cube-exact translation is unobservable on hardware and the warp
/// collapses to one scalar (docs/vibe/sys1/planner.md §1). Rows come straight
/// out of the table in wire layout; this class only ramps, blends and
/// rate-limits them.

#include <vector>

#include "sys1/clips.hpp"

namespace cpp_control {
namespace sys1 {

/// Live robot state the seam blends onto — everything it reads from sys0.
struct LiveState {
  std::vector<float> joint_pos_il;  ///< (29,) IL-ordered, radians
  std::vector<float> joint_vel_il;
};

class ReferenceWriter {
 public:
  ReferenceWriter(const ClipTable& table, const Cfg& cfg);

  /// Build the reference for `plan`. Returns rows [frames, cols] IL-ordered.
  /// Under v7 `entry_yaw` goes on the message and MotionClock applies it at
  /// engage; under v7.1 the ramp carries it and `ramped()` says so.
  const std::vector<float>& build(const Plan& plan, const LiveState& live);

  int frames() const { return frames_; }
  int cols() const { return cols_; }
  int lead_in_frames() const { return lead_; }
  /// v7.1 carried the heading residual in the ROWS, so the message must not
  /// also carry it in `entry_yaw_offset` — the two would compose.
  bool ramped() const { return ramp_; }

 private:
  void push_stand(int count);
  void push_lead_in(const Plan& plan, const float* target,
                    const LiveState& live);
  void blend_head(const LiveState& live, int count);

  const ClipTable& t_;
  Cfg cfg_;
  std::vector<float> rows_;
  int frames_ = 0, cols_ = 0, lead_ = 0;
  bool ramp_ = false;
};

}  // namespace sys1
}  // namespace cpp_control
