#pragma once

/// Plan -> the reference rows the controller plays. The whole seam, and it is
/// small: the tokenizer reads joint_pos, joint_vel and root ORIENTATION only,
/// so the sim's cube-exact translation is unobservable on hardware and the warp
/// collapses to one scalar (docs/planners/repose/planner.md §1). Rows come
/// straight out of the table in wire layout; this class only ramps, blends and
/// rate-limits them.

#include <vector>

#include "cpp_control/planners/repose/approach.hpp"
#include "cpp_control/planners/repose/clips.hpp"

namespace cpp_control {
namespace planners {
namespace repose {

/// Live robot state the seam blends onto — everything it reads from the
/// controller.
struct LiveState {
  std::vector<float> joint_pos_il;  ///< (29,) IL-ordered, radians
  std::vector<float> joint_vel_il;
  /// v7.1 only, and empty otherwise: the last row of the reference the
  /// controller is CURRENTLY playing. A ramp that starts from what the
  /// controller was actually told is C0 by construction, whatever wrote it —
  /// where the live pose re-opens the tracking error as a step at row 0.
  std::vector<float> held_row;
};

class ReferenceWriter {
 public:
  /// Which half of a v7.1 clip act to emit. FULL is v7 and every still: one
  /// reference, lead-in and clip together. v7.1 splits a clip in two so the
  /// clip RE-ENGAGES on the pose the ramp actually reached instead of dead
  /// reckoning from the pose it started at (docs/planners/repose/planner.md
  /// §7.1).
  enum class Stage { FULL, ENTER, CLIP };

  ReferenceWriter(const ClipTable& table, const Cfg& cfg,
                  const ApproachSource* approach = nullptr);

  /// Build the reference for `plan`. Returns rows [frames, cols] IL-ordered.
  /// Under v7 `entry_yaw` goes on the message and MotionClock applies it at
  /// engage; under v7.1 the ramp carries it and `ramped()` says so.
  const std::vector<float>& build(const Plan& plan, const LiveState& live,
                                  Stage stage = Stage::FULL);

  /// The last row of the reference just built — what the controller will be
  /// holding when it reports finished, and therefore where the next ramp
  /// starts.
  const float* final_row() const {
    return rows_.empty() ? nullptr : &rows_[rows_.size() - cols_];
  }

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
  const ApproachSource* approach_ = nullptr;
  Cfg cfg_;
  std::vector<float> rows_;
  int frames_ = 0, cols_ = 0, lead_ = 0;
  bool ramp_ = false;
};

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
