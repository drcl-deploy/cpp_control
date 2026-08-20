#pragma once

/// STILL or CLIP, one state integer, one ranking scalar. Port of vibe
/// `planner/clips.py::Clips` — the algorithm only; the clock lives in the
/// node, because on hardware the controller owns it (docs/planners/repose/planner.md §2).
///
/// Split in three, so the planner can think every tick and act only when the controller
/// is done: `observe()` folds a belief into the ladder, `decide()` is PURE and
/// may be called at any rate, `commit()` records what was actually published.

#include <set>
#include <string>

#include "cpp_control/planners/repose/belief.hpp"
#include "cpp_control/planners/repose/cfg.hpp"
#include "cpp_control/planners/repose/table.hpp"

namespace cpp_control {
namespace planners {
namespace repose {

/// Wire vocabulary. Structurally there are two: a CLIP, and a STILL whose yaw
/// happens to be zero (SETTLE) or swept (SCAN). Kept apart on the message
/// because a bag reads better for it, derived from the yaw everywhere else.
enum class Mode { INIT, SETTLE, SCAN, CLIP, KINEMATIC_SCAN };
const char* mode_name(Mode m);

struct Plan {
  Mode mode = Mode::INIT;
  int row = -1;    ///< index into the table — the seam reads its span from it
  int sym = 0;     ///< which of the cube's 4 spin symmetries won the match
  int frames = 0;  ///< length of the reference this plan publishes
  float yaw_offset = 0.f;  ///< SCAN: the swept turn. CLIP: solved from the cube
  float entry_yaw = 0.f;   ///< CLIP: heading residual at entry (the warp, §5)
  float cost = 0.f;
  char delta = '-';
  std::string label = "init";
};

class Clips {
 public:
  Clips(const ClipTable& table, const Cfg& cfg, int target_color);

  /// Fold the belief into the ladder: step a rung on an observed top-colour
  /// change, latch `done` on the target. Idempotent — safe every tick.
  void observe(const Belief& b);

  /// PURE. The whole planner, four branches, no side effects — so the node can
  /// call it every tick for a live intent preview and commit the same object
  /// later without the answer having drifted.
  Plan decide(const Belief& b) const;

  /// Record what was actually published: burn the clip and count the roll.
  void commit(const Plan& p);

  /// EVERY field, evidence included: a retained ladder plans the new episode
  /// from the old one's cube.
  void reset();

  Plan still(float yaw) const;

  void set_target_color(int c);
  int target_color() const { return target_; }

  int rung() const { return n_; }
  int tips() const { return tips_; }
  int stall() const { return stall_; }
  int rolls() const { return rolls_; }
  int episode() const { return episode_; }
  int burned() const { return static_cast<int>(tried_.size()); }
  char delta() const;  ///< the rung being asked for — PURE, never advances
  bool done() const { return done_; }

 private:
  void advance();
  Plan retrieve(const Sight& see) const;

  const ClipTable& t_;
  Cfg cfg_;
  int target_ = 4;
  int n_ = 0;      ///< THE state: which rung of the pattern
  int tips_ = 0;   ///< tips actually SEEN (telemetry only)
  int stall_ = 0;  ///< commits at this rung with no tip seen
  int rolls_ = 0;  ///< clips committed this episode
  int episode_ = 0;
  int last_color_ = -1;
  bool done_ = false;
  std::set<int> tried_;
};

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
