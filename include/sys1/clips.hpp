#pragma once

/// SETTLE -> read -> SCAN or CLIP. Three modes, one state integer, one ranking
/// scalar. Port of vibe `planner/clips.py::Sys1Clips` — the algorithm only; the
/// clock lives in the node, because on hardware sys0 owns it (docs/sys1.md §2).

#include <set>
#include <string>

#include "sys1/cfg.hpp"
#include "sys1/sight.hpp"
#include "sys1/table.hpp"

namespace cpp_control {
namespace sys1 {

enum class Mode { INIT, SETTLE, SCAN, CLIP };
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

class Sys1Clips {
 public:
  Sys1Clips(const ClipTable& table, const Cfg& cfg, int target_color);

  /// EVERY field, evidence included: a retained Sight plans the new episode
  /// from the old one's cube.
  void reset();

  /// One look. Cheap enough to call on every planner tick; only the read that
  /// `decide()` consumes can move the ladder (sys1_cpp.md §8.5 — a flickering
  /// read advanced three rungs from one still mode).
  const Sight& look(const cv::Mat& bgr, const cv::Mat& depth_m,
                    const Intrinsics& intr, const CameraPose& cam);

  /// Consume the latest look: count the tip, then choose the next mode.
  Plan decide();

  Plan still(Mode mode, float yaw) const;

  void set_target_color(int c);
  int target_color() const { return target_; }
  const Sight& sight() const { return see_; }
  const Plan& plan() const { return plan_; }
  const CubeSight& eye() const { return eye_; }
  int rung() const { return n_; }
  int tips() const { return tips_; }
  int stall() const { return stall_; }
  int burned() const { return static_cast<int>(tried_.size()); }
  char delta() const;  ///< the rung being asked for — PURE, never advances
  bool done() const { return done_; }
  std::string status() const;

 private:
  void advance();
  Plan retrieve();

  const ClipTable& t_;
  Cfg cfg_;
  CubeSight eye_;
  Sight see_;
  Plan plan_;
  int target_ = 4;
  int n_ = 0;      ///< THE state: which rung of the pattern
  int tips_ = 0;   ///< tips actually SEEN (telemetry only)
  int stall_ = 0;  ///< commits at this rung with no tip seen
  int last_color_ = -1;
  bool done_ = false;
  std::set<int> tried_;
};

}  // namespace sys1
}  // namespace cpp_control
