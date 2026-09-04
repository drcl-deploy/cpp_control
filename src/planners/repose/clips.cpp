#include "cpp_control/planners/repose/clips.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace cpp_control {
namespace planners {
namespace repose {

namespace {

constexpr float kPi = static_cast<float>(M_PI);

/// Angle into [-pi, pi).
float wrap(float a) { return std::atan2(std::sin(a), std::cos(a)); }

/// v5's escape when a delta keeps failing — swap the ROLL AXIS, not the sign.
char other_axis(char d) {
  switch (d) {
    case 'R':
    case 'L':
      return 'B';
    case 'B':
    case 'F':
      return 'R';
    default:
      return d;
  }
}

/// || Rot(90m) q_clip - q_live ||, translation + arm_radius * heading.
float se2(const ClipRow& r, const float q[3], int m, float arm_radius) {
  const float a = m * (kPi / 2.0f);
  const float c = std::cos(a), s = std::sin(a);
  const float dx = (c * r.qx - s * r.qy) - q[0];
  const float dy = (s * r.qx + c * r.qy) - q[1];
  const float dth = wrap(r.qth + a - q[2]);
  return std::hypot(dx, dy) + arm_radius * std::fabs(dth);
}

}  // namespace

const char* mode_name(Mode m) {
  switch (m) {
    case Mode::SETTLE:
      return "settle";
    case Mode::SCAN:
      return "scan";
    case Mode::CLIP:
      return "clip";
    case Mode::APPROACH:
      return "approach";
    default:
      return "init";
  }
}

Clips::Clips(const ClipTable& table, const Cfg& cfg, int target_color)
    : t_(table), cfg_(cfg), target_(target_color) {
  cfg_.validate();
  reset();
}

void Clips::reset() {
  ++episode_;
  n_ = 0;
  tips_ = 0;
  stall_ = 0;
  rolls_ = 0;
  tried_.clear();
  last_color_ = -1;
  done_ = false;
}

void Clips::set_target_color(int c) {
  if (c == target_) return;
  target_ = c;
  reset();
}

char Clips::delta() const {
  const char d = cfg_.pattern[n_ % cfg_.pattern.size()];
  return (cfg_.retry_limit > 0 && stall_ >= cfg_.retry_limit) ? other_axis(d)
                                                              : d;
}

void Clips::advance() {
  ++n_;
  stall_ = 0;
  tried_.clear();
}

// ── Belief -> ladder ─────────────────────────────────────────────

void Clips::observe(const Belief& b) {
  if (!b.valid()) return;
  // A TIP IS AN OBSERVED TOP-COLOUR CHANGE. It is read off the VOTED colour, so
  // it fires at most once per real tip no matter how often this runs — which is
  // what lets the ladder live out here rather than inside the one decision the
  // node happened to consume (sys1_cpp.md §8.5).
  if (last_color_ >= 0 && last_color_ != b.color()) {
    ++tips_;
    advance();
  }
  last_color_ = b.color();
  // DONE ON THE COLOUR, and LATCHED: a cut face names the top colour exactly,
  // and a solved cube must not be rolled away by the next read that flickers.
  // The operator clears it by re-picking a target on /vibe/sonic/goal_color.
  if (b.color() == target_) done_ = true;
}

// ── The decision ─────────────────────────────────────────────────

Plan Clips::decide(const Belief& b) const {
  // BLIND is not the same as "saw nothing". No accepted reads means the camera
  // is down, or the robot never went quiet enough to read — both are answered
  // by standing in the nominal stance, which is also the state that produces
  // the reads needed to get out of it. Turning would be a guess.
  if (done_ || b.n_reads() < cfg_.belief_min_votes) return still(0.0f);

  if (!b.valid() || !b.pose_ok()) {
    // No POSE: turn. Covers "no cube", "only side faces", "mid-tumble" and
    // "half in frame" with ONE branch — all four are answered by looking from
    // elsewhere. A quarter sweep when the top face IS in frame and merely cut:
    // that is a re-aim, not a search.
    const float sweep =
        cfg_.scan_sweep_deg * kPi / 180.0f * (b.valid() ? 0.25f : 1.0f);
    float yaw = std::clamp(b.newest().hint, -sweep, sweep);
    if (std::fabs(yaw) < 0.25f * sweep)
      yaw = 0.25f * sweep * (yaw < 0.0f ? -1.0f : 1.0f);
    return still(yaw);
  }
  return retrieve(b.pose());
}

Plan Clips::still(float yaw) const {
  Plan p;
  p.mode = yaw == 0.0f ? Mode::SETTLE : Mode::SCAN;
  p.yaw_offset = yaw;
  p.frames = yaw == 0.0f ? cfg_.settle_steps : cfg_.scan_steps;
  p.label = mode_name(p.mode);
  return p;
}

void Clips::commit(const Plan& p) {
  if (p.mode != Mode::CLIP) return;
  ++rolls_;
  ++stall_;
  tried_.insert(p.row);
  // Burned pool -> allow reuse. Done HERE, not in the ranking, so `decide()`
  // stays a pure function of the belief and the ladder.
  if (tried_.size() >= t_.pool(p.delta).size()) tried_.clear();
}

// ── THE ONE RANKING SCALAR ───────────────────────────────────────
//
// The warp is cube-exact, so what the controller must self-correct is exactly
// the stance residual: the robot's SE(2) pose in the CUBE's frame, ours minus
// the recording's. The cube's 4-fold symmetry gives four free warps, so take
// the best. Range, bearing and spin all fold into this one distance.
//
// Deliberately UNCAPPED. A clip committed with half a metre of residual is not
// a planner bug, it is the experiment: whatever closes that gap is the
// controller's implicit localisation, and `cost` is published so the bag can
// price it.

Plan Clips::retrieve(const Sight& see) const {
  const char d = delta();
  const std::vector<int>& k = t_.pool(d);
  if (k.empty()) return still(0.0f);

  std::vector<int> avail;
  avail.reserve(k.size());
  for (int i : k)
    if (!tried_.count(i)) avail.push_back(i);
  if (avail.empty()) avail = k;

  // Robot SE(2) in the cube frame. The robot is the base frame's origin looking
  // down +x, so this is pure perception.
  const float c = std::cos(-see.phi), s = std::sin(-see.phi);
  const float x = -see.pos[0], y = -see.pos[1];
  const float q[3] = {c * x - s * y, s * x + c * y, -see.phi};

  float best = std::numeric_limits<float>::max();
  int best_row = avail.front(), best_sym = 0;
  for (int i : avail) {
    const ClipRow& r = t_.rows()[i];
    // THE HORIZON, in the same metres as the rest: where the clip LEAVES us.
    // Independent of the warp symmetry, so it applies to all four.
    const float horizon =
        cfg_.horizon_gain * std::fabs(r.exit_range - cfg_.stance_band_m);
    for (int m = 0; m < 4; ++m) {
      const float cost = se2(r, q, m, cfg_.arm_radius) + horizon;
      if (cost < best) {
        best = cost;
        best_row = i;
        best_sym = m;
      }
    }
  }

  return candidate(see, best_row, best_sym, d);
}

Plan Clips::retarget(const Plan& anchor, const Sight& see) const {
  if (anchor.row < 0 || anchor.row >= static_cast<int>(t_.rows().size()) ||
      anchor.sym < 0 || anchor.sym >= 4)
    throw std::runtime_error(
        "repose planner: cannot retarget an invalid approach-turn anchor");
  return candidate(see, anchor.row, anchor.sym, anchor.delta);
}

Plan Clips::retarget_heading_locked(const Plan& anchor, const Sight& see,
                                    float robot_yaw,
                                    float target_world_yaw) const {
  if (anchor.row < 0 || anchor.row >= static_cast<int>(t_.rows().size()))
    throw std::runtime_error(
        "repose planner: cannot retarget an invalid heading lock");
  const ClipRow& r = t_.rows()[anchor.row];
  int best_sym = 0;
  float best_error = std::numeric_limits<float>::max();
  for (int sym = 0; sym < 4; ++sym) {
    const float world = robot_yaw + r.qth + sym * (kPi / 2.0f) + see.phi;
    const float error = std::fabs(wrap(world - target_world_yaw));
    if (error < best_error) {
      best_error = error;
      best_sym = sym;
    }
  }
  return candidate(see, anchor.row, best_sym, anchor.delta);
}

Plan Clips::candidate(const Sight& see, int row, int sym, char delta) const {
  const ClipRow& r = t_.rows()[row];
  // Robot SE(2) in the cube frame, as in retrieve(). Keeping this computation
  // here makes a locked candidate differ from a global retrieval in identity
  // only; its cost and entry geometry are always based on the newest sight.
  const float c = std::cos(-see.phi), s = std::sin(-see.phi);
  const float x = -see.pos[0], y = -see.pos[1];
  const float q[3] = {c * x - s * y, s * x + c * y, -see.phi};
  const float horizon =
      cfg_.horizon_gain * std::fabs(r.exit_range - cfg_.stance_band_m);

  Plan p;
  p.mode = Mode::CLIP;
  p.row = row;
  p.sym = sym;
  p.frames = r.span_len;
  p.cost = se2(r, q, sym, cfg_.arm_radius) + horizon;
  p.delta = delta;
  // Anchor on the cube, never the robot: rotating the reference about the cube
  // puts the residual on the reference ROBOT, which the controller tracks. This
  // scalar IS that rotation, and it is exactly the heading term the cost just
  // minimised.
  p.entry_yaw = wrap(r.qth + sym * (kPi / 2.0f) + see.phi);
  p.matched_entry_yaw = p.entry_yaw;
  // The same translation term `se2()` ranked, now expressed in the robot base
  // frame. It is the physical displacement that would put the live robot at
  // this clip's recorded entry stance; no odometry or total-cost weights enter.
  const float a = sym * (kPi / 2.0f);
  const float ca = std::cos(a), sa = std::sin(a);
  const float rx = ca * r.qx - sa * r.qy;
  const float ry = sa * r.qx + ca * r.qy;
  const float cp = std::cos(see.phi), sp = std::sin(see.phi);
  const float ex = see.pos[0] + cp * rx - sp * ry;
  const float ey = see.pos[1] + sp * rx + cp * ry;
  p.entry_translation = std::hypot(ex, ey);
  p.entry_bearing = std::atan2(ey, ex);
  p.entry_forward = ex;
  p.entry_lateral = ey;
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%c#%d", delta, r.clip);
  p.label = buf;
  return p;
}

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
