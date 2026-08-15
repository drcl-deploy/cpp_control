#include "sys1/clips.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace cpp_control {
namespace sys1 {

namespace {

constexpr float kPi = static_cast<float>(M_PI);

/// Angle into [-pi, pi).
float wrap(float a) {
  return std::atan2(std::sin(a), std::cos(a));
}

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
    case Mode::SETTLE: return "settle";
    case Mode::SCAN:   return "scan";
    case Mode::CLIP:   return "clip";
    default:           return "init";
  }
}

Sys1Clips::Sys1Clips(const ClipTable& table, const Cfg& cfg, int target_color)
    : t_(table), cfg_(cfg), eye_(cfg, PALETTE_SIM, table.half_extent()),
      target_(target_color) {
  cfg_.validate();
  reset();
}

void Sys1Clips::reset() {
  n_ = 0;
  tips_ = 0;
  stall_ = 0;
  tried_.clear();
  last_color_ = -1;
  done_ = false;
  see_ = Sight{};
  plan_ = Plan{};
}

void Sys1Clips::set_target_color(int c) {
  if (c == target_) return;
  target_ = c;
  done_ = false;
}

const Sight& Sys1Clips::look(const cv::Mat& bgr, const cv::Mat& depth_m,
                             const Intrinsics& intr, const CameraPose& cam) {
  see_ = eye_(bgr, depth_m, intr, cam);
  return see_;
}

char Sys1Clips::delta() const {
  const char d = cfg_.pattern[n_ % cfg_.pattern.size()];
  // stall_ drives BOTH escapes and they are mutually exclusive by config: v5
  // swaps the axis (retry_limit), v6 advances the rung (stall_limit).
  return (cfg_.retry_limit > 0 && stall_ >= cfg_.retry_limit) ? other_axis(d)
                                                              : d;
}

void Sys1Clips::advance() {
  ++n_;
  stall_ = 0;
  tried_.clear();
}

// ── The decision ─────────────────────────────────────────────────

Plan Sys1Clips::decide() {
  // A TIP IS AN OBSERVED TOP-COLOUR CHANGE, counted on the read this decision
  // consumes: one still mode yields several looks and crediting each of them
  // walked the ladder three rungs on a flicker (sys1_cpp.md §8.5).
  if (see_.color_ok) {
    if (last_color_ >= 0 && last_color_ != see_.color) {
      ++tips_;
      advance();  // it moved: ladder resets
    }
    last_color_ = see_.color;
  }

  if (see_.color_ok && see_.color == target_) {
    // DONE FIRST, and on the COLOUR: a cut face names the top colour exactly,
    // and refusing it costs a whole scan cycle to re-learn what was just read.
    done_ = true;
    plan_ = still(Mode::SETTLE, 0.0f);
  } else if (!see_.ok) {
    // No POSE: turn. Covers "no cube", "only side faces", "mid-tumble" and
    // "half in frame" with ONE branch — all four are answered by looking from
    // elsewhere. A quarter sweep when the top face IS in frame and merely cut:
    // that is a re-aim, not a search.
    const float sweep =
        cfg_.scan_sweep_deg * kPi / 180.0f * (see_.color_ok ? 0.25f : 1.0f);
    float yaw = std::clamp(see_.hint, -sweep, sweep);
    if (std::fabs(yaw) < 0.25f * sweep)
      yaw = 0.25f * sweep * (yaw < 0.0f ? -1.0f : 1.0f);
    plan_ = still(Mode::SCAN, yaw);
  } else {
    plan_ = retrieve();
  }
  return plan_;
}

Plan Sys1Clips::still(Mode mode, float yaw) const {
  Plan p;
  p.mode = mode;
  p.yaw_offset = yaw;
  p.frames = mode == Mode::SCAN ? cfg_.scan_steps : cfg_.settle_steps;
  p.label = mode_name(mode);
  return p;
}

// ── THE ONE RANKING SCALAR ───────────────────────────────────────
//
// The warp is cube-exact, so what sys0 must self-correct is exactly the stance
// residual: the robot's SE(2) pose in the CUBE's frame, ours minus the
// recording's. The cube's 4-fold symmetry gives four free warps, so take the
// best. Range, bearing and spin all fold into this one distance.

Plan Sys1Clips::retrieve() {
  if (cfg_.stall_limit > 0 && stall_ >= cfg_.stall_limit)
    advance();  // the ladder cannot stall on a lost tip (measured OFF)
  const char d = delta();
  const std::vector<int>& k = t_.pool(d);
  if (k.empty()) return still(Mode::SETTLE, 0.0f);

  std::vector<int> avail;
  avail.reserve(k.size());
  for (int i : k)
    if (!tried_.count(i)) avail.push_back(i);
  if (avail.empty()) {  // pool burned -> allow reuse
    tried_.clear();
    avail = k;
  }

  // Robot SE(2) in the cube frame. The robot is the base frame's origin looking
  // down +x, so this is pure perception.
  const float c = std::cos(-see_.phi), s = std::sin(-see_.phi);
  const float x = -see_.pos[0], y = -see_.pos[1];
  const float q[3] = {c * x - s * y, s * x + c * y, -see_.phi};

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

  tried_.insert(best_row);
  ++stall_;

  const ClipRow& r = t_.rows()[best_row];
  Plan p;
  p.mode = Mode::CLIP;
  p.row = best_row;
  p.sym = best_sym;
  p.frames = r.span_len;
  p.cost = best;
  p.delta = d;
  // Anchor on the cube, never the robot: rotating the reference about the cube
  // puts the residual on the reference ROBOT, which sys0 tracks. This scalar IS
  // that rotation, and it is exactly the heading term the cost just minimised.
  p.entry_yaw = wrap(r.qth + best_sym * (kPi / 2.0f) + see_.phi);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%c#%d", d, r.clip);
  p.label = buf;
  return p;
}

std::string Sys1Clips::status() const {
  char top[8] = "--";
  if (see_.color_ok) std::snprintf(top, sizeof(top), "c%d", see_.color);
  char buf[256];
  std::snprintf(
      buf, sizeof(buf),
      "%-6s n=%d d=%c %-8s | top %s %s r=%.2f b=%+.0f phi=%+.0f cost=%.2f",
      mode_name(plan_.mode), n_, delta(), plan_.label.c_str(), top,
      see_.reason.c_str(), see_.range_m(), see_.bearing() * 180.0f / kPi,
      see_.phi * 180.0f / kPi, plan_.cost);
  return buf;
}

}  // namespace sys1
}  // namespace cpp_control
