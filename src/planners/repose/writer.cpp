#include "cpp_control/planners/repose/writer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "common/g1/joint_orders.hpp"
#include "common/g1/motion.hpp"

namespace cpp_control {
namespace planners {
namespace repose {

namespace {

constexpr int J = g1::NUM_JOINTS;
constexpr int APOS = 2 * J, AQUAT = 2 * J + 3;
constexpr int ALIN = g1::WIRE_COLS_MIN, AANG = g1::WIRE_COLS_MIN + 3;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kPeak = 1.5f;  ///< a smoothstep's peak slope over its mean

/// Premultiply a wire row's anchor quat by Rz(psi) and turn its world twist
/// with it. Body-frame twist — what the adapter actually reads — is invariant
/// under this by construction, so the augmentation port never notices a warp.
void rotate_row(float* row, float psi) {
  const float h = 0.5f * psi, cw = std::cos(h), sw = std::sin(h);
  const float w = row[AQUAT], x = row[AQUAT + 1], y = row[AQUAT + 2],
              z = row[AQUAT + 3];
  row[AQUAT] = cw * w - sw * z;  // Rz(psi) * q, wxyz
  row[AQUAT + 1] = cw * x - sw * y;
  row[AQUAT + 2] = cw * y + sw * x;
  row[AQUAT + 3] = cw * z + sw * w;
  const float c = std::cos(psi), s = std::sin(psi);
  for (int at : {ALIN, AANG}) {
    const float vx = row[at], vy = row[at + 1];
    row[at] = c * vx - s * vy;
    row[at + 1] = s * vx + c * vy;
  }
}

}  // namespace

ReferenceWriter::ReferenceWriter(const ClipTable& table, const Cfg& cfg,
                                 const ApproachSource* approach)
    : t_(table), approach_(approach), cfg_(cfg), cols_(table.cols()) {}

void ReferenceWriter::push_stand(int count) {
  const size_t at = rows_.size();
  rows_.resize(at + static_cast<size_t>(count) * cols_);
  for (int f = 0; f < count; ++f)
    std::memcpy(&rows_[at + static_cast<size_t>(f) * cols_], t_.stand_row(),
                cols_ * sizeof(float));
}

/// Rate-limited ramp from the live pose onto `target`. The sim pays this as a
/// 12-frame blend; on hardware a reference that STEPS is a step input to a
/// balancing policy, so walk there instead — the same mechanism the
/// human-driven L1 prep provides, just automatic
/// (docs/planners/repose/planner.md §1 F4).
///
/// Under v7 this matters for STILL modes too, not only clips: the nominal
/// stance is the planner's own vocabulary and therefore sits further from a
/// clip's exit pose than the library frame it replaced, so the settle that
/// follows every clip is the largest joint step in the loop. Cost is
/// `lead_in_min_s` (0.2 s) when the delta is small, which is what a
/// settle-after-settle sees.
///
/// v7.1 puts the HEADING in the same ramp (`enter_yaw_rate_deg`), for clips
/// only. `MotionClock::engage` cancels the FRAME-0 heading and nothing else, so
/// a rotation that VARIES across the lead-in survives it — the same mechanism
/// the SCAN sweep already uses to command a turn. The residual then lives in
/// the rows and `entry_yaw_offset` goes to zero: one convention, both modes.
void ReferenceWriter::push_lead_in(const Plan& plan, const float* target,
                                   const LiveState& live) {
  if (cfg_.lead_in_rate <= 0.0f) return;
  ramp_ = cfg_.enter_yaw_rate_deg > 0.0f &&
          (plan.mode == Mode::CLIP || plan.mode == Mode::APPROACH);

  // v7.1 starts from what the controller was ACTUALLY commanded, so the ramp is
  // C0 with the still it leaves; v7 keeps the live pose it was measured with.
  const bool held = ramp_ && live.held_row.size() == static_cast<size_t>(cols_);
  const float* from = held ? live.held_row.data() : live.joint_pos_il.data();

  float max_dq = 0.0f;
  for (int j = 0; j < J; ++j)
    max_dq = std::max(max_dq, std::fabs(target[j] - from[j]));

  if (!ramp_) {
    const float secs = std::clamp(max_dq / cfg_.lead_in_rate,
                                  cfg_.lead_in_min_s, cfg_.lead_in_max_s);
    lead_ = std::max(2, static_cast<int>(std::lround(secs * t_.fps())));
  } else {
    // Whichever bound needs longer wins, and the answer is in FRAMES: the ramp
    // sweeps over `lead_ - 1` intervals, so rounding UP there is what keeps the
    // achieved peak under the ask rather than one frame over it.
    const float w_max = cfg_.enter_yaw_rate_deg * kPi / 180.0f;
    const float n = kPeak * t_.fps() *
                    std::max(std::fabs(plan.entry_yaw) / w_max,
                             max_dq / cfg_.enter_joint_rate);
    const int lo = std::max(
        2, static_cast<int>(std::lround(cfg_.lead_in_min_s * t_.fps())));
    const int hi = std::max(
        lo, static_cast<int>(std::lround(cfg_.lead_in_max_s * t_.fps())));
    lead_ = std::clamp(static_cast<int>(std::ceil(n)) + 1, lo, hi);
  }

  const float T = static_cast<float>(lead_ - 1) / t_.fps();  // swept seconds
  const float* v1 = target + J;  // the target's own joint velocity
  const size_t at = rows_.size();
  rows_.resize(at + static_cast<size_t>(lead_) * cols_);
  for (int f = 0; f < lead_; ++f) {
    float* row = &rows_[at + static_cast<size_t>(f) * cols_];
    // Root channels are the target's, so the heading the ramp holds is the one
    // the mode is about to start from; twist and contact stay zero.
    std::memcpy(row + APOS, target + APOS, (cols_ - APOS) * sizeof(float));
    std::memset(row + ALIN, 0, (cols_ - ALIN) * sizeof(float));
    const float a = static_cast<float>(f) / (lead_ - 1);
    const float s =
        a * a * (3.0f - 2.0f * a);  // smoothstep, as Motion::lead_in
    if (!ramp_) {
      for (int j = 0; j < J; ++j) row[j] = (1.0f - s) * from[j] + s * target[j];
      continue;
    }
    // Cubic Hermite. v0 is ZERO because a clip is only ever committed out of a
    // still — a commit clears the belief, so `decide()` cannot answer CLIP
    // again until a still has refilled it, and the quiescence gate has already
    // proved the robot was not moving while it did. v1 is the clip's own entry
    // velocity, which the smoothstep used to arrive at zero against, and
    // `joint_vel` is the ANALYTIC derivative: a position command and a
    // velocity command that disagree is a jerk with two sources, and the
    // tokenizer reads both. What this does NOT do is close the seam — see the
    // clamp below.
    const float a2 = a * a, a3 = a2 * a;
    const float h01 = -2.0f * a3 + 3.0f * a2, h11 = a3 - a2;
    const float g01 = 6.0f * a - 6.0f * a2, g11 = 3.0f * a2 - 2.0f * a;
    for (int j = 0; j < J; ++j) {
      const float p0 = from[j], d = target[j] - p0;
      // Fritsch-Carlson: keep the cubic MONOTONE between its endpoints. A clip
      // span is sliced mid-motion, so its entry frame is already moving fast —
      // med 4.8, p90 7.1 rad/s over this alphabet. Matched unclamped, the
      // curve detours ~0.8 rad backwards to wind up for it, which is a far
      // worse thing to command than the velocity step it removes. Clamped, the
      // ramp arrives moving wherever the geometry allows and never detours:
      // ~14% of the step goes, the rest is the alphabet's, not the seam's.
      const float m1 = std::clamp(T * v1[j], std::min(0.0f, 3.0f * d),
                                  std::max(0.0f, 3.0f * d));
      row[j] = p0 + h01 * d + h11 * m1;
      row[J + j] = (g01 * d + g11 * m1) / T;
    }
    // Root twist stays ZERO, as v7's lead-in has it. The ramp's root POSITION
    // is pinned at the entry anchor for every one of its frames, so a velocity
    // that ramps onto the clip's entry twist is a lie the robot acts on — and
    // over a ramp 5x longer than v7's lead-in it walks the clip off the anchor
    // the controller stamped at engage and never re-measures.
    rotate_row(row, -plan.entry_yaw * (1.0f - s));
    // ...and say so, as the sweep does: this is the smoothstep's derivative,
    // added to the clip's own yaw rate so both endpoints stay continuous.
    row[AANG + 2] += plan.entry_yaw * 6.0f * a * (1.0f - a) / T;
  }
  if (ramp_) return;  // jv is analytic above; a difference would fight it
  for (int f = 0; f + 1 < lead_; ++f)  // jv from jp, so the two cannot disagree
    for (int j = 0; j < J; ++j)
      rows_[at + static_cast<size_t>(f) * cols_ + J + j] =
          (rows_[at + static_cast<size_t>(f + 1) * cols_ + j] -
           rows_[at + static_cast<size_t>(f) * cols_ + j]) *
          t_.fps();
}

/// Motion matching's inertialization: pay the pose discontinuity, buy task
/// adherence, then decay the offset. Tokenizer channels only — contact flags
/// are binary and stay hard.
void ReferenceWriter::blend_head(const LiveState& live, int count) {
  for (int f = 0; f < count; ++f) {
    const float w = std::exp(-3.0f * f / static_cast<float>(cfg_.blend_frames));
    float* row = &rows_[static_cast<size_t>(f) * cols_];
    for (int j = 0; j < J; ++j) {
      row[j] += w * (live.joint_pos_il[j] - row[j]);
      row[J + j] += w * (live.joint_vel_il[j] - row[J + j]);
    }
  }
}

const std::vector<float>& ReferenceWriter::build(const Plan& plan,
                                                 const LiveState& live,
                                                 Stage stage) {
  if (live.joint_pos_il.size() != static_cast<size_t>(J) ||
      live.joint_vel_il.size() != static_cast<size_t>(J))
    throw std::runtime_error(
        "the planner writer: live state is not 29 IL joints");
  rows_.clear();
  lead_ = 0;
  ramp_ = false;

  // v7.1's second half: the ramp already walked the joints onto the entry pose
  // and paid most of the heading, so the clip is its own bare reference. No
  // lead-in (there is nothing left to lead in from) and no blend (that pins the
  // reference back to the live pose and re-opens what the ramp just closed —
  // the `_blend_entry` suppression, vibe docs/sys1_v7.md §7.1). What the ramp
  // did NOT pay rides `entry_yaw` on the message, as v7 always did, on an angle
  // the node has re-measured instead of assumed.
  if (stage == Stage::CLIP) {
    const float* span = nullptr;
    int count = 0;
    if (plan.mode == Mode::CLIP) {
      const ClipRow& r = t_.rows()[plan.row];
      span = t_.span(plan.row);
      count = r.span_len;
    } else if (plan.mode == Mode::APPROACH) {
      if (!approach_)
        throw std::runtime_error(
            "the planner writer: bare APPROACH has no walk source");
      span = approach_->span(plan.approach_window);
      count = approach_->frames(plan.approach_window);
    } else {
      throw std::runtime_error(
          "the planner writer: bare stage is only CLIP or APPROACH");
    }
    rows_.assign(span, span + static_cast<size_t>(count) * cols_);
    frames_ = count;
    return rows_;
  }

  if (plan.mode == Mode::APPROACH) {
    if (!approach_)
      throw std::runtime_error(
          "the planner writer: APPROACH requested without a walk source");
    if (approach_->cols() != cols_ ||
        std::fabs(approach_->fps() - t_.fps()) > 1e-3f)
      throw std::runtime_error(
          "the planner writer: approach/table wire or fps mismatch");
    const float* span = approach_->span(plan.approach_window);
    const int count = approach_->frames(plan.approach_window);
    push_lead_in(plan, span, live);
    if (stage == Stage::ENTER) {
      frames_ = static_cast<int>(rows_.size() / cols_);
      return rows_;
    }
    const size_t at = rows_.size();
    rows_.resize(at + static_cast<size_t>(count) * cols_);
    std::memcpy(&rows_[at], span,
                static_cast<size_t>(count) * cols_ * sizeof(float));
    frames_ = static_cast<int>(rows_.size() / cols_);
    if (lead_ == 0 && cfg_.blend_frames > 0)
      blend_head(live, std::min(cfg_.blend_frames, frames_));
    return rows_;
  }

  if (plan.mode != Mode::CLIP) {
    // A still mode is a clip whose frames happen to be identical — except for
    // the yaw, which RAMPS across them. That is what actually commands the
    // turn: the controller's window reads frame k at the yaw the sweep WILL
    // have reached, so the 6D row says "still turning", not "hold this
    // heading". A one-shot ask saturates the tracking error instead (28.5 deg
    // achieved per 90 asked).
    int hold = std::max(plan.frames, 1);
    int ramp = std::max(hold - 2 * cfg_.hold_tail, 1);
    if (cfg_.smooth_scan && plan.mode == Mode::SCAN &&
        std::fabs(plan.yaw_offset) > 1e-6f) {
      const float distance = std::fabs(plan.yaw_offset);
      const float vmax = cfg_.scan_yaw_rate_deg * kPi / 180.0f;
      const float accel = cfg_.scan_yaw_accel_deg * kPi / 180.0f;
      const float accel_distance = vmax * vmax / accel;
      const float seconds = distance <= accel_distance
                                ? 2.0f * std::sqrt(distance / accel)
                                : distance / vmax + vmax / accel;
      ramp = std::max(2, static_cast<int>(std::ceil(seconds * t_.fps())));
      hold = ramp + 2 * cfg_.hold_tail;
    }
    push_lead_in(plan, t_.stand_row(), live);
    const size_t at = rows_.size();  // the sweep is over the HELD frames only
    push_stand(hold);
    const float seconds = static_cast<float>(ramp) / t_.fps();
    for (int f = 0; f < hold; ++f) {
      const float a = std::min(static_cast<float>(f) / ramp, 1.0f);
      float angle = plan.yaw_offset * a;
      float rate = f < ramp ? plan.yaw_offset / seconds : 0.0f;
      if (cfg_.smooth_scan && plan.mode == Mode::SCAN) {
        const float distance = std::fabs(plan.yaw_offset);
        const float accel = cfg_.scan_yaw_accel_deg * kPi / 180.0f;
        const float disc =
            std::max(0.0f, seconds * seconds - 4.0f * distance / accel);
        const float accel_time = 0.5f * (seconds - std::sqrt(disc));
        const float peak = accel * accel_time;
        const float time = std::min(static_cast<float>(f) / t_.fps(), seconds);
        float travelled = distance;
        float speed = 0.0f;
        if (time < accel_time) {
          travelled = 0.5f * accel * time * time;
          speed = accel * time;
        } else if (time < seconds - accel_time) {
          travelled = 0.5f * accel * accel_time * accel_time +
                      peak * (time - accel_time);
          speed = peak;
        } else if (time < seconds) {
          const float left = seconds - time;
          travelled = distance - 0.5f * accel * left * left;
          speed = accel * left;
        }
        const float sign = std::copysign(1.0f, plan.yaw_offset);
        angle = sign * travelled;
        rate = sign * speed;
      }
      float* row = &rows_[at + static_cast<size_t>(f) * cols_];
      rotate_row(row, angle);
      // Smooth position and its analytic velocity agree, with zero velocity at
      // both ends. v8.5 stepped this channel to a constant at frame zero and
      // back to zero at the tail — exactly the yaw jerk visible on hardware.
      row[AANG + 2] = rate;
    }
    frames_ = static_cast<int>(rows_.size() / cols_);
    if (lead_ == 0 && cfg_.blend_frames > 0)
      blend_head(live, std::min(cfg_.blend_frames, frames_));
    return rows_;
  }

  const ClipRow& r = t_.rows()[plan.row];
  const float* span = t_.span(plan.row);
  push_lead_in(plan, span, live);

  if (stage == Stage::ENTER) {  // v7.1: the ramp is its own act, and ends here
    frames_ = static_cast<int>(rows_.size() / cols_);
    return rows_;
  }

  const size_t at = rows_.size();
  rows_.resize(at + static_cast<size_t>(r.span_len) * cols_);
  std::memcpy(&rows_[at], span,
              static_cast<size_t>(r.span_len) * cols_ * sizeof(float));
  frames_ = static_cast<int>(rows_.size() / cols_);
  if (lead_ == 0 && cfg_.blend_frames > 0)
    blend_head(live, std::min(cfg_.blend_frames, frames_));
  return rows_;
}

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
