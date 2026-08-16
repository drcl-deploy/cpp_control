#include "sys1/writer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "common/g1/joint_orders.hpp"
#include "common/g1/motion.hpp"

namespace cpp_control {
namespace sys1 {

namespace {

constexpr int J = g1::NUM_JOINTS;
constexpr int APOS = 2 * J, AQUAT = 2 * J + 3;
constexpr int ALIN = g1::WIRE_COLS_MIN, AANG = g1::WIRE_COLS_MIN + 3;

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

ReferenceWriter::ReferenceWriter(const ClipTable& table, const Cfg& cfg)
    : t_(table), cfg_(cfg), cols_(table.cols()) {}

void ReferenceWriter::push_stand(int count) {
  const size_t at = rows_.size();
  rows_.resize(at + static_cast<size_t>(count) * cols_);
  for (int f = 0; f < count; ++f)
    std::memcpy(&rows_[at + static_cast<size_t>(f) * cols_], t_.stand_row(),
                cols_ * sizeof(float));
}

/// Rate-limited ramp from the live pose onto `target`. The sim pays this as a
/// 12-frame blend; on hardware a reference that STEPS is a step input to a
/// balancing policy, so walk there instead — the same mechanism the human-driven
/// L1 prep provides, just automatic (docs/sys1.md §1 F4).
///
/// Under v7 this matters for STILL modes too, not only clips: the nominal stance
/// is sys1's own vocabulary and therefore sits further from a clip's exit pose
/// than the library frame it replaced, so the settle that follows every clip is
/// the largest joint step in the loop. Cost is `lead_in_min_s` (0.2 s) when the
/// delta is small, which is what a settle-after-settle sees.
void ReferenceWriter::push_lead_in(const float* target, const LiveState& live) {
  if (cfg_.lead_in_rate <= 0.0f) return;
  float max_dq = 0.0f;
  for (int j = 0; j < J; ++j)
    max_dq = std::max(max_dq, std::fabs(target[j] - live.joint_pos_il[j]));
  const float secs = std::clamp(max_dq / cfg_.lead_in_rate, cfg_.lead_in_min_s,
                                cfg_.lead_in_max_s);
  lead_ = std::max(2, static_cast<int>(std::lround(secs * t_.fps())));

  const size_t at = rows_.size();
  rows_.resize(at + static_cast<size_t>(lead_) * cols_);
  for (int f = 0; f < lead_; ++f) {
    float* row = &rows_[at + static_cast<size_t>(f) * cols_];
    // Root channels are the target's, so the heading the ramp holds is the one
    // the mode is about to start from; twist and contact stay zero.
    std::memcpy(row + APOS, target + APOS, (cols_ - APOS) * sizeof(float));
    std::memset(row + ALIN, 0, (cols_ - ALIN) * sizeof(float));
    const float a = static_cast<float>(f) / (lead_ - 1);
    const float s = a * a * (3.0f - 2.0f * a);  // smoothstep, as Motion::lead_in
    for (int j = 0; j < J; ++j)
      row[j] = (1.0f - s) * live.joint_pos_il[j] + s * target[j];
  }
  for (int f = 0; f + 1 < lead_; ++f)  // jv from jp, so the two cannot disagree
    for (int j = 0; j < J; ++j)
      rows_[at + static_cast<size_t>(f) * cols_ + J + j] =
          (rows_[at + static_cast<size_t>(f + 1) * cols_ + j] -
           rows_[at + static_cast<size_t>(f) * cols_ + j]) * t_.fps();
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
                                                 const LiveState& live) {
  if (live.joint_pos_il.size() != static_cast<size_t>(J) ||
      live.joint_vel_il.size() != static_cast<size_t>(J))
    throw std::runtime_error("sys1 writer: live state is not 29 IL joints");
  rows_.clear();
  lead_ = 0;

  if (plan.mode != Mode::CLIP) {
    // A still mode is a clip whose frames happen to be identical — except for
    // the yaw, which RAMPS across them. That is what actually commands the
    // turn: sys0's window reads frame k at the yaw the sweep WILL have reached,
    // so the 6D row says "still turning", not "hold this heading". A one-shot
    // ask saturates the tracking error instead (28.5 deg achieved per 90 asked).
    const int hold = std::max(plan.frames, 1);
    push_lead_in(t_.stand_row(), live);
    const size_t at = rows_.size();  // the sweep is over the HELD frames only
    push_stand(hold);
    const int ramp = std::max(hold - 2 * cfg_.read_tail, 1);
    const float rate = plan.yaw_offset / ramp;  // rad per frame
    for (int f = 0; f < hold; ++f) {
      const float frac = std::min(static_cast<float>(f) / ramp, 1.0f);
      float* row = &rows_[at + static_cast<size_t>(f) * cols_];
      rotate_row(row, plan.yaw_offset * frac);
      // The robot IS turning, so say so: zero here would lie to the adapter's
      // robot_root_ang_vel_cmd for the whole sweep.
      row[AANG + 2] = f < ramp ? rate * t_.fps() : 0.0f;
    }
    frames_ = static_cast<int>(rows_.size() / cols_);
    if (lead_ == 0 && cfg_.blend_frames > 0)
      blend_head(live, std::min(cfg_.blend_frames, frames_));
    return rows_;
  }

  const ClipRow& r = t_.rows()[plan.row];
  const float* span = t_.span(plan.row);
  push_lead_in(span, live);

  const size_t at = rows_.size();
  rows_.resize(at + static_cast<size_t>(r.span_len) * cols_);
  std::memcpy(&rows_[at], span,
              static_cast<size_t>(r.span_len) * cols_ * sizeof(float));
  frames_ = static_cast<int>(rows_.size() / cols_);
  if (lead_ == 0 && cfg_.blend_frames > 0)
    blend_head(live, std::min(cfg_.blend_frames, frames_));
  return rows_;
}

}  // namespace sys1
}  // namespace cpp_control
