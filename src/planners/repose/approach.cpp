#include "cpp_control/planners/repose/approach.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>

#include "common/g1/joint_orders.hpp"

namespace cpp_control {
namespace planners {
namespace repose {

namespace {

float wrap(float a) { return std::atan2(std::sin(a), std::cos(a)); }

float yaw_of(const std::array<float, 4>& q) {
  return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                    1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

}  // namespace

ApproachSource::ApproachSource(const std::string& motion_path, const Cfg& cfg)
    : motion_(g1::Motion::from_npz(motion_path, g1::MJ2IL, false)), cfg_(cfg) {
  if (motion_.num_joints != g1::NUM_JOINTS || motion_.num_bodies < 1 ||
      motion_.num_frames < 3)
    throw std::runtime_error(
        "repose approach: motion must carry >=3 frames, 29 joints and a root");
  if (!motion_.has_twist)
    throw std::runtime_error(
        "repose approach: motion has no root twist; a walking reference may "
        "not lie to the adapter with zero velocity");

  const int n = motion_.num_frames;
  std::vector<float> speed(n, 0.0f), smooth(n, 0.0f);
  for (int f = 1; f < n; ++f) {
    const auto a = motion_.root_pos(f - 1);
    const auto b = motion_.root_pos(f);
    speed[f] = std::hypot(b[0] - a[0], b[1] - a[1]) * motion_.fps;
  }
  const int radius = cfg_.approach_speed_smooth / 2;
  // Match torch conv1d's zero-padded moving mean used by ReachSource.
  for (int f = 0; f < n; ++f) {
    float sum = 0.0f;
    for (int k = -radius; k <= radius; ++k)
      if (f + k >= 0 && f + k < n) sum += speed[f + k];
    smooth[f] = sum / cfg_.approach_speed_smooth;
  }

  int end = -1;
  for (int f = 0; f < n; ++f) {
    if (smooth[f] < cfg_.approach_still_speed_m_s) continue;
    if (onset_ < 0) onset_ = f;
    end = f;
  }
  if (onset_ < 0 || end <= onset_ + 1)
    throw std::runtime_error(
        "repose approach: no moving segment above still_speed_m_s");

  const int seg_n = end - onset_;
  std::vector<float> net(seg_n, 0.0f), path(seg_n, 0.0f);
  const auto source_origin = motion_.root_pos(onset_);
  for (int k = 1; k < seg_n; ++k) {
    const auto a = motion_.root_pos(onset_ + k - 1);
    const auto b = motion_.root_pos(onset_ + k);
    net[k] = std::hypot(b[0] - source_origin[0], b[1] - source_origin[1]);
    path[k] = path[k - 1] + std::hypot(b[0] - a[0], b[1] - a[1]);
  }

  // One extra grid point brackets max_step_m for nearest-window selection.
  // Windows remain unscaled: their joint/root velocities keep their recorded
  // meaning, exactly as in OGMP ReachSource.
  std::set<int> cuts;
  for (float goal = cfg_.approach_window_step_m;
       goal <= cfg_.approach_max_step_m + cfg_.approach_window_step_m + 1e-6f;
       goal += cfg_.approach_window_step_m) {
    // The quantity the planner requests and later measures is NET robot
    // displacement. v8.5 cut on cumulative foot-path length and then labelled
    // the window by net displacement, so the opening weight shift consumed a
    // large part of every nominally short step. Search the first crossing: the
    // recording eventually turns home, so its full net-distance trace is not
    // globally sorted even though this short outbound prefix is monotone.
    const std::vector<float>& cut_metric =
        cfg_.approach_net_windows ? net : path;
    int k = 1;
    while (k + 1 < seg_n && cut_metric[k] < goal) ++k;
    const int lo = std::max(1, k - cfg_.approach_window_snap);
    const int hi = std::min(seg_n - 1, k + cfg_.approach_window_snap + 1);
    if (hi <= lo) continue;
    int best = lo;
    for (int j = lo + 1; j < hi; ++j)
      if (smooth[onset_ + j] < smooth[onset_ + best]) best = j;
    cuts.insert(best);
  }
  if (cuts.empty())
    throw std::runtime_error("repose approach: the walk produced no windows");

  const auto p0 = motion_.root_pos(onset_);
  const float root_yaw = yaw_of(motion_.root_quat(onset_));
  for (int length : cuts) {
    const auto p1 = motion_.root_pos(onset_ + length);
    const float heading = std::atan2(p1[1] - p0[1], p1[0] - p0[0]);
    const float travel_from_root = wrap(heading - root_yaw);
    if (std::fabs(travel_from_root) * 180.0f / static_cast<float>(M_PI) >
        cfg_.approach_window_heading_max_deg)
      continue;
    // Size by NET displacement, not cumulative footpath. The opening weight
    // shift is real path length but closes no clip-entry residual.
    const float net = std::hypot(p1[0] - p0[0], p1[1] - p0[1]);
    if (!windows_.empty() && net <= windows_.back().distance) continue;
    windows_.push_back({length, net, travel_from_root});
  }
  if (windows_.empty())
    throw std::runtime_error(
        "repose approach: every walk window violates window_heading_max_deg");

  // Bake the source into the controller's IL wire once. A walk has no cube,
  // therefore its robot<->object contact command is truthfully all zero even
  // if the recording directory happens to contain an object contact matrix.
  wire_.assign(static_cast<size_t>(n) * g1::WIRE_COLS_FULL, 0.0f);
  std::array<float, g1::NUM_JOINTS> jp{}, jv{};
  for (int f = 0; f < n; ++f) {
    float* row = &wire_[static_cast<size_t>(f) * g1::WIRE_COLS_FULL];
    motion_.jp_il(f, jp.data());
    motion_.jv_il(f, jv.data());
    std::copy(jp.begin(), jp.end(), row);
    std::copy(jv.begin(), jv.end(), row + g1::NUM_JOINTS);
    const auto p = motion_.root_pos(f);
    const auto q = motion_.root_quat(f);
    std::copy(p.begin(), p.end(), row + 2 * g1::NUM_JOINTS);
    std::copy(q.begin(), q.end(), row + 2 * g1::NUM_JOINTS + 3);
    const size_t root3 = static_cast<size_t>(f) * motion_.num_bodies * 3;
    std::copy_n(&motion_.body_lin_vel_w[root3], 3, row + g1::WIRE_COLS_MIN);
    std::copy_n(&motion_.body_ang_vel_w[root3], 3, row + g1::WIRE_COLS_MIN + 3);
  }

  std::printf(
      "[planner] approach: onset %d, %zu windows %.2f..%.2f m from %s\n",
      onset_, windows_.size(), windows_.front().distance,
      windows_.back().distance, motion_path.c_str());
}

Plan ApproachSource::plan(const Plan& candidate) const {
  if (candidate.mode != Mode::CLIP)
    throw std::runtime_error(
        "repose approach: only a clip has an entry stance");
  const float requested =
      std::clamp((cfg_.approach_anisotropic ? candidate.entry_forward
                                            : candidate.entry_translation) -
                     (cfg_.approach_anisotropic ? cfg_.approach_target_forward_m
                                                : cfg_.approach_target_m),
                 0.0f, cfg_.approach_max_step_m);
  int best = 0;
  float error = std::numeric_limits<float>::max();
  for (size_t i = 0; i < windows_.size(); ++i) {
    const float e = std::fabs(windows_[i].distance - requested);
    if (e < error) {
      error = e;
      best = static_cast<int>(i);
    }
  }

  const ApproachWindow& w = windows_[best];
  Plan p;
  p.mode = Mode::APPROACH;
  p.row = candidate.row;
  p.sym = candidate.sym;
  p.delta = candidate.delta;
  p.frames = w.length + 1;  // both endpoints
  p.cost = candidate.cost;
  p.entry_translation = candidate.entry_translation;
  p.entry_bearing = candidate.entry_bearing;
  p.entry_forward = candidate.entry_forward;
  p.entry_lateral = candidate.entry_lateral;
  p.matched_entry_yaw = candidate.entry_yaw;
  p.approach_requested = requested;
  p.approach_covered = w.distance;
  p.approach_window = best;
  // Normal MotionClock alignment already maps the source root heading onto the
  // live robot heading. Pay only the remaining source-travel-to-target angle.
  p.entry_yaw = wrap(candidate.entry_bearing - w.travel_from_root);
  char label[48];
  std::snprintf(label, sizeof(label), "approach %.2fm", w.distance);
  p.label = label;
  return p;
}

const float* ApproachSource::span(int window) const {
  if (window < 0 || window >= static_cast<int>(windows_.size()))
    throw std::runtime_error("repose approach: window index out of range");
  return &wire_[static_cast<size_t>(onset_) * g1::WIRE_COLS_FULL];
}

int ApproachSource::frames(int window) const {
  if (window < 0 || window >= static_cast<int>(windows_.size()))
    throw std::runtime_error("repose approach: window index out of range");
  return windows_[window].length + 1;
}

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
