#pragma once

/// What sys1 believes about the cube, right now: a vote over the last K reads
/// taken while the camera was quiet. Cleared at every commit, so a decision
/// only ever sees evidence gathered under the act it is about to replace
/// (docs/vibe/sys1/planner.md §3).

#include <array>
#include <vector>

#include "cpp_control/tasks/vibe/task_profile.hpp"
#include "sys1/cfg.hpp"
#include "sys1/sight.hpp"

namespace cpp_control {
namespace sys1 {

/// A colour vote plus the freshest pose that agrees with it. The vote is what
/// makes the ladder safe: a rung is stepped off the VOTED colour, which by
/// construction changes at most once per real cube tip, so a flickering read
/// cannot walk the ladder the way a single consumed read could.
class Belief {
 public:
  explicit Belief(const Cfg& cfg) : cfg_(cfg) {
    ring_.reserve(static_cast<size_t>(cfg.belief_window));
  }

  void clear() {
    ring_.clear();
    n_reads_ = 0;
    tally();
  }

  /// One accepted read. Rejected reads (camera moving) never reach here — they
  /// are not evidence, and treating blur as "I looked and saw nothing" is what
  /// would send the robot scanning after every clip.
  void push(const Sight& s) {
    ++n_reads_;
    ring_.push_back(s);  // newest at the back; window is 8, so this is free
    if (ring_.size() > static_cast<size_t>(cfg_.belief_window))
      ring_.erase(ring_.begin());
    tally();
  }

  /// Reads accumulated since the last commit — 0 means BLIND (camera down, or
  /// the robot never went quiet), which is not the same as "looked, saw
  /// nothing" and must not be answered by turning.
  int n_reads() const { return n_reads_; }
  int n_votes() const { return votes_; }

  /// Enough agreeing reads to act on.
  bool valid() const { return votes_ >= cfg_.belief_min_votes; }
  int color() const { return color_; }

  /// ...and one of those agreeing reads carried a face whole enough to place
  /// the cube.
  bool pose_ok() const { return pose_ != nullptr; }

  /// The freshest agreeing read. Freshest, not averaged: the robot moves
  /// between reads, so a stale pose is wrong in a way noise is not.
  const Sight& pose() const { return *pose_; }

  /// The newest read whatever it said — the telemetry line, and the SCAN's
  /// bearing hint, which stays meaningful even when the read carried no pose.
  const Sight& newest() const { return ring_.back(); }
  bool empty() const { return ring_.empty(); }

 private:
  void tally() {
    std::array<int, vibe::NUM_CUBE_COLORS> hist{};
    votes_ = 0;
    color_ = -1;
    pose_ = nullptr;
    for (const Sight& s : ring_)
      if (s.color_ok && s.color >= 0 && s.color < vibe::NUM_CUBE_COLORS)
        ++hist[s.color];
    for (int c = 0; c < vibe::NUM_CUBE_COLORS; ++c)
      if (hist[c] > votes_) {
        votes_ = hist[c];
        color_ = c;
      }
    if (color_ < 0) return;
    for (auto it = ring_.rbegin(); it != ring_.rend(); ++it)
      if (it->ok && it->color == color_) {
        pose_ = &*it;
        break;
      }
  }

  Cfg cfg_;
  std::vector<Sight> ring_;  ///< oldest first; capped at cfg_.belief_window
  int n_reads_ = 0;          ///< since the last clear, uncapped
  int votes_ = 0, color_ = -1;
  const Sight* pose_ = nullptr;  ///< into ring_; re-tallied on every mutation
};

}  // namespace sys1
}  // namespace cpp_control
