#include "cpp_control/planners/repose/cfg.hpp"

#include <cstring>
#include <stdexcept>

namespace cpp_control {
namespace planners {
namespace repose {

Cfg Cfg::preset(const std::string& name) {
  if (name == "v6") return Cfg::v6();
  if (name == "v7") return Cfg::v7();
  if (name == "v7.1") return Cfg::v7_1();
  throw std::runtime_error("repose planner: version must be v6 | v7 | v7.1, got '" +
                           name + "'");
}

void Cfg::validate() const {
  if (pattern.empty())
    throw std::runtime_error("repose planner: pattern is empty");
  for (char c : pattern)
    if (!std::strchr("FBLR", c))
      throw std::runtime_error(
          "repose planner: pattern must be drawn from FBLR (only the roll deltas are "
          "retrievable), got '" + pattern + "'");
  if (scan_steps <= 2 * hold_tail)
    throw std::runtime_error(
        "repose planner: a SCAN must outlast twice its hold tail — the sweep has to "
        "finish before the camera is quiet enough to read");
  if (settle_steps < 1)
    throw std::runtime_error("repose planner: settle_steps must be >= 1");
  if (min_visible <= 0.0f || min_visible > 1.0f)
    throw std::runtime_error("repose planner: min_visible must be in (0, 1]");
  if (belief_min_votes < 1 || belief_window < belief_min_votes)
    throw std::runtime_error(
        "repose planner: need 1 <= belief_min_votes <= belief_window");
  if (omega_still <= 0.0f)
    throw std::runtime_error(
        "repose planner: omega_still must be > 0 — a gate that never opens leaves the "
        "planner blind and standing still forever");
  if (blend_frames < 0 || lead_in_rate < 0.0f)
    throw std::runtime_error("repose planner: blend_frames / lead_in_rate must be >= 0");
  if (enter_yaw_rate_deg < 0.0f || enter_joint_rate <= 0.0f)
    throw std::runtime_error(
        "repose planner: need enter_yaw_rate_deg >= 0 (0 is v7, the ramp off) and "
        "enter_joint_rate > 0");
  if (lead_in_max_s < lead_in_min_s)
    throw std::runtime_error("repose planner: lead_in_max_s must be >= lead_in_min_s");
}

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
