#include "sys1/cfg.hpp"

#include <cstring>
#include <stdexcept>

namespace cpp_control {
namespace sys1 {

Cfg Cfg::preset(const std::string& name) {
  if (name == "v5") return Cfg::v5();
  if (name == "v6") return Cfg::v6();
  throw std::runtime_error("sys1: version must be v5 | v6, got '" + name + "'");
}

void Cfg::validate() const {
  if (pattern.empty())
    throw std::runtime_error("sys1: pattern is empty");
  for (char c : pattern)
    if (!std::strchr("FBLR", c))
      throw std::runtime_error(
          "sys1: pattern must be drawn from FBLR (only the roll deltas are "
          "retrievable), got '" + pattern + "'");
  if (settle_steps <= 2 * read_tail || scan_steps <= 2 * read_tail)
    throw std::runtime_error(
        "sys1: a still mode must outlast twice its read tail — the sweep has to "
        "finish before the camera is read");
  if (min_visible <= 0.0f || min_visible > 1.0f)
    throw std::runtime_error("sys1: min_visible must be in (0, 1]");
  if (blend_frames < 0 || lead_in_rate < 0.0f)
    throw std::runtime_error("sys1: blend_frames / lead_in_rate must be >= 0");
}

}  // namespace sys1
}  // namespace cpp_control
