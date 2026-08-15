#pragma once

/// sys1 knobs — the C++ twin of vibe `planner/clips.py::ClipCfg`.
/// Rationale for every default lives there; the migration spec is
/// vibe `docs/sys1_cpp.md`. v5 and v6 differ by four values and nothing else.

#include <string>

namespace cpp_control {
namespace sys1 {

/// Ladder rungs, indexing `pattern`. Only the four roll deltas are retrievable.
inline constexpr char SLOTS[] = "UDFBLR";

struct Cfg {
  // ── perception ──
  int proc_width = 0;  ///< 0 = work at the depth plane's own size (the wire's)
  float min_area_frac = 0.002f;
  float min_visible = 0.60f;  ///< shortest rect side / cube edge, for a POSE
  /// Same gate for the COLOUR channel, which needs far less — the plane test
  /// already proved the face is the top one. <0 follows min_visible (v5).
  float min_visible_color = -1.0f;
  int min_value = 30;
  float min_rel_sat = 0.22f;
  float up_dot_min = 0.90f;  ///< |n.z| for a fitted plane to BE the top face
  int settle_steps = 40;

  // ── the loop ──
  std::string pattern = "RRB";  ///< v6: "RRF" — the single biggest win
  int retry_limit = 3;          ///< commits at a rung before swapping the axis
  int stall_limit = 0;          ///< built, measured, OFF (clips.py post-mortem)
  float arm_radius = 0.30f;     ///< m/rad: heading residual -> one distance
  float horizon_gain = 0.0f;    ///< v6: 0.3 — penalise a clip's exit range
  float stance_band_m = 0.70f;
  float scan_sweep_deg = 90.0f;  ///< swept over one SCAN, signed by the hint
  int scan_steps = 90;           ///< hold for one SCAN; sets the sweep RATE
  int read_tail = 4;             ///< frames at a still's end the decision reads

  // ── the seam (no sim twin: hardware has no teleport) ──
  int blend_frames = 12;    ///< live->reference offset decay at a commit
  float lead_in_rate = 1.5f;  ///< rad/s ramp onto a clip's entry; 0 = blend only
  float lead_in_min_s = 0.2f;
  float lead_in_max_s = 0.6f;

  static Cfg v5() { return Cfg{}; }
  /// v6, as MEASURED — the horizon and the graded sighting, not the clock.
  static Cfg v6() {
    Cfg c;
    c.pattern = "RRF";
    c.horizon_gain = 0.3f;
    c.min_visible_color = 0.0f;
    return c;
  }
  static Cfg preset(const std::string& name);

  /// Effective colour gate (< 0 follows min_visible — that is v5).
  float color_gate() const {
    return min_visible_color < 0.0f ? min_visible : min_visible_color;
  }
  void validate() const;
};

}  // namespace sys1
}  // namespace cpp_control
