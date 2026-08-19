#pragma once

/// sys1 knobs — the C++ twin of vibe `planner/clips.py::ClipCfg`.
/// Rationale for every default lives there; the migration spec is
/// vibe `docs/sys1_cpp.md`. v6 and v7 differ by one boolean, so one binary
/// A/Bs the only ablation still worth running on the robot.

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
  /// already proved the face is the top one.
  float min_visible_color = 0.0f;
  int min_value = 30;
  float min_rel_sat = 0.22f;
  float up_dot_min = 0.90f;  ///< |n.z| for a fitted plane to BE the top face

  /// v7: hold the robot's NOMINAL STANCE in a still mode, not the library's
  /// stand frame. sys1's own vocabulary — the borrowed pose was picked for
  /// quietness and carries a waist that aims the head at the near ground.
  /// Measured on THIS deployment's mount quat and its own baked stand row:
  ///
  ///     library frame 12953   waist +26.7 deg -> cam 71.2 deg down, -27.2 yaw
  ///     nominal stance        waist 0         -> cam 45.0 deg down,   0.0 yaw
  ///
  /// 45.0 is the mount angle, recovered exactly because a nominal pose has
  /// waist = 0 and the torso is therefore vertical. Sim: cube-top band
  /// 0.02-0.57 m -> 0.26-1.35 m, clip exits in view 0% -> 100%, solve 62.5%
  /// -> 95% (n=120, z=6.15). False is v6.
  bool nominal_stand = true;

  // ── the belief ──
  //
  // One vote over the last `belief_window` reads replaces four rules the fused
  // observe/decide had to carry: the read window, eyes-shut-during-a-clip, the
  // tip counted only on the consumed read, and camera staleness as a park
  // condition (docs/vibe/sys1/planner.md §3).

  int belief_window = 8;     ///< reads kept; at 20 Hz this is the last 0.4 s
  int belief_min_votes = 3;  ///< agreeing reads before the belief may be acted on
  /// rad/s of CAMERA motion above which a read is blur, not evidence. A held
  /// stance sits near 0.05, a 90 deg sweep at 0.96 — the gap is an order of
  /// magnitude, so this needs no tuning to separate them.
  float omega_still = 0.15f;

  // ── the loop ──
  std::string pattern = "RRF";  ///< a Hamiltonian cycle on the cube's 6 faces
  int retry_limit = 3;          ///< commits at a rung before swapping the axis
  float arm_radius = 0.30f;     ///< m/rad: heading residual -> one distance
  float horizon_gain = 0.3f;    ///< penalise a clip's exit range
  float stance_band_m = 0.70f;
  float scan_sweep_deg = 90.0f;  ///< swept over one SCAN, signed by the hint
  int settle_steps = 40;         ///< hold for one SETTLE (yaw = 0 throughout)
  int scan_steps = 110;          ///< hold for one SCAN; sets the sweep RATE
  /// Quiescent frames a SCAN ends on: the sweep finishes `2 * hold_tail` before
  /// the reference does. It is the only thing that makes a SCAN observable at
  /// all, since every frame of the sweep itself fails `omega_still`. A SETTLE
  /// needs none — its yaw is zero, so the whole hold is quiescent.
  int hold_tail = 15;

  // ── the seam (no sim twin: hardware has no teleport) ──
  int blend_frames = 12;    ///< live->reference offset decay; only if no lead-in
  float lead_in_rate = 1.5f;  ///< rad/s ramp onto a clip's entry; 0 = blend only
  float lead_in_min_s = 0.2f;
  float lead_in_max_s = 0.6f;

  /// v6 — the library's stand frame, i.e. v7 without the gaze fix. The one
  /// ablation still worth running; everything else the versions used to carry
  /// is now the single shipping default.
  static Cfg v6() {
    Cfg c;
    c.nominal_stand = false;
    return c;
  }
  static Cfg v7() { return Cfg{}; }
  static Cfg preset(const std::string& name);

  float color_gate() const { return min_visible_color; }
  void validate() const;
};

}  // namespace sys1
}  // namespace cpp_control
