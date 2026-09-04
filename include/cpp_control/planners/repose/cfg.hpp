#pragma once

/// the planner knobs — the C++ twin of vibe `planner/clips.py::ClipCfg`.
/// Rationale for every default lives there; the migration spec is
/// vibe `docs/sys1_cpp.md`. Each version is one knob over the last, so one
/// binary A/Bs both ablations still worth running on the robot.

#include <array>
#include <string>

namespace YAML {
class Node;  // the loader takes one; nothing else here needs yaml-cpp
}

namespace cpp_control {
namespace planners {
namespace repose {

/// Ladder rungs, indexing `pattern`. Only the four roll deltas are retrievable.
inline constexpr char SLOTS[] = "UDFBLR";

/// Chromaticity references, 6 colours x 2 rows (lit, shaded), RGB.
/// 0 red 1 orange 2 green 3 yellow 4 blue 5 pink. A config value, not a
/// constant: the twelve defaults were measured off the SIM renderer, and a
/// deployment's own lighting is a different set of numbers entirely.
using Palette = std::array<float, 36>;
extern const Palette PALETTE_SIM;

struct Cfg {
  // ── perception ──
  int proc_width = 0;  ///< 0 = work at the depth plane's own size (the wire's)
  float min_area_frac = 0.002f;
  int min_px_floor =
      8;  ///< a blob is never smaller than this, whatever the frac
  float min_visible = 0.60f;  ///< shortest rect side / cube edge, for a POSE
  /// Same gate for the COLOUR channel, which needs far less — the plane test
  /// already proved the face is the top one.
  float min_visible_color = 0.0f;
  int min_value = 30;
  float min_rel_sat = 0.22f;
  float up_dot_min = 0.90f;  ///< |n.z| for a fitted plane to BE the top face
  float big_max =
      1.4f;  ///< longest rect side / cube edge: above this it is floor
  float z_min_m = 0.05f;  ///< depth nearer than this is not a measurement
  /// TOP SLAB depth, in cube edges, below the blob's `min_px`-th highest point.
  /// A same-coloured floor touching the cube is ONE component and the floor is
  /// the larger half, so the cut is by height.
  float slab_frac = 0.25f;
  Palette palette = PALETTE_SIM;

  /// How a frame becomes ONE answer. Two orders of the same gates.
  ///
  ///   BLOBS  colour first: six independent blob extractions, each gated on
  ///          geometry, and the HIGHEST surviving one wins. The shipping read.
  ///   MASK   geometry first in name only: one cube-top mask off every pixel
  ///          that already has a colour label, then the modal colour over it.
  ///   PLANE  depth first: fit the floor from all valid depth, lift it by the
  ///          known cube edge, fit a hole-tolerant square, THEN vote colour.
  ///
  /// Measured on bags/sys1_observe (180 labelled hardware frames, 60
  /// negatives): BLOBS 55%, and no palette or gate setting moves it — 54 of 83
  /// errors are the true blob passing every gate and losing the height contest
  /// to a spurious one. MASK with a re-measured palette, `chroma_reject` and a
  /// loosened `min_rel_sat` reaches 84% at the same 0/60 false positives.
  /// Neither half works alone: MASK on the sim palette is 49%.
  enum class Read { BLOBS, MASK, PLANE };
  Read read = Read::BLOBS;

  /// MASK only. The top face is a horizontal plane `mask_band_m` thick, found
  /// at the `mask_top_pct`-th percentile of live height — a percentile, not the
  /// max, because stray pixels put the max above the robot's root. Then the
  /// largest connected component, eroded by `mask_erode` to drop the mixed
  /// pixels a JPEG leaves on every edge.
  float mask_top_pct = 97.0f;
  float mask_band_m = 0.08f;
  int mask_erode = 1;

  /// PLANE only. The floor is the dominant horizontal plane among the lower
  /// `plane_floor_quantile` of valid points. The cube top is one known edge
  /// above it. Closing associates depth islands caused by the D435i's edge
  /// dropout; the square fit keeps only the densest edge-sized window, so the
  /// closing operation never invents 3-D measurements.
  float plane_floor_quantile = 0.70f;
  float plane_ransac_dist_m = 0.035f;
  float plane_top_band_m = 0.08f;
  int plane_close_px = 2;
  float plane_fit_slack = 0.15f;
  float plane_range_max_m = 2.30f;

  /// Chromaticity distance beyond which a pixel is NO colour rather than the
  /// nearest of six. 0 disables it, which is the shipping behaviour: the
  /// classifier has no "none" class, so every lit pixel in the frame votes.
  /// Applies to both reads; it is what lets MASK loosen `min_rel_sat` without
  /// letting the floor in.
  float chroma_reject = 0.0f;

  /// v7: hold the robot's NOMINAL STANCE in a still mode, not the library's
  /// stand frame. The planner's own vocabulary — the borrowed pose was picked
  /// for quietness and carries a waist that aims the head at the near ground.
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
  // condition (docs/planners/repose/planner.md §3).

  int belief_window = 8;  ///< reads kept; at 20 Hz this is the last 0.4 s
  int belief_min_votes =
      3;  ///< agreeing reads before the belief may be acted on
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

  // ── v8.5 approach ──
  // A clip candidate whose TRANSLATION-ONLY entry residual exceeds this gate
  // gets a robot-anchored walk before it is reconsidered. The old total cost
  // is deliberately not the gate: it also prices heading and the exit
  // horizon, neither of which says how far the feet need to move.
  bool approach_enabled = false;
  std::string approach_motion;
  float approach_enter_m = 0.25f;
  float approach_target_m = 0.13f;
  float approach_max_step_m = 0.30f;
  float approach_turn_max_deg = 20.0f;
  /// Hard stop for the preparatory heading correction. Two ideal 90-degree
  /// sweeps cover the full circle; the third allows tracking error to settle.
  int approach_turn_max_attempts = 3;
  float approach_min_progress_m = 0.05f;
  int approach_max_attempts = 2;
  /// ReachSource-compatible construction of the fixed walk vocabulary.
  float approach_window_step_m = 0.12f;
  int approach_window_snap = 4;
  float approach_still_speed_m_s = 0.05f;
  int approach_speed_smooth = 9;
  /// Reject short prefixes whose initial weight shift travels sideways. A
  /// heading warp on one of those would itself be an unsafe yaw step.
  float approach_window_heading_max_deg = 45.0f;

  // ── the seam (no sim twin: hardware has no teleport) ──
  int blend_frames = 12;  ///< live->reference offset decay; only if no lead-in
  float lead_in_rate =
      1.5f;  ///< rad/s ramp onto a clip's entry; 0 = blend only
  float lead_in_min_s = 0.2f;
  float lead_in_max_s = 0.6f;

  /// v7.1: put the HEADING in the ramp too. 0 is v7, every new path
  /// short-circuited.
  ///
  /// The lead-in has always walked the JOINTS onto a clip's entry pose. The
  /// heading never joined them: `MotionClock::engage` pins frame 0 to
  /// `robot_yaw + entry_yaw`, so the whole residual lands in one frame.
  /// Measured over the clean pools a commit asks med 9-22 deg, p90 21-45 —
  /// 157-321 deg/s delivered that way, 3-6x the only rate the planner has ever
  /// measured as followable (the SCAN sweep, 50 deg/s). Sim v7.1: clip hit at
  /// stance residual >= 0.35 m, 0.745 -> 0.854 (z = 2.52), and below it the
  /// arms are identical — the mechanism's signature. Solve rate is a wash;
  /// what moves is speed and jerk (vibe `docs/sys1_v7.md` §7.1).
  ///
  /// A RATE, because what is bounded is what the controller can follow.
  float enter_yaw_rate_deg = 0.0f;
  /// The same bound on the largest single joint the ramp moves, rad/s. Both
  /// enter_* knobs are PEAK rates — a smoothstep peaks at 1.5x its mean and
  /// the sizing pays for that. Distinct from `lead_in_rate`, which is v7's
  /// mean-rate knob and keeps its meaning; whichever term needs longer wins.
  float enter_joint_rate = 2.0f;

  /// v6 — the library's stand frame, i.e. v7 without the gaze fix. The one
  /// ablation still worth running; everything else the versions used to carry
  /// is now the single shipping default.
  static Cfg v6() {
    Cfg c;
    c.nominal_stand = false;
    return c;
  }
  static Cfg v7() { return Cfg{}; }
  /// v7.1 — v7 + the ENTER ramp. Built ON v7(), so a v7 revision reaches it
  /// and the pair cannot disagree about what they share. The ceiling moves
  /// with it: 0.6 s truncates even the MEDIAN heading ask (22 deg at 50 deg/s,
  /// peak-sized, is 0.66 s).
  static Cfg v7_1() {
    Cfg c = v7();
    c.enter_yaw_rate_deg = 50.0f;
    c.lead_in_max_s = 1.2f;
    return c;
  }
  /// v8 — v7's planner and controller, with only the depth-first observation
  /// path changed. This deliberately does not add walking or alter the
  /// scan/settle decision ladder: hardware can accept or reject localization
  /// in isolation first.
  static Cfg v8() {
    Cfg c = v7();
    c.read = Read::PLANE;
    return c;
  }
  /// v8.5 — v8 localization plus a guarded robot-anchored walk. APPROACH
  /// always returns to SETTLE and re-observes; it never carries a stale clip
  /// candidate across dead-reckoned locomotion.
  static Cfg v8_5() {
    Cfg c = v8();
    c.approach_enabled = true;
    c.approach_motion =
        "data/sys1_walk/walk_forward_amateur_001__A001/motion.npz";
    return c;
  }
  static Cfg preset(const std::string& name);

  /// The yaml IS the truth. `version` picks a preset for the defaults; every
  /// field below may override it, and an unknown key is an error rather than a
  /// silent no-op. One loader, so the planner and the observe rig cannot drift
  /// into reading the same file two different ways.
  static Cfg from_yaml(const YAML::Node& root);

  float color_gate() const { return min_visible_color; }
  void validate() const;

  /// Field-for-field. Exists so `repose_planner_selftest` can assert that the
  /// SHIPPED yaml round-trips to the preset it names — which turns "the config
  /// refactor is bit-identical" from a claim into a test.
  bool operator==(const Cfg& o) const;
  bool operator!=(const Cfg& o) const { return !(*this == o); }
};

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
