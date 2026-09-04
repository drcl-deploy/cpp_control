/**
 * Repose planner unit checks, the replay rig, and the bake — no robot, no ROS.
 *
 *   repose_planner_selftest                                            # unit
 * checks
 *   repose_planner_selftest --table sys1_clips.npz \
 *                 --library sys1_library.npz --frames <retargeted_root>
 *   repose_planner_selftest ... --bake bundle.npz     # freeze the spans for
 * the Orin repose_planner_selftest --table t.npz --frames bundle.npz          #
 * read it back repose_planner_selftest ... --replay dir/ # recorded reads
 *
 * The replay rig is the acceptance gate for the perception port: it settles the
 * palette and depth-realism blockers with the robot merely standing still.
 */

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "cnpy/cnpy.h"
#include "common/g1/joint_orders.hpp"
#include "common/g1/motion.hpp"
#include "cpp_control/planners/repose/approach.hpp"
#include "cpp_control/planners/repose/belief.hpp"
#include "cpp_control/planners/repose/clips.hpp"
#include "cpp_control/planners/repose/kinematics.hpp"
#include "cpp_control/planners/repose/sight.hpp"
#include "cpp_control/planners/repose/table.hpp"
#include "cpp_control/planners/repose/writer.hpp"

using namespace cpp_control;
using namespace cpp_control::planners::repose;

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what.c_str());
  if (!ok) ++failures;
}

/// A gate that does not reject is not a gate — every `refuses`-style check runs
/// through here so a silently-accepted bad config fails the build, not the
/// robot.
template <typename F>
bool threw(F&& f) {
  try {
    f();
  } catch (const std::exception&) {
    return true;
  }
  return false;
}

// ── Unit checks: the invariants the seam depends on ──────────────

void check_cfg() {
  std::puts("cfg");
  const Cfg v6 = Cfg::v6(), v7 = Cfg::v7(), v8 = Cfg::v8(), v85 = Cfg::v8_5(),
            v9 = Cfg::v9(), v91 = Cfg::v9_1(), v92 = Cfg::v9_2();
  check(!v6.nominal_stand && v7.nominal_stand,
        "v6 and v7 differ by the gaze fix and nothing else");
  check(v6.pattern == v7.pattern && v6.horizon_gain == v7.horizon_gain,
        "everything the old versions used to split is now one default");
  check(v7.scan_steps > 2 * v7.hold_tail,
        "a SCAN outlasts twice its hold tail, so the sweep ends while quiet");
  auto refuses = [](void (*mangle)(Cfg&)) {
    Cfg bad;
    mangle(bad);
    try {
      bad.validate();
    } catch (const std::exception&) {
      return true;
    }
    return false;
  };
  check(refuses([](Cfg& c) { c.pattern = "RRU"; }),
        "a pattern outside FBLR is refused");
  check(refuses([](Cfg& c) { c.hold_tail = c.scan_steps; }),
        "a SCAN that never goes quiet is refused");
  check(refuses([](Cfg& c) { c.belief_min_votes = c.belief_window + 1; }),
        "a vote that can never carry is refused");
  check(
      refuses([](Cfg& c) { c.approach_turn_max_attempts = 0; }),
      "a preparatory turn without a finite positive attempt budget is refused");
  const Cfg r = Cfg::v7_1();
  check(r.enter_yaw_rate_deg > 0.0f && v7.enter_yaw_rate_deg == 0.0f,
        "v7.1 is v7 plus one rate; 0 is the ramp off");
  check(r.nominal_stand == v7.nominal_stand && r.pattern == v7.pattern &&
            r.hold_tail == v7.hold_tail,
        "and shares everything else with v7, so a v7 revision reaches it");
  check(r.lead_in_max_s > v7.lead_in_max_s,
        "with a ceiling that does not truncate the median heading ask");
  check(Cfg::preset("v7.1").enter_yaw_rate_deg == r.enter_yaw_rate_deg,
        "the preset name resolves");
  check(v8.read == Cfg::Read::PLANE && v8.nominal_stand == v7.nominal_stand &&
            v8.pattern == v7.pattern &&
            v8.enter_yaw_rate_deg == v7.enter_yaw_rate_deg,
        "v8 changes the observation read, not planning or control");
  check(v85.read == v8.read && v85.approach_enabled && !v8.approach_enabled &&
            v85.approach_enter_m == 0.25f && !v85.approach_anisotropic &&
            !v85.approach_net_windows && !v85.stateful_scan &&
            !v85.lock_candidate_identity && !v85.smooth_scan &&
            !v85.plane_color_pool && v85.enter_yaw_rate_deg == 0.0f &&
            v85.approach_no_progress_limit == 2,
        "v8.5 is v8 plus the guarded approach mode");
  check(v9.approach_enabled && v9.approach_anisotropic && v9.stateful_scan &&
            v9.plane_color_pool && !v9.plane_color_pool_fallback &&
            !v9.search_left_first && v9.enter_yaw_rate_deg > 0.0f,
        "v9 enables stable search, axis-wise approach and smooth dynamic acts");
  check(v91.plane_color_pool_fallback && v91.search_left_first &&
            v91.omega_still == 0.25f && v91.settle_steps == 60 &&
            v91.min_visible == 0.55f &&
            v91.approach_enter_forward_m == v9.approach_enter_forward_m &&
            v91.approach_window_step_m == v9.approach_window_step_m &&
            v91.approach_min_window_m == 0.0f &&
            !v91.approach_escalate_window && !v91.approach_forward_only &&
            !v91.approach_latch_blocked,
        "v9.1 changes observation/search admission without changing approach");
  check(v92.approach_enter_forward_m == 0.25f &&
            v92.approach_min_window_m == 0.30f &&
            v92.approach_no_progress_limit == v92.approach_max_attempts &&
            v92.approach_escalate_window && v92.approach_forward_only &&
            v92.approach_latch_blocked &&
            v92.plane_color_pool_fallback == v91.plane_color_pool_fallback &&
            v92.omega_still == v91.omega_still,
        "v9.2 changes bounded approach recovery, not v9.1 perception");
  check(refuses([](Cfg& c) { c.enter_joint_rate = 0.0f; }),
        "a zero joint rate is a divide, not a config");
  check(refuses([](Cfg& c) { c.omega_still = 0.0f; }),
        "a quiescence gate that never opens is refused");
  check(refuses([](Cfg& c) { c.plane_floor_quantile = 1.0f; }),
        "a plane floor quantile outside (0, 1) is refused");
  check(refuses([](Cfg& c) {
          c.approach_enabled = true;
          c.approach_motion.clear();
        }),
        "an enabled approach without a deployable motion is refused");
  check(refuses([](Cfg& c) { c.approach_target_m = c.approach_enter_m; }),
        "an approach target outside its admission gate is refused");
  check(refuses([](Cfg& c) { c.approach_min_window_m = -0.01f; }),
        "a negative minimum approach window is refused");
  check(refuses([](Cfg& c) {
          c.approach_escalate_window = true;
          c.approach_no_progress_limit = 1;
        }),
        "an escalation policy with only one allowed failure is refused");
}

/// A depth-only scene for v8. The camera looks forward in +base-x; lower image
/// rows intersect a horizontal floor, and an edge-sized elevated square
/// occludes it. Sparse missing top depth models the D435i fringe/interior holes
/// that made the colour-seeded MASK rectangle collapse on hardware.
void check_plane_read() {
  std::puts("plane read");
  constexpr int h = 120, w = 160;
  constexpr float edge = 0.6096f, half = 0.5f * edge;
  const Intrinsics intr{120.0f, 120.0f, 79.5f, 10.0f};
  CameraPose cam;
  cam.R = {0.0f, 0.0f, -1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
  cam.t = {0.0f, 0.0f, 1.0f};
  cv::Mat bgr(h, w, CV_8UC3, cv::Scalar(20, 20, 20));
  cv::Mat depth(h, w, CV_32FC1,
                cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  cv::Mat top(h, w, CV_8UC1, cv::Scalar(0));
  const float cube_x = 0.90f, cube_y = 0.05f, phi = 0.22f;
  const float cp = std::cos(phi), sp = std::sin(phi);
  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u) {
      const float rz = -(static_cast<float>(v) - intr.cy) / intr.fy;
      if (rz >= -1e-5f) continue;
      const float floor_d = -cam.t[2] / rz;
      depth.at<float>(v, u) = floor_d;
      const float top_d = (edge - cam.t[2]) / rz;
      const float x = top_d;
      const float y = -(static_cast<float>(u) - intr.cx) / intr.fx * top_d;
      const float dx = x - cube_x, dy = y - cube_y;
      const float qx = cp * dx + sp * dy;
      const float qy = -sp * dx + cp * dy;
      if (std::fabs(qx) > half || std::fabs(qy) > half) continue;
      top.at<uint8_t>(v, u) = 255;
      bgr.at<cv::Vec3b>(v, u) = {240, 90, 250};
      // Structured holes: geometry must associate the remaining islands, but
      // must never manufacture a point where depth is missing.
      if ((u + 2 * v) % 7 == 0)
        depth.at<float>(v, u) = std::numeric_limits<float>::quiet_NaN();
      else
        depth.at<float>(v, u) = top_d;
    }
  }

  Cfg cfg = Cfg::v8();
  cfg.min_visible = 0.70f;
  CubeSight eye(cfg, half);
  const Sight seen = eye(bgr, depth, intr, cam);
  check(seen.ok && seen.color_ok && seen.color == 5,
        "a holey top yields one usable pink square pose");
  check(std::fabs(seen.pos[0] - cube_x) < 0.08f &&
            std::fabs(seen.pos[1] - cube_y) < 0.08f &&
            std::fabs(seen.phi - phi) < 0.10f,
        "the constrained square recovers its metric centre and spin");

  cv::Mat gray(h, w, CV_8UC3, cv::Scalar(80, 80, 80));
  CubeSight depth_only(cfg, half);
  const Sight unlabelled = depth_only(gray, depth, intr, cam);
  check(
      unlabelled.ok && !unlabelled.color_ok && unlabelled.reason == "no_color",
      "rejected RGB cannot starve depth geometry");

  cv::Mat hole_color(h, w, CV_8UC3, cv::Scalar(80, 80, 80));
  for (int i = 0; i < h * w; ++i)
    if (top.ptr<uint8_t>()[i] && !std::isfinite(depth.ptr<float>()[i]))
      hole_color.ptr<cv::Vec3b>()[i] = {240, 90, 250};
  Cfg v9 = Cfg::v9();
  v9.min_visible = 0.70f;
  CubeSight pooled(v9, half);
  const Sight from_holes = pooled(hole_color, depth, intr, cam);
  check(from_holes.ok && from_holes.color_ok && from_holes.color == 5,
        "v9 pools RGB inside the square even where top depth is missing");

  Cfg destructive = Cfg::v9();
  destructive.min_visible = 0.70f;
  destructive.plane_color_inset_frac = 0.499f;
  CubeSight tiny_pool(destructive, half);
  const Sight pool_miss = tiny_pool(bgr, depth, intr, cam);
  Cfg fallback = Cfg::v9_1();
  fallback.min_visible = 0.70f;
  fallback.plane_color_inset_frac = 0.499f;
  CubeSight guarded_pool(fallback, half);
  const Sight depth_retained = guarded_pool(bgr, depth, intr, cam);
  check(!pool_miss.color_ok && depth_retained.color_ok &&
            depth_retained.color == 5,
        "v9.1 projected pooling cannot erase a valid depth colour vote");

  cv::Mat floor_depth = depth.clone();
  for (int i = 0; i < h * w; ++i)
    if (top.ptr<uint8_t>()[i]) {
      const int v = i / w;
      const float rz = -(static_cast<float>(v) - intr.cy) / intr.fy;
      floor_depth.ptr<float>()[i] = -cam.t[2] / rz;
    }
  CubeSight empty(cfg, half);
  const Sight no_cube = empty(gray, floor_depth, intr, cam);
  check(!no_cube.ok && !no_cube.color_ok,
        "a floor without an elevated square is not a cube");
}

/// The belief is what makes the ladder safe, so assert the two properties the
/// planner leans on: a minority read cannot carry, and the pose that comes back
/// is the FRESHEST one that agrees with the vote, not merely the first.
void check_belief() {
  std::puts("belief");
  Cfg cfg;
  cfg.belief_window = 4;
  cfg.belief_min_votes = 3;
  Belief b(cfg);
  check(b.n_reads() == 0 && !b.valid(),
        "an empty belief is BLIND, not 'saw nothing'");

  auto read = [](int color, bool ok, float x) {
    Sight s;
    s.color = color;
    s.color_ok = color >= 0;
    s.ok = ok;
    s.pos = {x, 0.f, 0.f};
    return s;
  };

  b.push(read(4, true, 1.0f));
  b.push(read(4, false, 0.f));
  check(b.n_reads() == 2 && !b.valid(), "two agreeing reads do not carry");
  b.push(read(1, false, 0.f));
  check(!b.valid(), "and a flicker in the middle does not help");
  b.push(read(4, true, 2.0f));
  check(b.valid() && b.color() == 4, "three of four carries the vote");
  check(b.pose_ok() && b.pose().pos[0] == 2.0f,
        "the pose is the freshest agreeing read");

  // Roll the window past every blue read; the vote must follow the evidence.
  for (int i = 0; i < 4; ++i) b.push(read(2, true, 3.0f));
  check(b.color() == 2 && b.n_reads() == 8,
        "the window forgets, the count does not");
  b.clear();
  check(b.n_reads() == 0 && !b.valid(), "a commit clears the evidence");
}

/// decide() must be PURE — the node calls it every tick and commits the answer
/// only when the controller finishes, so a decision that burned a clip on the
/// way out would empty the pool twenty times a second.
void check_decide_is_pure(const ClipTable& t) {
  std::puts("decide");
  Cfg cfg;
  Clips clips(t, cfg, 4);
  Belief b(cfg);

  check(clips.decide(b).mode == Mode::SETTLE,
        "a blind planner stands in the nominal stance, it does not turn");

  Sight seen;  // a cube, seen, but no pose: only a turn can fix that
  seen.color = 2;
  seen.color_ok = true;
  seen.hint = 0.4f;
  for (int i = 0; i < cfg.belief_min_votes; ++i) b.push(seen);
  check(clips.decide(b).mode == Mode::SCAN, "colour without a pose scans");

  Sight placed = seen;
  placed.ok = true;
  placed.pos = {0.6f, 0.1f, 0.05f};
  b.clear();
  for (int i = 0; i < cfg.belief_min_votes; ++i) b.push(placed);
  const Plan a = clips.decide(b);
  const Plan c = clips.decide(b);
  check(a.mode == Mode::CLIP, "a placed cube retrieves a clip");
  check(a.row == c.row && a.sym == c.sym && a.cost == c.cost &&
            clips.burned() == 0,
        "and twenty calls burn nothing and answer the same");
  if (a.mode == Mode::CLIP) {
    const ClipRow& r = t.rows()[a.row];
    const float q = a.sym * static_cast<float>(M_PI) / 2.0f;
    const float rx = std::cos(q) * r.qx - std::sin(q) * r.qy;
    const float ry = std::sin(q) * r.qx + std::cos(q) * r.qy;
    const float ex =
        placed.pos[0] + std::cos(placed.phi) * rx - std::sin(placed.phi) * ry;
    const float ey =
        placed.pos[1] + std::sin(placed.phi) * rx + std::cos(placed.phi) * ry;
    check(std::fabs(a.entry_translation - std::hypot(ex, ey)) < 1e-6f &&
              std::fabs(a.entry_bearing - std::atan2(ey, ex)) < 1e-6f,
          "the published entry vector is exactly the ranking's translation "
          "term in robot coordinates");
  }
  clips.commit(a);
  check(clips.burned() == 1 && clips.rolls() == 1,
        "the commit is what burns the row and counts the roll");

  // The target's colour ends the episode, and stays ended: a solved cube must
  // survive the next read that flickers.
  Sight target = placed;
  target.color = 4;
  b.clear();
  for (int i = 0; i < cfg.belief_min_votes; ++i) b.push(target);
  clips.observe(b);
  check(clips.done() && clips.decide(b).mode == Mode::SETTLE,
        "the target ends it");
  b.clear();
  for (int i = 0; i < cfg.belief_min_votes; ++i) b.push(placed);
  clips.observe(b);
  check(clips.done() && clips.decide(b).mode == Mode::SETTLE,
        "and done is LATCHED — one bad read cannot roll a solved cube away");

  // The ladder steps on the voted colour, once per change however often it
  // runs.
  Clips fresh(t, cfg, 5);
  Belief b2(cfg);
  for (int i = 0; i < cfg.belief_min_votes; ++i) b2.push(placed);
  for (int i = 0; i < 10; ++i) fresh.observe(b2);
  check(fresh.rung() == 0 && fresh.tips() == 0,
        "the first colour is not a tip");
  Sight other = placed;
  other.color = 3;
  b2.clear();
  for (int i = 0; i < cfg.belief_min_votes; ++i) b2.push(other);
  for (int i = 0; i < 10; ++i) fresh.observe(b2);
  check(fresh.tips() == 1 && fresh.rung() == 1,
        "a colour change steps exactly one rung, however often observe() runs");
}

/// A preparatory turn changes the coordinates of every candidate but must not
/// change which physical entry stance the robot is pursuing. This exact pose
/// is a witness from the shipping R pool: unconstrained retrieval alternates
/// forever between two locally cheapest rows after ideal turns.
void check_approach_turn_continuity(const ClipTable& t) {
  std::puts("approach turn continuity");
  Cfg cfg = Cfg::v8_5();
  Clips clips(t, cfg, 4);
  Belief belief(cfg);
  constexpr float kPi = static_cast<float>(M_PI);
  const float range = 1.19f;
  const float world_bearing = 154.7f * kPi / 180.0f;
  const float world_phi = 100.9f * kPi / 180.0f;
  auto wrap = [](float a) { return std::atan2(std::sin(a), std::cos(a)); };
  auto see_at = [&](float robot_yaw) {
    Sight s;
    s.ok = true;
    s.color_ok = true;
    s.color = 2;
    const float b = wrap(world_bearing - robot_yaw);
    s.pos = {range * std::cos(b), range * std::sin(b), 0.3048f};
    s.phi = wrap(world_phi - robot_yaw);
    return s;
  };
  auto global_at = [&](float yaw) {
    belief.clear();
    const Sight s = see_at(yaw);
    for (int i = 0; i < cfg.belief_min_votes; ++i) belief.push(s);
    return clips.decide(belief);
  };

  float yaw = 0.0f;
  const Plan anchor = global_at(yaw);
  check(anchor.mode == Mode::CLIP && anchor.entry_translation > 0.25f &&
            std::fabs(anchor.entry_bearing) > 20.0f * kPi / 180.0f,
        "the witness requires a preparatory turn");

  Plan fixed = anchor;
  int turns = 0;
  for (; turns < cfg.approach_turn_max_attempts; ++turns) {
    fixed = clips.retarget(anchor, see_at(yaw));
    if (std::fabs(fixed.entry_bearing) <=
        cfg.approach_turn_max_deg * kPi / 180.0f)
      break;
    const float sweep = cfg.scan_sweep_deg * kPi / 180.0f;
    yaw = wrap(yaw + std::clamp(fixed.entry_bearing, -sweep, sweep));
  }
  check(turns <= 2 && fixed.row == anchor.row && fixed.sym == anchor.sym &&
            std::fabs(fixed.entry_bearing) < 1e-4f,
        "a locked row/sym converges after at most two ideal turns");
  check(std::fabs(fixed.entry_translation - anchor.entry_translation) < 1e-4f,
        "turning recomputes coordinates without inventing translation");

  const Plan switched = global_at(yaw);
  check(switched.row != anchor.row || switched.sym != anchor.sym,
        "the same final sight makes unconstrained retrieval switch targets");
}

/// CubeSight's square angle is modulo 90 degrees. Crossing the fold must
/// change only the integer symmetry label, not the physical entry heading.
void check_fold_aware_lock(const ClipTable& t) {
  std::puts("fold-aware candidate lock");
  Cfg cfg = Cfg::v9();
  Clips clips(t, cfg, 4);
  constexpr float kPi = static_cast<float>(M_PI);
  Sight before;
  before.ok = before.color_ok = true;
  before.color = 2;
  before.pos = {0.85f, 0.08f, 0.3048f};
  before.phi = 44.0f * kPi / 180.0f;
  Belief belief(cfg);
  for (int i = 0; i < cfg.belief_min_votes; ++i) belief.push(before);
  const Plan anchor = clips.decide(belief);
  const float target = anchor.entry_yaw;  // robot yaw is zero

  Sight after = before;
  after.phi = -44.0f * kPi / 180.0f;  // +2 deg physically, modulo 90
  const Plan fixed = clips.retarget_heading_locked(anchor, after, 0.0f, target);
  const float error = std::atan2(std::sin(fixed.entry_yaw - target),
                                 std::cos(fixed.entry_yaw - target));
  check(fixed.row == anchor.row && fixed.sym != anchor.sym &&
            std::fabs(error) < 3.0f * kPi / 180.0f,
        "a +/-45 degree fold changes symmetry without a 90 degree target jump");
}

/// THE v7 RESULT, asserted offline: a nominal stance has waist = 0, so the
/// torso is vertical and the camera reads back its mount angle. The library's
/// stand frame does not, which is why v5/v6 aimed at the near ground and saw
/// 44-47% side faces. No robot, no camera — three joint angles and the mount
/// quat.
void check_gaze() {
  std::puts("gaze");
  const Mount mount;  // the xml's, same defaults the node falls back to
  const std::array<float, 4> level{1.f, 0.f, 0.f, 0.f};
  auto look = [&](float yaw, float roll, float pitch) {
    return gaze_of(camera_pose(level, {yaw, roll, pitch}, mount));
  };

  const Gaze nom = look(0.f, 0.f, 0.f);
  std::printf("       nominal (waist 0)      %5.1f deg down  %+6.1f off-axis\n",
              nom.pitch_deg, nom.yaw_deg);
  check(std::fabs(nom.pitch_deg - 45.0f) < 0.5f,
        "a nominal stance recovers the 45 deg mount angle");
  check(std::fabs(nom.yaw_deg) < 0.5f, "and looks straight ahead");

  // Library stand frame 12953, the pose v5/v6 held. Measured off the baked row.
  const Gaze lib = look(-0.2435f, -0.0780f, 0.4666f);
  std::printf("       library frame 12953    %5.1f deg down  %+6.1f off-axis\n",
              lib.pitch_deg, lib.yaw_deg);
  check(lib.pitch_deg > 65.0f, "the library stand aims at the near ground");
  check(std::fabs(lib.yaw_deg) > 20.0f,
        "and its waist yaw skews the search window too");

  // The gaze must be a pure function of the waist: any base tilt the IMU
  // reports is already removed before perception, so it cannot enter here.
  const float h = 0.5f * 0.4f;
  const Gaze yawed = gaze_of(camera_pose({std::cos(h), 0.f, 0.f, std::sin(h)},
                                         {0.f, 0.f, 0.f}, mount));
  check(std::fabs(yawed.pitch_deg - nom.pitch_deg) < 1e-3f &&
            std::fabs(yawed.yaw_deg - nom.yaw_deg) < 1e-3f,
        "heading is removed before the gaze, so a turned robot sees the same");
}

/// v7's still pose must BE the nominal stance and nothing else: the sim's
/// `_fk_nominal` channel for channel, so a sim result transfers.
void check_nominal_stand(ClipTable& t) {
  std::puts("nominal stand");
  std::vector<float> nominal(g1::NUM_JOINTS, 0.0f);
  for (size_t i = 0; i < g1::MJ_JOINTS.size(); ++i)
    nominal[i] =
        0.1f * static_cast<float>(i % 3);  // any stance, waist included
  for (int i : g1::WAIST_JOINT_INDICES) nominal[i] = 0.0f;

  t.set_nominal_stand(nominal);
  const float* s = t.stand_row();
  const int cols = t.cols(), J = g1::NUM_JOINTS;

  float dq = 0.0f;
  for (int j = 0; j < J; ++j)
    dq = std::max(dq, std::fabs(s[j] - nominal[g1::IL2MJ[j]]));
  check(dq < 1e-6f, "joints ARE the nominal stance, IL-ordered");

  float rest = 0.0f;
  for (int c = J; c < cols; ++c)
    if (c != 2 * J + 3) rest = std::max(rest, std::fabs(s[c]));
  check(rest < 1e-6f,
        "velocity, root position, twist and contact are all zero");
  check(
      std::fabs(s[2 * J + 3] - 1.0f) < 1e-6f,
      "the anchor quat is identity — upright, and engage rebases the heading");
}

/// The warp must be exactly the heading term the retrieval minimises, or the
/// planner is ranking one thing and commanding another.
void check_warp_identity() {
  std::puts("warp");
  const float qth = 0.31f, phi = -0.12f;
  for (int sym = 0; sym < 4; ++sym) {
    const float a = sym * static_cast<float>(M_PI) / 2.0f;
    const float dth =
        std::atan2(std::sin(qth + a + phi), std::cos(qth + a + phi));
    const float entry_yaw =
        std::atan2(std::sin(qth + a + phi), std::cos(qth + a + phi));
    check(std::fabs(dth - entry_yaw) < 1e-6f,
          "sym " + std::to_string(sym) +
              ": entry_yaw == the cost's heading term");
  }
}

/// A still mode's rows must be identical apart from the yaw ramp, and the ramp
/// must be over before the frames the decision is read from.
void check_still_rows(const ClipTable& t, const Cfg& cfg) {
  std::puts("still rows");
  ReferenceWriter w(t, cfg);
  Plan p;
  p.mode = Mode::SCAN;
  p.yaw_offset = 1.2f;
  p.frames = cfg.scan_steps;
  LiveState live;
  live.joint_pos_il.assign(g1::NUM_JOINTS, 0.0f);
  live.joint_vel_il.assign(g1::NUM_JOINTS, 0.0f);
  const auto& rows = w.build(p, live);
  const int cols = t.cols(), lead = w.lead_in_frames();
  const int held = w.frames() - lead;
  check(held > 2 * cfg.hold_tail && (cfg.smooth_scan || held == cfg.scan_steps),
        "the turn is followed by its full quiet read tail");

  // Only the yaw RELATIVE to frame 0 is a command: MotionClock::engage aligns
  // frame 0 onto the robot, so the stand pose's own recorded heading cancels.
  // The sweep is over the HELD frames — the lead-in holds the entry heading.
  const int aq = 2 * g1::NUM_JOINTS + 3;
  auto yaw_of = [&](int f) {
    const float* q = &rows[static_cast<size_t>(lead + f) * cols + aq];
    return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  };
  auto swept = [&](int f) {
    return std::atan2(std::sin(yaw_of(f) - yaw_of(0)),
                      std::cos(yaw_of(f) - yaw_of(0)));
  };
  const int ramp = held - 2 * cfg.hold_tail;
  check(std::fabs(swept(0)) < 1e-5f, "frame 0 is the zero of the sweep");
  check(std::fabs(swept(ramp) - p.yaw_offset) < 1e-3f, "the sweep completes");
  check(std::fabs(swept(held - 1) - p.yaw_offset) < 1e-3f,
        "and holds through the read tail");
  check(swept(ramp / 2) > 0.1f && swept(ramp / 2) < p.yaw_offset,
        "and is monotone across the window, not a step");
  const int aang = g1::WIRE_COLS_MIN + 3;
  auto ang_z = [&](int f) {
    return rows[static_cast<size_t>(lead + f) * cols + aang + 2];
  };
  check(ang_z(ramp / 2) > 0.0f, "a turning reference commands a turn rate");
  check(std::fabs(ang_z(held - 1)) < 1e-6f, "and zero once it is done turning");
  if (cfg.smooth_scan) {
    float peak = 0.0f;
    for (int f = 0; f < held; ++f) peak = std::max(peak, std::fabs(ang_z(f)));
    check(peak <=
              cfg.scan_yaw_rate_deg * static_cast<float>(M_PI) / 180.0f + 1e-4f,
          "the planner turn obeys the stand-yaw rate bound");
    check(std::fabs(ang_z(0)) < 1e-6f,
          "the planner turn starts at zero yaw rate");
  }
  float head = 0.0f;
  for (int j = 0; j < g1::NUM_JOINTS; ++j)
    head = std::max(head, std::fabs(rows[j] - live.joint_pos_il[j]));
  check(head < 1e-5f,
        "row 0 IS the live pose — a still commit ramps, not steps");
}

/// The lead-in must be continuous with the live pose and land on the clip.
void check_lead_in(const ClipTable& t, const Cfg& cfg) {
  std::puts("lead-in");
  if (t.pool('R').empty()) {
    check(false, "table has an R pool");
    return;
  }
  ReferenceWriter w(t, cfg);
  Plan p;
  p.mode = Mode::CLIP;
  p.row = t.pool('R').front();
  p.frames = t.rows()[p.row].span_len;
  LiveState live;
  live.joint_pos_il.assign(g1::NUM_JOINTS, 0.35f);
  live.joint_vel_il.assign(g1::NUM_JOINTS, 0.0f);
  const auto& rows = w.build(p, live);
  const int cols = t.cols(), lead = w.lead_in_frames();
  check(lead >= 2, "a clip commit ramps rather than steps");
  float head = 0.0f, tail = 0.0f;
  const float* span = t.span(p.row);
  for (int j = 0; j < g1::NUM_JOINTS; ++j) {
    head = std::max(head, std::fabs(rows[j] - live.joint_pos_il[j]));
    tail = std::max(
        tail,
        std::fabs(rows[static_cast<size_t>(lead - 1) * cols + j] - span[j]));
  }
  check(head < 1e-5f, "row 0 IS the live pose");
  check(tail < 1e-5f, "the last lead-in row IS the clip's entry pose");
  check(w.frames() == lead + p.frames, "clip follows the ramp intact");
}

/// v7.1: the heading joins the ramp, the seam is C1, and 0 is v7 exactly.
void check_enter_ramp(const ClipTable& t) {
  std::puts("enter ramp (v7.1)");
  if (t.pool('R').empty()) {
    check(false, "table has an R pool");
    return;
  }
  Plan p;
  p.mode = Mode::CLIP;
  p.row = t.pool('R').front();
  p.frames = t.rows()[p.row].span_len;
  p.entry_yaw = 0.60f;  // 34 deg: between the measured median and p90
  LiveState live;
  live.joint_pos_il.assign(g1::NUM_JOINTS, 0.35f);
  live.joint_vel_il.assign(g1::NUM_JOINTS, 0.0f);

  // 1. the knob at zero is v7, byte for byte — the new paths short-circuit.
  Cfg off = Cfg::v7_1();
  off.enter_yaw_rate_deg = 0.0f;
  off.lead_in_max_s = Cfg::v7().lead_in_max_s;
  ReferenceWriter wv7(t, Cfg::v7()), woff(t, off);
  const std::vector<float> a = wv7.build(p, live), b = woff.build(p, live);
  check(a == b && !woff.ramped(),
        "enter_yaw_rate_deg = 0 IS v7, byte for byte");

  const Cfg cfg = Cfg::v7_1();
  ReferenceWriter w(t, cfg);
  const auto& rows = w.build(p, live);
  const int cols = t.cols(), lead = w.lead_in_frames(), J = g1::NUM_JOINTS;
  const float* span = t.span(p.row);
  check(w.ramped() && lead > 2, "a clip commit ramps its heading");
  check(w.frames() == lead + p.frames, "clip follows the ramp intact");

  // 2. the heading is swept, not stepped. Only yaw RELATIVE to frame 0 is a
  // command — engage() aligns frame 0 onto the robot, cancelling the rest.
  const int aq = 2 * J + 3, alin = g1::WIRE_COLS_MIN, aang = alin + 3;
  auto yaw_at = [&](int f) {
    const float* q = &rows[static_cast<size_t>(f) * cols + aq];
    return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  };
  auto swept = [&](int f) {
    return std::atan2(std::sin(yaw_at(f) - yaw_at(0)),
                      std::cos(yaw_at(f) - yaw_at(0)));
  };
  check(std::fabs(swept(lead - 1) - p.entry_yaw) < 1e-3f,
        "the ramp lands on entry_yaw, so the clip plays where v7 put it");
  check(std::fabs(swept(lead) - p.entry_yaw) < 1e-3f,
        "and the clip's own rows carry it on unchanged");
  check(swept(lead / 2) > 0.1f && swept(lead / 2) < p.entry_yaw,
        "monotone across the window, not a step");

  // 3. both bounds hold, and they are PEAK rates.
  float peak_yaw = 0.0f, peak_dq = 0.0f;
  for (int f = 0; f + 1 < lead; ++f) {
    peak_yaw = std::max(peak_yaw, std::fabs(swept(f + 1) - swept(f)) * t.fps());
    for (int j = 0; j < J; ++j)
      peak_dq = std::max(peak_dq,
                         std::fabs(rows[static_cast<size_t>(f + 1) * cols + j] -
                                   rows[static_cast<size_t>(f) * cols + j]) *
                             t.fps());
  }
  const float w_max = cfg.enter_yaw_rate_deg * 3.14159265f / 180.0f;
  check(peak_yaw <= w_max * 1.02f,
        "peak commanded yaw rate is inside enter_yaw_rate_deg");
  check(peak_dq <= cfg.enter_joint_rate * 1.02f,
        "peak commanded joint rate is inside enter_joint_rate");
  check(peak_yaw > 0.4f * w_max || peak_dq > 0.4f * cfg.enter_joint_rate,
        "and one of them is actually binding — the ramp is not padded");

  // 4. C1 at both ends: the whole point.
  float head = 0.0f, tail = 0.0f, dv = 0.0f;
  for (int j = 0; j < J; ++j) {
    head = std::max(head, std::fabs(rows[j] - live.joint_pos_il[j]));
    const float* last = &rows[static_cast<size_t>(lead - 1) * cols];
    tail = std::max(tail, std::fabs(last[j] - span[j]));
    dv = std::max(dv, std::fabs(last[J + j] - span[J + j]));
  }
  check(head < 1e-5f, "row 0 IS the live pose");
  check(tail < 1e-5f, "the last ramp row IS the clip's entry pose");
  // The position path is MONOTONE, and that is the binding constraint: a clip
  // span is sliced mid-motion, so its entry velocity (this alphabet: med 4.8,
  // p90 7.1 rad/s) can only be matched by winding backwards first. The ramp
  // arrives moving as far as it can without doing that, and no further.
  float over = 0.0f, step_v7 = 0.0f;
  for (int j = 0; j < J; ++j) {
    step_v7 = std::max(step_v7, std::fabs(span[J + j]));
    const float lo = std::min(live.joint_pos_il[j], span[j]);
    const float hi = std::max(live.joint_pos_il[j], span[j]);
    for (int f = 0; f < lead; ++f) {
      const float x = rows[static_cast<size_t>(f) * cols + j];
      over = std::max(over, std::max(lo - x, x - hi));
    }
  }
  check(over < 1e-4f, "the ramp never overshoots either endpoint");
  check(dv < 0.95f * step_v7, "and arrives moving, where v7 arrived from rest");
  check(std::fabs(rows[J]) < 1e-5f, "the ramp starts from rest, as a still is");

  // The ramp's root POSITION is pinned at the entry anchor, so the only twist
  // it may command is its own yaw sweep — and that returns to zero at both
  // ends.
  float twist = 0.0f, yend = 0.0f;
  for (int f = 0; f < lead; ++f) {
    const float* row = &rows[static_cast<size_t>(f) * cols];
    for (int k = alin; k < aang + 3; ++k)
      if (k != aang + 2) twist = std::max(twist, std::fabs(row[k]));
    if (f == 0 || f == lead - 1)
      yend = std::max(yend, std::fabs(row[aang + 2]));
  }
  check(twist < 1e-6f, "the ramp commands no root twist but its own turn");
  check(yend < 1e-5f, "and that turn rate is zero at both ends of the sweep");

  // The split (v7.1): the ramp is one act, the clip is another, so the clip
  // re-engages on the pose the ramp reached instead of dead reckoning from the
  // pose it started at. That is the whole fix — an anchor stamped 1.2 s late.
  ReferenceWriter we(t, cfg), wc(t, cfg);
  const auto& er = we.build(p, live, ReferenceWriter::Stage::ENTER);
  check(we.ramped() && we.frames() == lead &&
            er.size() == static_cast<size_t>(lead) * cols,
        "the ENTER stage is the ramp and nothing else");
  check(std::memcmp(er.data(), rows.data(),
                    static_cast<size_t>(lead) * cols * sizeof(float)) == 0,
        "and is the same ramp the one-shot build lays down");
  const auto& cr = wc.build(p, live, ReferenceWriter::Stage::CLIP);
  check(!wc.ramped() && wc.frames() == p.frames &&
            std::memcmp(cr.data(), span,
                        static_cast<size_t>(p.frames) * cols * sizeof(float)) ==
                0,
        "the CLIP stage is the raw span — no lead-in, no blend");

  // The ramp starts from what the controller was TOLD, not from where it is: C0
  // with the still it leaves, whatever tracking error that still was carrying.
  LiveState held = live;
  held.held_row.assign(t.stand_row(), t.stand_row() + cols);
  ReferenceWriter wh(t, cfg);
  const auto& hr = wh.build(p, held, ReferenceWriter::Stage::ENTER);
  float d0 = 0.0f;
  for (int j = 0; j < J; ++j)
    d0 = std::max(d0, std::fabs(hr[j] - t.stand_row()[j]));
  check(d0 < 1e-5f, "a held row, when given, is where the ramp starts");

  // 5. a still is untouched by the ramp: v7.1 is a clip-entry change only.
  Plan q = p;
  q.mode = Mode::SETTLE;
  q.frames = cfg.settle_steps;
  q.entry_yaw = 0.0f;
  ReferenceWriter ws(t, cfg), wr(t, Cfg::v7());
  check(ws.build(q, live) == wr.build(q, live) && !ws.ramped(),
        "a still builds the same rows under v7 and v7.1");
}

void check_table(const ClipTable& t) {
  std::puts("table");
  check(!t.rows().empty(), "rows loaded");
  check(t.cols() == g1::WIRE_COLS_FULL, "rows are in MotionReference layout");
  check(t.half_extent() > 0.1f && t.fps() > 1.0f,
        "half_extent and fps are sane");
  size_t pooled = 0;
  for (char d : std::string("FBLR")) pooled += t.pool(d).size();
  check(pooled > 0, "at least one delta has a clean pool");
  if (!t.has_frames()) return;
  int worst = 0;
  for (const auto& r : t.rows())
    if (r.clean && r.span_len != r.exit - r.entry + 1) ++worst;
  check(worst == 0, "every clean row's span matches its [entry, exit]");
}

void check_approach_source(const std::string& path, const ClipTable& t,
                           const Cfg& cfg) {
  std::puts("approach source");
  ApproachSource source(path, cfg);
  check(!source.windows().empty(), "the fixed walk yields distance windows");
  bool monotone = true;
  for (size_t i = 1; i < source.windows().size(); ++i)
    monotone = monotone &&
               source.windows()[i].distance > source.windows()[i - 1].distance;
  check(monotone, "approach windows increase monotonically in covered metres");
  if (cfg.approach_min_window_m > 0.0f)
    check(
        source.windows().front().distance + 1e-6f >= cfg.approach_min_window_m,
        "v9.2 removes every incomplete approach window");

  Plan clip;
  clip.mode = Mode::CLIP;
  clip.cost = 0.56f;
  clip.entry_translation = 0.556f;  // the failed R#53 hardware candidate
  clip.entry_bearing = -3.2f * static_cast<float>(M_PI) / 180.0f;
  clip.entry_forward = clip.entry_translation * std::cos(clip.entry_bearing);
  clip.entry_lateral = clip.entry_translation * std::sin(clip.entry_bearing);
  const Plan p = source.plan(clip);
  check(p.mode == Mode::APPROACH &&
            p.approach_requested <= cfg.approach_max_step_m + 1e-6f,
        "the failed hardware residual becomes one bounded approach leg");
  check(p.approach_covered > 0.0f && p.frames > 1,
        "a real unscaled source window supplies the command");
  if (cfg.approach_escalate_window) {
    const Plan retry = source.plan(clip, p.approach_window + 1);
    check(retry.approach_window > p.approach_window &&
              retry.approach_covered > p.approach_covered,
          "one failed complete gait escalates to the next longer window");
    const Plan longest = source.plan(clip, 1000000);
    check(longest.approach_window ==
              static_cast<int>(source.windows().size()) - 1,
          "escalation past the vocabulary clamps to its longest safe gait");
  }

  Plan near = clip;
  near.entry_translation = 0.24f;
  near.entry_forward = 0.24f;
  near.entry_lateral = 0.01f;
  check(approach_ready(near, cfg),
        "the observed 0.24 m residual goes directly to manipulation");
  Plan behind = clip;
  behind.entry_forward = -0.30f;
  behind.entry_lateral = 0.0f;
  check(
      !approach_ready(behind, cfg) && !approach_forward_reachable(behind, cfg),
      "a forward-only gait refuses an entry stance behind the robot");

  LiveState live;
  live.joint_pos_il.assign(g1::NUM_JOINTS, 0.0f);
  live.joint_vel_il.assign(g1::NUM_JOINTS, 0.0f);
  ReferenceWriter writer(t, cfg, &source);
  const auto& rows = writer.build(p, live);
  const int lead = writer.lead_in_frames(), cols = writer.cols();
  check(writer.frames() == lead + p.frames,
        "the approach follows its live-to-walk lead-in intact");

  const g1::Motion source_motion = g1::Motion::from_npz(path, g1::MJ2IL, false);
  std::array<float, g1::NUM_JOINTS> expected_jp{};
  source_motion.jp_il(source.onset(), expected_jp.data());
  float joint_order_error = 0.0f;
  const float* first_walk = &rows[static_cast<size_t>(lead) * cols];
  for (int j = 0; j < g1::NUM_JOINTS; ++j)
    joint_order_error =
        std::max(joint_order_error, std::fabs(first_walk[j] - expected_jp[j]));
  check(joint_order_error < 1e-6f,
        "the IL source survives MJ storage and returns in IL wire order");

  float contacts = 0.0f, twist = 0.0f;
  for (int f = lead; f < writer.frames(); ++f) {
    const float* row = &rows[static_cast<size_t>(f) * cols];
    for (int k = 0; k < 6; ++k)
      twist = std::max(twist, std::fabs(row[g1::WIRE_COLS_MIN + k]));
    for (int k = 0; k < g1::NUM_CONTACT_BODIES; ++k)
      contacts = std::max(contacts, std::fabs(row[g1::WIRE_COLS_MIN + 6 + k]));
  }
  check(twist > 0.01f, "the walking reference carries truthful root twist");
  check(contacts == 0.0f, "an approach commands zero robot-to-object contacts");

  const g1::Motion motion = g1::Motion::from_wire(writer.frames(), rows.data(),
                                                  cols, true, true, t.fps());
  g1::MotionClock clock(motion, 0);
  clock.engage({1.0f, 0.0f, 0.0f, 0.0f}, 0,
               writer.ramped() ? 0.0f : p.entry_yaw);
  const auto end = clock.aligned_root_pos(writer.frames() - 1);
  const float distance = std::hypot(end[0], end[1]);
  const float bearing = std::atan2(end[1], end[0]);
  check(std::fabs(distance - p.approach_covered) < 1e-4f,
        "the aligned reference covers the selected metric distance");
  check(std::fabs(std::atan2(std::sin(bearing - clip.entry_bearing),
                             std::cos(bearing - clip.entry_bearing))) < 1e-4f,
        "and its net displacement points at the candidate entry bearing");
  if (cfg.enter_yaw_rate_deg > 0.0f) {
    writer.build(p, live, ReferenceWriter::Stage::ENTER);
    check(writer.ramped() && writer.frames() == writer.lead_in_frames(),
          "v9 APPROACH publishes its heading/joint ramp as a separate act");
    writer.build(p, live, ReferenceWriter::Stage::CLIP);
    check(!writer.ramped() && writer.lead_in_frames() == 0 &&
              writer.frames() == p.frames,
          "and re-engages the bare walk after that ramp finishes");
  }
}

// ── Replay rig ───────────────────────────────────────────────────
//
// Each npz holds one recorded read: bgr (H,W,3) uint8, depth (h,w) float32
// metres, intrinsics (4,), R_bc (9,), t_bc (3,). Dump them from the sim or from
// a standing robot; the point is that C++ and the Python oracle see the SAME
// frames.
//
// An optional int32 `label` (0-5, or -1 for a negative/no-cube frame) turns the
// rig into a SCORER: the number printed is the production read's, under the
// production config, so a tuning sweep cannot score itself against a second
// implementation that has quietly drifted.

int replay(const std::string& dir, ClipTable& t, const Cfg& cfg, bool summary) {
  std::vector<std::string> files;
  for (int i = 0; i < 10000; ++i) {
    char name[512];
    std::snprintf(name, sizeof(name), "%s/read_%04d.npz", dir.c_str(), i);
    if (!std::ifstream(name).good()) break;
    files.emplace_back(name);
  }
  if (files.empty()) {
    std::printf("replay: no read_XXXX.npz under %s\n", dir.c_str());
    return 1;
  }
  Clips clips(t, cfg, 4);
  CubeSight eye(cfg, t.half_extent());  // palette rides in the cfg now
  Belief belief(cfg);
  int conf[7][7] = {};  // [label + 1][answer + 1], index 0 is "no cube"/"none"
  bool scored = false;
  int color_reads = 0, pose_reads = 0, action_reads = 0;
  int pose_on_positive = 0, joint_right = 0;
  if (!summary) std::printf("replay: %zu reads\n", files.size());
  for (const auto& f : files) {
    cnpy::npz_t z = cnpy::npz_load(f);
    const auto& bgr_a = z.at("bgr");
    const auto& depth_a = z.at("depth");
    const cv::Mat bgr(static_cast<int>(bgr_a.shape[0]),
                      static_cast<int>(bgr_a.shape[1]), CV_8UC3,
                      const_cast<uint8_t*>(bgr_a.data<uint8_t>()));
    const cv::Mat depth(static_cast<int>(depth_a.shape[0]),
                        static_cast<int>(depth_a.shape[1]), CV_32FC1,
                        const_cast<float*>(depth_a.data<float>()));
    const float* k = z.at("intrinsics").data<float>();
    const float* R = z.at("R_bc").data<float>();
    const float* tr = z.at("t_bc").data<float>();
    CameraPose cam;
    std::copy(R, R + 9, cam.R.begin());
    std::copy(tr, tr + 3, cam.t.begin());
    // Every recorded read is a standing robot, so all of them are quiescent by
    // construction — the rig feeds the belief the way the node's gate would.
    const Sight s = eye(bgr, depth, {k[0], k[1], k[2], k[3]}, cam);
    color_reads += s.color_ok;
    pose_reads += s.ok;
    action_reads += s.color_ok && s.ok;
    if (z.count("label")) {
      scored = true;
      const int lab = *z.at("label").data<int32_t>();
      const int ans = s.color_ok ? s.color : -1;
      if (lab >= -1 && lab < 6 && ans >= -1 && ans < 6)
        ++conf[lab + 1][ans + 1];
      if (lab >= 0) {
        pose_on_positive += s.ok;
        joint_right += s.ok && s.color_ok && s.color == lab;
      }
    }
    belief.push(s);
    clips.observe(belief);
    const Plan p = clips.decide(belief);
    if (!summary)
      std::printf(
          "%-24s %-10s c%d pos %+.4f %+.4f %+.4f phi %+.5f | vote %d/%d "
          "c%d | %s cost %.3f yaw %+.4f row %d sym %d\n",
          f.substr(f.find_last_of('/') + 1).c_str(), s.reason.c_str(), s.color,
          s.pos[0], s.pos[1], s.pos[2], s.phi, belief.n_votes(),
          belief.n_reads(), belief.color(), p.label.c_str(), p.cost,
          p.entry_yaw, p.row, p.sym);
  }
  std::printf(
      "\nOBSERVE frames %zu  color_ok %d  pose_ok %d  action_ready %d\n",
      files.size(), color_reads, pose_reads, action_reads);
  if (!scored) return 0;

  static const char* kName[6] = {"red",    "orange", "green",
                                 "yellow", "blue",   "pink"};
  int right = 0, n_pos = 0;
  for (int c = 0; c < 6; ++c)
    for (int a = -1; a < 6; ++a) {
      n_pos += conf[c + 1][a + 1];
      if (a == c) right += conf[c + 1][a + 1];
    }
  int n_neg = 0, fp = 0;
  for (int a = -1; a < 6; ++a) {
    n_neg += conf[0][a + 1];
    if (a >= 0) fp += conf[0][a + 1];
  }
  std::printf("\n%-8s", "exp\\got");
  for (int a = 0; a < 6; ++a) std::printf("%8s", kName[a]);
  std::printf("%8s%8s%8s\n", "none", "n", "recall");
  for (int c = 0; c < 6; ++c) {
    int n = 0;
    for (int a = -1; a < 6; ++a) n += conf[c + 1][a + 1];
    if (!n) continue;
    std::printf("%-8s", kName[c]);
    for (int a = 0; a < 6; ++a) std::printf("%8d", conf[c + 1][a + 1]);
    std::printf("%8d%8d%7.1f%%\n", conf[c + 1][0], n,
                100.0 * conf[c + 1][c + 1] / n);
  }
  if (n_neg) {
    std::printf("%-8s", "no cube");
    for (int a = 0; a < 6; ++a) std::printf("%8d", conf[0][a + 1]);
    std::printf("%8d%8d%7.1f%%\n", conf[0][0], n_neg,
                100.0 * conf[0][0] / n_neg);
  }
  // The one line a sweep reads back.
  std::printf("\nSCORE accuracy %.4f (%d/%d)  false_positives %d/%d\n",
              n_pos ? static_cast<double>(right) / n_pos : 0.0, right, n_pos,
              fp, n_neg);
  std::printf("POSE pose_ok %.4f (%d/%d)  joint_color_pose %.4f (%d/%d)\n",
              n_pos ? static_cast<double>(pose_on_positive) / n_pos : 0.0,
              pose_on_positive, n_pos,
              n_pos ? static_cast<double>(joint_right) / n_pos : 0.0,
              joint_right, n_pos);
  return 0;
}

/// The SHIPPED yaml must reproduce the preset it names, field for field.
/// This is what makes "the config refactor changed nothing" checkable: the
/// schema can grow, but the day a key is renamed and its reader is not, this
/// fails instead of the robot quietly running stock numbers.
void check_config_roundtrip(const std::string& path, bool parity) {
  std::puts("config");
  const YAML::Node root = YAML::LoadFile(path);
  const std::string named =
      root["version"] ? root["version"].as<std::string>() : "v7";
  check(!threw([&] { Cfg::from_yaml(root); }),
        "the config loads and validates");
  const Cfg loaded = Cfg::from_yaml(root);
  if (named == "v9.1" || named == "v9.2") {
    check(loaded.plane_color_pool_fallback && loaded.search_left_first &&
              loaded.omega_still == 0.25f && loaded.settle_steps == 60 &&
              loaded.min_visible == 0.55f,
          "the shipped v9.1 hardware-evidence knobs survive yaml parsing");
  }
  if (named == "v9.2") {
    check(loaded.approach_enter_forward_m == 0.25f &&
              loaded.approach_min_window_m == 0.30f &&
              loaded.approach_no_progress_limit == 2 &&
              loaded.approach_escalate_window && loaded.approach_forward_only &&
              loaded.approach_latch_blocked,
          "the shipped v9.2 bounded-approach knobs survive yaml parsing");
  }

  // Parity is asserted only for a file that CLAIMS to be untuned — the shipped
  // base. A deployment config exists precisely to differ from its preset, and
  // asserting otherwise would make tuning a test failure.
  if (parity) {
    check(loaded == Cfg::preset(named),
          "the shipped yaml round-trips to the preset it names");
    // V6--v9 remain one-line ablations of the base file. V9.1 deliberately
    // changes explicit observation thresholds, so its own shipped file has a
    // separate full-parity test.
    for (const char* v : {"v6", "v7", "v7.1", "v8", "v8.5", "v9"}) {
      YAML::Node n = YAML::Clone(root);
      n["version"] = v;
      check(Cfg::from_yaml(n) == Cfg::preset(v),
            std::string("...and so does ") + v);
    }
  }
  check(threw([&] {
          YAML::Node n = YAML::Clone(root);
          n["version"] = "v10";
          Cfg::from_yaml(n);
        }),
        "an unknown version is refused rather than defaulted");

  // ...and the schema is strict, or "tuned" and "ignored" look the same.
  YAML::Node typo = YAML::Clone(root);
  typo["observe"]["gates"]["min_rel_saturation"] = 0.4;
  check(threw([&] { Cfg::from_yaml(typo); }),
        "a misspelt knob is an error, not a silent no-op");

  YAML::Node legacy;
  legacy["knobs"]["min_rel_sat"] = 0.4;
  check(threw([&] { Cfg::from_yaml(legacy); }),
        "the pre-split flat 'knobs:' schema is refused by name");

  // One override reaches the struct — the whole point of the file.
  YAML::Node tuned = YAML::Clone(root);
  tuned["observe"]["palette"]["blue"]["lit"] = std::vector<float>{1, 181, 255};
  tuned["observe"]["palette"]["blue"]["shaded"] =
      std::vector<float>{10, 10, 51};
  check(Cfg::from_yaml(tuned).palette[24] == 1.0f &&
            Cfg::from_yaml(tuned).palette[25] == 181.0f,
        "a palette row written in yaml is the row the classifier gets");
}

}  // namespace

int main(int argc, char** argv) {
  std::string table_path, frames_path, library_path, bake_path, replay_dir,
      config_path, approach_path, version = "v7";  // v6 is the gaze ablation
  bool summary = false, check_parity = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--table")
      table_path = next();
    else if (a == "--frames")
      frames_path = next();
    else if (a == "--library")
      library_path = next();
    else if (a == "--bake")
      bake_path = next();
    else if (a == "--replay")
      replay_dir = next();
    else if (a == "--version")
      version = next();
    else if (a == "--config")
      config_path = next();
    else if (a == "--approach")
      approach_path = next();
    else if (a == "--summary")
      summary = true;
    else if (a == "--check-preset-parity")
      check_parity = true;
    else {
      std::printf("unknown argument '%s'\n", a.c_str());
      return 2;
    }
  }

  // A config file wins over --version: the whole point of the replay scorer is
  // that it reads the file a run would actually deploy.
  const Cfg cfg = config_path.empty()
                      ? Cfg::preset(version)
                      : Cfg::from_yaml(YAML::LoadFile(config_path));
  check_cfg();
  check_belief();
  check_plane_read();
  check_gaze();
  check_warp_identity();
  if (!config_path.empty()) check_config_roundtrip(config_path, check_parity);

  if (table_path.empty()) {
    std::printf(
        "\n%d failure(s). Pass --table <sys1_clips.npz> for the rest.\n",
        failures);
    return failures ? 1 : 0;
  }

  ClipTable table = ClipTable::load(table_path);
  if (!frames_path.empty()) {
    const bool baked = frames_path.size() > 4 &&
                       frames_path.rfind(".npz") == frames_path.size() - 4;
    if (baked) {
      table.load_frames_baked(frames_path);
    } else if (library_path.empty()) {
      std::puts("--frames <dir> also needs --library <sys1_library.npz>");
      return 2;
    } else {
      table.load_frames_retargeted(library_path, frames_path);
    }
  }
  check_table(table);
  // BEFORE the checks below, which overwrite the stand row: a bundle carries
  // the library's pose and nothing else. v7's stance is per-robot and arrives
  // at runtime from the controller's manifest, so the bundle stays
  // version-agnostic.
  if (!bake_path.empty()) {
    table.bake(bake_path);
    std::printf("baked -> %s\n", bake_path.c_str());
  }
  if (table.has_frames()) {
    // Deliberately BEFORE the two below, so they exercise the still pose the
    // configured version actually ships.
    if (cfg.nominal_stand) check_nominal_stand(table);
    check_still_rows(table, cfg);
    check_lead_in(table, cfg);
    check_enter_ramp(table);
    check_decide_is_pure(table);
    check_approach_turn_continuity(table);
    check_fold_aware_lock(table);
    if (!approach_path.empty())
      check_approach_source(approach_path, table, cfg);
  }
  if (!replay_dir.empty() && replay(replay_dir, table, cfg, summary) != 0)
    ++failures;

  std::printf("\n%d failure(s)\n", failures);
  return failures ? 1 : 0;
}
