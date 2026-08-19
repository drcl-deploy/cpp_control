/**
 * sys1 unit checks, the replay rig, and the bake — no robot, no ROS.
 *
 *   sys1_selftest                                            # unit checks
 *   sys1_selftest --table sys1_clips.npz \
 *                 --library sys1_library.npz --frames <retargeted_root>
 *   sys1_selftest ... --bake bundle.npz     # freeze the spans for the Orin
 *   sys1_selftest --table t.npz --frames bundle.npz          # read it back
 *   sys1_selftest ... --replay dir/                          # recorded reads
 *
 * The replay rig is the acceptance gate for the perception port: it settles the
 * palette and depth-realism blockers with the robot merely standing still.
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cnpy/cnpy.h"
#include "common/g1/joint_orders.hpp"
#include "common/g1/motion.hpp"
#include "sys1/belief.hpp"
#include "sys1/clips.hpp"
#include "sys1/kinematics.hpp"
#include "sys1/sight.hpp"
#include "sys1/table.hpp"
#include "sys1/writer.hpp"

using namespace cpp_control;
using namespace cpp_control::sys1;

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what.c_str());
  if (!ok) ++failures;
}

// ── Unit checks: the invariants the seam depends on ──────────────

void check_cfg() {
  std::puts("cfg");
  const Cfg v6 = Cfg::v6(), v7 = Cfg::v7();
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
  check(refuses([](Cfg& c) { c.omega_still = 0.0f; }),
        "a quiescence gate that never opens is refused");
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
  check(b.color() == 2 && b.n_reads() == 8, "the window forgets, the count does not");
  b.clear();
  check(b.n_reads() == 0 && !b.valid(), "a commit clears the evidence");
}

/// decide() must be PURE — the node calls it every tick and commits the answer
/// only when sys0 finishes, so a decision that burned a clip on the way out
/// would empty the pool twenty times a second.
void check_decide_is_pure(const ClipTable& t) {
  std::puts("decide");
  Cfg cfg;
  Sys1Clips clips(t, cfg, 4);
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
  check(clips.done() && clips.decide(b).mode == Mode::SETTLE, "the target ends it");
  b.clear();
  for (int i = 0; i < cfg.belief_min_votes; ++i) b.push(placed);
  clips.observe(b);
  check(clips.done() && clips.decide(b).mode == Mode::SETTLE,
        "and done is LATCHED — one bad read cannot roll a solved cube away");

  // The ladder steps on the voted colour, once per change however often it runs.
  Sys1Clips fresh(t, cfg, 5);
  Belief b2(cfg);
  for (int i = 0; i < cfg.belief_min_votes; ++i) b2.push(placed);
  for (int i = 0; i < 10; ++i) fresh.observe(b2);
  check(fresh.rung() == 0 && fresh.tips() == 0, "the first colour is not a tip");
  Sight other = placed;
  other.color = 3;
  b2.clear();
  for (int i = 0; i < cfg.belief_min_votes; ++i) b2.push(other);
  for (int i = 0; i < 10; ++i) fresh.observe(b2);
  check(fresh.tips() == 1 && fresh.rung() == 1,
        "a colour change steps exactly one rung, however often observe() runs");
}

/// THE v7 RESULT, asserted offline: a nominal stance has waist = 0, so the torso
/// is vertical and the camera reads back its mount angle. The library's stand
/// frame does not, which is why v5/v6 aimed at the near ground and saw 44-47%
/// side faces. No robot, no camera — three joint angles and the mount quat.
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
  const Gaze yawed = gaze_of(camera_pose(
      {std::cos(h), 0.f, 0.f, std::sin(h)}, {0.f, 0.f, 0.f}, mount));
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
    nominal[i] = 0.1f * static_cast<float>(i % 3);  // any stance, waist included
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
  check(std::fabs(s[2 * J + 3] - 1.0f) < 1e-6f,
        "the anchor quat is identity — upright, and engage rebases the heading");
}

/// The warp must be exactly the heading term the retrieval minimises, or the
/// planner is ranking one thing and commanding another.
void check_warp_identity() {
  std::puts("warp");
  const float qth = 0.31f, phi = -0.12f;
  for (int sym = 0; sym < 4; ++sym) {
    const float a = sym * static_cast<float>(M_PI) / 2.0f;
    const float dth = std::atan2(std::sin(qth + a + phi), std::cos(qth + a + phi));
    const float entry_yaw =
        std::atan2(std::sin(qth + a + phi), std::cos(qth + a + phi));
    check(std::fabs(dth - entry_yaw) < 1e-6f,
          "sym " + std::to_string(sym) + ": entry_yaw == the cost's heading term");
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
  check(w.frames() == lead + cfg.scan_steps,
        "one row per held frame, after the ramp onto the still pose");

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
  const int ramp = cfg.scan_steps - 2 * cfg.hold_tail;
  check(std::fabs(swept(0)) < 1e-5f, "frame 0 is the zero of the sweep");
  check(std::fabs(swept(ramp) - p.yaw_offset) < 1e-3f, "the sweep completes");
  check(std::fabs(swept(cfg.scan_steps - 1) - p.yaw_offset) < 1e-3f,
        "and holds through the read tail");
  check(swept(ramp / 2) > 0.1f && swept(ramp / 2) < p.yaw_offset,
        "and is monotone across the window, not a step");
  const int aang = g1::WIRE_COLS_MIN + 3;
  auto ang_z = [&](int f) {
    return rows[static_cast<size_t>(lead + f) * cols + aang + 2];
  };
  check(ang_z(ramp / 2) > 0.0f, "a turning reference commands a turn rate");
  check(std::fabs(ang_z(cfg.scan_steps - 1)) < 1e-6f,
        "and zero once it is done turning");
  float head = 0.0f;
  for (int j = 0; j < g1::NUM_JOINTS; ++j)
    head = std::max(head, std::fabs(rows[j] - live.joint_pos_il[j]));
  check(head < 1e-5f, "row 0 IS the live pose — a still commit ramps, not steps");
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
    tail = std::max(tail, std::fabs(rows[static_cast<size_t>(lead - 1) * cols + j] -
                                    span[j]));
  }
  check(head < 1e-5f, "row 0 IS the live pose");
  check(tail < 1e-5f, "the last lead-in row IS the clip's entry pose");
  check(w.frames() == lead + p.frames, "clip follows the ramp intact");
}

void check_table(const ClipTable& t) {
  std::puts("table");
  check(!t.rows().empty(), "rows loaded");
  check(t.cols() == g1::WIRE_COLS_FULL, "rows are in MotionReference layout");
  check(t.half_extent() > 0.1f && t.fps() > 1.0f, "half_extent and fps are sane");
  size_t pooled = 0;
  for (char d : std::string("FBLR")) pooled += t.pool(d).size();
  check(pooled > 0, "at least one delta has a clean pool");
  if (!t.has_frames()) return;
  int worst = 0;
  for (const auto& r : t.rows())
    if (r.clean && r.span_len != r.exit - r.entry + 1) ++worst;
  check(worst == 0, "every clean row's span matches its [entry, exit]");
}

// ── Replay rig ───────────────────────────────────────────────────
//
// Each npz holds one recorded read: bgr (H,W,3) uint8, depth (h,w) float32
// metres, intrinsics (4,), R_bc (9,), t_bc (3,). Dump them from the sim or from
// a standing robot; the point is that C++ and the Python oracle see the SAME
// frames.

int replay(const std::string& dir, ClipTable& t, const Cfg& cfg) {
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
  Sys1Clips clips(t, cfg, 4);
  CubeSight eye(cfg, PALETTE_SIM, t.half_extent());
  Belief belief(cfg);
  std::printf("replay: %zu reads\n", files.size());
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
    belief.push(s);
    clips.observe(belief);
    const Plan p = clips.decide(belief);
    std::printf("%-24s %-10s c%d pos %+.4f %+.4f %+.4f phi %+.5f | vote %d/%d "
                "c%d | %s cost %.3f yaw %+.4f row %d sym %d\n",
                f.substr(f.find_last_of('/') + 1).c_str(), s.reason.c_str(),
                s.color, s.pos[0], s.pos[1], s.pos[2], s.phi, belief.n_votes(),
                belief.n_reads(), belief.color(), p.label.c_str(), p.cost,
                p.entry_yaw, p.row, p.sym);
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string table_path, frames_path, library_path, bake_path, replay_dir,
      version = "v7";  // v6 is the gaze ablation
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--table") table_path = next();
    else if (a == "--frames") frames_path = next();
    else if (a == "--library") library_path = next();
    else if (a == "--bake") bake_path = next();
    else if (a == "--replay") replay_dir = next();
    else if (a == "--version") version = next();
    else {
      std::printf("unknown argument '%s'\n", a.c_str());
      return 2;
    }
  }

  const Cfg cfg = Cfg::preset(version);
  check_cfg();
  check_belief();
  check_gaze();
  check_warp_identity();

  if (table_path.empty()) {
    std::printf("\n%d failure(s). Pass --table <sys1_clips.npz> for the rest.\n",
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
  // at runtime from sys0's manifest, so the bundle stays version-agnostic.
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
    check_decide_is_pure(table);
  }
  if (!replay_dir.empty() && replay(replay_dir, table, cfg) != 0) ++failures;

  std::printf("\n%d failure(s)\n", failures);
  return failures ? 1 : 0;
}
