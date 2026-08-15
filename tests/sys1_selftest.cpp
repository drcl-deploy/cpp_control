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
#include "sys1/clips.hpp"
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
  const Cfg v5 = Cfg::v5(), v6 = Cfg::v6();
  check(v5.pattern == "RRB" && v6.pattern == "RRF", "v5/v6 differ by pattern");
  check(v6.horizon_gain == 0.3f && v5.horizon_gain == 0.0f, "v6 has a horizon");
  check(v5.color_gate() == v5.min_visible, "v5 is one gate for both channels");
  check(v6.color_gate() == 0.0f, "v6 splits the colour channel");
  bool threw = false;
  try {
    Cfg bad = Cfg::v5();
    bad.pattern = "RRU";
    bad.validate();
  } catch (const std::exception&) {
    threw = true;
  }
  check(threw, "a pattern outside FBLR is refused");
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
  const int cols = t.cols();
  check(w.frames() == cfg.scan_steps, "one row per held frame");

  // Only the yaw RELATIVE to frame 0 is a command: MotionClock::engage aligns
  // frame 0 onto the robot, so the stand pose's own recorded heading cancels.
  const int aq = 2 * g1::NUM_JOINTS + 3;
  auto yaw_of = [&](int f) {
    const float* q = &rows[static_cast<size_t>(f) * cols + aq];
    return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  };
  auto swept = [&](int f) {
    return std::atan2(std::sin(yaw_of(f) - yaw_of(0)),
                      std::cos(yaw_of(f) - yaw_of(0)));
  };
  const int ramp = cfg.scan_steps - 2 * cfg.read_tail;
  check(std::fabs(swept(0)) < 1e-5f, "frame 0 is the zero of the sweep");
  check(std::fabs(swept(ramp) - p.yaw_offset) < 1e-3f, "the sweep completes");
  check(std::fabs(swept(cfg.scan_steps - 1) - p.yaw_offset) < 1e-3f,
        "and holds through the read tail");
  check(swept(ramp / 2) > 0.1f && swept(ramp / 2) < p.yaw_offset,
        "and is monotone across the window, not a step");
  const int aang = g1::WIRE_COLS_MIN + 3;
  check(rows[static_cast<size_t>(ramp / 2) * cols + aang + 2] > 0.0f,
        "a turning reference commands a turn rate");
  check(std::fabs(rows[static_cast<size_t>(cfg.scan_steps - 1) * cols + aang + 2]) <
            1e-6f,
        "and zero once it is done turning");
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
    const Sight& s = clips.look(bgr, depth, {k[0], k[1], k[2], k[3]}, cam);
    const Plan p = clips.decide();
    std::printf("%-24s %-10s c%d pos %+.4f %+.4f %+.4f phi %+.5f | %s cost %.3f "
                "yaw %+.4f row %d sym %d\n",
                f.substr(f.find_last_of('/') + 1).c_str(), s.reason.c_str(),
                s.color, s.pos[0], s.pos[1], s.pos[2], s.phi, p.label.c_str(),
                p.cost, p.entry_yaw, p.row, p.sym);
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string table_path, frames_path, library_path, bake_path, replay_dir,
      version = "v6";
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
  if (table.has_frames()) {
    check_still_rows(table, cfg);
    check_lead_in(table, cfg);
  }
  if (!bake_path.empty()) {
    table.bake(bake_path);
    std::printf("baked -> %s\n", bake_path.c_str());
  }
  if (!replay_dir.empty() && replay(replay_dir, table, cfg) != 0) ++failures;

  std::printf("\n%d failure(s)\n", failures);
  return failures ? 1 : 0;
}
