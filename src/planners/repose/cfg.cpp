#include "cpp_control/planners/repose/cfg.hpp"

#include <yaml-cpp/yaml.h>

#include <cstring>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace cpp_control {
namespace planners {
namespace repose {

Cfg Cfg::preset(const std::string& name) {
  if (name == "v6") return Cfg::v6();
  if (name == "v7") return Cfg::v7();
  if (name == "v7.1") return Cfg::v7_1();
  if (name == "v8") return Cfg::v8();
  if (name == "v8.5") return Cfg::v8_5();
  if (name == "v9") return Cfg::v9();
  if (name == "v9.1") return Cfg::v9_1();
  if (name == "v9.2") return Cfg::v9_2();
  throw std::runtime_error(
      "repose planner: version must be v6 | v7 | v7.1 | v8 | v8.5 | v9 | "
      "v9.1 | v9.2, got '" +
      name + "'");
}

// ── The loader ───────────────────────────────────────────────────
//
// One reader for one schema. Every knob the perception and planning stack has
// is reachable from here, so a deployment is a FILE and not a rebuild; the
// preset named by `version` supplies the defaults, because a preset is a
// measured point in the ablation ledger and a half-written yaml should fall
// back to one, not to zeros.
//
// Unknown keys are an ERROR. A silently ignored knob is the worst outcome
// available: it reads as tuned and behaves as stock.

namespace {

/// Colour order IS the palette's row order, and the goal-colour one-hot's.
const char* const kColors[6] = {"red",    "orange", "green",
                                "yellow", "blue",   "pink"};

void reject_unknown(const YAML::Node& n, const char* section,
                    const std::set<std::string>& known) {
  if (!n) return;
  if (!n.IsMap())
    throw std::runtime_error(std::string("repose planner: '") + section +
                             "' must be a map");
  for (const auto& kv : n) {
    const std::string key = kv.first.as<std::string>();
    if (!known.count(key))
      throw std::runtime_error(
          std::string("repose planner: unknown key '") + section + "." + key +
          "'. A knob that is not read is worse than one that "
          "is wrong — fix the name or delete the line");
  }
}

struct Reader {
  const YAML::Node& n;
  void operator()(const char* k, float& v) const {
    if (n && n[k]) v = n[k].as<float>();
  }
  void operator()(const char* k, int& v) const {
    if (n && n[k]) v = n[k].as<int>();
  }
  void operator()(const char* k, bool& v) const {
    if (n && n[k]) v = n[k].as<bool>();
  }
  void operator()(const char* k, std::string& v) const {
    if (n && n[k]) v = n[k].as<std::string>();
  }
};

std::array<float, 3> rgb(const YAML::Node& n, const std::string& what) {
  if (!n || !n.IsSequence() || n.size() != 3)
    throw std::runtime_error("repose planner: palette." + what +
                             " must be a 3-element RGB sequence");
  return {n[0].as<float>(), n[1].as<float>(), n[2].as<float>()};
}

/// 6 colours x {lit, shaded}. Partial is allowed — an unlisted colour keeps the
/// preset's row, so a deployment that only re-measured blue writes only blue.
void read_palette(const YAML::Node& n, Palette& p) {
  if (!n) return;
  reject_unknown(
      n, "observe.palette",
      {kColors[0], kColors[1], kColors[2], kColors[3], kColors[4], kColors[5]});
  for (int c = 0; c < 6; ++c) {
    const YAML::Node row = n[kColors[c]];
    if (!row) continue;
    reject_unknown(row, (std::string("observe.palette.") + kColors[c]).c_str(),
                   {"lit", "shaded"});
    if (!row["lit"] || !row["shaded"])
      throw std::runtime_error(
          std::string("repose planner: palette.") + kColors[c] +
          " needs both 'lit' and 'shaded' — the pair is what "
          "survives shading, and one alone silently halves it");
    const auto lit = rgb(row["lit"], std::string(kColors[c]) + ".lit");
    const auto sha = rgb(row["shaded"], std::string(kColors[c]) + ".shaded");
    for (int i = 0; i < 3; ++i) {
      p[(c * 2) * 3 + i] = lit[i];
      p[(c * 2 + 1) * 3 + i] = sha[i];
    }
  }
}

}  // namespace

Cfg Cfg::from_yaml(const YAML::Node& root) {
  if (root["knobs"])
    throw std::runtime_error(
        "repose planner: 'knobs:' is the pre-split flat schema and is no "
        "longer read. "
        "Move its entries under observe.gates / observe.geometry / belief / "
        "loop / "
        "seam — see config/repose_planner/g1.yaml");

  Cfg c =
      Cfg::preset(root["version"] ? root["version"].as<std::string>() : "v7");

  const YAML::Node obs = root["observe"];
  reject_unknown(
      obs, "observe",
      {"read", "proc_width", "palette", "gates", "geometry", "mask", "plane"});
  Reader{obs}("proc_width", c.proc_width);
  if (obs && obs["read"]) {
    const std::string r = obs["read"].as<std::string>();
    if (r == "blobs")
      c.read = Cfg::Read::BLOBS;
    else if (r == "mask")
      c.read = Cfg::Read::MASK;
    else if (r == "plane")
      c.read = Cfg::Read::PLANE;
    else
      throw std::runtime_error(
          "repose planner: observe.read must be blobs | mask | plane, got '" +
          r + "'");
  }
  read_palette(obs ? obs["palette"] : YAML::Node(), c.palette);

  const YAML::Node mk = obs ? obs["mask"] : YAML::Node();
  reject_unknown(mk, "observe.mask", {"top_pct", "band_m", "erode"});
  Reader r_mk{mk};
  r_mk("top_pct", c.mask_top_pct);
  r_mk("band_m", c.mask_band_m);
  r_mk("erode", c.mask_erode);

  const YAML::Node pl = obs ? obs["plane"] : YAML::Node();
  reject_unknown(
      pl, "observe.plane",
      {"floor_quantile", "ransac_dist_m", "top_band_m", "close_px", "fit_slack",
       "range_max_m", "color_pool", "color_pool_fallback", "color_inset_frac"});
  Reader r_pl{pl};
  r_pl("floor_quantile", c.plane_floor_quantile);
  r_pl("ransac_dist_m", c.plane_ransac_dist_m);
  r_pl("top_band_m", c.plane_top_band_m);
  r_pl("close_px", c.plane_close_px);
  r_pl("fit_slack", c.plane_fit_slack);
  r_pl("range_max_m", c.plane_range_max_m);
  r_pl("color_pool", c.plane_color_pool);
  r_pl("color_pool_fallback", c.plane_color_pool_fallback);
  r_pl("color_inset_frac", c.plane_color_inset_frac);

  const YAML::Node g = obs ? obs["gates"] : YAML::Node();
  reject_unknown(g, "observe.gates",
                 {"min_value", "min_rel_sat", "min_area_frac", "min_px_floor",
                  "chroma_reject"});
  Reader r_g{g};
  r_g("min_value", c.min_value);
  r_g("min_rel_sat", c.min_rel_sat);
  r_g("min_area_frac", c.min_area_frac);
  r_g("min_px_floor", c.min_px_floor);
  r_g("chroma_reject", c.chroma_reject);

  const YAML::Node geo = obs ? obs["geometry"] : YAML::Node();
  reject_unknown(geo, "observe.geometry",
                 {"up_dot_min", "min_visible", "min_visible_color", "big_max",
                  "slab_frac", "z_min_m"});
  Reader r_geo{geo};
  r_geo("up_dot_min", c.up_dot_min);
  r_geo("min_visible", c.min_visible);
  r_geo("min_visible_color", c.min_visible_color);
  r_geo("big_max", c.big_max);
  r_geo("slab_frac", c.slab_frac);
  r_geo("z_min_m", c.z_min_m);

  const YAML::Node b = root["belief"];
  reject_unknown(b, "belief", {"window", "min_votes", "omega_still"});
  Reader r_b{b};
  r_b("window", c.belief_window);
  r_b("min_votes", c.belief_min_votes);
  r_b("omega_still", c.omega_still);

  const YAML::Node l = root["loop"];
  reject_unknown(
      l, "loop",
      {"nominal_stand", "pattern", "retry_limit", "arm_radius", "horizon_gain",
       "stance_band_m", "scan_sweep_deg", "settle_steps", "scan_steps",
       "hold_tail", "stateful_scan", "search_left_first",
       "near_reframe_range_m", "near_reframe_deg", "lock_candidate_identity",
       "smooth_scan", "scan_yaw_rate_deg", "scan_yaw_accel_deg"});
  Reader r_l{l};
  r_l("nominal_stand", c.nominal_stand);
  r_l("pattern", c.pattern);
  r_l("retry_limit", c.retry_limit);
  r_l("arm_radius", c.arm_radius);
  r_l("horizon_gain", c.horizon_gain);
  r_l("stance_band_m", c.stance_band_m);
  r_l("scan_sweep_deg", c.scan_sweep_deg);
  r_l("settle_steps", c.settle_steps);
  r_l("scan_steps", c.scan_steps);
  r_l("hold_tail", c.hold_tail);
  r_l("stateful_scan", c.stateful_scan);
  r_l("search_left_first", c.search_left_first);
  r_l("near_reframe_range_m", c.near_reframe_range_m);
  r_l("near_reframe_deg", c.near_reframe_deg);
  r_l("lock_candidate_identity", c.lock_candidate_identity);
  r_l("smooth_scan", c.smooth_scan);
  r_l("scan_yaw_rate_deg", c.scan_yaw_rate_deg);
  r_l("scan_yaw_accel_deg", c.scan_yaw_accel_deg);

  const YAML::Node a = root["approach"];
  reject_unknown(a, "approach",
                 {"enabled",
                  "motion",
                  "enter_m",
                  "target_m",
                  "max_step_m",
                  "turn_max_deg",
                  "turn_max_attempts",
                  "min_progress_m",
                  "max_attempts",
                  "window_step_m",
                  "min_window_m",
                  "window_snap",
                  "still_speed_m_s",
                  "speed_smooth",
                  "window_heading_max_deg",
                  "anisotropic",
                  "enter_forward_m",
                  "enter_lateral_m",
                  "target_forward_m",
                  "no_progress_limit",
                  "net_windows",
                  "escalate_window",
                  "forward_only",
                  "latch_blocked"});
  Reader r_a{a};
  r_a("enabled", c.approach_enabled);
  r_a("motion", c.approach_motion);
  r_a("enter_m", c.approach_enter_m);
  r_a("target_m", c.approach_target_m);
  r_a("max_step_m", c.approach_max_step_m);
  r_a("turn_max_deg", c.approach_turn_max_deg);
  r_a("turn_max_attempts", c.approach_turn_max_attempts);
  r_a("min_progress_m", c.approach_min_progress_m);
  r_a("max_attempts", c.approach_max_attempts);
  r_a("window_step_m", c.approach_window_step_m);
  r_a("min_window_m", c.approach_min_window_m);
  r_a("window_snap", c.approach_window_snap);
  r_a("still_speed_m_s", c.approach_still_speed_m_s);
  r_a("speed_smooth", c.approach_speed_smooth);
  r_a("window_heading_max_deg", c.approach_window_heading_max_deg);
  r_a("anisotropic", c.approach_anisotropic);
  r_a("enter_forward_m", c.approach_enter_forward_m);
  r_a("enter_lateral_m", c.approach_enter_lateral_m);
  r_a("target_forward_m", c.approach_target_forward_m);
  r_a("no_progress_limit", c.approach_no_progress_limit);
  r_a("net_windows", c.approach_net_windows);
  r_a("escalate_window", c.approach_escalate_window);
  r_a("forward_only", c.approach_forward_only);
  r_a("latch_blocked", c.approach_latch_blocked);

  const YAML::Node s = root["seam"];
  reject_unknown(s, "seam",
                 {"blend_frames", "lead_in_rate", "lead_in_min_s",
                  "lead_in_max_s", "enter_yaw_rate_deg", "enter_joint_rate"});
  Reader r_s{s};
  r_s("blend_frames", c.blend_frames);
  r_s("lead_in_rate", c.lead_in_rate);
  r_s("lead_in_min_s", c.lead_in_min_s);
  r_s("lead_in_max_s", c.lead_in_max_s);
  r_s("enter_yaw_rate_deg", c.enter_yaw_rate_deg);
  r_s("enter_joint_rate", c.enter_joint_rate);

  c.validate();
  return c;
}

bool Cfg::operator==(const Cfg& o) const {
  return read == o.read && mask_top_pct == o.mask_top_pct &&
         mask_band_m == o.mask_band_m && mask_erode == o.mask_erode &&
         plane_floor_quantile == o.plane_floor_quantile &&
         plane_ransac_dist_m == o.plane_ransac_dist_m &&
         plane_top_band_m == o.plane_top_band_m &&
         plane_close_px == o.plane_close_px &&
         plane_fit_slack == o.plane_fit_slack &&
         plane_range_max_m == o.plane_range_max_m &&
         plane_color_pool == o.plane_color_pool &&
         plane_color_pool_fallback == o.plane_color_pool_fallback &&
         plane_color_inset_frac == o.plane_color_inset_frac &&
         chroma_reject == o.chroma_reject && proc_width == o.proc_width &&
         min_area_frac == o.min_area_frac && min_px_floor == o.min_px_floor &&
         min_visible == o.min_visible &&
         min_visible_color == o.min_visible_color && min_value == o.min_value &&
         min_rel_sat == o.min_rel_sat && up_dot_min == o.up_dot_min &&
         big_max == o.big_max && z_min_m == o.z_min_m &&
         slab_frac == o.slab_frac && palette == o.palette &&
         nominal_stand == o.nominal_stand && belief_window == o.belief_window &&
         belief_min_votes == o.belief_min_votes &&
         omega_still == o.omega_still && pattern == o.pattern &&
         retry_limit == o.retry_limit && arm_radius == o.arm_radius &&
         horizon_gain == o.horizon_gain && stance_band_m == o.stance_band_m &&
         scan_sweep_deg == o.scan_sweep_deg && settle_steps == o.settle_steps &&
         scan_steps == o.scan_steps && hold_tail == o.hold_tail &&
         stateful_scan == o.stateful_scan &&
         search_left_first == o.search_left_first &&
         lock_candidate_identity == o.lock_candidate_identity &&
         smooth_scan == o.smooth_scan &&
         scan_yaw_rate_deg == o.scan_yaw_rate_deg &&
         scan_yaw_accel_deg == o.scan_yaw_accel_deg &&
         near_reframe_range_m == o.near_reframe_range_m &&
         near_reframe_deg == o.near_reframe_deg &&
         approach_enabled == o.approach_enabled &&
         approach_motion == o.approach_motion &&
         approach_enter_m == o.approach_enter_m &&
         approach_target_m == o.approach_target_m &&
         approach_max_step_m == o.approach_max_step_m &&
         approach_turn_max_deg == o.approach_turn_max_deg &&
         approach_turn_max_attempts == o.approach_turn_max_attempts &&
         approach_min_progress_m == o.approach_min_progress_m &&
         approach_max_attempts == o.approach_max_attempts &&
         approach_window_step_m == o.approach_window_step_m &&
         approach_min_window_m == o.approach_min_window_m &&
         approach_window_snap == o.approach_window_snap &&
         approach_still_speed_m_s == o.approach_still_speed_m_s &&
         approach_speed_smooth == o.approach_speed_smooth &&
         approach_window_heading_max_deg == o.approach_window_heading_max_deg &&
         approach_anisotropic == o.approach_anisotropic &&
         approach_enter_forward_m == o.approach_enter_forward_m &&
         approach_enter_lateral_m == o.approach_enter_lateral_m &&
         approach_target_forward_m == o.approach_target_forward_m &&
         approach_no_progress_limit == o.approach_no_progress_limit &&
         approach_net_windows == o.approach_net_windows &&
         approach_escalate_window == o.approach_escalate_window &&
         approach_forward_only == o.approach_forward_only &&
         approach_latch_blocked == o.approach_latch_blocked &&
         blend_frames == o.blend_frames && lead_in_rate == o.lead_in_rate &&
         lead_in_min_s == o.lead_in_min_s && lead_in_max_s == o.lead_in_max_s &&
         enter_yaw_rate_deg == o.enter_yaw_rate_deg &&
         enter_joint_rate == o.enter_joint_rate;
}

void Cfg::validate() const {
  if (pattern.empty())
    throw std::runtime_error("repose planner: pattern is empty");
  for (char c : pattern)
    if (!std::strchr("FBLR", c))
      throw std::runtime_error(
          "repose planner: pattern must be drawn from FBLR (only the roll "
          "deltas are "
          "retrievable), got '" +
          pattern + "'");
  if (scan_steps <= 2 * hold_tail)
    throw std::runtime_error(
        "repose planner: a SCAN must outlast twice its hold tail — the sweep "
        "has to "
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
        "repose planner: omega_still must be > 0 — a gate that never opens "
        "leaves the "
        "planner blind and standing still forever");
  if (blend_frames < 0 || lead_in_rate < 0.0f)
    throw std::runtime_error(
        "repose planner: blend_frames / lead_in_rate must be >= 0");
  if (enter_yaw_rate_deg < 0.0f || enter_joint_rate <= 0.0f)
    throw std::runtime_error(
        "repose planner: need enter_yaw_rate_deg >= 0 (0 is v7, the ramp off) "
        "and "
        "enter_joint_rate > 0");
  if (lead_in_max_s < lead_in_min_s)
    throw std::runtime_error(
        "repose planner: lead_in_max_s must be >= lead_in_min_s");
  if (approach_enabled && approach_motion.empty())
    throw std::runtime_error(
        "repose planner: approach.motion is required when approach is enabled");
  if (approach_enter_m <= 0.0f || approach_target_m < 0.0f ||
      approach_target_m >= approach_enter_m)
    throw std::runtime_error(
        "repose planner: approach needs 0 <= target_m < enter_m");
  if (approach_max_step_m <= 0.0f || approach_turn_max_deg <= 0.0f ||
      approach_turn_max_deg > 180.0f || approach_turn_max_attempts < 1)
    throw std::runtime_error(
        "repose planner: approach max_step_m must be > 0, turn_max_deg in "
        "(0, 180], and turn_max_attempts >= 1");
  if (approach_min_progress_m < 0.0f || approach_max_attempts < 1)
    throw std::runtime_error(
        "repose planner: approach needs min_progress_m >= 0 and max_attempts "
        ">= 1");
  if (approach_window_step_m <= 0.0f || approach_min_window_m < 0.0f ||
      approach_min_window_m >
          approach_max_step_m + approach_window_step_m + 1e-6f ||
      approach_window_snap < 0 || approach_still_speed_m_s <= 0.0f ||
      approach_speed_smooth < 1 || approach_speed_smooth % 2 == 0 ||
      approach_window_heading_max_deg <= 0.0f ||
      approach_window_heading_max_deg > 180.0f)
    throw std::runtime_error(
        "repose planner: approach window_step/still_speed must be positive, "
        "min_window in [0, max_step + window_step], window_snap non-negative, "
        "speed_smooth a positive odd number, and window_heading_max_deg in "
        "(0, 180]");
  if (approach_enter_forward_m <= 0.0f || approach_enter_lateral_m <= 0.0f ||
      approach_target_forward_m < 0.0f ||
      approach_target_forward_m >= approach_enter_forward_m ||
      approach_no_progress_limit < 1)
    throw std::runtime_error(
        "repose planner: v9 approach needs positive forward/lateral gates, "
        "0 <= target_forward_m < enter_forward_m, and no_progress_limit >= 1");
  if (approach_escalate_window &&
      (approach_max_attempts < 2 || approach_no_progress_limit < 2))
    throw std::runtime_error(
        "repose planner: approach window escalation needs at least two "
        "attempts and two no-progress results");
  if (near_reframe_range_m <= 0.0f || near_reframe_deg <= 0.0f ||
      near_reframe_deg > scan_sweep_deg)
    throw std::runtime_error(
        "repose planner: near reframe needs a positive range and an angle in "
        "(0, scan_sweep_deg]");
  if (scan_yaw_rate_deg <= 0.0f || scan_yaw_accel_deg <= 0.0f)
    throw std::runtime_error(
        "repose planner: scan yaw rate and acceleration must be positive");

  // ── the knobs the yaml only just gained a way to get wrong ──
  if (min_value < 0 || min_value > 255)
    throw std::runtime_error(
        "repose planner: min_value is an 8-bit level, [0, 255]");
  if (min_rel_sat < 0.0f || min_rel_sat >= 1.0f)
    throw std::runtime_error("repose planner: min_rel_sat must be in [0, 1)");
  if (min_area_frac <= 0.0f || min_area_frac > 1.0f)
    throw std::runtime_error("repose planner: min_area_frac must be in (0, 1]");
  if (min_px_floor < 3)
    throw std::runtime_error(
        "repose planner: min_px_floor must be >= 3 — a plane fit needs three "
        "points");
  if (up_dot_min <= 0.0f || up_dot_min > 1.0f)
    throw std::runtime_error("repose planner: up_dot_min must be in (0, 1]");
  if (min_visible_color < 0.0f || min_visible_color > 1.0f)
    throw std::runtime_error(
        "repose planner: min_visible_color must be in [0, 1]");
  if (big_max < min_visible)
    throw std::runtime_error(
        "repose planner: big_max is the LONGEST rect side and min_visible the "
        "shortest, so big_max < min_visible rejects every square");
  if (slab_frac <= 0.0f || slab_frac > 1.0f)
    throw std::runtime_error(
        "repose planner: slab_frac is a depth in cube edges and must be in (0, "
        "1]");
  if (z_min_m <= 0.0f)
    throw std::runtime_error("repose planner: z_min_m must be > 0");
  if (chroma_reject < 0.0f)
    throw std::runtime_error(
        "repose planner: chroma_reject must be >= 0 (0 is off, and is the "
        "shipping read)");
  if (mask_top_pct <= 0.0f || mask_top_pct > 100.0f)
    throw std::runtime_error(
        "repose planner: observe.mask.top_pct must be in (0, 100]");
  if (mask_band_m <= 0.0f)
    throw std::runtime_error("repose planner: observe.mask.band_m must be > 0");
  if (mask_erode < 0)
    throw std::runtime_error("repose planner: observe.mask.erode must be >= 0");
  if (plane_floor_quantile <= 0.0f || plane_floor_quantile >= 1.0f)
    throw std::runtime_error(
        "repose planner: observe.plane.floor_quantile must be in (0, 1)");
  if (plane_ransac_dist_m <= 0.0f || plane_top_band_m <= 0.0f)
    throw std::runtime_error(
        "repose planner: observe.plane ransac_dist_m / top_band_m must be > 0");
  if (plane_close_px < 0)
    throw std::runtime_error(
        "repose planner: observe.plane.close_px must be >= 0");
  if (plane_fit_slack < 0.0f || plane_fit_slack > 0.5f)
    throw std::runtime_error(
        "repose planner: observe.plane.fit_slack must be in [0, 0.5]");
  if (plane_range_max_m <= 0.0f)
    throw std::runtime_error(
        "repose planner: observe.plane.range_max_m must be > 0");
  if (plane_color_inset_frac < 0.0f || plane_color_inset_frac >= 0.5f)
    throw std::runtime_error(
        "repose planner: observe.plane.color_inset_frac must be in [0, 0.5)");
  for (float v : palette)
    if (!(v >= 0.0f && v <= 255.0f))
      throw std::runtime_error(
          "repose planner: palette entries are 8-bit RGB levels, [0, 255]");
}

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
