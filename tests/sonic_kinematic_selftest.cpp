#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "common/g1/joint_orders.hpp"
#include "cpp_control/planners/sonic_kinematic.hpp"

namespace {

using cpp_control::g1::Motion;
using cpp_control::planners::SonicKinematicCommand;
using cpp_control::planners::SonicKinematicPlanner;

constexpr float kPi = 3.14159265358979323846f;

void check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void check_near(float actual, float expected, const std::string& message) {
  if (std::fabs(actual - expected) > 1e-5f)
    throw std::runtime_error(message + ": got " + std::to_string(actual) +
                             ", expected " + std::to_string(expected));
}

Motion fixture(int frames, float offset) {
  Motion m;
  m.num_frames = frames;
  m.num_joints = cpp_control::g1::NUM_JOINTS;
  m.num_bodies = 1;
  m.fps = SonicKinematicPlanner::kOutputFps;
  m.has_twist = true;
  m.has_contact = true;
  m.joint_pos.resize(static_cast<size_t>(frames) * m.num_joints);
  m.joint_vel.resize(static_cast<size_t>(frames) * m.num_joints);
  m.body_pos_w.resize(static_cast<size_t>(frames) * 3);
  m.body_quat_w.assign(static_cast<size_t>(frames) * 4, 0.0f);
  m.body_lin_vel_w.resize(static_cast<size_t>(frames) * 3);
  m.body_ang_vel_w.resize(static_cast<size_t>(frames) * 3);
  m.bodywise_contact.assign(
      static_cast<size_t>(frames) * cpp_control::g1::NUM_CONTACT_BODIES, 0.0f);
  for (int f = 0; f < frames; ++f) {
    for (int j = 0; j < m.num_joints; ++j) {
      m.joint_pos[static_cast<size_t>(f) * m.num_joints + j] =
          offset + static_cast<float>(f) + 0.01f * j;
      m.joint_vel[static_cast<size_t>(f) * m.num_joints + j] = 50.0f;
    }
    m.body_pos_w[static_cast<size_t>(f) * 3] = offset + 0.1f * f;
    m.body_pos_w[static_cast<size_t>(f) * 3 + 2] = 0.79f;
    m.body_quat_w[static_cast<size_t>(f) * 4] = 1.0f;
    m.body_lin_vel_w[static_cast<size_t>(f) * 3] = 5.0f;
  }
  return m;
}

void test_modes() {
  for (int mode = 0; mode < SonicKinematicPlanner::kNumModes; ++mode)
    check(std::string(SonicKinematicPlanner::mode_name(mode)) != "Invalid",
          "mode name missing");
  check(SonicKinematicPlanner::is_locomotion_mode(3), "run must locomote");
  check(SonicKinematicPlanner::is_ground_mode(14),
        "elbow crawl must be a ground mode");
  check(SonicKinematicPlanner::is_static_mode(4), "squat must be static");
}

void test_splice_and_wire() {
  const Motion old_motion = fixture(20, 0.0f);
  const Motion generated = fixture(10, 100.0f);
  const Motion out = SonicKinematicPlanner::splice(
      old_motion, /*handoff_frame=*/5, generated, /*generation_frame=*/7,
      /*blend_frames=*/4);
  check(out.num_frames == 12, "spliced reference length");
  check_near(out.jp(0)[0], old_motion.jp(5)[0], "handoff continuity");
  check_near(out.jp(2)[0], old_motion.jp(7)[0], "blend starts on old path");
  check_near(out.jp(6)[0], generated.jp(4)[0], "blend lands on new path");

  const auto rows = SonicKinematicPlanner::to_wire_rows(out);
  check(rows.size() == static_cast<size_t>(out.num_frames) *
                           cpp_control::g1::WIRE_COLS_FULL,
        "full MotionReference row width");
  for (int il = 0; il < cpp_control::g1::NUM_JOINTS; ++il)
    check_near(rows[il], out.jp(0)[cpp_control::g1::IL2MJ[il]],
               "MJ to IL wire mapping");

  const Motion decoded = Motion::from_wire(
      out.num_frames, rows.data(), cpp_control::g1::WIRE_COLS_FULL,
      /*has_twist=*/true, /*has_contact=*/true, out.fps);
  for (int mj = 0; mj < cpp_control::g1::NUM_JOINTS; ++mj)
    check_near(decoded.jp(0)[mj], out.jp(0)[mj], "wire round trip");
}

void smoke_model(const std::string& model_path) {
  SonicKinematicPlanner planner(model_path);
  std::array<float, cpp_control::g1::NUM_JOINTS> joints{};
  planner.initialize(joints);
  double total_ms = 0.0;
  double max_ms = 0.0;
  for (int mode = 0; mode < SonicKinematicPlanner::kNumModes; ++mode) {
    SonicKinematicCommand command;
    command.mode = mode;
    if (SonicKinematicPlanner::is_locomotion_mode(mode)) {
      command.target_speed = 0.2f;
      command.movement_direction = {1.0f, 0.0f, 0.0f};
    }
    if (SonicKinematicPlanner::is_ground_mode(mode)) command.height = 0.4f;
    const auto result = planner.plan(command, nullptr, 0);
    check(result.motion.num_frames >= 2,
          std::string("model generated no motion for ") +
              SonicKinematicPlanner::mode_name(mode));
    check(result.motion.num_joints == cpp_control::g1::NUM_JOINTS,
          "model generated wrong joint count");
    total_ms += result.inference_ms;
    max_ms = std::max(max_ms, result.inference_ms);
  }
  std::cout << "model smoke: all " << SonicKinematicPlanner::kNumModes
            << " modes, mean " << total_ms / SonicKinematicPlanner::kNumModes
            << " ms, max " << max_ms << " ms\n";

  // Repose v7.2 depends on a less obvious model contract: locomotion mode with
  // a positive speed and ZERO movement direction is a stepping turn, not Idle
  // and not a forward walk. Pin the released model's semantic, not only its
  // tensor shapes, so replacing the artifact cannot quietly turn SCAN into
  // translation.
  SonicKinematicCommand scan;
  scan.mode = 2;  // Walk
  scan.target_speed = 0.10f;
  scan.movement_direction = {0.0f, 0.0f, 0.0f};
  scan.facing_direction = {std::sqrt(0.5f), std::sqrt(0.5f), 0.0f};
  const auto turn = planner.plan(scan, nullptr, 0);
  const auto q0 = turn.motion.root_quat(0);
  const auto q1 = turn.motion.root_quat(turn.motion.num_frames - 1);
  const auto yaw = [](const std::array<float, 4>& q) {
    return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  };
  const float dyaw = std::atan2(std::sin(yaw(q1) - yaw(q0)),
                                std::cos(yaw(q1) - yaw(q0)));
  const auto p0 = turn.motion.root_pos(0);
  const auto p1 = turn.motion.root_pos(turn.motion.num_frames - 1);
  const float translation = std::hypot(p1[0] - p0[0], p1[1] - p0[1]);
  check(dyaw > 20.0f * kPi / 180.0f,
        "stepping SCAN did not turn toward its facing target");
  check(translation < 0.10f,
        "stepping SCAN translated instead of turning in place");
  std::cout << "stepping scan: yaw " << dyaw * 180.0f / kPi
            << " deg, translation " << translation << " m\n";

  // v7.3 generates the next segment from measured joints while the controller
  // holds this one's final pose. Pin that re-entry seam and make sure repeated
  // zero-translation commands retain the turn semantic.
  std::array<float, cpp_control::g1::NUM_JOINTS> final_joints{};
  std::copy_n(turn.motion.jp(turn.motion.num_frames - 1),
              cpp_control::g1::NUM_JOINTS, final_joints.begin());
  planner.initialize(final_joints);
  const auto next = planner.plan(scan, nullptr, 0);
  float max_joint_step = 0.0f;
  for (int j = 0; j < cpp_control::g1::NUM_JOINTS; ++j)
    max_joint_step = std::max(
        max_joint_step, std::fabs(next.motion.jp(0)[j] - final_joints[j]));
  const auto q2_0 = next.motion.root_quat(0);
  const auto q2_1 = next.motion.root_quat(next.motion.num_frames - 1);
  const float next_dyaw = std::atan2(std::sin(yaw(q2_1) - yaw(q2_0)),
                                     std::cos(yaw(q2_1) - yaw(q2_0)));
  const auto p2_0 = next.motion.root_pos(0);
  const auto p2_1 = next.motion.root_pos(next.motion.num_frames - 1);
  const float next_translation =
      std::hypot(p2_1[0] - p2_0[0], p2_1[1] - p2_0[1]);
  check(max_joint_step < 0.35f,
        "consecutive stepping SCAN has a large joint-position seam");
  check(next_dyaw > 20.0f * kPi / 180.0f,
        "consecutive stepping SCAN lost the turn direction");
  check(next_translation < 0.10f,
        "consecutive stepping SCAN translated instead of turning");
  std::cout << "next scan: seam " << max_joint_step << " rad, yaw "
            << next_dyaw * 180.0f / kPi << " deg, translation "
            << next_translation << " m\n";

  // v7.3 deliberately leaves the zero-vector fallback for a short command on
  // Sonic's native gamepad surface: Slow Walk starts at 0.20 m/s, then stick
  // release selects Idle. Pin the two-inference splice before hardware relies
  // on it as a finite creep-turn rather than an endless walking reference.
  planner.initialize(joints);
  SonicKinematicCommand creep;
  creep.mode = 1;  // Slow Walk
  creep.target_speed = 0.20f;
  constexpr float kTurn = 22.5f * kPi / 180.0f;
  creep.facing_direction = {std::cos(kTurn), std::sin(kTurn), 0.0f};
  creep.movement_direction = creep.facing_direction;
  const auto walking = planner.plan(creep, nullptr, 0);
  SonicKinematicCommand idle;
  idle.mode = 0;
  idle.target_speed = -1.0f;
  idle.facing_direction = creep.facing_direction;
  const int stop_frame = static_cast<int>(0.30f * walking.motion.fps);
  const auto stopping = planner.plan(idle, &walking.motion, stop_frame);
  const Motion creep_stop = SonicKinematicPlanner::splice(
      walking.motion, 0, stopping.motion, stopping.generation_frame, 8);
  const float creep_yaw = std::atan2(
      std::sin(yaw(creep_stop.root_quat(creep_stop.num_frames - 1)) -
               yaw(creep_stop.root_quat(0))),
      std::cos(yaw(creep_stop.root_quat(creep_stop.num_frames - 1)) -
               yaw(creep_stop.root_quat(0))));
  const auto cp0 = creep_stop.root_pos(0);
  const auto cp1 = creep_stop.root_pos(creep_stop.num_frames - 1);
  const float creep_translation =
      std::hypot(cp1[0] - cp0[0], cp1[1] - cp0[1]);
  check(creep_yaw > 15.0f * kPi / 180.0f,
        "native creep + Idle failed to preserve the bounded turn");
  check(creep_translation > 0.02f && creep_translation < 0.10f,
        "native creep + Idle left its finite displacement envelope");
  std::cout << "native creep + Idle: yaw " << creep_yaw * 180.0f / kPi
            << " deg, translation " << creep_translation << " m\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    test_modes();
    test_splice_and_wire();
    if (argc > 1) smoke_model(argv[1]);
    std::cout << "sonic_kinematic_selftest: PASS\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "sonic_kinematic_selftest: FAIL: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}
