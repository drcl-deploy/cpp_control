#include "common/g1/stand_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "common/math_utils.hpp"

namespace cpp_control {
namespace g1 {

namespace {

static_assert(StandPolicy::NUM_OBS == 3 + 3 + 3 * StandPolicy::NUM_JOINTS + 3);

constexpr std::array<float, StandPolicy::NUM_JOINTS> DEFAULT_JOINT_POS = {
    -0.312f, 0.000f, 0.000f,  0.669f, -0.363f, 0.000f, -0.312f, 0.000f,
    0.000f,  0.669f, -0.363f, 0.000f, 0.000f,  0.000f, 0.000f,  0.200f,
    0.200f,  0.000f, 0.600f,  0.000f, 0.000f,  0.000f, 0.200f,  -0.200f,
    0.000f,  0.600f, 0.000f,  0.000f, 0.000f};

constexpr std::array<float, StandPolicy::NUM_JOINTS> ACTION_SCALE = {
    0.548f, 0.351f, 0.548f, 0.351f, 0.439f, 0.439f, 0.548f, 0.351f,
    0.548f, 0.351f, 0.439f, 0.439f, 0.548f, 0.439f, 0.439f, 0.439f,
    0.439f, 0.439f, 0.439f, 0.439f, 0.075f, 0.075f, 0.439f, 0.439f,
    0.439f, 0.439f, 0.439f, 0.075f, 0.075f};

constexpr std::array<float, StandPolicy::NUM_JOINTS> STIFFNESS = {
    40.179f, 99.098f, 40.179f, 99.098f, 28.501f, 28.501f, 40.179f, 99.098f,
    40.179f, 99.098f, 28.501f, 28.501f, 40.179f, 28.501f, 28.501f, 14.251f,
    14.251f, 14.251f, 14.251f, 14.251f, 16.778f, 16.778f, 14.251f, 14.251f,
    14.251f, 14.251f, 14.251f, 16.778f, 16.778f};

constexpr std::array<float, StandPolicy::NUM_JOINTS> DAMPING = {
    2.558f, 6.309f, 2.558f, 6.309f, 1.814f, 1.814f, 2.558f, 6.309f,
    2.558f, 6.309f, 1.814f, 1.814f, 2.558f, 1.814f, 1.814f, 0.907f,
    0.907f, 0.907f, 0.907f, 0.907f, 1.068f, 1.068f, 0.907f, 0.907f,
    0.907f, 0.907f, 0.907f, 1.068f, 1.068f};

}  // namespace

StandPolicy::StandPolicy(const std::string& onnx_path)
    : session_(std::make_unique<deploy::OnnxSession>(onnx_path)),
      default_joint_pos_(DEFAULT_JOINT_POS),
      action_scale_(ACTION_SCALE),
      stiffness_(STIFFNESS),
      damping_(DAMPING) {
  if (session_->input_names() != std::vector<std::string>{"obs"} ||
      session_->input_dim("obs") != NUM_OBS)
    throw std::runtime_error(
        "g1 stand policy: expected one input 'obs' with 96 floats");
  if (session_->output_names() != std::vector<std::string>{"actions"} ||
      session_->output_dim("actions") != NUM_JOINTS)
    throw std::runtime_error(
        "g1 stand policy: expected one output 'actions' with 29 floats");

  reset();
}

void StandPolicy::reset() {
  last_actions_.fill(0.0f);
  command_valid_ = false;
  cached_command_.motor_commands.clear();
}

RobotCommand StandPolicy::step(const RobotState& state) {
  if (state.joint_positions.size() < NUM_JOINTS ||
      state.joint_velocities.size() < NUM_JOINTS)
    throw std::runtime_error(
        "g1 stand policy: robot state has fewer than 29 joints");

  // BaseNode uses a wall timer, whereas simulation state stops advancing while
  // MuJoCo is paused. Do not recursively feed new actions through last_action
  // unless the physical-state token has advanced as well.
  if (command_valid_ && state.tick == last_state_tick_)
    return cached_command_;

  // Export contract (96): ang_vel | projected_gravity | joint_pos_rel |
  // joint_vel | last_action | zero velocity command.
  float* obs = session_->input("obs");
  int offset = 0;
  std::copy(state.imu_gyroscope.begin(), state.imu_gyroscope.end(),
            obs + offset);
  offset += 3;
  const auto gravity = math::get_projected_gravity(state.imu_quaternion);
  std::copy(gravity.begin(), gravity.end(), obs + offset);
  offset += 3;
  for (int i = 0; i < NUM_JOINTS; ++i)
    obs[offset + i] = state.joint_positions[i] - default_joint_pos_[i];
  offset += NUM_JOINTS;
  std::copy_n(state.joint_velocities.begin(), NUM_JOINTS, obs + offset);
  offset += NUM_JOINTS;
  std::copy(last_actions_.begin(), last_actions_.end(), obs + offset);
  offset += NUM_JOINTS;
  std::fill_n(obs + offset, 3, 0.0f);

  session_->run();
  const float* actions = session_->output("actions");

  RobotCommand command;
  command.motor_commands.resize(NUM_JOINTS);
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (!std::isfinite(actions[i]))
      throw std::runtime_error("g1 stand policy: non-finite action");
    last_actions_[i] = actions[i];
    auto& motor = command.motor_commands[i];
    motor.q = default_joint_pos_[i] + action_scale_[i] * actions[i];
    motor.kp = stiffness_[i];
    motor.kd = damping_[i];
  }
  last_state_tick_ = state.tick;
  cached_command_ = command;
  command_valid_ = true;
  return cached_command_;
}

}  // namespace g1
}  // namespace cpp_control
