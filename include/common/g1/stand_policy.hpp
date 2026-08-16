#pragma once

#include <array>
#include <memory>
#include <string>

#include "common/onnx_session.hpp"
#include "common/types.hpp"

namespace cpp_control {
namespace g1 {

/**
 * @brief Self-contained actively-balancing G1 stand policy.
 *
 * Owns its ONNX session, observation buffer, action history, and checkpoint
 * action metadata. It deliberately shares no action or gain state with the
 * task controller that owns the G1Node.
 */
class StandPolicy {
 public:
  static constexpr int NUM_JOINTS = 29;
  static constexpr int NUM_OBS = 96;
  static constexpr double STEP_DT = 0.02;

  explicit StandPolicy(const std::string& onnx_path);

  void reset();
  RobotCommand step(const RobotState& state);

 private:
  std::unique_ptr<deploy::OnnxSession> session_;
  std::array<float, NUM_JOINTS> last_actions_{};

  // last_action is part of the policy observation, so it may only advance with
  // the physical state. Reuse the most recent command when a wall-clock control
  // timer fires again for the same robot-state tick (for example, while MuJoCo
  // is paused).
  uint32_t last_state_tick_ = 0;
  bool command_valid_ = false;
  RobotCommand cached_command_;

  // Immutable checkpoint deployment metadata, copied once from the export.
  std::array<float, NUM_JOINTS> default_joint_pos_;
  std::array<float, NUM_JOINTS> action_scale_;
  std::array<float, NUM_JOINTS> stiffness_;
  std::array<float, NUM_JOINTS> damping_;
};

}  // namespace g1
}  // namespace cpp_control
