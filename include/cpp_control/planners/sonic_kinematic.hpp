#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "common/g1/motion.hpp"

namespace cpp_control {
namespace planners {

/// High-level command accepted by the GEAR-SONIC V2 kinematic planner.
struct SonicKinematicCommand {
  int mode = 0;
  float target_speed = -1.0f;
  float height = -1.0f;
  std::array<float, 3> movement_direction{0.0f, 0.0f, 0.0f};
  std::array<float, 3> facing_direction{1.0f, 0.0f, 0.0f};
};

struct SonicKinematicResult {
  g1::Motion motion;
  int context_frame = 0;
  int generation_frame = 0;
  int source_frames = 0;
  double inference_ms = 0.0;
};

/// ROS-free adapter around the published GEAR-SONIC V2 ONNX planner.
///
/// The model emits 30 Hz MuJoCo qpos. This class owns the mixed-dtype ONNX
/// ABI, builds the four-frame recurrent context, and returns the repository's
/// canonical g1::Motion at 50 Hz. It deliberately owns no operator state,
/// ROS topics, or gamepad mapping.
class SonicKinematicPlanner {
 public:
  static constexpr int kNumModes = 27;
  static constexpr int kQposWidth = 36;
  static constexpr float kModelFps = 30.0f;
  static constexpr float kOutputFps = 50.0f;

  explicit SonicKinematicPlanner(const std::string& model_path,
                                 float default_height = 0.788740f,
                                 int lookahead_frames = 2,
                                 int64_t random_seed = 1234);
  ~SonicKinematicPlanner();

  SonicKinematicPlanner(const SonicKinematicPlanner&) = delete;
  SonicKinematicPlanner& operator=(const SonicKinematicPlanner&) = delete;

  /// Seed a fresh episode from measured MJ-order joint positions. As in the
  /// reference deploy stack, root translation/yaw are canonical at episode
  /// start; the tracking controller performs the hardware heading alignment.
  void initialize(const std::array<float, g1::NUM_JOINTS>& joints_mj);
  bool initialized() const { return initialized_; }

  /// Generate a new reference. `previous` is the last reference published by
  /// this planner and `current_frame` is the tracker's playback frame in it.
  SonicKinematicResult plan(const SonicKinematicCommand& command,
                            const g1::Motion* previous, int current_frame);

  /// Rebase `previous` at the handoff frame and cross-fade `generated` onto it.
  /// generation_frame is expressed on the previous reference's timeline.
  static g1::Motion splice(const g1::Motion& previous, int handoff_frame,
                           const g1::Motion& generated, int generation_frame,
                           int blend_frames = 8);

  /// Encode one canonical Motion into MotionReference's FULL, IL-order rows.
  static std::vector<float> to_wire_rows(const g1::Motion& motion);

  static const char* mode_name(int mode);
  static bool is_static_mode(int mode);
  static bool is_ground_mode(int mode);
  static bool is_locomotion_mode(int mode);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;

  void initialize_context(const std::array<float, g1::NUM_JOINTS>& joints_mj);
  void update_context(const g1::Motion& motion, int current_frame);

  float default_height_;
  int lookahead_frames_;
  bool initialized_ = false;
  std::array<float, 4 * kQposWidth> context_{};
};

}  // namespace planners
}  // namespace cpp_control
