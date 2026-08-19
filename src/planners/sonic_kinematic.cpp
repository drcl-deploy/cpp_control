#include "cpp_control/planners/sonic_kinematic.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "common/g1/joint_orders.hpp"
#include "common/math_utils.hpp"

namespace cpp_control {
namespace planners {

namespace {

constexpr int kJ = g1::NUM_JOINTS;
constexpr int kQ = SonicKinematicPlanner::kQposWidth;

const std::array<const char*, SonicKinematicPlanner::kNumModes> kModeNames = {
    "Idle",          "Slow Walk",       "Walk",           "Run",
    "Squat",         "Kneel Two Legs",  "Kneel One Leg",  "Lying Facedown",
    "Hand Crawling", "Idle Boxing",     "Walk Boxing",    "Left Jab",
    "Right Jab",     "Random Punches",  "Elbow Crawling", "Left Hook",
    "Right Hook",    "Forward Jump",    "Stealth Walk",   "Injured Walk",
    "Ledge Walking", "Object Carrying", "Stealth Walk 2", "Happy Dance Walk",
    "Zombie Walk",   "Gun Walk",        "Scare Walk",
};

std::array<float, 4> normalized(std::array<float, 4> q) {
  float n2 = 0.0f;
  for (float v : q) n2 += v * v;
  if (!std::isfinite(n2) || n2 < 1e-10f)
    throw std::runtime_error("sonic kinematic planner: invalid quaternion");
  const float s = 1.0f / std::sqrt(n2);
  for (float& v : q) v *= s;
  return q;
}

g1::Motion empty_motion(int frames, float fps) {
  if (frames < 1)
    throw std::runtime_error("sonic kinematic planner: empty motion");
  g1::Motion m;
  m.num_frames = frames;
  m.num_joints = kJ;
  m.num_bodies = 1;
  m.fps = fps;
  m.has_twist = true;
  m.has_contact = true;  // no object: zero is the truthful contact command
  m.joint_pos.assign(static_cast<size_t>(frames) * kJ, 0.0f);
  m.joint_vel.assign(static_cast<size_t>(frames) * kJ, 0.0f);
  m.body_pos_w.assign(static_cast<size_t>(frames) * 3, 0.0f);
  m.body_quat_w.assign(static_cast<size_t>(frames) * 4, 0.0f);
  m.body_lin_vel_w.assign(static_cast<size_t>(frames) * 3, 0.0f);
  m.body_ang_vel_w.assign(static_cast<size_t>(frames) * 3, 0.0f);
  m.bodywise_contact.assign(
      static_cast<size_t>(frames) * g1::NUM_CONTACT_BODIES, 0.0f);
  return m;
}

void derive_velocities(g1::Motion& m) {
  const int T = m.num_frames;
  if (T < 2) return;
  const float fps = m.fps;
  for (int f = 0; f + 1 < T; ++f) {
    for (int j = 0; j < kJ; ++j)
      m.joint_vel[static_cast<size_t>(f) * kJ + j] =
          (m.joint_pos[static_cast<size_t>(f + 1) * kJ + j] -
           m.joint_pos[static_cast<size_t>(f) * kJ + j]) *
          fps;
    for (int xyz = 0; xyz < 3; ++xyz)
      m.body_lin_vel_w[static_cast<size_t>(f) * 3 + xyz] =
          (m.body_pos_w[static_cast<size_t>(f + 1) * 3 + xyz] -
           m.body_pos_w[static_cast<size_t>(f) * 3 + xyz]) *
          fps;

    auto q0 = m.root_quat(f);
    auto q1 = m.root_quat(f + 1);
    auto dq = normalized(math::qmul(q1, math::qinv(q0)));
    if (dq[0] < 0.0f)
      for (float& v : dq) v = -v;
    const float half = std::acos(std::clamp(dq[0], -1.0f, 1.0f));
    const float sin_half = std::sin(half);
    if (std::fabs(sin_half) > 1e-6f) {
      const float scale = 2.0f * half * fps / sin_half;
      for (int xyz = 0; xyz < 3; ++xyz)
        m.body_ang_vel_w[static_cast<size_t>(f) * 3 + xyz] =
            dq[xyz + 1] * scale;
    }
  }
  std::copy_n(&m.joint_vel[static_cast<size_t>(T - 2) * kJ], kJ,
              &m.joint_vel[static_cast<size_t>(T - 1) * kJ]);
  std::copy_n(&m.body_lin_vel_w[static_cast<size_t>(T - 2) * 3], 3,
              &m.body_lin_vel_w[static_cast<size_t>(T - 1) * 3]);
  std::copy_n(&m.body_ang_vel_w[static_cast<size_t>(T - 2) * 3], 3,
              &m.body_ang_vel_w[static_cast<size_t>(T - 1) * 3]);
}

g1::Motion resample_qpos(const std::vector<float>& qpos, int frames_30hz) {
  if (frames_30hz < 2 || qpos.size() < static_cast<size_t>(frames_30hz) * kQ)
    throw std::runtime_error(
        "sonic kinematic planner: model returned an invalid trajectory");
  const int frames_50hz = std::max(
      2, static_cast<int>(
             std::floor(frames_30hz * SonicKinematicPlanner::kOutputFps /
                        SonicKinematicPlanner::kModelFps)));
  g1::Motion out = empty_motion(frames_50hz, SonicKinematicPlanner::kOutputFps);
  for (int f = 0; f < frames_50hz; ++f) {
    const float at = static_cast<float>(f) * SonicKinematicPlanner::kModelFps /
                     SonicKinematicPlanner::kOutputFps;
    const int f0 = std::min(static_cast<int>(std::floor(at)), frames_30hz - 1);
    const int f1 = std::min(f0 + 1, frames_30hz - 1);
    const float w1 = at - std::floor(at), w0 = 1.0f - w1;
    for (int xyz = 0; xyz < 3; ++xyz)
      out.body_pos_w[static_cast<size_t>(f) * 3 + xyz] =
          w0 * qpos[static_cast<size_t>(f0) * kQ + xyz] +
          w1 * qpos[static_cast<size_t>(f1) * kQ + xyz];
    std::array<float, 4> q0{}, q1{};
    for (int i = 0; i < 4; ++i) {
      q0[i] = qpos[static_cast<size_t>(f0) * kQ + 3 + i];
      q1[i] = qpos[static_cast<size_t>(f1) * kQ + 3 + i];
    }
    const auto q =
        normalized(math::quat_slerp(normalized(q0), normalized(q1), w1));
    std::copy(q.begin(), q.end(), &out.body_quat_w[static_cast<size_t>(f) * 4]);
    // ONNX qpos is MuJoCo tree order, which is cpp_control's canonical storage.
    for (int j = 0; j < kJ; ++j)
      out.joint_pos[static_cast<size_t>(f) * kJ + j] =
          w0 * qpos[static_cast<size_t>(f0) * kQ + 7 + j] +
          w1 * qpos[static_cast<size_t>(f1) * kQ + 7 + j];
  }
  derive_velocities(out);
  return out;
}

size_t element_count(const std::vector<int64_t>& shape) {
  size_t n = 1;
  for (int64_t d : shape) n *= static_cast<size_t>(d);
  return n;
}

std::vector<int64_t> fixed_shape(std::vector<int64_t> shape, size_t expected,
                                 const std::string& name) {
  for (int64_t& d : shape)
    if (d < 0) d = 1;
  if (element_count(shape) != expected)
    throw std::runtime_error("sonic kinematic planner: tensor '" + name +
                             "' has unexpected shape");
  return shape;
}

}  // namespace

class SonicKinematicPlanner::Impl {
 public:
  Impl(const std::string& model_path, int64_t random_seed)
      : env_(ORT_LOGGING_LEVEL_WARNING, "sonic_kinematic"),
        random_seed_{random_seed} {
    options_.SetIntraOpNumThreads(1);
    options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ =
        std::make_unique<Ort::Session>(env_, model_path.c_str(), options_);
    inspect_model();
    allowed_tokens_ = {0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0};
  }

  struct Output {
    std::vector<float> qpos;
    int frames = 0;
    double inference_ms = 0.0;
  };

  Output run(const std::array<float, 4 * kQ>& context,
             const SonicKinematicCommand& command) {
    target_speed_[0] = command.target_speed;
    mode_[0] = command.mode;
    movement_ = command.movement_direction;
    facing_ = command.facing_direction;
    height_[0] = command.height;

    auto memory =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> values;
    values.reserve(input_names_.size());
    for (const std::string& name : input_names_) {
      const auto& shape = input_shapes_.at(name);
      if (name == "context_mujoco_qpos")
        values.push_back(Ort::Value::CreateTensor<float>(
            memory, const_cast<float*>(context.data()), context.size(),
            shape.data(), shape.size()));
      else if (name == "target_vel")
        values.push_back(float_tensor(memory, target_speed_, shape));
      else if (name == "mode")
        values.push_back(int_tensor(memory, mode_, shape));
      else if (name == "movement_direction")
        values.push_back(float_tensor(memory, movement_, shape));
      else if (name == "facing_direction")
        values.push_back(float_tensor(memory, facing_, shape));
      else if (name == "random_seed")
        values.push_back(int_tensor(memory, random_seed_, shape));
      else if (name == "height")
        values.push_back(float_tensor(memory, height_, shape));
      else if (name == "has_specific_target")
        values.push_back(int_tensor(memory, has_specific_target_, shape));
      else if (name == "specific_target_positions")
        values.push_back(float_tensor(memory, target_positions_, shape));
      else if (name == "specific_target_headings")
        values.push_back(float_tensor(memory, target_headings_, shape));
      else if (name == "allowed_pred_num_tokens")
        values.push_back(int_tensor(memory, allowed_tokens_, shape));
      else
        throw std::runtime_error("sonic kinematic planner: unhandled input '" +
                                 name + "'");
    }

    const char* output_names[] = {"mujoco_qpos", "num_pred_frames"};
    const auto start = std::chrono::steady_clock::now();
    auto output = session_->Run(Ort::RunOptions{nullptr}, input_cnames_.data(),
                                values.data(), values.size(), output_names, 2);
    const auto stop = std::chrono::steady_clock::now();
    if (output.size() != 2 || !output[0].IsTensor() || !output[1].IsTensor())
      throw std::runtime_error(
          "sonic kinematic planner: invalid model outputs");

    const auto qinfo = output[0].GetTensorTypeAndShapeInfo();
    const auto qshape = qinfo.GetShape();
    if (qinfo.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        qshape.size() != 3 || qshape.back() != kQ)
      throw std::runtime_error(
          "sonic kinematic planner: bad mujoco_qpos output");
    int frames = 0;
    const auto ntype = output[1].GetTensorTypeAndShapeInfo().GetElementType();
    if (ntype == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32)
      frames = output[1].GetTensorData<int32_t>()[0];
    else if (ntype == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64)
      frames = static_cast<int>(output[1].GetTensorData<int64_t>()[0]);
    else
      throw std::runtime_error(
          "sonic kinematic planner: num_pred_frames is not int32/int64");
    const size_t available = qinfo.GetElementCount() / kQ;
    if (frames < 2 || frames > 64 || static_cast<size_t>(frames) > available)
      throw std::runtime_error(
          "sonic kinematic planner: num_pred_frames outside valid output");

    const float* q = output[0].GetTensorData<float>();
    Output result;
    result.frames = frames;
    result.qpos.assign(q, q + static_cast<size_t>(frames) * kQ);
    if (!std::all_of(result.qpos.begin(), result.qpos.end(),
                     [](float v) { return std::isfinite(v); }))
      throw std::runtime_error(
          "sonic kinematic planner: output contains NaN/Inf");
    result.inference_ms =
        std::chrono::duration<double, std::milli>(stop - start).count();
    return result;
  }

 private:
  template <size_t N>
  static Ort::Value float_tensor(const Ort::MemoryInfo& memory,
                                 std::array<float, N>& values,
                                 const std::vector<int64_t>& shape) {
    return Ort::Value::CreateTensor<float>(memory, values.data(), values.size(),
                                           shape.data(), shape.size());
  }

  template <size_t N>
  static Ort::Value int_tensor(const Ort::MemoryInfo& memory,
                               std::array<int64_t, N>& values,
                               const std::vector<int64_t>& shape) {
    return Ort::Value::CreateTensor<int64_t>(
        memory, values.data(), values.size(), shape.data(), shape.size());
  }

  void inspect_model() {
    static const std::unordered_map<std::string, size_t> kExpected = {
        {"context_mujoco_qpos", 144},
        {"target_vel", 1},
        {"mode", 1},
        {"movement_direction", 3},
        {"facing_direction", 3},
        {"random_seed", 1},
        {"height", 1},
        {"has_specific_target", 1},
        {"specific_target_positions", 12},
        {"specific_target_headings", 4},
        {"allowed_pred_num_tokens", 11},
    };
    Ort::AllocatorWithDefaultOptions allocator;
    if (session_->GetInputCount() != kExpected.size())
      throw std::runtime_error(
          "sonic kinematic planner: V2 model must have 11 inputs");
    for (size_t i = 0; i < session_->GetInputCount(); ++i) {
      auto name = session_->GetInputNameAllocated(i, allocator);
      const std::string key = name.get();
      const auto expected = kExpected.find(key);
      if (expected == kExpected.end())
        throw std::runtime_error("sonic kinematic planner: unknown input '" +
                                 key + "'");
      // Keep TypeInfo alive while using the tensor metadata. Older ORT C++
      // wrappers expose the nested handle as a non-owning view.
      auto type_info = session_->GetInputTypeInfo(i);
      auto info = type_info.GetTensorTypeAndShapeInfo();
      const bool is_int = key == "mode" || key == "random_seed" ||
                          key == "has_specific_target" ||
                          key == "allowed_pred_num_tokens";
      const auto wanted = is_int ? ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64
                                 : ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
      if (info.GetElementType() != wanted)
        throw std::runtime_error("sonic kinematic planner: bad dtype for '" +
                                 key + "' (got " +
                                 std::to_string(info.GetElementType()) +
                                 ", expected " + std::to_string(wanted) + ")");
      input_shapes_[key] = fixed_shape(info.GetShape(), expected->second, key);
      input_names_.push_back(key);
    }
    for (const std::string& name : input_names_)
      input_cnames_.push_back(name.c_str());

    bool qpos = false, count = false;
    for (size_t i = 0; i < session_->GetOutputCount(); ++i) {
      auto name = session_->GetOutputNameAllocated(i, allocator);
      qpos = qpos || std::string(name.get()) == "mujoco_qpos";
      count = count || std::string(name.get()) == "num_pred_frames";
    }
    if (!qpos || !count)
      throw std::runtime_error(
          "sonic kinematic planner: required outputs are missing");
  }

  Ort::Env env_;
  Ort::SessionOptions options_;
  std::unique_ptr<Ort::Session> session_;
  std::vector<std::string> input_names_;
  std::vector<const char*> input_cnames_;
  std::unordered_map<std::string, std::vector<int64_t>> input_shapes_;

  std::array<float, 1> target_speed_{};
  std::array<int64_t, 1> mode_{};
  std::array<float, 3> movement_{};
  std::array<float, 3> facing_{};
  std::array<int64_t, 1> random_seed_{};
  std::array<float, 1> height_{};
  std::array<int64_t, 1> has_specific_target_{};
  std::array<float, 12> target_positions_{};
  std::array<float, 4> target_headings_{};
  std::array<int64_t, 11> allowed_tokens_{};
};

SonicKinematicPlanner::SonicKinematicPlanner(const std::string& model_path,
                                             float default_height,
                                             int lookahead_frames,
                                             int64_t random_seed)
    : impl_(std::make_unique<Impl>(model_path, random_seed)),
      default_height_(default_height),
      lookahead_frames_(lookahead_frames) {
  if (lookahead_frames_ < 0)
    throw std::runtime_error(
        "sonic kinematic planner: lookahead_frames must be non-negative");
}

SonicKinematicPlanner::~SonicKinematicPlanner() = default;

void SonicKinematicPlanner::initialize(
    const std::array<float, g1::NUM_JOINTS>& joints_mj) {
  if (!std::all_of(joints_mj.begin(), joints_mj.end(),
                   [](float v) { return std::isfinite(v); }))
    throw std::runtime_error(
        "sonic kinematic planner: measured joints contain NaN/Inf");
  initialize_context(joints_mj);
  initialized_ = true;
}

void SonicKinematicPlanner::initialize_context(
    const std::array<float, g1::NUM_JOINTS>& joints_mj) {
  for (int n = 0; n < 4; ++n) {
    float* q = &context_[static_cast<size_t>(n) * kQ];
    std::fill(q, q + kQ, 0.0f);
    q[2] = default_height_;
    q[3] = 1.0f;
    std::copy(joints_mj.begin(), joints_mj.end(), q + 7);
  }
}

void SonicKinematicPlanner::update_context(const g1::Motion& motion,
                                           int current_frame) {
  if (motion.num_frames < 1 || motion.num_joints != kJ || motion.num_bodies < 1)
    throw std::runtime_error(
        "sonic kinematic planner: previous motion is not a G1 reference");
  const int generation_frame = current_frame + lookahead_frames_;
  const float generation_time = generation_frame / motion.fps;
  for (int n = 0; n < 4; ++n) {
    const float sample =
        (generation_time + static_cast<float>(n) / kModelFps) * motion.fps;
    const int f0 = std::clamp(static_cast<int>(std::floor(sample)), 0,
                              motion.num_frames - 1);
    const int f1 = std::min(f0 + 1, motion.num_frames - 1);
    const float w1 = sample - std::floor(sample), w0 = 1.0f - w1;
    float* qpos = &context_[static_cast<size_t>(n) * kQ];
    const auto p0 = motion.root_pos(f0), p1 = motion.root_pos(f1);
    const auto q0 = motion.root_quat(f0), q1 = motion.root_quat(f1);
    for (int i = 0; i < 3; ++i) qpos[i] = w0 * p0[i] + w1 * p1[i];
    const auto q = normalized(math::quat_slerp(q0, q1, w1));
    std::copy(q.begin(), q.end(), qpos + 3);
    for (int j = 0; j < kJ; ++j)
      qpos[7 + j] = w0 * motion.jp(f0)[j] + w1 * motion.jp(f1)[j];
  }
}

SonicKinematicResult SonicKinematicPlanner::plan(
    const SonicKinematicCommand& command, const g1::Motion* previous,
    int current_frame) {
  if (!initialized_)
    throw std::runtime_error(
        "sonic kinematic planner: initialize() not called");
  if (command.mode < 0 || command.mode >= kNumModes)
    throw std::runtime_error("sonic kinematic planner: mode outside V2 range");
  if (previous) update_context(*previous, current_frame);
  const int context_frame = previous ? current_frame : 0;
  const int generation_frame = context_frame + lookahead_frames_;
  const auto raw = impl_->run(context_, command);

  SonicKinematicResult result;
  result.motion = resample_qpos(raw.qpos, raw.frames);
  result.context_frame = context_frame;
  result.generation_frame = generation_frame;
  result.source_frames = raw.frames;
  result.inference_ms = raw.inference_ms;
  return result;
}

g1::Motion SonicKinematicPlanner::splice(const g1::Motion& previous,
                                         int handoff_frame,
                                         const g1::Motion& generated,
                                         int generation_frame,
                                         int blend_frames) {
  if (previous.num_joints != kJ || generated.num_joints != kJ ||
      previous.num_bodies < 1 || generated.num_bodies < 1)
    throw std::runtime_error(
        "sonic kinematic planner: cannot splice incompatible motions");
  handoff_frame = std::clamp(handoff_frame, 0, previous.num_frames - 1);
  const int length = generation_frame - handoff_frame + generated.num_frames;
  if (length < 2) return generated;
  g1::Motion out = empty_motion(length, generated.fps);
  const int blend_start = std::max(0, generation_frame - handoff_frame);
  const float width = static_cast<float>(std::max(1, blend_frames));
  for (int f = 0; f < length; ++f) {
    const int old_frame =
        std::clamp(f + handoff_frame, 0, previous.num_frames - 1);
    const int new_frame = std::clamp(f + handoff_frame - generation_frame, 0,
                                     generated.num_frames - 1);
    const float wn = std::clamp((f - blend_start) / width, 0.0f, 1.0f);
    const float wo = 1.0f - wn;
    for (int j = 0; j < kJ; ++j)
      out.joint_pos[static_cast<size_t>(f) * kJ + j] =
          wo * previous.jp(old_frame)[j] + wn * generated.jp(new_frame)[j];
    const auto po = previous.root_pos(old_frame);
    const auto pn = generated.root_pos(new_frame);
    for (int xyz = 0; xyz < 3; ++xyz)
      out.body_pos_w[static_cast<size_t>(f) * 3 + xyz] =
          wo * po[xyz] + wn * pn[xyz];
    const auto q = normalized(math::quat_slerp(
        previous.root_quat(old_frame), generated.root_quat(new_frame), wn));
    std::copy(q.begin(), q.end(), &out.body_quat_w[static_cast<size_t>(f) * 4]);
  }
  derive_velocities(out);
  return out;
}

std::vector<float> SonicKinematicPlanner::to_wire_rows(
    const g1::Motion& motion) {
  if (motion.num_joints != kJ || motion.num_bodies < 1)
    throw std::runtime_error(
        "sonic kinematic planner: cannot encode incompatible motion");
  std::vector<float> rows(
      static_cast<size_t>(motion.num_frames) * g1::WIRE_COLS_FULL, 0.0f);
  std::array<float, kJ> jp_il{}, jv_il{};
  for (int f = 0; f < motion.num_frames; ++f) {
    float* row = &rows[static_cast<size_t>(f) * g1::WIRE_COLS_FULL];
    motion.jp_il(f, jp_il.data());
    motion.jv_il(f, jv_il.data());
    std::copy(jp_il.begin(), jp_il.end(), row);
    std::copy(jv_il.begin(), jv_il.end(), row + kJ);
    const auto p = motion.root_pos(f);
    const auto q = motion.root_quat(f);
    std::copy(p.begin(), p.end(), row + 2 * kJ);
    std::copy(q.begin(), q.end(), row + 2 * kJ + 3);
    std::copy_n(&motion.body_lin_vel_w[static_cast<size_t>(f) * 3], 3,
                row + g1::WIRE_COLS_MIN);
    std::copy_n(&motion.body_ang_vel_w[static_cast<size_t>(f) * 3], 3,
                row + g1::WIRE_COLS_MIN + 3);
    std::copy_n(motion.contact(f), g1::NUM_CONTACT_BODIES,
                row + g1::WIRE_COLS_MIN + 6);
  }
  return rows;
}

const char* SonicKinematicPlanner::mode_name(int mode) {
  return mode >= 0 && mode < kNumModes ? kModeNames[mode] : "Invalid";
}

bool SonicKinematicPlanner::is_static_mode(int mode) {
  return mode == 0 || (mode >= 4 && mode <= 7) || mode == 9;
}

bool SonicKinematicPlanner::is_ground_mode(int mode) {
  return (mode >= 4 && mode <= 8) || mode == 14;
}

bool SonicKinematicPlanner::is_locomotion_mode(int mode) {
  return mode == 1 || mode == 2 || mode == 3 || mode == 8 || mode == 10 ||
         mode == 14 || (mode >= 18 && mode <= 26);
}

}  // namespace planners
}  // namespace cpp_control
