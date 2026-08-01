#include "common/g1/sonic_stand.hpp"

#include <cstring>
#include <stdexcept>

#include "common/math_utils.hpp"

namespace cpp_control
{
namespace g1
{

SonicStand::SonicStand(const std::string& onnx_path, std::string manifest_path)
{
    if (manifest_path.empty())
        manifest_path = onnx_path.substr(0, onnx_path.rfind(".onnx")) + ".manifest.json";
    manifest_ = deploy::DeployManifest::load(manifest_path);

    num_joints_ = static_cast<int>(manifest_.action.joint_names.size());
    actions_.assign(num_joints_, 0.0f);
    stand_motion_ = Motion::stand(manifest_.action.default_joint_pos);
    clock_ = std::make_unique<MotionClock>(stand_motion_, 0);

    session_ = std::make_unique<deploy::OnnxSession>(onnx_path);
    output_name_ = manifest_.outputs.front();
    bind_ports();
}

void SonicStand::bind_ports()
{
    for (const auto& port : manifest_.inputs)
    {
        if (!session_->has_input(port.name))
            throw std::runtime_error("sonic_stand: manifest port '" + port.name +
                                     "' not found in the ONNX graph");
        float* buf = session_->input(port.name);
        if (static_cast<int>(session_->input_dim(port.name)) != port.dim())
            throw std::runtime_error("sonic_stand: port '" + port.name + "' dim mismatch");
        for (const auto& term : port.terms)
            bindings_.push_back(make_binding(buf + term.offset, term));
    }
}

SonicStand::Binding SonicStand::make_binding(float* dst, const deploy::TermSpec& spec)
{
    const int J = num_joints_;

    auto history_binding = [&](int width,
                               std::function<void(const RobotState&, float*)> compute) -> Binding {
        if (spec.history <= 0 || spec.dim != width * spec.history)
            throw std::runtime_error("sonic_stand: term '" + spec.name + "' dim/history mismatch");
        auto hist = std::make_unique<obs::HistoryTerm>(width, spec.history);
        obs::HistoryTerm* h = hist.get();
        histories_.push_back(std::move(hist));
        history_updates_.push_back([h, compute](const RobotState& s) {
            float value[64];
            compute(s, value);
            h->push(value);
        });
        return {dst, [h](const RobotState&, float* out) { h->write(out); }};
    };

    if (spec.name == "base_ang_vel")
        return history_binding(3, [](const RobotState& s, float* v) {
            v[0] = s.imu_gyroscope[0]; v[1] = s.imu_gyroscope[1]; v[2] = s.imu_gyroscope[2];
        });
    if (spec.name == "joint_pos")
        return history_binding(J, [this, J](const RobotState& s, float* v) {
            const auto& d = manifest_.action.default_joint_pos;
            for (int i = 0; i < J; ++i)
                v[i] = s.joint_positions[i] - d[i];
        });
    if (spec.name == "joint_vel")
        return history_binding(J, [J](const RobotState& s, float* v) {
            for (int i = 0; i < J; ++i)
                v[i] = s.joint_velocities[i];
        });
    if (spec.name == "actions")
        return history_binding(J, [this, J](const RobotState&, float* v) {
            std::memcpy(v, actions_.data(), J * sizeof(float));
        });
    if (spec.name == "gravity_dir")
        return history_binding(3, [](const RobotState& s, float* v) {
            auto g = math::get_projected_gravity(s.imu_quaternion);
            v[0] = g[0]; v[1] = g[1]; v[2] = g[2];
        });

    // tokenizer over the 1-frame stand reference: every future step is frame 0
    if (spec.name == "g1_tokenizer")
    {
        if (spec.dim % (2 * J + 6) != 0)
            throw std::runtime_error("sonic_stand: tokenizer dim not divisible by 2J+6");
        const int F = spec.dim / (2 * J + 6);
        return {dst, [this, F, J](const RobotState& s, float* out) {
                    const int row = 2 * J + 6;
                    auto rot_dif = math::qmul(math::qinv(s.imu_quaternion),
                                              clock_->aligned_root_quat(0));
                    auto r6d = math::quat_to_rotation_6d(rot_dif);
                    for (int f = 0; f < F; ++f)
                    {
                        std::memcpy(out + f * row, stand_motion_.jp(0), J * sizeof(float));
                        std::memcpy(out + f * row + J, stand_motion_.jv(0), J * sizeof(float));
                        std::memcpy(out + f * row + 2 * J, r6d.data(), 6 * sizeof(float));
                    }
                }};
    }

    // adapter exports: motion_cmd = [jp_future.flat | jv_future.flat] — all frame 0
    if (spec.name == "motion_cmd")
    {
        if (spec.dim % (2 * J) != 0)
            throw std::runtime_error("sonic_stand: motion_cmd dim not divisible by 2J");
        const int F = spec.dim / (2 * J);
        return {dst, [this, F, J](const RobotState&, float* out) {
                    for (int f = 0; f < F; ++f)
                    {
                        std::memcpy(out + f * J, stand_motion_.jp(0), J * sizeof(float));
                        std::memcpy(out + (F + f) * J, stand_motion_.jv(0), J * sizeof(float));
                    }
                }};
    }

    throw std::runtime_error("sonic_stand: no writer for term '" + spec.name +
                             "' — the stand engine serves base-SONIC exports only");
}

void SonicStand::engage(const RobotState& state)
{
    clock_->engage(state.imu_quaternion, 0);
    for (auto& h : histories_)
        h->reset();
    std::fill(actions_.begin(), actions_.end(), 0.0f);
}

RobotCommand SonicStand::tick(const RobotState& state, double dt)
{
    for (auto& update : history_updates_)
        update(state);
    for (auto& b : bindings_)
        b.write(state, b.dst);

    session_->run();
    const float* act = session_->output(output_name_);
    std::memcpy(actions_.data(), act, num_joints_ * sizeof(float));

    const auto& a = manifest_.action;
    RobotCommand cmd;
    cmd.motor_commands.resize(num_joints_);
    for (int i = 0; i < num_joints_; ++i)
    {
        auto& mc = cmd.motor_commands[i];
        mc.q = a.default_joint_pos[i] + a.scale[i] * actions_[i];
        mc.kp = a.stiffness[i];
        mc.kd = a.damping[i];
    }
    clock_->step(dt);
    return cmd;
}

}  // namespace g1
}  // namespace cpp_control
