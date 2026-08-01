#include "common/g1/motion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include "cnpy/cnpy.h"
#include "common/g1/joint_orders.hpp"
#include "common/math_utils.hpp"

namespace cpp_control
{
namespace g1
{

namespace
{

/// npz arrays may be float32 or float64; normalize to float32.
std::vector<float> to_floats(const cnpy::NpyArray& arr)
{
    std::vector<float> out(arr.num_vals);
    if (arr.word_size == sizeof(float))
    {
        const float* p = arr.data<float>();
        std::copy(p, p + arr.num_vals, out.begin());
    }
    else if (arr.word_size == sizeof(double))
    {
        const double* p = arr.data<double>();
        for (size_t i = 0; i < arr.num_vals; ++i)
            out[i] = static_cast<float>(p[i]);
    }
    else
    {
        throw std::runtime_error("Motion: unsupported npz dtype (word_size=" +
                                 std::to_string(arr.word_size) + ")");
    }
    return out;
}

}  // namespace

Motion Motion::from_npz(const std::string& path, const std::vector<int>& joint_perm)
{
    cnpy::npz_t npz = cnpy::npz_load(path);

    auto require = [&](const char* key) -> cnpy::NpyArray& {
        auto it = npz.find(key);
        if (it == npz.end())
            throw std::runtime_error("Motion: '" + std::string(key) + "' missing in " + path);
        return it->second;
    };

    Motion m;
    const auto& jp = require("joint_pos");
    const auto& jv = require("joint_vel");
    const auto& bq = require("body_quat_w");
    if (jp.shape.size() != 2 || bq.shape.size() != 3 || bq.shape[2] != 4)
        throw std::runtime_error("Motion: unexpected array shapes in " + path);

    m.num_frames = static_cast<int>(jp.shape[0]);
    m.num_joints = static_cast<int>(jp.shape[1]);
    m.num_bodies = static_cast<int>(bq.shape[1]);
    m.joint_pos = to_floats(jp);
    m.joint_vel = to_floats(jv);
    m.body_quat_w = to_floats(bq);

    const size_t tb3 = static_cast<size_t>(m.num_frames) * m.num_bodies * 3;
    m.body_pos_w = npz.count("body_pos_w") ? to_floats(npz.at("body_pos_w"))
                                           : std::vector<float>(tb3, 0.0f);
    m.has_twist = npz.count("body_lin_vel_w") && npz.count("body_ang_vel_w");
    m.body_lin_vel_w = m.has_twist ? to_floats(npz.at("body_lin_vel_w"))
                                   : std::vector<float>(tb3, 0.0f);
    m.body_ang_vel_w = m.has_twist ? to_floats(npz.at("body_ang_vel_w"))
                                   : std::vector<float>(tb3, 0.0f);

    if (npz.count("fps"))
    {
        // fps dtype varies by producer (mjlab demo: float64; retargeted dataset:
        // int64). cnpy keeps only word_size, so disambiguate by sane range —
        // an int64 misread as double is denormal (~1e-322), never a real rate.
        const auto& arr = npz.at("fps");
        auto sane = [](double v) { return v > 0.5 && v < 10000.0; };
        double fps = 0.0;
        if (arr.word_size == 8)
        {
            const double as_f64 = arr.data<double>()[0];
            const auto as_i64 = static_cast<double>(arr.data<int64_t>()[0]);
            fps = sane(as_f64) ? as_f64 : as_i64;
        }
        else if (arr.word_size == 4)
        {
            const double as_f32 = arr.data<float>()[0];
            const auto as_i32 = static_cast<double>(arr.data<int32_t>()[0]);
            fps = sane(as_f32) ? as_f32 : as_i32;
        }
        if (!sane(fps))
            throw std::runtime_error("Motion: unreadable fps in " + path);
        m.fps = static_cast<float>(fps);
    }

    if (!joint_perm.empty())
    {
        if (static_cast<int>(joint_perm.size()) != m.num_joints)
            throw std::runtime_error("Motion: joint_perm size mismatch");
        std::vector<float> jp2(m.joint_pos.size()), jv2(m.joint_vel.size());
        for (int t = 0; t < m.num_frames; ++t)
            for (int j = 0; j < m.num_joints; ++j)
            {
                jp2[t * m.num_joints + j] = m.joint_pos[t * m.num_joints + joint_perm[j]];
                jv2[t * m.num_joints + j] = m.joint_vel[t * m.num_joints + joint_perm[j]];
            }
        m.joint_pos = std::move(jp2);
        m.joint_vel = std::move(jv2);
    }

    return m;
}

Motion Motion::from_wire(int num_frames, const float* rows)
{
    const int J = NUM_JOINTS;
    const int cols = 2 * J + 3 + 4;

    Motion m;
    m.num_frames = num_frames;
    m.num_joints = J;
    m.num_bodies = 1;
    m.joint_pos.resize(static_cast<size_t>(num_frames) * J);
    m.joint_vel.resize(static_cast<size_t>(num_frames) * J);
    m.body_pos_w.resize(static_cast<size_t>(num_frames) * 3);
    m.body_quat_w.resize(static_cast<size_t>(num_frames) * 4);
    m.body_lin_vel_w.assign(static_cast<size_t>(num_frames) * 3, 0.0f);
    m.body_ang_vel_w.assign(static_cast<size_t>(num_frames) * 3, 0.0f);

    for (int t = 0; t < num_frames; ++t)
    {
        const float* row = rows + static_cast<size_t>(t) * cols;
        for (int j = 0; j < J; ++j)  // wire is IL-ordered
        {
            m.joint_pos[t * J + j] = row[MJ2IL[j]];
            m.joint_vel[t * J + j] = row[J + MJ2IL[j]];
        }
        std::memcpy(&m.body_pos_w[t * 3], row + 2 * J, 3 * sizeof(float));
        std::memcpy(&m.body_quat_w[t * 4], row + 2 * J + 3, 4 * sizeof(float));
    }
    return m;
}

Motion Motion::stand(const std::vector<float>& default_angles_mj, float fps)
{
    Motion m;
    m.num_frames = 1;
    m.num_joints = static_cast<int>(default_angles_mj.size());
    m.num_bodies = 1;
    m.fps = fps;
    m.has_twist = true;  // a stand reference truly commands zero twist
    m.joint_pos = default_angles_mj;
    m.joint_vel.assign(m.num_joints, 0.0f);
    m.body_pos_w = {0.0f, 0.0f, 0.0f};
    m.body_quat_w = {1.0f, 0.0f, 0.0f, 0.0f};
    m.body_lin_vel_w = {0.0f, 0.0f, 0.0f};
    m.body_ang_vel_w = {0.0f, 0.0f, 0.0f};
    return m;
}

void Motion::jp_il(int f, float* out) const
{
    const float* src = jp(f);
    for (int j = 0; j < num_joints; ++j)
        out[j] = src[IL2MJ[j]];
}

void Motion::jv_il(int f, float* out) const
{
    const float* src = jv(f);
    for (int j = 0; j < num_joints; ++j)
        out[j] = src[IL2MJ[j]];
}

std::array<float, 3> Motion::root_pos(int f, int body) const
{
    const size_t base = (static_cast<size_t>(f) * num_bodies + body) * 3;
    return {body_pos_w[base], body_pos_w[base + 1], body_pos_w[base + 2]};
}

std::array<float, 4> Motion::root_quat(int f, int body) const
{
    const size_t base = (static_cast<size_t>(f) * num_bodies + body) * 4;
    return {body_quat_w[base], body_quat_w[base + 1], body_quat_w[base + 2],
            body_quat_w[base + 3]};
}

std::array<float, 3> Motion::root_lin_vel_b(int f, int body) const
{
    const size_t base = (static_cast<size_t>(f) * num_bodies + body) * 3;
    return math::quat_rotate_inverse(
        root_quat(f, body),
        {body_lin_vel_w[base], body_lin_vel_w[base + 1], body_lin_vel_w[base + 2]});
}

std::array<float, 3> Motion::root_ang_vel_b(int f, int body) const
{
    const size_t base = (static_cast<size_t>(f) * num_bodies + body) * 3;
    return math::quat_rotate_inverse(
        root_quat(f, body),
        {body_ang_vel_w[base], body_ang_vel_w[base + 1], body_ang_vel_w[base + 2]});
}

// ── MotionClock ──────────────────────────────────────────────────

MotionClock::MotionClock(const Motion& motion, int anchor_body)
    : motion_(motion), anchor_body_(anchor_body)
{
    if (anchor_body_ < 0 || anchor_body_ >= motion_.num_bodies)
        throw std::runtime_error("MotionClock: anchor_body out of range");
}

void MotionClock::engage(const std::array<float, 4>& robot_quat, int start_frame)
{
    phase_ = static_cast<double>(std::clamp(start_frame, 0, motion_.num_frames - 1));
    const int f0 = static_cast<int>(phase_);
    ref_start_pos_ = motion_.root_pos(f0, anchor_body_);
    heading_offset_ = math::qmul(
        math::heading_quat(robot_quat),
        math::qinv(math::heading_quat(motion_.root_quat(f0, anchor_body_))));
}

void MotionClock::step(double dt)
{
    phase_ = std::min(phase_ + motion_.fps * dt,
                      static_cast<double>(motion_.num_frames - 1));
}

int MotionClock::frame() const
{
    return static_cast<int>(phase_);
}

int MotionClock::future_frame(int frames_ahead) const
{
    return std::min(frame() + frames_ahead, motion_.num_frames - 1);
}

std::array<float, 4> MotionClock::aligned_root_quat(int f) const
{
    return math::qmul(heading_offset_, motion_.root_quat(f, anchor_body_));
}

std::array<float, 3> MotionClock::aligned_root_pos(int f) const
{
    // Displacement from the engage frame, yaw-rotated into the robot's heading.
    const auto p = motion_.root_pos(f, anchor_body_);
    return math::quat_rotate(heading_offset_,
                             {p[0] - ref_start_pos_[0], p[1] - ref_start_pos_[1],
                              p[2] - ref_start_pos_[2]});
}

}  // namespace g1
}  // namespace cpp_control
