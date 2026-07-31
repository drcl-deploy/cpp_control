#include "common/motion_clip.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "cnpy/cnpy.h"
#include "common/math_utils.hpp"

namespace cpp_control
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
        throw std::runtime_error("MotionClip: unsupported npz dtype (word_size=" +
                                 std::to_string(arr.word_size) + ")");
    }
    return out;
}

}  // namespace

MotionClip MotionClip::load(const std::string& path, const std::vector<int>& joint_perm)
{
    cnpy::npz_t npz = cnpy::npz_load(path);

    auto require = [&](const char* key) -> cnpy::NpyArray& {
        auto it = npz.find(key);
        if (it == npz.end())
            throw std::runtime_error("MotionClip: '" + std::string(key) + "' missing in " + path);
        return it->second;
    };

    MotionClip clip;
    const auto& jp = require("joint_pos");
    const auto& jv = require("joint_vel");
    const auto& bq = require("body_quat_w");
    if (jp.shape.size() != 2 || bq.shape.size() != 3 || bq.shape[2] != 4)
        throw std::runtime_error("MotionClip: unexpected array shapes in " + path);

    clip.num_frames = static_cast<int>(jp.shape[0]);
    clip.num_joints = static_cast<int>(jp.shape[1]);
    clip.num_bodies = static_cast<int>(bq.shape[1]);
    clip.joint_pos = to_floats(jp);
    clip.joint_vel = to_floats(jv);
    clip.body_quat_w = to_floats(bq);

    if (npz.count("fps"))
        clip.fps = to_floats(npz.at("fps"))[0];

    if (!joint_perm.empty())
    {
        if (static_cast<int>(joint_perm.size()) != clip.num_joints)
            throw std::runtime_error("MotionClip: joint_perm size mismatch");
        std::vector<float> jp2(clip.joint_pos.size()), jv2(clip.joint_vel.size());
        for (int t = 0; t < clip.num_frames; ++t)
            for (int j = 0; j < clip.num_joints; ++j)
            {
                jp2[t * clip.num_joints + j] = clip.joint_pos[t * clip.num_joints + joint_perm[j]];
                jv2[t * clip.num_joints + j] = clip.joint_vel[t * clip.num_joints + joint_perm[j]];
            }
        clip.joint_pos = std::move(jp2);
        clip.joint_vel = std::move(jv2);
    }

    return clip;
}

std::array<float, 4> MotionClip::body_quat(int t, int body) const
{
    const size_t base = (static_cast<size_t>(t) * num_bodies + body) * 4;
    return {body_quat_w[base], body_quat_w[base + 1], body_quat_w[base + 2],
            body_quat_w[base + 3]};
}

// ── MotionPlayback ───────────────────────────────────────────────

MotionPlayback::MotionPlayback(const MotionClip& clip, int anchor_body)
    : clip_(clip), anchor_body_(anchor_body)
{
    if (anchor_body_ < 0 || anchor_body_ >= clip_.num_bodies)
        throw std::runtime_error("MotionPlayback: anchor_body out of range");
}

void MotionPlayback::start(const std::array<float, 4>& robot_quat, int start_frame)
{
    phase_ = static_cast<double>(std::clamp(start_frame, 0, clip_.num_frames - 1));
    auto ref0 = clip_.body_quat(static_cast<int>(phase_), anchor_body_);
    heading_offset_ = math::qmul(math::heading_quat(robot_quat),
                                 math::qinv(math::heading_quat(ref0)));
}

void MotionPlayback::step(double dt)
{
    phase_ = std::min(phase_ + clip_.fps * dt,
                      static_cast<double>(clip_.num_frames - 1));
}

int MotionPlayback::frame() const
{
    return static_cast<int>(phase_);
}

int MotionPlayback::future_frame(int frames_ahead) const
{
    return std::min(frame() + frames_ahead, clip_.num_frames - 1);
}

std::array<float, 4> MotionPlayback::aligned_anchor_quat(int f) const
{
    return math::qmul(heading_offset_, clip_.body_quat(f, anchor_body_));
}

}  // namespace cpp_control
