#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace cpp_control
{

/// Reference motion clip — the same npz mjlab's MotionLoader reads
/// (fps, joint_pos (T,J), joint_vel (T,J), body_quat_w (T,B,4)).
/// Optional joint permutation for IL-ordered clips (mocke MjMotionLoader).
struct MotionClip
{
    int num_frames = 0;
    int num_joints = 0;
    int num_bodies = 0;
    float fps = 50.0f;

    std::vector<float> joint_pos;    ///< (T*J) row-major
    std::vector<float> joint_vel;    ///< (T*J)
    std::vector<float> body_quat_w;  ///< (T*B*4) wxyz

    /// joint_perm: source column for each output joint (empty = identity).
    static MotionClip load(const std::string& path,
                           const std::vector<int>& joint_perm = {});

    const float* jp(int t) const { return &joint_pos[static_cast<size_t>(t) * num_joints]; }
    const float* jv(int t) const { return &joint_vel[static_cast<size_t>(t) * num_joints]; }
    std::array<float, 4> body_quat(int t, int body) const;
};

/// Playback clock + heading alignment over a MotionClip.
///
/// Deployment cannot teleport the robot to the clip's start pose the way
/// mjlab play does, so the clip is brought to the robot instead: at start(),
/// a yaw-only offset `yaw(robot) * yaw(ref[0])^-1` is baked in, and every
/// anchor quat is pre-multiplied by it (sim2real's `_heading_offset` pattern).
///
/// Future indices mirror mocke FutureMotionCommand.future_frames: frame + k,
/// clamped to the clip end.
class MotionPlayback
{
public:
    MotionPlayback(const MotionClip& clip, int anchor_body);

    void start(const std::array<float, 4>& robot_quat, int start_frame = 0);
    void step(double dt);  ///< advance the clock by fps*dt motion frames

    int frame() const;
    int future_frame(int frames_ahead) const;  ///< clamped to num_frames-1
    bool finished() const { return frame() >= clip_.num_frames - 1; }

    /// Heading-aligned anchor orientation at a motion frame.
    std::array<float, 4> aligned_anchor_quat(int f) const;

private:
    const MotionClip& clip_;
    int anchor_body_;
    double phase_ = 0.0;  ///< playback position in motion frames
    std::array<float, 4> heading_offset_ = {1.0f, 0.0f, 0.0f, 0.0f};
};

}  // namespace cpp_control
