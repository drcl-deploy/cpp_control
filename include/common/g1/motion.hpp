#pragma once

#include <array>
#include <string>
#include <vector>

namespace cpp_control
{
namespace g1
{

/// The one reference-motion container — every tracker reads its motion through
/// this (no per-node parallel buffers). Storage is MJ-canonical joint order;
/// IL views gather through joint_orders.hpp. Sources:
///
///   from_npz()   retargeted-dataset / mjlab npz (fps, joint_pos, joint_vel,
///                body_{pos,quat,lin_vel,ang_vel}_w) — mocke MjMotionLoader twin
///   from_wire()  textop motion topic rows [jp29 | jv29 | anchor_pos3 | anchor_quat4],
///                IL-ordered, anchor only (no twist)
///   stand()      1-frame nominal reference (sonic/textop stand modes)
///
/// root_*_vel_b are the sys1 command twists (orcs robot_root_{lin,ang}_vel_cmd):
/// reference-anchor-frame, pure clip functions — heading-rebase invariant, so
/// they never touch live robot state.
struct Motion
{
    int num_frames = 0;
    int num_joints = 0;
    int num_bodies = 0;
    float fps = 50.0f;
    bool has_twist = false;  ///< body_*_vel_w came from the source (vs zero-filled)

    std::vector<float> joint_pos;       ///< (T*J) MJ order
    std::vector<float> joint_vel;       ///< (T*J) MJ order
    std::vector<float> body_pos_w;      ///< (T*B*3)
    std::vector<float> body_quat_w;     ///< (T*B*4) wxyz
    std::vector<float> body_lin_vel_w;  ///< (T*B*3)
    std::vector<float> body_ang_vel_w;  ///< (T*B*3)

    /// joint_perm: source column for each output joint (empty = identity;
    /// g1::MJ2IL for IL-ordered clips). Body arrays load as-is.
    static Motion from_npz(const std::string& path,
                           const std::vector<int>& joint_perm = {});
    static Motion from_wire(int num_frames, const float* rows);
    static Motion stand(const std::vector<float>& default_angles_mj,
                        float fps = 50.0f);

    // ── joint views ──
    const float* jp(int f) const { return &joint_pos[static_cast<size_t>(f) * num_joints]; }
    const float* jv(int f) const { return &joint_vel[static_cast<size_t>(f) * num_joints]; }
    void jp_il(int f, float* out) const;  ///< IL-ordered gather into out[num_joints]
    void jv_il(int f, float* out) const;

    // ── root (anchor body) views ──
    std::array<float, 3> root_pos(int f, int body = 0) const;
    std::array<float, 4> root_quat(int f, int body = 0) const;
    std::array<float, 3> root_lin_vel_b(int f, int body = 0) const;  ///< ref-anchor frame
    std::array<float, 3> root_ang_vel_b(int f, int body = 0) const;  ///< ref-anchor frame
};

/// Playback clock + heading alignment over a Motion — the one `t` every
/// consumer samples through.
///
/// Deployment cannot teleport the robot to the clip's start pose the way
/// mjlab play does, so the reference is brought to the robot instead: at
/// engage(), a yaw-only offset `yaw(robot) * yaw(ref[start])^-1` is baked in
/// and the aligned_* views pre-apply it (sim2real `_heading_offset` pattern).
///
/// Future indices mirror mocke FutureMotionCommand.future_frames: frame + k,
/// clamped to the clip end.
class MotionClock
{
public:
    MotionClock(const Motion& motion, int anchor_body);

    void engage(const std::array<float, 4>& robot_quat, int start_frame = 0);
    void step(double dt);  ///< advance by fps*dt motion frames

    int frame() const;
    int future_frame(int frames_ahead) const;  ///< clamped to num_frames-1
    bool finished() const { return frame() >= motion_.num_frames - 1; }

    /// Heading-aligned root views at a motion frame.
    std::array<float, 4> aligned_root_quat(int f) const;
    std::array<float, 3> aligned_root_pos(int f) const;

private:
    const Motion& motion_;
    int anchor_body_;
    double phase_ = 0.0;  ///< playback position in motion frames
    std::array<float, 4> heading_offset_ = {1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> ref_start_pos_ = {0.0f, 0.0f, 0.0f};
};

}  // namespace g1
}  // namespace cpp_control
