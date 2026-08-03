#pragma once

#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace cpp_control
{
namespace g1
{

/// G1 29-dof joint orderings — the single source for every controller.
///
/// IL = IsaacLab BFS order (training checkpoints, retargeted-dataset npz).
/// MJ = MuJoCo DFS order (== unitree_hg motor index == mjlab MJ index).
///
/// Gather semantics:
///   v_mj[i] = v_il[MJ2IL[i]]        v_il[i] = v_mj[IL2MJ[i]]

constexpr int NUM_JOINTS = 29;

inline const std::vector<std::string> IL_JOINTS = {
    "left_hip_pitch_joint",      "right_hip_pitch_joint",     "waist_yaw_joint",
    "left_hip_roll_joint",       "right_hip_roll_joint",      "waist_roll_joint",
    "left_hip_yaw_joint",        "right_hip_yaw_joint",       "waist_pitch_joint",
    "left_knee_joint",           "right_knee_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",  "right_shoulder_roll_joint",
    "left_ankle_roll_joint",     "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",   "right_shoulder_yaw_joint",
    "left_elbow_joint",          "right_elbow_joint",
    "left_wrist_roll_joint",     "right_wrist_roll_joint",
    "left_wrist_pitch_joint",    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",      "right_wrist_yaw_joint",
};

inline const std::vector<std::string> MJ_JOINTS = {
    "left_hip_pitch_joint",       "left_hip_roll_joint",       "left_hip_yaw_joint",
    "left_knee_joint",            "left_ankle_pitch_joint",    "left_ankle_roll_joint",
    "right_hip_pitch_joint",      "right_hip_roll_joint",      "right_hip_yaw_joint",
    "right_knee_joint",           "right_ankle_pitch_joint",   "right_ankle_roll_joint",
    "waist_yaw_joint",            "waist_roll_joint",          "waist_pitch_joint",
    "left_shoulder_pitch_joint",  "left_shoulder_roll_joint",  "left_shoulder_yaw_joint",
    "left_elbow_joint",           "left_wrist_roll_joint",     "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint",          "right_wrist_roll_joint",    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
};

namespace detail
{
inline std::vector<int> index_of(const std::vector<std::string>& what,
                                 const std::vector<std::string>& in)
{
    std::vector<int> out(what.size());
    for (size_t i = 0; i < what.size(); ++i)
        out[i] = static_cast<int>(
            std::find(in.begin(), in.end(), what[i]) - in.begin());
    return out;
}
}  // namespace detail

/// Built from the name tables at static init — cannot drift from them.
inline const std::vector<int> MJ2IL = detail::index_of(MJ_JOINTS, IL_JOINTS);
inline const std::vector<int> IL2MJ = detail::index_of(IL_JOINTS, MJ_JOINTS);

}  // namespace g1
}  // namespace cpp_control
