#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace cpp_control
{

/**
 * Computes J^T * F for both arm end-effectors using pinocchio (fixed-base model).
 * Loads the robot URDF, maps the 29-DOF control joints to pinocchio indices,
 * and on each compute() call returns the feedforward torque vector.
 * Only translational forces are used (top 3 rows of the 6×nv Jacobian).
 */
class ArmForceController
{
public:
    ArmForceController(const std::string& urdf_path,
                       const std::vector<std::string>& joint_names,
                       const std::string& left_ee_link,
                       const std::string& right_ee_link);

    // Returns 29-DOF tau_ff. joint_pos is in the same order as joint_names.
    // Forces are in the pelvis (URDF root) frame, in Newtons.
    std::vector<float> compute(const std::vector<float>& joint_pos,
                               const Eigen::Vector3d& left_force,
                               const Eigen::Vector3d& right_force);

private:
    pinocchio::Model model_;
    pinocchio::Data  data_;
    pinocchio::FrameIndex left_ee_id_;
    pinocchio::FrameIndex right_ee_id_;
    int n_ctrl_;
    std::vector<int> q_idx_;  // ctrl joint i → pinocchio q index (-1 = not in URDF)
};

}  // namespace cpp_control
