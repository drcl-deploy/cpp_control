#include "common/arm_force_controller.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <iostream>
#include <stdexcept>

namespace cpp_control
{

ArmForceController::ArmForceController(const std::string& urdf_path,
                                       const std::vector<std::string>& joint_names,
                                       const std::string& left_ee_link,
                                       const std::string& right_ee_link)
{
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    left_ee_id_  = model_.getFrameId(left_ee_link);
    right_ee_id_ = model_.getFrameId(right_ee_link);

    if (left_ee_id_ >= static_cast<pinocchio::FrameIndex>(model_.nframes))
        throw std::runtime_error("ArmForceController: left EE frame not found: " + left_ee_link);
    if (right_ee_id_ >= static_cast<pinocchio::FrameIndex>(model_.nframes))
        throw std::runtime_error("ArmForceController: right EE frame not found: " + right_ee_link);

    n_ctrl_ = static_cast<int>(joint_names.size());
    q_idx_.assign(n_ctrl_, -1);
    for (int i = 0; i < n_ctrl_; ++i)
    {
        if (model_.existJointName(joint_names[i]))
            q_idx_[i] = model_.joints[model_.getJointId(joint_names[i])].idx_q();
        else
            std::cerr << "ArmForceController: joint not in URDF: " << joint_names[i] << "\n";
    }

    std::cout << "ArmForceController: nq=" << model_.nq << " nv=" << model_.nv
              << " left_ee=" << left_ee_link << " right_ee=" << right_ee_link << "\n";
}

std::vector<float> ArmForceController::compute(const std::vector<float>& joint_pos,
                                                const Eigen::Vector3d& left_force,
                                                const Eigen::Vector3d& right_force)
{
    Eigen::VectorXd q = pinocchio::neutral(model_);
    for (int i = 0; i < n_ctrl_ && i < static_cast<int>(joint_pos.size()); ++i)
        if (q_idx_[i] >= 0)
            q[q_idx_[i]] = static_cast<double>(joint_pos[i]);

    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    Eigen::MatrixXd J_left  = Eigen::MatrixXd::Zero(6, model_.nv);
    Eigen::MatrixXd J_right = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::getFrameJacobian(model_, data_, left_ee_id_,
                                pinocchio::LOCAL_WORLD_ALIGNED, J_left);
    pinocchio::getFrameJacobian(model_, data_, right_ee_id_,
                                pinocchio::LOCAL_WORLD_ALIGNED, J_right);

    Eigen::VectorXd tau = J_left.topRows<3>().transpose()  * left_force
                        + J_right.topRows<3>().transpose() * right_force;

    std::vector<float> tau_ctrl(n_ctrl_, 0.0f);
    for (int i = 0; i < n_ctrl_; ++i)
        if (q_idx_[i] >= 0 && q_idx_[i] < static_cast<int>(tau.size()))
            tau_ctrl[i] = static_cast<float>(tau[q_idx_[i]]);

    return tau_ctrl;
}

}  // namespace cpp_control
