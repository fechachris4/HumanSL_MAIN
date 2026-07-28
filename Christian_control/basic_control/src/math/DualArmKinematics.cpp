//
// DualArmKinematics: explicit 14-model / 7-controller adapter.
//

#include "math/DualArmKinematics.h"

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace
{
    constexpr std::array<const char*, 7> kRightJointNames{
        "Actuator1", "Actuator2", "Actuator3", "Actuator4",
        "Actuator5", "Actuator6", "Actuator7"
    };
    constexpr std::array<const char*, 7> kLeftJointNames{
        "leftActuator1", "leftActuator2", "leftActuator3", "leftActuator4",
        "leftActuator5", "leftActuator6", "leftActuator7"
    };

    void ResolveJoints(const pinocchio::Model& model,
                       const std::array<const char*, 7>& names,
                       std::array<int, 7>& q_indices,
                       std::array<int, 7>& v_indices)
    {
        for (int i = 0; i < 7; ++i) {
            const std::string name(names[static_cast<std::size_t>(i)]);
            if (!model.existJointName(name))
                throw std::runtime_error("dual model has no joint named '" + name + "'");
            const pinocchio::JointIndex joint_id = model.getJointId(name);
            if (model.nqs[joint_id] != 1 || model.nvs[joint_id] != 1)
                throw std::runtime_error(
                    "dual-model adapter requires one configuration/velocity variable for '" +
                    name + "'");
            q_indices[static_cast<std::size_t>(i)] = model.idx_qs[joint_id];
            v_indices[static_cast<std::size_t>(i)] = model.idx_vs[joint_id];
        }
    }

    void ValidateCover(const std::array<int, 7>& right,
                       const std::array<int, 7>& left, int size,
                       const char* label)
    {
        std::vector<bool> seen(static_cast<std::size_t>(size), false);
        for (int index : right) {
            if (index < 0 || index >= size || seen[static_cast<std::size_t>(index)])
                throw std::runtime_error(std::string("invalid/duplicate right ") + label +
                                         " index in dual-model adapter");
            seen[static_cast<std::size_t>(index)] = true;
        }
        for (int index : left) {
            if (index < 0 || index >= size || seen[static_cast<std::size_t>(index)])
                throw std::runtime_error(std::string("invalid/duplicate left ") + label +
                                         " index in dual-model adapter");
            seen[static_cast<std::size_t>(index)] = true;
        }
        for (bool present : seen)
            if (!present)
                throw std::runtime_error(std::string("dual-model adapter does not cover every ") +
                                         label + " variable");
    }
} // namespace

DualArmKinematics::DualArmKinematics(
    Dynamics& dynamics, const JointVector& left_nominal_rad,
    const std::string& right_end_effector_frame)
    : dynamics_(dynamics), right_frame_id_(0),
      left_nominal_rad_(left_nominal_rad),
      q_full_(pinocchio::neutral(dynamics.model_))
{
    if (dynamics_.model_.nq != kFullDofs || dynamics_.model_.nv != kFullDofs)
        throw std::runtime_error(
            "dual runtime URDF must have nq = nv = 14; got nq = " +
            std::to_string(dynamics_.model_.nq) + ", nv = " +
            std::to_string(dynamics_.model_.nv));
    if (!dynamics_.model_.existFrame(right_end_effector_frame))
        throw std::runtime_error("dual model has no right end-effector frame named '" +
                                 right_end_effector_frame + "'");

    ResolveJoints(dynamics_.model_, kRightJointNames,
                  right_q_indices_, right_v_indices_);
    ResolveJoints(dynamics_.model_, kLeftJointNames,
                  left_q_indices_, left_v_indices_);
    ValidateCover(right_q_indices_, left_q_indices_, dynamics_.model_.nq, "q");
    ValidateCover(right_v_indices_, left_v_indices_, dynamics_.model_.nv, "v");
    right_frame_id_ = dynamics_.model_.getFrameId(right_end_effector_frame);

    for (int i = 0; i < 7; ++i) {
        const double value = left_nominal_rad_[static_cast<std::size_t>(i)];
        if (!std::isfinite(value))
            throw std::runtime_error("left nominal configuration must be finite");
        const int q_index = left_q_indices_[static_cast<std::size_t>(i)];
        if (value < dynamics_.model_.lowerPositionLimit[q_index] ||
            value > dynamics_.model_.upperPositionLimit[q_index])
            throw std::runtime_error(
                "left nominal joint " + std::to_string(i + 1) +
                " lies outside the dual URDF position limits");
        q_full_[q_index] = value;
    }
}

const Eigen::VectorXd& DualArmKinematics::FullConfigurationForRight(
    const Eigen::Matrix<double, 7, 1>& right_q_rad)
{
    if (!right_q_rad.allFinite())
        throw std::runtime_error("right measured joint configuration must be finite");
    for (int i = 0; i < 7; ++i)
        q_full_[right_q_indices_[static_cast<std::size_t>(i)]] = right_q_rad[i];
    return q_full_;
}

void DualArmKinematics::UpdateFullKinematics(
    const Eigen::Matrix<double, 7, 1>& right_q_rad,
    KinematicsWorkspace& workspace)
{
    const Eigen::VectorXd& q = FullConfigurationForRight(right_q_rad);
    pinocchio::computeJointJacobians(dynamics_.model_, dynamics_.data_, q);
    pinocchio::updateFramePlacements(dynamics_.model_, dynamics_.data_);
    workspace.jacobian_full.setZero();
    pinocchio::getFrameJacobian(
        dynamics_.model_, dynamics_.data_, right_frame_id_,
        pinocchio::LOCAL_WORLD_ALIGNED, workspace.jacobian_full);
}

PositionJacobian DualArmKinematics::RightPositionAndJacobian(
    const Eigen::Matrix<double, 7, 1>& right_q_rad,
    KinematicsWorkspace& workspace)
{
    UpdateFullKinematics(right_q_rad, workspace);
    PositionJacobian result;
    result.position = dynamics_.data_.oMf[right_frame_id_].translation();
    result.rotation = dynamics_.data_.oMf[right_frame_id_].rotation();
    for (int i = 0; i < 7; ++i)
        result.jacobian_p.col(i) =
            workspace.jacobian_full.topRows<3>().col(
                right_v_indices_[static_cast<std::size_t>(i)]);
    return result;
}

PoseJacobian DualArmKinematics::RightPoseAndJacobian(
    const Eigen::Matrix<double, 7, 1>& right_q_rad,
    KinematicsWorkspace& workspace)
{
    UpdateFullKinematics(right_q_rad, workspace);
    PoseJacobian result;
    result.position = dynamics_.data_.oMf[right_frame_id_].translation();
    result.rotation = dynamics_.data_.oMf[right_frame_id_].rotation();
    for (int i = 0; i < 7; ++i)
        result.jacobian.col(i) =
            workspace.jacobian_full.col(
                right_v_indices_[static_cast<std::size_t>(i)]);
    return result;
}
