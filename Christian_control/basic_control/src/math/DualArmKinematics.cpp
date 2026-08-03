//
// DualArmKinematics: explicit 14-DoF model / 7-controller adapter.
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
    constexpr std::array<int, 7> kVelocitySizes{1, 1, 1, 1, 1, 1, 1};

    void ResolveJoints(const pinocchio::Model& model,
                       const std::array<const char*, 7>& names,
                       const std::array<int, 7>& expected_q_sizes,
                       std::array<int, 7>& q_indices,
                       std::array<int, 7>& v_indices)
    {
        for (int i = 0; i < 7; ++i) {
            const std::string name(names[static_cast<std::size_t>(i)]);
            if (!model.existJointName(name))
                throw std::runtime_error("dual model has no joint named '" + name + "'");
            const pinocchio::JointIndex joint_id = model.getJointId(name);
            const int expected_q_size =
                expected_q_sizes[static_cast<std::size_t>(i)];
            if (model.nqs[joint_id] != expected_q_size || model.nvs[joint_id] != 1)
                throw std::runtime_error(
                    "unexpected Pinocchio representation for '" + name +
                    "': expected nq=" + std::to_string(expected_q_size) +
                    ", nv=1; got nq=" + std::to_string(model.nqs[joint_id]) +
                    ", nv=" + std::to_string(model.nvs[joint_id]));
            q_indices[static_cast<std::size_t>(i)] = model.idx_qs[joint_id];
            v_indices[static_cast<std::size_t>(i)] = model.idx_vs[joint_id];
        }
    }

    void ValidateCover(const std::array<int, 7>& right,
                       const std::array<int, 7>& left,
                       const std::array<int, 7>& widths, int size,
                       const char* label)
    {
        std::vector<bool> seen(static_cast<std::size_t>(size), false);
        const auto mark = [&](const std::array<int, 7>& indices,
                              const char* side) {
            for (int i = 0; i < 7; ++i) {
                const int index = indices[static_cast<std::size_t>(i)];
                const int width = widths[static_cast<std::size_t>(i)];
                if (width <= 0 || index < 0 || index + width > size)
                    throw std::runtime_error(
                        std::string("invalid ") + side + " " + label +
                        " range in dual-model adapter");
                for (int offset = 0; offset < width; ++offset) {
                    const std::size_t slot =
                        static_cast<std::size_t>(index + offset);
                    if (seen[slot])
                        throw std::runtime_error(
                            std::string("duplicate ") + side + " " + label +
                            " range in dual-model adapter");
                    seen[slot] = true;
                }
            }
        };
        mark(right, "right");
        mark(left, "left");
        for (bool present : seen)
            if (!present)
                throw std::runtime_error(std::string("dual-model adapter does not cover every ") +
                                         label + " variable");
    }

    void SetJointAngle(Eigen::VectorXd& q, int q_index, int q_size,
                       double angle_rad)
    {
        if (q_size == 1) {
            q[q_index] = angle_rad;
            return;
        }
        if (q_size == 2) {
            q[q_index] = std::cos(angle_rad);
            q[q_index + 1] = std::sin(angle_rad);
            return;
        }
        throw std::runtime_error("unsupported joint configuration size");
    }
} // namespace

DualArmKinematics::DualArmKinematics(
    Dynamics& dynamics, const JointVector& left_nominal_rad,
    const std::string& right_base_frame,
    const std::string& right_end_effector_frame)
    : dynamics_(dynamics), right_base_frame_id_(0), right_frame_id_(0),
      left_nominal_rad_(left_nominal_rad),
      q_full_(pinocchio::neutral(dynamics.model_))
{
    if (dynamics_.model_.nq != kFullConfigurationSize ||
        dynamics_.model_.nv != kFullDofs)
        throw std::runtime_error(
            "dual runtime URDF must have nq = 22 and nv = 14; got nq = " +
            std::to_string(dynamics_.model_.nq) + ", nv = " +
            std::to_string(dynamics_.model_.nv));
    if (!dynamics_.model_.existFrame(right_base_frame))
        throw std::runtime_error("dual model has no right base frame named '" +
                                 right_base_frame + "'");
    if (!dynamics_.model_.existFrame(right_end_effector_frame))
        throw std::runtime_error("dual model has no right end-effector frame named '" +
                                 right_end_effector_frame + "'");

    ResolveJoints(dynamics_.model_, kRightJointNames, kJointConfigurationSizes,
                  right_q_indices_, right_v_indices_);
    ResolveJoints(dynamics_.model_, kLeftJointNames, kJointConfigurationSizes,
                  left_q_indices_, left_v_indices_);
    ValidateCover(right_q_indices_, left_q_indices_, kJointConfigurationSizes,
                  dynamics_.model_.nq, "q");
    ValidateCover(right_v_indices_, left_v_indices_, kVelocitySizes,
                  dynamics_.model_.nv, "v");
    right_base_frame_id_ = dynamics_.model_.getFrameId(right_base_frame);
    right_frame_id_ = dynamics_.model_.getFrameId(right_end_effector_frame);

    for (int i = 0; i < 7; ++i) {
        const double value = left_nominal_rad_[static_cast<std::size_t>(i)];
        if (!std::isfinite(value))
            throw std::runtime_error("left nominal configuration must be finite");
        const int q_index = left_q_indices_[static_cast<std::size_t>(i)];
        const int q_size = kJointConfigurationSizes[static_cast<std::size_t>(i)];
        if (q_size == 1 &&
            (value < dynamics_.model_.lowerPositionLimit[q_index] ||
             value > dynamics_.model_.upperPositionLimit[q_index]))
            throw std::runtime_error(
                "left nominal joint " + std::to_string(i + 1) +
                " lies outside the dual URDF position limits");
        SetJointAngle(q_full_, q_index, q_size, value);
    }
}

const Eigen::VectorXd& DualArmKinematics::FullConfigurationForRight(
    const Eigen::Matrix<double, 7, 1>& right_q_rad)
{
    if (!right_q_rad.allFinite())
        throw std::runtime_error("right measured joint configuration must be finite");
    for (int i = 0; i < 7; ++i) {
        const std::size_t joint = static_cast<std::size_t>(i);
        SetJointAngle(q_full_, right_q_indices_[joint],
                      kJointConfigurationSizes[joint], right_q_rad[i]);
    }
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

    // LOCAL_WORLD_ALIGNED gives the tool-point twist in model-root axes.
    // Rotate both linear and angular rows into right-base axes. The point is
    // unchanged (the tool origin), so no translational adjoint term applies.
    const Eigen::Matrix3d base_R_world =
        dynamics_.data_.oMf[right_base_frame_id_].rotation().transpose();
    for (int col = 0; col < workspace.jacobian_full.cols(); ++col) {
        const Eigen::Vector3d linear_world =
            workspace.jacobian_full.template block<3, 1>(0, col);
        const Eigen::Vector3d angular_world =
            workspace.jacobian_full.template block<3, 1>(3, col);
        workspace.jacobian_full.template block<3, 1>(0, col) =
            base_R_world * linear_world;
        workspace.jacobian_full.template block<3, 1>(3, col) =
            base_R_world * angular_world;
    }
}

PositionJacobian DualArmKinematics::RightPositionAndJacobian(
    const Eigen::Matrix<double, 7, 1>& right_q_rad,
    KinematicsWorkspace& workspace)
{
    UpdateFullKinematics(right_q_rad, workspace);
    const pinocchio::SE3 base_M_tool =
        dynamics_.data_.oMf[right_base_frame_id_].inverse() *
        dynamics_.data_.oMf[right_frame_id_];
    PositionJacobian result;
    result.position = base_M_tool.translation();
    result.rotation = base_M_tool.rotation();
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
    const pinocchio::SE3 base_M_tool =
        dynamics_.data_.oMf[right_base_frame_id_].inverse() *
        dynamics_.data_.oMf[right_frame_id_];
    PoseJacobian result;
    result.position = base_M_tool.translation();
    result.rotation = base_M_tool.rotation();
    for (int i = 0; i < 7; ++i)
        result.jacobian.col(i) =
            workspace.jacobian_full.col(
                right_v_indices_[static_cast<std::size_t>(i)]);
    return result;
}
