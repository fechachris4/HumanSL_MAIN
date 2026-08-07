//
// Kinematics — implementations for Kinematics.h.
//

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "Kinematics.h"

// ---------------------------------------------------------------
// Kinematics — generic Pinocchio FK and Jacobian
// ---------------------------------------------------------------

//
// Kinematics: forward kinematics via Pinocchio (model already loaded in Dynamics).
//




Pose forward_kinematics(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                        const std::string& frame_name)
{
    if (!dynamics.model_.existFrame(frame_name))
        throw std::runtime_error("forward_kinematics: no frame named '" + frame_name +
                                 "' in the model (check the URDF link names)");

    // Walk the kinematic tree: place every joint for configuration q_pin,
    // then update the frames attached to them.
    pinocchio::framesForwardKinematics(dynamics.model_, dynamics.data_, q_pin);

    // oMf = "origin-to-frame": the frame pose in the model root frame.
    const auto frame_id = dynamics.model_.getFrameId(frame_name);
    const pinocchio::SE3& oMf = dynamics.data_.oMf[frame_id];

    return Pose{oMf.translation(), oMf.rotation()};
}

// ---------------------------------------------------------------
// DualArmKinematics — the 14-model/7-controller adapter
// ---------------------------------------------------------------

//
// DualArmKinematics: explicit 14-DoF model / 7-controller adapter.
//




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
    Dynamics& dynamics, Arm controlled_arm,
    const JointVector& other_arm_nominal_rad,
    const std::string& right_base_frame,
    const std::string& right_end_effector_frame,
    const std::string& left_base_frame,
    const std::string& left_end_effector_frame)
    : dynamics_(dynamics), controlled_arm_(controlled_arm),
      right_base_frame_id_(0), right_frame_id_(0),
      left_base_frame_id_(0), left_frame_id_(0),
      other_arm_nominal_rad_(other_arm_nominal_rad),
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
    if (!dynamics_.model_.existFrame(left_base_frame))
        throw std::runtime_error("dual model has no left base frame named '" +
                                 left_base_frame + "'");
    if (!dynamics_.model_.existFrame(left_end_effector_frame))
        throw std::runtime_error("dual model has no left end-effector frame named '" +
                                 left_end_effector_frame + "'");

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
    left_base_frame_id_ = dynamics_.model_.getFrameId(left_base_frame);
    left_frame_id_ = dynamics_.model_.getFrameId(left_end_effector_frame);

    // Both mounts are FIXED joints onto the `mount` root, so each base frame's
    // placement is the same for every configuration. Evaluate once here (at
    // the neutral configuration) and cache; nothing downstream has to redo FK
    // just to convert a point between mount and a base frame.
    pinocchio::framesForwardKinematics(dynamics_.model_, dynamics_.data_,
                                       pinocchio::neutral(dynamics_.model_));
    mount_from_right_base_ = dynamics_.data_.oMf[right_base_frame_id_];
    mount_from_left_base_ = dynamics_.data_.oMf[left_base_frame_id_];

    // Seed the OTHER (uncontrolled) arm's slots once; they are never
    // touched again (no connection in this process). The controlled arm's
    // slots stay at pinocchio::neutral() until the first
    // FullConfigurationForControlled call, same as before this arm was
    // configurable — the loop always makes that call before reading FK.
    const std::array<int, 7>& other_q_indices =
        controlled_arm_ == Arm::kRight ? left_q_indices_ : right_q_indices_;
    const char* other_label = controlled_arm_ == Arm::kRight ? "left" : "right";
    for (int i = 0; i < 7; ++i) {
        const double value = other_arm_nominal_rad_[static_cast<std::size_t>(i)];
        if (!std::isfinite(value))
            throw std::runtime_error(std::string(other_label) +
                                     " nominal configuration must be finite");
        const int q_index = other_q_indices[static_cast<std::size_t>(i)];
        const int q_size = kJointConfigurationSizes[static_cast<std::size_t>(i)];
        if (q_size == 1 &&
            (value < dynamics_.model_.lowerPositionLimit[q_index] ||
             value > dynamics_.model_.upperPositionLimit[q_index]))
            throw std::runtime_error(
                std::string(other_label) + " nominal joint " +
                std::to_string(i + 1) + " lies outside the dual URDF position limits");
        SetJointAngle(q_full_, q_index, q_size, value);
    }
}

const Eigen::VectorXd& DualArmKinematics::FullConfigurationForControlled(
    const Eigen::Matrix<double, 7, 1>& controlled_q_rad)
{
    if (!controlled_q_rad.allFinite())
        throw std::runtime_error("controlled-arm measured joint configuration must be finite");
    const std::array<int, 7>& q_indices =
        controlled_arm_ == Arm::kRight ? right_q_indices_ : left_q_indices_;
    for (int i = 0; i < 7; ++i) {
        const std::size_t joint = static_cast<std::size_t>(i);
        SetJointAngle(q_full_, q_indices[joint],
                      kJointConfigurationSizes[joint], controlled_q_rad[i]);
    }
    return q_full_;
}

const Eigen::VectorXd& DualArmKinematics::FullConfiguration(
    const Eigen::Matrix<double, 7, 1>& right_q_rad,
    const Eigen::Matrix<double, 7, 1>& left_q_rad)
{
    // Both arms explicit — independent of controlled_arm_, unlike
    // FullConfigurationForControlled. Writes both halves directly rather
    // than routing through it, so this never depends on which arm this
    // process happens to control.
    if (!right_q_rad.allFinite())
        throw std::runtime_error("right joint configuration must be finite");
    if (!left_q_rad.allFinite())
        throw std::runtime_error("left joint configuration must be finite");
    for (int i = 0; i < 7; ++i) {
        const std::size_t joint = static_cast<std::size_t>(i);
        SetJointAngle(q_full_, right_q_indices_[joint],
                      kJointConfigurationSizes[joint], right_q_rad[i]);
        SetJointAngle(q_full_, left_q_indices_[joint],
                      kJointConfigurationSizes[joint], left_q_rad[i]);
    }
    return q_full_;
}

// ---------------------------------------------------------------
// World frame
// ---------------------------------------------------------------

const pinocchio::SE3& DualArmKinematics::MountFromBase(Arm arm) const
{
    return arm == Arm::kRight ? mount_from_right_base_ : mount_from_left_base_;
}

Eigen::Vector3d DualArmKinematics::PointBaseToMount(
    Arm arm, const Eigen::Vector3d& p_base) const
{
    return MountFromBase(arm).act(p_base);
}

Eigen::Vector3d DualArmKinematics::PointMountToBase(
    Arm arm, const Eigen::Vector3d& p_mount) const
{
    return MountFromBase(arm).actInv(p_mount);
}

Pose DualArmKinematics::ToolPoseInMount(
    Arm arm, const Eigen::Matrix<double, 7, 1>& right_q_rad,
    const Eigen::Matrix<double, 7, 1>& left_q_rad)
{
    const Eigen::VectorXd& q = FullConfiguration(right_q_rad, left_q_rad);
    pinocchio::framesForwardKinematics(dynamics_.model_, dynamics_.data_, q);
    // oMf is already the mount-frame placement: `mount` is the model root.
    const pinocchio::SE3& mount_M_tool =
        dynamics_.data_.oMf[arm == Arm::kRight ? right_frame_id_ : left_frame_id_];
    return Pose{mount_M_tool.translation(), mount_M_tool.rotation()};
}

Pose DualArmKinematics::ToolPoseInMount(
    Arm arm, const Eigen::Matrix<double, 7, 1>& controlled_q_rad)
{
    Eigen::Matrix<double, 7, 1> other_q_rad;
    for (int i = 0; i < 7; ++i)
        other_q_rad[i] = other_arm_nominal_rad_[static_cast<std::size_t>(i)];
    if (controlled_arm_ == Arm::kRight)
        return ToolPoseInMount(arm, controlled_q_rad, other_q_rad);
    return ToolPoseInMount(arm, other_q_rad, controlled_q_rad);
}

void DualArmKinematics::UpdateFullKinematics(
    const Eigen::Matrix<double, 7, 1>& controlled_q_rad,
    KinematicsWorkspace& workspace)
{
    const Eigen::VectorXd& q = FullConfigurationForControlled(controlled_q_rad);
    pinocchio::computeJointJacobians(dynamics_.model_, dynamics_.data_, q);
    pinocchio::updateFramePlacements(dynamics_.model_, dynamics_.data_);
    workspace.jacobian_full.setZero();
    const pinocchio::FrameIndex controlled_frame_id =
        controlled_arm_ == Arm::kRight ? right_frame_id_ : left_frame_id_;
    const pinocchio::FrameIndex controlled_base_frame_id =
        controlled_arm_ == Arm::kRight ? right_base_frame_id_ : left_base_frame_id_;
    pinocchio::getFrameJacobian(
        dynamics_.model_, dynamics_.data_, controlled_frame_id,
        pinocchio::LOCAL_WORLD_ALIGNED, workspace.jacobian_full);

    // LOCAL_WORLD_ALIGNED gives the tool-point twist in model-root axes.
    // Rotate both linear and angular rows into the controlled arm's base
    // axes. The point is unchanged (the tool origin), so no translational
    // adjoint term applies.
    const Eigen::Matrix3d base_R_mount =
        dynamics_.data_.oMf[controlled_base_frame_id].rotation().transpose();
    for (int col = 0; col < workspace.jacobian_full.cols(); ++col) {
        const Eigen::Vector3d linear_mount =
            workspace.jacobian_full.template block<3, 1>(0, col);
        const Eigen::Vector3d angular_mount =
            workspace.jacobian_full.template block<3, 1>(3, col);
        workspace.jacobian_full.template block<3, 1>(0, col) =
            base_R_mount * linear_mount;
        workspace.jacobian_full.template block<3, 1>(3, col) =
            base_R_mount * angular_mount;
    }
}

PositionJacobian DualArmKinematics::ControlledPositionAndJacobian(
    const Eigen::Matrix<double, 7, 1>& controlled_q_rad,
    KinematicsWorkspace& workspace)
{
    UpdateFullKinematics(controlled_q_rad, workspace);
    const pinocchio::FrameIndex controlled_frame_id =
        controlled_arm_ == Arm::kRight ? right_frame_id_ : left_frame_id_;
    const pinocchio::FrameIndex controlled_base_frame_id =
        controlled_arm_ == Arm::kRight ? right_base_frame_id_ : left_base_frame_id_;
    const std::array<int, 7>& controlled_v_indices =
        controlled_arm_ == Arm::kRight ? right_v_indices_ : left_v_indices_;
    const pinocchio::SE3 base_M_tool =
        dynamics_.data_.oMf[controlled_base_frame_id].inverse() *
        dynamics_.data_.oMf[controlled_frame_id];
    PositionJacobian result;
    result.position = base_M_tool.translation();
    result.rotation = base_M_tool.rotation();
    for (int i = 0; i < 7; ++i)
        result.jacobian_p.col(i) =
            workspace.jacobian_full.topRows<3>().col(
                controlled_v_indices[static_cast<std::size_t>(i)]);
    return result;
}

PoseJacobian DualArmKinematics::ControlledPoseAndJacobian(
    const Eigen::Matrix<double, 7, 1>& controlled_q_rad,
    KinematicsWorkspace& workspace)
{
    UpdateFullKinematics(controlled_q_rad, workspace);
    const pinocchio::FrameIndex controlled_frame_id =
        controlled_arm_ == Arm::kRight ? right_frame_id_ : left_frame_id_;
    const pinocchio::FrameIndex controlled_base_frame_id =
        controlled_arm_ == Arm::kRight ? right_base_frame_id_ : left_base_frame_id_;
    const std::array<int, 7>& controlled_v_indices =
        controlled_arm_ == Arm::kRight ? right_v_indices_ : left_v_indices_;
    const pinocchio::SE3 base_M_tool =
        dynamics_.data_.oMf[controlled_base_frame_id].inverse() *
        dynamics_.data_.oMf[controlled_frame_id];
    PoseJacobian result;
    result.position = base_M_tool.translation();
    result.rotation = base_M_tool.rotation();
    for (int i = 0; i < 7; ++i)
        result.jacobian.col(i) =
            workspace.jacobian_full.col(
                controlled_v_indices[static_cast<std::size_t>(i)]);
    return result;
}

