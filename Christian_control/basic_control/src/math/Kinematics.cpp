//
// Kinematics: forward kinematics via Pinocchio (model already loaded in Dynamics).
//

#include "math/Kinematics.h"

#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

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

PositionJacobian position_and_jacobian(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                                       pinocchio::FrameIndex frame_id,
                                       KinematicsWorkspace& workspace)
{
    // One tree walk computes the joint Jacobians AND the joint placements;
    // updateFramePlacements then positions the frames. Position and
    // Jacobian therefore describe the same configuration q_pin.
    pinocchio::computeJointJacobians(dynamics.model_, dynamics.data_, q_pin);
    pinocchio::updateFramePlacements(dynamics.model_, dynamics.data_);
    pinocchio::getFrameJacobian(dynamics.model_, dynamics.data_, frame_id,
                                pinocchio::LOCAL_WORLD_ALIGNED, workspace.jacobian_full);

    PositionJacobian result;
    result.position = dynamics.data_.oMf[frame_id].translation();
    result.rotation = dynamics.data_.oMf[frame_id].rotation();
    // Rows 0-2 of the 6D frame Jacobian are the translational part; in
    // LOCAL_WORLD_ALIGNED they are expressed in model-root axes.
    result.jacobian_p = workspace.jacobian_full.topRows<3>();
    return result;
}

PoseJacobian pose_and_jacobian(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                               pinocchio::FrameIndex frame_id,
                               KinematicsWorkspace& workspace)
{
    // Same one-tree-walk pattern as position_and_jacobian: pose and Jacobian
    // describe the same configuration q_pin.
    pinocchio::computeJointJacobians(dynamics.model_, dynamics.data_, q_pin);
    pinocchio::updateFramePlacements(dynamics.model_, dynamics.data_);
    pinocchio::getFrameJacobian(dynamics.model_, dynamics.data_, frame_id,
                                pinocchio::LOCAL_WORLD_ALIGNED, workspace.jacobian_full);

    PoseJacobian result;
    result.position = dynamics.data_.oMf[frame_id].translation();
    result.rotation = dynamics.data_.oMf[frame_id].rotation();
    result.jacobian = workspace.jacobian_full;
    return result;
}
