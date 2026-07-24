//
// Kinematics: forward kinematics via Pinocchio (model already loaded in Dynamics).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_KINEMATICS_H
#define HUMANSL_MASTERS_PROJECT_2025_KINEMATICS_H

#include <string>

#include "Dynamics.h"

// Pose of one frame of the robot, expressed in the base frame.
struct Pose {
    Eigen::Vector3d position; // meters
    Eigen::Matrix3d rotation; // orientation as a rotation matrix
};

// Forward kinematics: where is `frame_name` when the joints are at q_pin?
// q_pin is the Pinocchio configuration vector (from measure_configuration).
// Frame names come from the URDF, e.g. "EndEffector_Link", "gripper_link",
// "Bracelet_Link".
Pose forward_kinematics(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                        const std::string& frame_name = "EndEffector_Link");

// Preallocated workspace for the per-cycle kinematics: the full 6×nv frame
// Jacobian lives here so the cyclic loop never allocates. Construct once,
// outside the loop, from the model that will be queried.
struct KinematicsWorkspace {
    explicit KinematicsWorkspace(const Dynamics& dynamics)
        : jacobian_full(6, dynamics.model_.nv)
    {
        jacobian_full.setZero(); // getFrameJacobian only writes nonzeros
    }
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_full;
};

// Position of `frame_id` and its translational Jacobian Jp (3×7, base
// frame, LOCAL_WORLD_ALIGNED), both from the SAME configuration q_pin —
// the Jacobian must describe the exact configuration the position error is
// computed at. Requires model_.nv == 7 (checked by the caller at startup).
struct PositionJacobian {
    Eigen::Vector3d position;                 // meters, base frame
    Eigen::Matrix<double, 3, 7> jacobian_p;   // rows: x,y,z; cols: joints 1-7
};

PositionJacobian position_and_jacobian(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                                       pinocchio::FrameIndex frame_id,
                                       KinematicsWorkspace& workspace);

// Full pose of `frame_id` and its 6×7 frame Jacobian (rows: linear x,y,z
// then angular x,y,z; base frame, LOCAL_WORLD_ALIGNED), all from the SAME
// configuration q_pin — the 6-DoF counterpart of position_and_jacobian for
// the reactive pose controller. Requires model_.nv == 7 (checked at startup).
struct PoseJacobian {
    Eigen::Vector3d position;                // meters, base frame
    Eigen::Matrix3d rotation;                // base-frame rotation matrix
    Eigen::Matrix<double, 6, 7> jacobian;    // [linear; angular] × joints 1-7
};

PoseJacobian pose_and_jacobian(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                               pinocchio::FrameIndex frame_id,
                               KinematicsWorkspace& workspace);

#endif // HUMANSL_MASTERS_PROJECT_2025_KINEMATICS_H
