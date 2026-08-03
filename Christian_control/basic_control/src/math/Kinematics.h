//
// Kinematics: forward kinematics via Pinocchio (model already loaded in Dynamics).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_KINEMATICS_H
#define HUMANSL_MASTERS_PROJECT_2025_KINEMATICS_H

#include <string>

#include "Dynamics.h"

// Pose of one frame of the robot, expressed in the Pinocchio model root frame.
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

// Cartesian pose and translational Jacobian result. Position, rotation, and
// Jacobian rows must use the same declared frame. The generic producer below
// uses model-root axes; DualArmKinematics transforms this result to base_link.
// Pose and Jacobian must describe the same configuration q_pin.
struct PositionJacobian {
    Eigen::Vector3d position;                 // meters, declared Cartesian frame
    Eigen::Matrix3d rotation;                 // orientation in that frame
    Eigen::Matrix<double, 3, 7> jacobian_p;   // rows: x,y,z; cols: joints 1-7
};

PositionJacobian position_and_jacobian(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                                       pinocchio::FrameIndex frame_id,
                                       KinematicsWorkspace& workspace);

// Full-pose counterpart of PositionJacobian. All six Jacobian rows
// ([linear; angular]) use the same declared axes as position and rotation.
struct PoseJacobian {
    Eigen::Vector3d position;                // meters, declared Cartesian frame
    Eigen::Matrix3d rotation;                // orientation in that frame
    Eigen::Matrix<double, 6, 7> jacobian;    // [linear; angular] x joints 1-7
};

PoseJacobian pose_and_jacobian(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                               pinocchio::FrameIndex frame_id,
                               KinematicsWorkspace& workspace);

#endif // HUMANSL_MASTERS_PROJECT_2025_KINEMATICS_H
