//
// Kinematics — the Pinocchio robot model: frame FK and Jacobians, and the
// explicit 14-DoF-model / 7-joint-controller dual-arm adapter. The
// controller that uses this model lives in Controller.h/.cpp.
//
// Separated from the pure-Eigen headers so the portable tests need not link
// Pinocchio.
//

#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "Config.h"
#include "State.h"
#include "Dynamics.h" // TrajectoryExecution's Pinocchio model wrapper

// ---------------------------------------------------------------
// Kinematics — generic Pinocchio FK and Jacobian
// ---------------------------------------------------------------

//
// Kinematics: forward kinematics via Pinocchio (model already loaded in Dynamics).
//




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

// ---------------------------------------------------------------
// DualArmKinematics — the 14-model/7-controller adapter
// ---------------------------------------------------------------

//
// DualArmKinematics: explicit 14-DoF model / 7-controller adapter.
//
// The Pinocchio model contains both mounted Gen3 arms (14 velocity DoFs).
// Kinova's continuous joints 1/3/5/7 each use Pinocchio's two-value
// (cos(angle), sin(angle)) configuration representation, so model.nq is 22.
// The controller and Kortex command path remain seven-wide for the right arm.
// This adapter assembles q_full = [right measured, left nominal] by JOINT NAME,
// computes the full mounted-model kinematics, expresses the right tool pose
// and Jacobian in the right base frame, and selects the seven right-arm
// Jacobian columns in Kortex actuator order.
//




class DualArmKinematics
{
public:
    static constexpr int kArmDofs = 7;
    static constexpr int kFullDofs = 14;
    static constexpr int kFullConfigurationSize = 22;
    static constexpr std::array<int, 7> kJointConfigurationSizes{
        2, 1, 2, 1, 2, 1, 2
    };

    DualArmKinematics(Dynamics& dynamics, const JointVector& left_nominal_rad,
                      const std::string& right_base_frame,
                      const std::string& right_end_effector_frame);

    PositionJacobian RightPositionAndJacobian(
        const Eigen::Matrix<double, 7, 1>& right_q_rad,
        KinematicsWorkspace& workspace);
    PoseJacobian RightPoseAndJacobian(
        const Eigen::Matrix<double, 7, 1>& right_q_rad,
        KinematicsWorkspace& workspace);

    // Exposed for hardware-free structural tests. The returned reference is
    // owned by this adapter and is overwritten by the next call.
    const Eigen::VectorXd& FullConfigurationForRight(
        const Eigen::Matrix<double, 7, 1>& right_q_rad);

    Dynamics& dynamics() { return dynamics_; }
    pinocchio::FrameIndex right_base_frame_id() const { return right_base_frame_id_; }
    pinocchio::FrameIndex right_frame_id() const { return right_frame_id_; }
    const std::array<int, 7>& right_q_indices() const { return right_q_indices_; }
    const std::array<int, 7>& right_v_indices() const { return right_v_indices_; }
    const std::array<int, 7>& left_q_indices() const { return left_q_indices_; }
    const std::array<int, 7>& left_v_indices() const { return left_v_indices_; }
    const JointVector& left_nominal_rad() const { return left_nominal_rad_; }

private:
    void UpdateFullKinematics(const Eigen::Matrix<double, 7, 1>& right_q_rad,
                              KinematicsWorkspace& workspace);

    Dynamics& dynamics_;
    pinocchio::FrameIndex right_base_frame_id_;
    pinocchio::FrameIndex right_frame_id_;
    std::array<int, 7> right_q_indices_{};
    std::array<int, 7> right_v_indices_{};
    std::array<int, 7> left_q_indices_{};
    std::array<int, 7> left_v_indices_{};
    JointVector left_nominal_rad_;
    Eigen::VectorXd q_full_;
};
