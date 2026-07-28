//
// DualArmKinematics: explicit 14-model / 7-controller adapter.
//
// The Pinocchio model contains both mounted Gen3 arms (14 revolute joints).
// The controller and Kortex command path remain seven-wide for the right arm.
// This adapter assembles q_full = [right measured, left nominal] by JOINT NAME,
// computes the full mounted-model kinematics, and selects the seven right-arm
// Jacobian columns in Kortex actuator order.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_DUAL_ARM_KINEMATICS_H
#define HUMANSL_MASTERS_PROJECT_2025_DUAL_ARM_KINEMATICS_H

#include <array>
#include <string>

#include "JointVector.h"
#include "math/Kinematics.h"

class DualArmKinematics
{
public:
    static constexpr int kArmDofs = 7;
    static constexpr int kFullDofs = 14;

    DualArmKinematics(Dynamics& dynamics, const JointVector& left_nominal_rad,
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
    pinocchio::FrameIndex right_frame_id_;
    std::array<int, 7> right_q_indices_{};
    std::array<int, 7> right_v_indices_{};
    std::array<int, 7> left_q_indices_{};
    std::array<int, 7> left_v_indices_{};
    JointVector left_nominal_rad_;
    Eigen::VectorXd q_full_;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_DUAL_ARM_KINEMATICS_H
