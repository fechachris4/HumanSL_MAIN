//
// ReactivePose: the reactive 6-DoF pose controller (see header).
//

#include "control/ReactivePose.h"

#include <cmath>
ReactivePose::ReactivePose(DualArmKinematics& model, PoseTargetStore& targets,
                           const ReactivePoseGains& gains, double arrival_tolerance_m,
                           const Eigen::Matrix<double, 7, 1>& null_midpoint_rad,
                           const Eigen::Matrix<double, 7, 1>& null_centering_mask)
    : model_(model), targets_(targets), gains_(gains),
      arrival_tolerance_m_(arrival_tolerance_m), workspace_(model.dynamics()),
      null_midpoint_rad_(null_midpoint_rad),
      null_centering_mask_(null_centering_mask)
{}

void ReactivePose::Reset(const RobotState& state)
{
    const PoseJacobian ee =
        model_.RightPoseAndJacobian(state.q_rad, workspace_);
    // Hold here: position AND orientation seeded from the measured pose;
    // anything typed before takeover is discarded.
    targets_.Store(ee.position, ee.rotation);
    last_target_sequence_ = targets_.Get().sequence;
    arrival_reported_ = true;
}

Eigen::Matrix<double, 7, 1> ReactivePose::DesiredVelocity(const RobotState& state,
                                                          double /*dt_s*/,
                                                          ControllerStatus& status)
{
    // The adapter composes the SAME full q from measured right joints and the
    // fixed left nominal, then selects only the right 7 Jacobian columns.
    const PoseJacobian ee =
        model_.RightPoseAndJacobian(state.q_rad, workspace_);

    // Equation 1: pose error, reference minus actual (ReactiveLaw.h).
    const PoseTargetStore::Snapshot target = targets_.Get();
    if (target.sequence != last_target_sequence_) {
        last_target_sequence_ = target.sequence;
        arrival_reported_ = false;
    }
    const Eigen::Vector3d e_pos = target.p_desired - ee.position;
    const Eigen::Vector3d e_rot = RotationLog(target.rotation * ee.rotation.transpose());

    // Equation 2: twist error against a zero reference twist (static
    // targets); the measured end-effector twist is J q̇_measured.
    const Eigen::Matrix<double, 6, 1> measured_twist = ee.jacobian * state.qdot_rad_s;
    const Eigen::Vector3d e_v = -measured_twist.head<3>();
    const Eigen::Vector3d e_w = -measured_twist.tail<3>();

    // Arrival notice (position-based, like ResolvedRate): edge-triggered
    // data — the Runner prints.
    if (!arrival_reported_ && e_pos.norm() < arrival_tolerance_m_) {
        arrival_reported_ = true;
        status.arrived_edge = true;
        status.arrival_error_m = e_pos.norm();
    }
    status.p_desired = target.p_desired;
    status.p_current = ee.position;
    status.rot_error_rad = e_rot.norm();
    status.tool_quat = Eigen::Quaterniond(ee.rotation);
    if (status.tool_quat.w() < 0.0)
        status.tool_quat.coeffs() = -status.tool_quat.coeffs();

    // σ_min of the FULL 6×7 task Jacobian — 6×6 self-adjoint solve, no
    // allocation (same construction as ResolvedRate's 3×3, decision 13).
    // The full Jacobian mixes meters-per-radian and radian-per-radian rows,
    // so this σ_min is NOT comparable with ResolvedRate's translational one
    // — watch its trend, not its absolute value.
    const Eigen::Matrix<double, 6, 6> jjt = ee.jacobian * ee.jacobian.transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(jjt);
    status.sigma_min = std::sqrt(std::max(0.0, eigensolver.eigenvalues()(0)));

    // Equations 3-6: task twist -> DLS -> null-space (ReactiveLaw.h).
    return SolveReactiveVelocity(ee.jacobian, e_pos, e_rot, e_v, e_w, state.q_rad,
                                 null_midpoint_rad_, null_centering_mask_, gains_);
}
