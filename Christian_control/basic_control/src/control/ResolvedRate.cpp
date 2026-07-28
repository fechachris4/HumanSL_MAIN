//
// ResolvedRate: the Cartesian resolved-rate controller (see header).
//

#include "control/ResolvedRate.h"

#include "math/Dls.h"

#include <algorithm>
#include <cmath>
ResolvedRate::ResolvedRate(DualArmKinematics& model, TargetStore& targets, double kp,
                           double dls_lambda, double arrival_tolerance_m)
    : model_(model), targets_(targets), kp_(kp), dls_lambda_(dls_lambda),
      arrival_tolerance_m_(arrival_tolerance_m),
      workspace_(model.dynamics())
{}

void ResolvedRate::Reset(const RobotState& state)
{
    const PositionJacobian ee =
        model_.RightPositionAndJacobian(state.q_rad, workspace_);
    targets_.Store(ee.position); // anything typed before takeover is discarded
    last_target_sequence_ = targets_.Get().sequence;
    arrival_reported_ = true;
}

Eigen::Matrix<double, 7, 1> ResolvedRate::DesiredVelocity(const RobotState& state,
                                                          double /*dt_s*/,
                                                          ControllerStatus& status)
{
    // The adapter composes the SAME full q from measured right joints and the
    // fixed left nominal, then selects only the right 7 Jacobian columns.
    const PositionJacobian ee =
        model_.RightPositionAndJacobian(state.q_rad, workspace_);

    // e = p_desired - p(q_measured);  v_d = Kp e;  q̇_raw = DLS(Jp, v_d).
    const TargetStore::Snapshot target = targets_.Get();

    // A new operator target re-arms the arrival notice.
    if (target.sequence != last_target_sequence_)
    {
        last_target_sequence_ = target.sequence;
        arrival_reported_ = false;
    }
    const Eigen::Vector3d position_error_m = target.p_desired - ee.position;
    const Eigen::Vector3d v_desired = kp_ * position_error_m;

    if (!arrival_reported_ && position_error_m.norm() < arrival_tolerance_m_)
    {
        arrival_reported_ = true;
        status.arrived_edge = true;
        status.arrival_error_m = position_error_m.norm();
    }
    status.p_desired = target.p_desired;
    status.p_current = ee.position;

    // Measured tool orientation for the log; the rotation matrix is already
    // computed by the FK above, so this is one 3x3 -> quaternion conversion.
    // Hemisphere fix: q and -q are the same rotation — pin w >= 0.
    status.tool_quat = Eigen::Quaterniond(ee.rotation);
    if (status.tool_quat.w() < 0.0)
        status.tool_quat.coeffs() = -status.tool_quat.coeffs();

    // σ_min(Jp) = sqrt of the smallest eigenvalue of Jp Jpᵀ — fixed-size
    // 3×3 self-adjoint solve, no allocation (decision 13).
    const Eigen::Matrix3d jjt = ee.jacobian_p * ee.jacobian_p.transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(jjt);
    status.sigma_min = std::sqrt(std::max(0.0, eigensolver.eigenvalues()(0)));

    return DampedLeastSquares(ee.jacobian_p, v_desired, dls_lambda_);
}
