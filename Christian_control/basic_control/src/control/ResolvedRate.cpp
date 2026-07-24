//
// ResolvedRate: the Cartesian resolved-rate controller (see header).
//

#include "control/ResolvedRate.h"

#include "math/Dls.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

ResolvedRate::ResolvedRate(Dynamics& dynamics, TargetStore& targets, double kp,
                           double dls_lambda, double arrival_tolerance_m,
                           const std::string& ee_frame_name,
                           const CylinderKeepout& cylinder_keepout)
    : dynamics_(dynamics), targets_(targets), kp_(kp), dls_lambda_(dls_lambda),
      arrival_tolerance_m_(arrival_tolerance_m), ee_frame_(0),
      workspace_(dynamics), q_measured_rad_(7),
      route_follower_(cylinder_keepout)
{
    // Precondition: model_.nv == 7 — validated once in main.cpp.
    if (!dynamics.model_.existFrame(ee_frame_name))
        throw std::runtime_error("no frame named '" + ee_frame_name + "' in the model");
    ee_frame_ = dynamics.model_.getFrameId(ee_frame_name);
}

void ResolvedRate::Reset(const RobotState& state)
{
    const PositionJacobian ee = position_and_jacobian(
        dynamics_, dynamics_.convertJointAnglesToConfig(state.q_rad), ee_frame_,
        workspace_);
    targets_.Store(ee.position); // anything typed before takeover is discarded
    last_target_sequence_ = targets_.Get().sequence;
    arrival_reported_ = true;
    route_follower_.Reset(ee.position);
}

Eigen::Matrix<double, 7, 1> ResolvedRate::DesiredVelocity(const RobotState& state,
                                                          double /*dt_s*/,
                                                          ControllerStatus& status)
{
    // FK and Jacobian use the SAME q_measured.
    for (int i = 0; i < 7; ++i)
        q_measured_rad_[i] = state.q_rad[i];
    const PositionJacobian ee = position_and_jacobian(
        dynamics_, dynamics_.convertJointAnglesToConfig(q_measured_rad_), ee_frame_,
        workspace_);

    // e = p_desired - p(q_measured);  v_d = Kp e;  q̇_raw = DLS(Jp, v_d).
    const TargetStore::Snapshot target = targets_.Get();

    // Construct a route only when the operator target changes. The normal
    // direct route has one waypoint; obstacle crossings get fixed-size
    // around/over waypoints and are never rejected.
    if (target.sequence != last_target_sequence_)
    {
        last_target_sequence_ = target.sequence;
        arrival_reported_ = false;
        route_follower_.SetTarget(ee.position, target.p_desired);
        const CylinderRoute& route = route_follower_.route();
        status.route_changed = route_follower_.enabled();
        status.route_kind = route.kind;
        status.route_target_adjusted = route.target_adjusted;
        status.route_waypoint_count = route.size;
        status.route_requested_target = route.requested_target;
        status.route_effective_target = route.effective_target;
    }
    const Eigen::Vector3d routed_target = route_follower_.Update(ee.position);
    const Eigen::Vector3d position_error_m = routed_target - ee.position;
    const Eigen::Vector3d v_desired = kp_ * position_error_m;

    // Arrival notice fires only at the final waypoint.
    if (!arrival_reported_ && route_follower_.at_final_waypoint() &&
        position_error_m.norm() < arrival_tolerance_m_)
    {
        arrival_reported_ = true;
        status.arrived_edge = true;
        status.arrival_error_m = position_error_m.norm();
    }
    status.p_desired = routed_target;
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
