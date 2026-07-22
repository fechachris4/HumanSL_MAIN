//
// Dls: damped least-squares (Levenberg-style) resolution of a desired
// Cartesian velocity into joint velocities. Header-only, Eigen-only — no
// robot, no Pinocchio — so it is testable in isolation.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_DLS_H
#define HUMANSL_MASTERS_PROJECT_2025_DLS_H

#include <algorithm>

#include <Eigen/Dense>

// qdot = Jpᵀ (Jp Jpᵀ + λ² I₃)⁻¹ v_desired
//
// for the translational Jacobian Jp (3×7) and desired linear velocity
// v_desired (m/s). Follows Pinocchio's official inverse-kinematics example:
// build the small 3×3 system, add λ² on the diagonal, solve with LDLT — no
// explicit matrix inverse. λ (meters-per-radian scale) trades tracking
// accuracy near singularities for bounded joint velocities: at a singular
// pose the solution stays finite instead of blowing up.
//
// Fixed-size types throughout: no heap allocation, safe in the 1 kHz loop.
inline Eigen::Matrix<double, 7, 1>
DampedLeastSquares(const Eigen::Matrix<double, 3, 7>& jacobian_position,
                   const Eigen::Vector3d& v_desired, double lambda)
{
    Eigen::Matrix3d jjt = jacobian_position * jacobian_position.transpose();
    jjt.diagonal().array() += lambda * lambda;
    return jacobian_position.transpose() * jjt.ldlt().solve(v_desired);
}

// Integration time step for the resolved-rate integrator: the measured
// elapsed cycle time, but never more than twice the nominal period — a
// scheduler stall must not integrate into one large position jump (the
// base's tracking safety faults on those; see Config.h's kQdotLimitDegS
// comment).
inline double ClampedCycleDt(double measured_dt_s, double nominal_dt_s)
{
    return std::min(measured_dt_s, 2.0 * nominal_dt_s);
}

#endif // HUMANSL_MASTERS_PROJECT_2025_DLS_H
