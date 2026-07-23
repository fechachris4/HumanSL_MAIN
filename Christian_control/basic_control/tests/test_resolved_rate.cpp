//
// Tests for the ResolvedRate controller against the real URDF model:
// hold-at-seed produces zero velocity, a known Cartesian error reproduces
// the closed-form DLS of the Pinocchio Jacobian, and the arrival edge fires
// exactly once per new target. Needs the bundled Pinocchio (Linux hardware
// machine only).
//

#include <cmath>
#include <iostream>
#include <string>

#include "control/ResolvedRate.h"
#include "math/Kinematics.h"
#include "Dynamics.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }
} // namespace

int main()
{
    const double kp = 1.0;
    const double lambda = 0.1;
    const double arrival_tol_m = 0.001;

    Dynamics dynamics(GEN3_URDF_PATH);
    TargetStore targets;
    ResolvedRate controller(dynamics, targets, kp, lambda, arrival_tol_m,
                            "EndEffector_Link");

    RobotState state;
    state.qdot_rad_s.setZero();
    for (int i = 0; i < 7; ++i)
        state.q_rad[i] = 0.3 + 0.2 * i; // an unremarkable, non-singular pose

    // Reset seeds p_desired = p(q): holding commands zero velocity.
    controller.Reset(state);
    ControllerStatus status;
    auto qdot_hold = controller.DesiredVelocity(state, 0.01, status);
    Check(qdot_hold.norm() < 1e-12, "hold at the seeded target commands zero velocity");
    Check(!status.arrived_edge, "the seeded hold target never fires the arrival edge");

    // A known error must reproduce the closed-form DLS of the Jacobian.
    const Eigen::Vector3d offset(0.05, -0.02, 0.03);
    targets.Store(status.p_current + offset);
    ControllerStatus status2;
    auto qdot = controller.DesiredVelocity(state, 0.01, status2);

    KinematicsWorkspace workspace(dynamics);
    const pinocchio::FrameIndex frame =
        dynamics.model_.getFrameId("EndEffector_Link");
    Eigen::VectorXd q(7);
    for (int i = 0; i < 7; ++i)
        q[i] = state.q_rad[i];
    const PositionJacobian ee = position_and_jacobian(
        dynamics, dynamics.convertJointAnglesToConfig(q), frame, workspace);
    Eigen::Matrix3d jjt = ee.jacobian_p * ee.jacobian_p.transpose();
    jjt.diagonal().array() += lambda * lambda;
    const Eigen::Matrix<double, 7, 1> reference =
        ee.jacobian_p.transpose() * jjt.inverse() * (kp * offset);
    Check((qdot - reference).norm() < 1e-9,
          "DesiredVelocity matches closed-form DLS of the Pinocchio Jacobian");
    Check((status2.p_current - ee.position).norm() < 1e-12,
          "status reports the FK position of the same q");

    // sigma_min (decision 13): sqrt of the smallest eigenvalue of Jp Jpᵀ.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(ee.jacobian_p *
                                                      ee.jacobian_p.transpose());
    const double sigma_ref = std::sqrt(std::max(0.0, es.eigenvalues()(0)));
    Check(std::isfinite(status2.sigma_min) && status2.sigma_min > 0.0,
          "sigma_min is finite and positive away from singularity");
    Check(std::abs(status2.sigma_min - sigma_ref) < 1e-12,
          "sigma_min matches the independent eigen-solve");

    // Arrival edge: a NEW target already within tolerance fires exactly once.
    targets.Store(status2.p_current);
    ControllerStatus status3;
    controller.DesiredVelocity(state, 0.01, status3);
    Check(status3.arrived_edge, "a new target within tolerance fires the arrival edge");
    Check(status3.arrival_error_m < arrival_tol_m, "arrival error is under the tolerance");
    ControllerStatus status4;
    controller.DesiredVelocity(state, 0.01, status4);
    Check(!status4.arrived_edge, "the arrival edge fires only once per target");

    if (failures == 0) {
        std::cout << "all resolved-rate tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
