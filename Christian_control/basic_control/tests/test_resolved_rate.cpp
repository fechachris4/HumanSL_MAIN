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
#include "app/Config.h"
#include "math/DualArmKinematics.h"
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

    Dynamics dynamics(GEN3_DUAL_URDF_PATH);
    DualArmKinematics model(
        dynamics, config::kLeftNominalRad,
        config::kRightBaseFrame,
        config::kRightEndEffectorFrame);
    TargetStore targets;
    ResolvedRate controller(model, targets, kp, lambda, arrival_tol_m);

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
    const PositionJacobian ee =
        model.RightPositionAndJacobian(state.q_rad, workspace);
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

    // Measured tool orientation: unit Hamilton quaternion, hemisphere-fixed
    // to w >= 0, equal (up to the q/-q sign ambiguity) to the quaternion of
    // the FK rotation at the same configuration.
    Check((ee.rotation * ee.rotation.transpose() - Eigen::Matrix3d::Identity())
                  .norm() < 1e-9,
          "PositionJacobian.rotation is orthonormal");
    Check(std::abs(status2.tool_quat.norm() - 1.0) < 1e-9,
          "tool_quat has unit norm");
    Check(status2.tool_quat.w() >= 0.0, "tool_quat is hemisphere-fixed (w >= 0)");
    {
        Eigen::Quaterniond fk_quat(ee.rotation);
        if (fk_quat.w() < 0.0)
            fk_quat.coeffs() = -fk_quat.coeffs();
        Check((status2.tool_quat.coeffs() - fk_quat.coeffs()).norm() < 1e-9,
              "tool_quat matches the FK rotation");
    }

    // Same checks at a second, distinct configuration (guards against the
    // quaternion being stale or computed from the wrong q).
    {
        RobotState state_b = state;
        for (int i = 0; i < 7; ++i)
            state_b.q_rad[i] = -0.4 + 0.15 * i;
        ControllerStatus status_b;
        controller.DesiredVelocity(state_b, 0.01, status_b);

        const PoseJacobian pose_b =
            model.RightPoseAndJacobian(state_b.q_rad, workspace);
        Eigen::Quaterniond fk_quat_b(pose_b.rotation);
        if (fk_quat_b.w() < 0.0)
            fk_quat_b.coeffs() = -fk_quat_b.coeffs();
        Check(std::abs(status_b.tool_quat.norm() - 1.0) < 1e-9,
              "tool_quat has unit norm at a second configuration");
        Check(status_b.tool_quat.w() >= 0.0,
              "tool_quat stays hemisphere-fixed at a second configuration");
        Check((status_b.tool_quat.coeffs() - fk_quat_b.coeffs()).norm() < 1e-9,
              "tool_quat matches forward_kinematics at a second configuration");
    }

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
