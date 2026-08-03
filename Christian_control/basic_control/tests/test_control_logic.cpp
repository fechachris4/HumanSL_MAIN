//
// Hardware-free tests for the controller's safety-relevant pure logic:
// damped-least-squares resolution, target parsing, TargetStore snapshots,
// PositionIntegration actuation. No robot, no sessions, no Pinocchio.
// Returns nonzero on the first failure.
//

#include <cmath>
#include <iostream>
#include <string>

#include "actuation/PositionIntegration.h"
#include "control/Target.h"
#include "math/Dls.h"

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

    void TestDampedLeastSquares()
    {
        // Identity-like Jacobian on the first three joints: with lambda = 0
        // the solution must reproduce v exactly on those joints.
        Eigen::Matrix<double, 3, 7> jacobian = Eigen::Matrix<double, 3, 7>::Zero();
        jacobian(0, 0) = 1.0;
        jacobian(1, 1) = 1.0;
        jacobian(2, 2) = 1.0;
        const Eigen::Vector3d v(0.1, -0.2, 0.3);

        auto qdot = DampedLeastSquares(jacobian, v, 0.0);
        Check((qdot.head<3>() - v).norm() < 1e-12, "undamped exact solution on square part");
        Check(qdot.tail<4>().norm() < 1e-12, "joints outside the Jacobian stay at zero");

        // Damping shrinks the solution but keeps its direction.
        auto qdot_damped = DampedLeastSquares(jacobian, v, 0.5);
        Check(qdot_damped.norm() < qdot.norm(), "damping reduces the velocity norm");
        Check(qdot_damped.head<3>().dot(v) > 0, "damped solution keeps the direction");

        // Verify against the definition qdot = Jt (J Jt + l^2 I)^-1 v.
        const double lambda = 0.3;
        Eigen::Matrix3d jjt = jacobian * jacobian.transpose();
        jjt.diagonal().array() += lambda * lambda;
        Eigen::Matrix<double, 7, 1> reference = jacobian.transpose() * jjt.inverse() * v;
        auto qdot_check = DampedLeastSquares(jacobian, v, lambda);
        Check((qdot_check - reference).norm() < 1e-12, "matches the closed-form definition");

        // Singular Jacobian (rank 1): undamped least squares would blow up
        // asking for velocity orthogonal to the reachable direction; the
        // damped solution must stay finite and bounded.
        Eigen::Matrix<double, 3, 7> singular = Eigen::Matrix<double, 3, 7>::Zero();
        singular(0, 0) = 1.0; // only x is reachable
        auto qdot_singular =
            DampedLeastSquares(singular, Eigen::Vector3d(0.0, 1.0, 0.0), 0.1);
        Check(qdot_singular.allFinite(), "singular Jacobian yields finite velocities");
        Check(qdot_singular.norm() < 1e-9,
              "unreachable direction commands ~zero, not a blow-up");
    }

    void TestParseCartesianTarget()
    {
        std::string error;
        auto ok = ParseCartesianTarget("0.4 0.1 0.3", error);
        Check(ok.has_value(), "valid x y z accepted");
        Check(ok && (*ok - Eigen::Vector3d(0.4, 0.1, 0.3)).norm() == 0.0,
              "parsed values are exact");

        Check(!ParseCartesianTarget("0.4 0.1", error).has_value(), "too few rejected");
        Check(!ParseCartesianTarget("0.4 0.1 0.3 0.2", error).has_value(),
              "too many rejected");
        Check(!ParseCartesianTarget("0.4 x 0.3", error).has_value(), "non-numeric rejected");
        Check(!ParseCartesianTarget("nan 0 0", error).has_value(), "NaN rejected");
        Check(!ParseCartesianTarget("inf 0 0", error).has_value(), "Inf rejected");
        Check(ParseCartesianTarget("-0.4 -0.1 0.0", error).has_value(),
              "negative coordinates accepted");
    }

    void TestClampedCycleDt()
    {
        Check(ClampedCycleDt(0.010, 0.010) == 0.010, "nominal dt passes through");
        Check(ClampedCycleDt(0.012, 0.010) == 0.012, "small jitter passes through");
        Check(ClampedCycleDt(0.250, 0.010) == 0.020, "a stall clamps to 2x nominal");
    }

    void TestTargetStore()
    {
        TargetStore store;
        Check(store.Get().sequence == 0, "fresh store has sequence 0");
        const Eigen::Vector3d p(0.4, 0.1, 0.3);
        store.Store(p);
        TargetStore::Snapshot snap = store.Get();
        Check(snap.sequence == 1, "sequence increments on Store");
        Check((snap.p_desired - p).norm() == 0.0, "snapshot returns the stored position");
        store.Store(p);
        Check(store.Get().sequence == 2, "sequence increments again");
    }

    void TestPositionIntegration()
    {
        constexpr double kDegToRad = M_PI / 180.0;
        constexpr double kRadToDeg = 180.0 / M_PI;

        RobotState seed;
        seed.qdot_rad_s.setZero();
        for (int i = 0; i < 7; ++i)
            seed.q_rad[i] = 0.1 * (i + 1); // rad

        PositionIntegration act;
        act.Prepare(seed);

        // Zero velocity holds the seeded position exactly, reports zero.
        JointVector setpoints{};
        JointVector velocity{};
        act.Apply(Eigen::Matrix<double, 7, 1>::Zero(), seed, 0.01,
                  setpoints, velocity);
        Check(std::abs(setpoints[0] - 0.1 * kRadToDeg) < 1e-12,
              "hold keeps the seeded position");
        Check(velocity[3] == 0.0, "hold reports zero velocity");

        // Integration arithmetic: q_command += q̇·dt, outputs in degrees.
        const Eigen::Matrix<double, 7, 1> qdot =
            Eigen::Matrix<double, 7, 1>::Constant(0.5); // rad/s
        act.Apply(qdot, seed, 0.02, setpoints, velocity);
        Check(std::abs(setpoints[0] - (0.1 + 0.5 * 0.02) * kRadToDeg) < 1e-12,
              "Apply integrates q̇·dt onto the persistent command");
        Check(std::abs(velocity[0] - 0.5 * kRadToDeg) < 1e-12,
              "Apply reports the applied velocity in deg/s");

        // Tracking error: measurement still at the seed, command has moved.
        auto error = act.TrackingErrorDeg(seed);
        Check(error.has_value(), "PositionIntegration provides a tracking guard");
        Check(std::abs((*error)[0] - 0.5 * 0.02 * kRadToDeg) < 1e-9,
              "tracking error = |command - measured|");

        // Wrap: measured 359 deg vs command 1 deg is 2 deg apart, not 358.
        PositionIntegration act_wrap;
        RobotState near_zero;
        near_zero.qdot_rad_s.setZero();
        near_zero.q_rad.setZero();
        near_zero.q_rad[0] = 1.0 * kDegToRad;
        act_wrap.Prepare(near_zero);
        RobotState wrapped = near_zero;
        wrapped.q_rad[0] = 359.0 * kDegToRad;
        auto wrap_error = act_wrap.TrackingErrorDeg(wrapped);
        Check(std::abs((*wrap_error)[0] - 2.0) < 1e-9,
              "tracking error shifts the measurement to within ±180 deg");

        // Runtime lead limiter: a stationary plant can never accumulate an
        // unbounded command gap. The returned status is telemetry — it
        // records what was requested and which joints the limiter changed.
        PositionIntegration limited(1.0);
        RobotState stationary;
        stationary.q_rad.setZero();
        stationary.qdot_rad_s.setZero();
        limited.Prepare(stationary);
        const auto limited_status = limited.Apply(
            Eigen::Matrix<double, 7, 1>::Constant(1.0), stationary, 0.1,
            setpoints, velocity);
        Check(limited_status.lead_limited[0],
              "lead limiter reports an active constraint");
        Check(std::abs(setpoints[0] - 1.0) < 1e-9,
              "stationary feedback bounds the position command to 1 deg lead");
        // requested is the unconstrained proposal: 1 rad/s for 0.1 s is
        // 0.1 rad = 5.7296 deg, well past the 1 deg the limiter allowed.
        Check(std::abs(limited_status.requested_deg[0] -
                       0.1 * 180.0 / M_PI) < 1e-9,
              "requested setpoint is recorded before the lead limiter");
        Check(limited_status.requested_deg[0] > setpoints[0],
              "requested exceeds sent exactly where the limiter engaged");

        // And where the limiter does NOT engage, requested == sent, so
        // offline tooling can trust req - cmd as the limiter's whole effect.
        PositionIntegration free_run(1.0);
        free_run.Prepare(stationary);
        const auto free_status = free_run.Apply(
            Eigen::Matrix<double, 7, 1>::Constant(0.001), stationary, 0.1,
            setpoints, velocity);
        Check(!free_status.lead_limited[0],
              "small step leaves the lead limiter inactive");
        Check(std::abs(free_status.requested_deg[0] - setpoints[0]) < 1e-12,
              "requested equals sent when no constraint is active");
    }

} // namespace

int main()
{
    TestDampedLeastSquares();
    TestClampedCycleDt();
    TestParseCartesianTarget();
    TestTargetStore();
    TestPositionIntegration();
    if (failures == 0) {
        std::cout << "all control-logic tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
