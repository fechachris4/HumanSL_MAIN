//
// Hardware-free tests for the controller's safety-relevant pure logic:
// damped-least-squares resolution, target parsing, TargetStore snapshots.
// No robot, no sessions, no Pinocchio. Returns nonzero on the first failure.
//

#include <cmath>
#include <iostream>
#include <string>

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

} // namespace

int main()
{
    TestDampedLeastSquares();
    TestClampedCycleDt();
    TestParseCartesianTarget();
    TestTargetStore();
    if (failures == 0) {
        std::cout << "all control-logic tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
