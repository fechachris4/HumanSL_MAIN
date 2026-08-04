//
// Hardware-free tests for the controller's safety-relevant pure logic:
// damped-least-squares resolution, cycle-dt clamping, PositionIntegration
// actuation. No robot, no sessions, no Pinocchio.
// Returns nonzero on the first failure.
//

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "Actuation.h"
#include "ReactiveLaw.h"

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

    void TestClampedCycleDt()
    {
        Check(ClampedCycleDt(0.010, 0.010) == 0.010, "nominal dt passes through");
        Check(ClampedCycleDt(0.012, 0.010) == 0.012, "small jitter passes through");
        Check(ClampedCycleDt(0.250, 0.010) == 0.020, "a stall clamps to 2x nominal");
        Check(ClampedCycleDt(-0.001, 0.010) == 0.0,
              "a negative measured dt fails safe to zero");
        Check(ClampedCycleDt(std::numeric_limits<double>::quiet_NaN(), 0.010) == 0.0,
              "a non-finite measured dt fails safe to zero");
    }

    void TestInvalidNominalDtHoldsPositionIntegration()
    {
        constexpr double kDegToRad = M_PI / 180.0;
        constexpr double kPreviousDeg = 43.1163;
        RobotState seed;
        seed.q_rad.setZero();
        seed.qdot_rad_s.setZero();
        seed.q_rad[0] = kPreviousDeg * kDegToRad;
        RobotState discontinuous_feedback = seed;
        discontinuous_feedback.q_rad[0] = 41.1788 * kDegToRad;
        Eigen::Matrix<double, 7, 1> qdot =
            Eigen::Matrix<double, 7, 1>::Zero();
        qdot[0] = 14.0916 * kDegToRad;

        for (const double invalid_nominal_dt_s : {
                 0.0,
                 -0.002,
                 std::numeric_limits<double>::quiet_NaN(),
                 -std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(),
             })
        {
            const double dt_s = ClampedCycleDt(0.002, invalid_nominal_dt_s);
            Check(dt_s == 0.0, "an invalid nominal dt fails safe to zero");

            PositionIntegration actuation(1.0);
            actuation.Prepare(seed);
            JointVector setpoints{};
            JointVector velocity{};
            const auto status = actuation.Apply(qdot, discontinuous_feedback, dt_s,
                                                setpoints, velocity);
            Check(std::abs(status.requested_deg[0] - kPreviousDeg) < 1e-12,
                  "invalid nominal dt records an exact requested hold");
            Check(std::abs(setpoints[0] - kPreviousDeg) < 1e-12,
                  "invalid nominal dt holds the sent position command");
            Check(velocity[0] == 0.0,
                  "invalid nominal dt reports zero applied velocity");
        }
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

        // A discontinuous feedback step must not make the lead projection
        // overwrite the clamped per-cycle velocity. This recreates joint 6
        // from the P2 log: the previous command was 43.1163 deg, feedback
        // suddenly read 41.1788 deg, and the Runner had already clipped the
        // requested velocity to +14.0916 deg/s for this 2 ms cycle.
        constexpr int kJoint6 = 5;
        constexpr double kPreviousDeg = 43.1163;
        constexpr double kFeedbackDeg = 41.1788;
        constexpr double kClampedDegS = 14.0916;
        constexpr double kDtS = 0.002;
        constexpr double kMaxStepDeg = kClampedDegS * kDtS;
        RobotState discontinuous_seed;
        discontinuous_seed.q_rad.setZero();
        discontinuous_seed.qdot_rad_s.setZero();
        discontinuous_seed.q_rad[kJoint6] = kPreviousDeg * kDegToRad;
        RobotState discontinuous_feedback = discontinuous_seed;
        discontinuous_feedback.q_rad[kJoint6] = kFeedbackDeg * kDegToRad;
        Eigen::Matrix<double, 7, 1> discontinuous_qdot =
            Eigen::Matrix<double, 7, 1>::Zero();
        discontinuous_qdot[kJoint6] = kClampedDegS * kDegToRad;
        PositionIntegration discontinuous(1.0);
        discontinuous.Prepare(discontinuous_seed);
        const auto discontinuous_status = discontinuous.Apply(
            discontinuous_qdot, discontinuous_feedback, kDtS, setpoints, velocity);
        Check(discontinuous_status.lead_limited[kJoint6],
              "a discontinuous feedback step reports the active lead constraint");
        Check(std::abs(discontinuous_status.requested_deg[kJoint6] -
                       (kPreviousDeg + kMaxStepDeg)) < 1e-12,
              "requested setpoint remains before all constraints");
        Check(std::abs(setpoints[kJoint6] - kPreviousDeg) <= kMaxStepDeg + 1e-12,
              "a discontinuous feedback step cannot snap the setpoint by 0.94 deg");
        Check(std::abs(velocity[kJoint6]) <= kClampedDegS + 1e-12,
              "applied setpoint velocity cannot exceed the clamped velocity magnitude");
        Check(std::abs(velocity[kJoint6] -
                       (setpoints[kJoint6] - kPreviousDeg) / kDtS) < 1e-9,
              "applied velocity reports the final constrained setpoint step");

        // Sign-mirror the discontinuity fixture: a negative clamped qdot
        // must get the same final rate envelope and never produce a snap.
        RobotState negative_seed = discontinuous_seed;
        negative_seed.q_rad[kJoint6] = -kPreviousDeg * kDegToRad;
        RobotState negative_feedback = negative_seed;
        negative_feedback.q_rad[kJoint6] = -kFeedbackDeg * kDegToRad;
        Eigen::Matrix<double, 7, 1> negative_qdot =
            Eigen::Matrix<double, 7, 1>::Zero();
        negative_qdot[kJoint6] = -kClampedDegS * kDegToRad;
        PositionIntegration negative_discontinuous(1.0);
        negative_discontinuous.Prepare(negative_seed);
        const auto negative_status = negative_discontinuous.Apply(
            negative_qdot, negative_feedback, kDtS, setpoints, velocity);
        Check(negative_status.lead_limited[kJoint6],
              "negative qdot reports the active lead constraint");
        Check(std::abs(setpoints[kJoint6] + kPreviousDeg) <= kMaxStepDeg + 1e-12,
              "negative qdot cannot snap the setpoint across a discontinuity");
        Check(std::abs(velocity[kJoint6]) <= kClampedDegS + 1e-12,
              "negative qdot respects the clamped velocity magnitude");

        // With no permitted step, lead recovery must wait for a later cycle
        // rather than snapping the command toward the discontinuous feedback.
        PositionIntegration zero_step(1.0);
        zero_step.Prepare(discontinuous_seed);
        const auto zero_step_status = zero_step.Apply(
            Eigen::Matrix<double, 7, 1>::Zero(), discontinuous_feedback, kDtS,
            setpoints, velocity);
        Check(zero_step_status.lead_limited[kJoint6],
              "zero qdot still reports an active lead constraint");
        Check(std::abs(setpoints[kJoint6] - kPreviousDeg) < 1e-12,
              "zero qdot cannot snap toward discontinuous feedback");
        Check(velocity[kJoint6] == 0.0,
              "zero qdot reports zero applied velocity after both constraints");

        // Apply is also fail-safe when called directly: invalid dt must not
        // move the persistent command, reverse a positive qdot, or emit a
        // nonzero applied velocity even with discontinuous feedback.
        for (const double invalid_dt_s : {
                 0.0,
                 -kDtS,
                 std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::infinity(),
             })
        {
            PositionIntegration invalid_dt(1.0);
            invalid_dt.Prepare(discontinuous_seed);
            const auto invalid_status = invalid_dt.Apply(
                discontinuous_qdot, discontinuous_feedback, invalid_dt_s,
                setpoints, velocity);
            Check(std::abs(invalid_status.requested_deg[kJoint6] - kPreviousDeg) < 1e-12,
                  "invalid dt records an exact hold as the requested setpoint");
            Check(std::abs(setpoints[kJoint6] - kPreviousDeg) < 1e-12,
                  "invalid dt holds the command despite discontinuous feedback");
            Check(velocity[kJoint6] == 0.0,
                  "invalid dt reports zero applied velocity");
        }

        // Wrapped feedback still uses the nearest turn and follows the same
        // final rate envelope, rather than treating 359 to 1 deg as a jump.
        RobotState wrap_seed;
        wrap_seed.q_rad.setZero();
        wrap_seed.qdot_rad_s.setZero();
        wrap_seed.q_rad[0] = 359.0 * kDegToRad;
        RobotState wrap_feedback = wrap_seed;
        wrap_feedback.q_rad[0] = 1.0 * kDegToRad;
        Eigen::Matrix<double, 7, 1> wrap_qdot =
            Eigen::Matrix<double, 7, 1>::Zero();
        wrap_qdot[0] = 1.0 * kDegToRad;
        PositionIntegration wrapped(1.0);
        wrapped.Prepare(wrap_seed);
        const auto wrap_status = wrapped.Apply(wrap_qdot, wrap_feedback, 0.1,
                                               setpoints, velocity);
        Check(wrap_status.lead_limited[0],
              "wrapped feedback still detects an excessive command lead");
        Check(std::abs(setpoints[0] - 359.1) < 1e-12,
              "wrapped feedback respects the final positive rate envelope");
        Check(std::abs(velocity[0] - 1.0) < 1e-12,
              "wrapped feedback reports the final applied velocity");
    }

    void TestJointLimitWarningGuard()
    {
        constexpr double kDegToRad = M_PI / 180.0;
        constexpr double kDtS = 0.1;
        constexpr int kJoint1 = 0;
        constexpr int kJoint4 = 3;
        constexpr int kJoint6 = 5;

        const auto state_at_deg = [](int joint, double deg)
        {
            RobotState state;
            state.q_rad.setZero();
            state.qdot_rad_s.setZero();
            state.q_rad[joint] = deg * kDegToRad;
            return state;
        };
        const auto qdot_deg_s = [](int joint, double deg_s)
        {
            Eigen::Matrix<double, 7, 1> qdot =
                Eigen::Matrix<double, 7, 1>::Zero();
            qdot[joint] = deg_s * kDegToRad;
            return qdot;
        };

        // A high-side warning crossing on bounded joint 6 blocks the whole
        // seven-joint command frame before it can be sent.
        RobotState high_seed = state_at_deg(kJoint6, 117.0);
        high_seed.q_rad[kJoint1] = 10.0 * kDegToRad;
        PositionIntegration high_guard;
        high_guard.Prepare(high_seed);
        Eigen::Matrix<double, 7, 1> high_qdot = qdot_deg_s(kJoint6, 20.0);
        high_qdot[kJoint1] = 10.0 * kDegToRad;
        JointVector setpoints{};
        JointVector velocity{};
        const auto high_status =
            high_guard.Apply(high_qdot, high_seed, kDtS, setpoints, velocity);
        Check(high_status.joint_limit_warning_joint == kJoint6,
              "high warning crossing identifies the bounded joint for the stop");
        Check(std::abs(setpoints[kJoint6] - 117.0) < 1e-12,
              "high warning crossing holds the bounded joint's last safe command");
        Check(std::abs(setpoints[kJoint1] - 10.0) < 1e-12,
              "high warning crossing holds every other joint in the frame");
        for (double v : velocity)
            Check(v == 0.0, "high warning crossing reports zero applied velocity");

        // The symmetric low-side crossing uses Kortex's raw 215 deg form of
        // signed -145 deg for joint 4 and must be blocked too.
        const RobotState low_seed = state_at_deg(kJoint4, 215.0);
        PositionIntegration low_guard;
        low_guard.Prepare(low_seed);
        const auto low_status = low_guard.Apply(qdot_deg_s(kJoint4, -20.0), low_seed,
                                                kDtS, setpoints, velocity);
        Check(low_status.joint_limit_warning_joint == kJoint4,
              "low warning crossing identifies the bounded joint for the stop");
        Check(std::abs(setpoints[kJoint4] - 215.0) < 1e-12,
              "low warning crossing holds a raw-wrapped bounded command");
        Check(velocity[kJoint4] == 0.0,
              "low warning crossing reports zero bounded-joint velocity");

        // Motion fully inside a bounded joint's warning band remains live.
        const RobotState inside_seed = state_at_deg(kJoint6, 116.0);
        PositionIntegration inside_guard;
        inside_guard.Prepare(inside_seed);
        const auto inside_status = inside_guard.Apply(qdot_deg_s(kJoint6, 10.0),
                                                      inside_seed, kDtS,
                                                      setpoints, velocity);
        Check(!inside_status.joint_limit_warning_joint,
              "inside-warning motion does not request a joint-limit stop");
        Check(std::abs(setpoints[kJoint6] - 117.0) < 1e-12,
              "inside-warning motion remains allowed");

        // Joint 4's raw 355 deg is signed -5 deg. A positive step across
        // the 360-degree representation boundary must not look like a
        // positive-limit crossing.
        const RobotState wrap_seed = state_at_deg(kJoint4, 355.0);
        PositionIntegration wrap_guard;
        wrap_guard.Prepare(wrap_seed);
        const auto wrap_status = wrap_guard.Apply(qdot_deg_s(kJoint4, 60.0),
                                                  wrap_seed, kDtS,
                                                  setpoints, velocity);
        Check(!wrap_status.joint_limit_warning_joint,
              "raw wrap does not request a joint-limit stop");
        Check(std::abs(setpoints[kJoint4] - 361.0) < 1e-12,
              "raw 355-degree bounded feedback is normalized before warning checks");

        // Continuous joints have a zero warning sentinel and remain free to
        // cross their angular representation boundary.
        const RobotState continuous_seed = state_at_deg(kJoint1, 179.0);
        PositionIntegration continuous_guard;
        continuous_guard.Prepare(continuous_seed);
        const auto continuous_status =
            continuous_guard.Apply(qdot_deg_s(kJoint1, 20.0), continuous_seed, kDtS,
                                   setpoints, velocity);
        Check(!continuous_status.joint_limit_warning_joint,
              "continuous joints do not request a joint-limit stop");
        Check(std::abs(setpoints[kJoint1] - 181.0) < 1e-12,
              "continuous joints are excluded from the position warning guard");

        // The reproduced failure mode: joint 6 is already beyond warning at
        // 123 deg, so another outward proposal must still be blocked.
        const RobotState beyond_warning_seed = state_at_deg(kJoint6, 123.0);
        PositionIntegration beyond_warning_guard;
        beyond_warning_guard.Prepare(beyond_warning_seed);
        const auto beyond_warning_status = beyond_warning_guard.Apply(
            qdot_deg_s(kJoint6, 20.0), beyond_warning_seed, kDtS,
            setpoints, velocity);
        Check(beyond_warning_status.joint_limit_warning_joint == kJoint6,
              "outward motion beyond warning requests the joint-limit stop");
        Check(std::abs(setpoints[kJoint6] - 123.0) < 1e-12,
              "outward motion beyond warning holds the last safe command");

        // A joint that starts beyond warning may still move back inward.
        const RobotState recovery_seed = state_at_deg(kJoint6, 123.0);
        PositionIntegration recovery_guard;
        recovery_guard.Prepare(recovery_seed);
        const auto recovery_status =
            recovery_guard.Apply(qdot_deg_s(kJoint6, -20.0), recovery_seed, kDtS,
                                 setpoints, velocity);
        Check(!recovery_status.joint_limit_warning_joint,
              "inward recovery does not request a joint-limit stop");
        Check(std::abs(setpoints[kJoint6] - 121.0) < 1e-12,
              "inward recovery remains allowed beyond the warning threshold");
    }

} // namespace

int main()
{
    TestDampedLeastSquares();
    TestClampedCycleDt();
    TestInvalidNominalDtHoldsPositionIntegration();
    TestPositionIntegration();
    TestJointLimitWarningGuard();
    if (failures == 0) {
        std::cout << "all control-logic tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
