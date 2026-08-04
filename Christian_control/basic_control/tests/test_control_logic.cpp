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
#include "Arrival.h"
#include "Freshness.h"
#include "ReactiveLaw.h"
#include "StopPriority.h"

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
        constexpr int kJoint2 = 1;
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

        // Joint 2 must stop at the conservative software guard of 126.9
        // deg, before the independently configured 130 deg firmware
        // warning. This is deliberately a crossing that the firmware
        // threshold alone would still admit.
        const RobotState joint2_seed = state_at_deg(kJoint2, 126.8);
        PositionIntegration joint2_guard;
        joint2_guard.Prepare(joint2_seed);
        const auto joint2_status = joint2_guard.Apply(qdot_deg_s(kJoint2, 2.0),
                                                       joint2_seed, kDtS,
                                                       setpoints, velocity);
        Check(config::kJointLimitWarnDeg[kJoint2] == 130.0,
              "joint 2 firmware warning remains its independently configured 130 deg");
        Check(joint2_status.joint_limit_warning_joint == kJoint2,
              "joint 2 blocks at its 126.9 deg software bound before firmware warning");
        Check(std::abs(setpoints[kJoint2] - 126.8) < 1e-12,
              "joint 2 software-bound crossing holds the last safe command");

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

    void TestJointPositionConfiguration()
    {
        Check(config::kJointLowerDeg ==
                  JointVector{0, -128.9, 0, -147.8, 0, -120.3, 0},
              "Table 39 lower position concepts match the Gen3 7-DoF values");
        Check(config::kJointUpperDeg ==
                  JointVector{0, 128.9, 0, 147.8, 0, 120.3, 0},
              "Table 39 upper position concepts match the Gen3 7-DoF values");
        Check(config::kJointBoundedMask == JointVector{0, 1, 0, 1, 0, 1, 0},
              "only joints 2, 4, and 6 are position-bounded");
        Check(config::kNullMidpointDeg == JointVector{0, 0, 0, 0, 0, 0, 0},
              "null midpoint is derived from the bounded position concepts");
        Check(config::kNullCenteringMask == config::kJointBoundedMask,
              "null centering mask aliases the bounded-joint mask");
        Check(config::kJointSoftwareLimitDeg ==
                  JointVector{0, 126.9, 0, 145.0, 0, 118.0, 0},
              "software position bounds apply the 2 deg margin without widening firmware warnings");
    }

    void TestStopPriority()
    {
        const auto warning_following = ResolveStopPriority(
            true, false, false, true, false);
        Check(warning_following.reason == StopPriorityReason::kFollowingError,
              "following error wins over a simultaneous held-frame warning");
        Check(!warning_following.live_fault_observed,
              "warning plus following error does not invent a live fault");

        const auto warning_left_low = ResolveStopPriority(
            false, false, true, true, false);
        Check(warning_left_low.reason == StopPriorityReason::kLeftLowLevel,
              "leaving low-level servoing wins over a simultaneous held-frame warning");

        const auto warning_ignored_fault = ResolveStopPriority(
            false, true, false, true, false);
        Check(warning_ignored_fault.reason == StopPriorityReason::kJointLimitWarning &&
                  warning_ignored_fault.live_fault_observed,
              "ignored live fault taints but does not mask a simultaneous warning");

        const auto warning_stopping_fault = ResolveStopPriority(
            false, true, false, true, true);
        Check(warning_stopping_fault.reason == StopPriorityReason::kRobotFault &&
                  warning_stopping_fault.live_fault_observed,
              "enabled live-fault stop wins over a simultaneous warning");

        const auto warning_fault_left_low = ResolveStopPriority(
            false, true, true, true, false);
        Check(warning_fault_left_low.reason == StopPriorityReason::kLeftLowLevel &&
                  warning_fault_left_low.live_fault_observed,
              "unconditional servo loss wins over an ignored fault and warning");

        const auto following_fault = ResolveStopPriority(
            true, true, false, false, false);
        Check(following_fault.reason == StopPriorityReason::kFollowingError &&
                  following_fault.live_fault_observed,
              "following error wins while a simultaneous live fault still taints");
    }

    void TestStaleAcknowledgementGuard()
    {
        constexpr int kStopCycles = config::kStaleFeedbackStopCycles;
        constexpr int kFrozenJoint = 4;
        FeedbackFreshnessMonitor freshness;
        std::array<std::uint32_t, 7> acknowledgement{};

        Check(kStopCycles == 25,
              "stale-feedback guard is configured for 25 cycles (50 ms at 500 Hz)");

        // The first returned cyclic frame only establishes the per-actuator
        // command-ID baseline; it must not consume any 50 ms stop budget.
        freshness.Update(acknowledgement);
        Check(!StaleAcknowledgementJoint(freshness.unchanged_cycles(), kStopCycles),
              "the acknowledgement seeding frame cannot request a stale-feedback stop");

        // Advancing command acknowledgements are healthy, even while other
        // feedback fields happen to be stationary.
        for (auto& value : acknowledgement)
            ++value;
        freshness.Update(acknowledgement);
        Check(!StaleAcknowledgementJoint(freshness.unchanged_cycles(), kStopCycles),
              "advancing acknowledgements remain below the stale-feedback stop");

        // A single frozen actuator reaches the stop exactly on sample 25,
        // not one completed feedback cycle earlier.
        for (int cycle = 1; cycle < kStopCycles; ++cycle)
        {
            for (int i = 0; i < 7; ++i)
                if (i != kFrozenJoint)
                    ++acknowledgement[i];
            freshness.Update(acknowledgement);
            Check(!StaleAcknowledgementJoint(freshness.unchanged_cycles(), kStopCycles),
                  "a frozen acknowledgement remains below the stop before 25 samples");
        }
        for (int i = 0; i < 7; ++i)
            if (i != kFrozenJoint)
                ++acknowledgement[i];
        freshness.Update(acknowledgement);
        Check(StaleAcknowledgementJoint(freshness.unchanged_cycles(), kStopCycles) ==
                  kFrozenJoint,
              "the 25th unchanged acknowledgement stops the exact frozen joint");

        // The condition is not latched: an advancing acknowledgement resets
        // this joint's count and removes the stale indication.
        ++acknowledgement[kFrozenJoint];
        freshness.Update(acknowledgement);
        Check(!StaleAcknowledgementJoint(freshness.unchanged_cycles(), kStopCycles),
              "an advancing acknowledgement resets the stale-feedback guard");

        // Detection is indexed by Kortex actuator order, not a hard-coded
        // joint: every one of the seven counters can independently stop.
        for (int joint = 0; joint < 7; ++joint)
        {
            std::array<int, 7> counts{};
            counts[joint] = kStopCycles;
            Check(StaleAcknowledgementJoint(counts, kStopCycles) == joint,
                  "each actuator acknowledgement counter can identify its own stale joint");
        }

        const auto stale_after_warning = ResolveStopPriority(
            false, false, false, true, true, false);
        Check(stale_after_warning.reason == StopPriorityReason::kJointLimitWarning,
              "joint-boundary warning outranks stale feedback in the same reply");
        const auto stale_after_following = ResolveStopPriority(
            true, false, false, false, false, true);
        Check(stale_after_following.reason == StopPriorityReason::kFollowingError,
              "following error outranks stale feedback in the same reply");
        const auto stale_after_servo_loss = ResolveStopPriority(
            false, false, true, false, false, true);
        Check(stale_after_servo_loss.reason == StopPriorityReason::kLeftLowLevel,
              "low-level servo loss outranks stale feedback in the same reply");
        const auto stale_after_enabled_fault = ResolveStopPriority(
            false, true, false, false, true, true);
        Check(stale_after_enabled_fault.reason == StopPriorityReason::kRobotFault,
              "an enabled live-fault stop outranks stale feedback in the same reply");
        const auto stale_before_counters = ResolveStopPriority(
            false, false, false, false, false, true);
        Check(stale_before_counters.reason == StopPriorityReason::kStaleFeedback,
              "stale feedback becomes the next stop after live-state and joint-boundary priority");
    }

    void TestArrivalSettling()
    {
        // 2 ms cycles at 500 Hz; the gate requires 100 ms inside tolerance.
        constexpr double kDt = 0.002;
        constexpr double kHold = 0.1;

        // A single in-tolerance cycle is the defect this gate exists to
        // reject: one noisy sample must not report arrival.
        ArrivalSettlingMonitor single(kHold);
        Check(!single.Update(true, kDt),
              "one in-tolerance cycle does not report arrival");

        // Arrival is reported once the accumulated in-tolerance time reaches
        // the threshold, and stays reported while it remains in tolerance.
        ArrivalSettlingMonitor settling(kHold);
        int cycles_to_settle = 0;
        for (int i = 0; i < 100; ++i) {
            if (settling.Update(true, kDt)) {
                cycles_to_settle = i + 1;
                break;
            }
        }
        Check(cycles_to_settle == 50,
              "100 ms of in-tolerance cycles at 2 ms settles on cycle 50");
        Check(settling.Update(true, kDt),
              "arrival stays reported while the error remains in tolerance");

        // One excursion outside tolerance discards the accumulated time: the
        // full window must be re-earned.
        ArrivalSettlingMonitor excursion(kHold);
        for (int i = 0; i < 40; ++i)
            excursion.Update(true, kDt);
        Check(!excursion.Update(false, kDt),
              "leaving tolerance clears the settled time");
        Check(excursion.settled_s() == 0.0,
              "an excursion resets the accumulator to zero");
        for (int i = 0; i < 49; ++i)
            Check(!excursion.Update(true, kDt),
                  "the settling window is re-earned in full after an excursion");
        Check(excursion.Update(true, kDt),
              "arrival is reported once the re-earned window completes");

        // A new target re-arms the gate.
        ArrivalSettlingMonitor rearmed(kHold);
        for (int i = 0; i < 60; ++i)
            rearmed.Update(true, kDt);
        rearmed.Rearm();
        Check(rearmed.settled_s() == 0.0, "Rearm clears the settled time");
        Check(!rearmed.Update(true, kDt),
              "a re-armed monitor does not report arrival on its first cycle");

        // Broken timing must not manufacture arrival, and must not destroy
        // genuine settling either: a bad dt neither accumulates nor resets.
        ArrivalSettlingMonitor bad_dt(kHold);
        for (int i = 0; i < 40; ++i)
            bad_dt.Update(true, kDt);
        const double settled_before = bad_dt.settled_s();
        bad_dt.Update(true, 0.0);
        bad_dt.Update(true, -kDt);
        bad_dt.Update(true, std::numeric_limits<double>::quiet_NaN());
        bad_dt.Update(true, std::numeric_limits<double>::infinity());
        Check(bad_dt.settled_s() == settled_before,
              "non-finite and non-positive dt neither accumulate nor reset");

        // A non-positive window disables the gate, matching the freshness
        // guard's convention, so the instantaneous behaviour stays reachable.
        ArrivalSettlingMonitor disabled(0.0);
        Check(disabled.Update(true, kDt),
              "a non-positive window reports arrival on the first in-tolerance cycle");
        Check(!disabled.Update(false, kDt),
              "a disabled gate still tracks the tolerance state itself");
    }

    void TestArrivalTimeout()
    {
        // 2 ms cycles at 500 Hz; a 0.1 s timeout is 50 cycles. The production
        // wiring uses kTargetHoldS (2.0 s); 0.1 s here is only test arithmetic.
        constexpr double kDt = 0.002;
        constexpr double kTimeout = 0.1;

        // Below the threshold, the non-arrival edge must not fire.
        ArrivalTimeoutMonitor early(kTimeout);
        for (int i = 0; i < 49; ++i)
            Check(!early.Update(true, kDt),
                  "no non-arrival edge before the timeout elapses");

        // A single one-shot edge exactly when the wait reaches the threshold,
        // and no re-fire while still waiting.
        ArrivalTimeoutMonitor fire(kTimeout);
        int fired_cycle = 0;
        for (int i = 0; i < 100; ++i) {
            if (fire.Update(true, kDt)) {
                fired_cycle = i + 1;
                break;
            }
        }
        Check(fired_cycle == 50,
              "100 ms of waiting at 2 ms fires the non-arrival edge on cycle 50");
        Check(!fire.Update(true, kDt),
              "the non-arrival edge does not re-fire while still waiting");

        // Reaching the target before the timeout (waiting goes false) cancels
        // the alarm and resets the accumulator.
        ArrivalTimeoutMonitor arrived(kTimeout);
        for (int i = 0; i < 40; ++i)
            arrived.Update(true, kDt);
        Check(!arrived.Update(false, kDt),
              "reaching the target before the timeout cancels the alarm");
        Check(arrived.waited_s() == 0.0,
              "leaving the waiting state resets the timeout accumulator");

        // Rearm clears both the accumulator and the fired latch for a new
        // target, and the full timeout must then be re-earned.
        ArrivalTimeoutMonitor rearmed(kTimeout);
        for (int i = 0; i < 60; ++i)
            rearmed.Update(true, kDt);
        rearmed.Rearm();
        Check(rearmed.waited_s() == 0.0, "Rearm clears the timeout accumulator");
        for (int i = 0; i < 49; ++i)
            Check(!rearmed.Update(true, kDt),
                  "the full timeout is re-earned after Rearm");
        Check(rearmed.Update(true, kDt),
              "the non-arrival edge fires again once the re-earned timeout elapses");

        // Broken timing neither accumulates nor resets.
        ArrivalTimeoutMonitor bad_dt(kTimeout);
        for (int i = 0; i < 40; ++i)
            bad_dt.Update(true, kDt);
        const double waited_before = bad_dt.waited_s();
        bad_dt.Update(true, 0.0);
        bad_dt.Update(true, -kDt);
        bad_dt.Update(true, std::numeric_limits<double>::quiet_NaN());
        bad_dt.Update(true, std::numeric_limits<double>::infinity());
        Check(bad_dt.waited_s() == waited_before,
              "non-finite and non-positive dt neither accumulate nor reset the timeout");

        // A non-positive timeout disables the gate entirely.
        ArrivalTimeoutMonitor disabled(0.0);
        for (int i = 0; i < 100; ++i)
            Check(!disabled.Update(true, kDt),
                  "a non-positive timeout never fires");
    }

} // namespace

int main()
{
    TestDampedLeastSquares();
    TestClampedCycleDt();
    TestInvalidNominalDtHoldsPositionIntegration();
    TestPositionIntegration();
    TestJointLimitWarningGuard();
    TestJointPositionConfiguration();
    TestStopPriority();
    TestStaleAcknowledgementGuard();
    TestArrivalSettling();
    TestArrivalTimeout();
    if (failures == 0) {
        std::cout << "all control-logic tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
