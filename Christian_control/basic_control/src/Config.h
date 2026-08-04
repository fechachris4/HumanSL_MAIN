//
// Config.h — JointVector and the one compiled source of truth for
// controller behaviour. --log is the only runtime argument, so these
// values are the only thing between the operator and the arm: read this
// file before a session. Rationale: ../../docs/decisions/, indexed by
// compiled-config.md.
//

#pragma once

#include <array>
#include <chrono>
#include <cstddef>

// ---------------------------------------------------------------
// The joint-space value type
// ---------------------------------------------------------------

// One value per joint in Kortex actuator order: index 0 = joint 1 ...
// index 6 = joint 7, matching feedback.actuators(i) / command.actuators(i).
// Fixed size, so "exactly 7 values" is a compile-time guarantee.
using JointVector = std::array<double, 7>;

// ---------------------------------------------------------------
// The settings
// ---------------------------------------------------------------

namespace config
{
    // Right-only by design: the URDF models both mounted arms, but only
    // this IP gets a connection and only its 7 joints reach the loop.
    inline constexpr const char* kRightRobotIp = "192.168.1.10";
    inline constexpr const char* kRightBaseFrame = "base_link";
    inline constexpr const char* kRightEndEffectorFrame = "EndEffector_Link";

    // The left arm is model-only: held here whenever the 14-joint
    // configuration is assembled. No left connection, feedback or command.
    inline constexpr JointVector kLeftNominalRad = {0, 0, 0, 0, 0, 0, 0};

    // Both right-arm sessions (TCP + UDP, Hardware.h). Timeouts are what
    // the base waits before dropping an idle session/connection.
    inline constexpr const char* kSessionUsername = "admin";
    inline constexpr const char* kSessionPassword = "admin";
    inline constexpr int kSessionInactivityTimeoutMs = 60000;
    inline constexpr int kConnectionInactivityTimeoutMs = 2000;

    // Commanded-speed clip, deg/s — the program's single speed limit.
    // TEMPORARY 45 uniform, below the 79.64/69.91 Table 40 model limit
    // (qdot-limit-raise.md; raising it back is a deliberate decision).
    inline constexpr JointVector kModelVelocityLimitsDegS = {
        45, 45, 45, 45,
        45, 45, 45
    };

    // Timing, single source of truth: 500 Hz — one Send/Feedback exchange
    // every 2 ms. Loop grid, frequency and log sizing all derive from it.
    inline constexpr double kControlDtS = 0.002;
    inline constexpr double kControlFrequencyHz = 1.0 / kControlDtS;
    inline constexpr std::chrono::microseconds kCyclePeriod{
        static_cast<long>(kControlDtS * 1e6)
    };

    // Freshness gate: refuse the run if the compiled fixed target is
    // farther than this from the measured end-effector position at
    // startup. The arm can be moved (dashboard jog, physical push) any
    // time after a build, and the law drives the WHOLE gap at clip speed
    // (2026-08-04: 37 cm of drift between compile and run).
    inline constexpr double kMaxFixedTargetDistanceM = 0.15;

    // The fixed target in right-arm base_link, in METRES. This controller's
    // target contract is position-only: every target preserves the
    // orientation captured at takeover.
    // -3 cm in -z from the arm's 2026-08-04 01:5x pose (EE 0.3834
    // -0.4051 0.7525, joints 359.32 61.54 120.07 69.83 338.16 11.80
    // 335.94): probe_direction at THAT configuration shows -z drives
    // joint 6 INWARD (-7.46 deg/s per 0.1 m/s; ~-2 deg over this move),
    // safe while j6's JOINT_LIMIT band cannot be read or restored. j6
    // sits at +11.8 deg with only ~12 deg of inward window before zero,
    // so keep any move here SMALL and re-probe after the arm moves.
    inline constexpr std::array<double, 3> kFixedTargetM = {0.3834, -0.4051, 0.7225};
    // Retained for the existing CSV key. It is a compile-time invariant, not
    // an operator-selectable orientation mode.
    inline constexpr bool kFixedTargetUseRpy = false;
    static_assert(!kFixedTargetUseRpy,
                  "basic_control targets must preserve takeover orientation");
    // Legacy conditional CSV field, retained only to preserve the log schema.
    // It is not a target orientation command and is never emitted while the
    // enforced position-only invariant above is false.
    inline constexpr std::array<double, 3> kFixedTargetRpyRad = {
        1.5707963267948966, 0.0, 1.5707963267948966
    };

    // v_desired = kKpCartesian * (p_desired - p_current), 1/s. At 10.0 a
    // 10 cm error commands 1.0 m/s — aggressive; the per-joint clip is what
    // actually bounds it. No Cartesian velocity/acceleration/workspace
    // limiting exists (cartesian-velocity-controller.md).
    inline constexpr double kKpCartesian = 10.0;

    // DLS damping λ (ReactiveLaw.h). Larger = slower but better conditioned
    // near singularities. Also damps the null-space projector.
    inline constexpr double kDlsLambda = 0.1;

    // Reactive-pose law gains (reactive-pose-port.md). Staging reached:
    // orientation and the velocity (Kd) term ON, null-space centering OFF.
    inline constexpr double kKpRotation = 10.0; // 1/s on the rotation-log error
    inline constexpr double kKdPosition = 0.5; // on the linear-velocity error
    inline constexpr double kKdRotation = 0.5; // on the angular-velocity error
    inline constexpr double kNullGain = 2.0; // 1/s on the centering error
    inline constexpr bool kOrientationEnabled = true;
    inline constexpr bool kVelocityTermEnabled = true; // Kd term
    inline constexpr bool kNullSpaceEnabled = false;

    // Null-space centering targets, deg. Bounded joints 2/4/6 have URDF
    // ranges symmetric about 0; continuous joints 1/3/5/7 are masked out.
    // CAUTION before enabling kNullSpaceEnabled: this arm's CONFIGURED soft
    // limits sit far inside the URDF range, so centering toward 0 can push
    // joint 4 into its limit. Confirm them in the web dashboard first.
    inline constexpr JointVector kNullMidpointDeg = {0, 0, 0, 0, 0, 0, 0};
    inline constexpr JointVector kNullCenteringMask = {0, 1, 0, 1, 0, 1, 0};

    // Base-link reach boundary. Stdin target parsing rejects positions beyond
    // this conservative sphere; telemetry also flags any desired position
    // beyond it. This remains a reach screen, not a collision or IK check.
    inline constexpr std::array<double, 3> kRightBaseOriginControlM = {0, 0, 0};
    inline constexpr double kReachRadiusM = 0.902; // Gen3 7-DOF max reach (Kinova spec)
    inline constexpr double kReachMarginM = 0.05; // near full extension is singular anyway

    // The clip applied to q̇ before integration. Equal to the model limits
    // above by construction, so the two cannot disagree.
    inline constexpr JointVector kQdotLimitDegS = kModelVelocityLimitsDegS;

    // JOINT_LIMIT thresholds re-applied on EVERY connection, because SDK
    // writes do not survive a power cycle (compiled-config.md).
    // Magnitudes in deg, sign applied per HIGH/LOW; 0 = leave that joint
    // alone. These sit just outside the rated range, so software stays
    // primary and firmware is the backstop.
    // Joint 6 is set to 0 (leave alone) since 2026-08-04: its config
    // service is wedged, so the gate's RPCs to it can only time out —
    // ~15-20 s of dead wait per run buying nothing. Its band therefore
    // stays whatever it is (presumed degenerate 0/0): ONLY j6-INWARD
    // MOVES ARE SAFE until the service recovers; run probe_direction
    // before choosing targets. Restore 118/123 here when j6 answers again.
    inline constexpr JointVector kJointLimitWarnDeg = {0, 0, 0, 145.0, 0, 0, 0};
    inline constexpr JointVector kJointLimitErrorDeg = {0, 0, 0, 150.0, 0, 0, 0};

    // false = A LIVE FAULT DOES NOT END THE RUN. Faults are still decoded,
    // printed and logged, and they taint the exit code, but the loop keeps
    // commanding. ATTENDED USE ONLY: the operator is the stop. Compile-time
    // only, never runtime-settable (qdot-limit-raise.md).
    inline constexpr bool kStopOnFault = false;

    // ---- Guard overrides. All three default false = every guard active. ----
    // Each one trades a protection for the ability to run through a fault.
    // They are compile-time only and echoed into the CSV preamble, so a run
    // recorded with one enabled is identifiable afterwards. Turn them back
    // off the moment the hardware problem they work around is fixed.

    // true = the startup gates ACCEPT an actuator whose configuration
    // service does not answer, instead of refusing the takeover. The
    // joints that do answer are still verified and still get their limits
    // restored. CONSEQUENCE: the unreachable joint is commanded with its
    // control mode UNVERIFIED and its JOINT_LIMIT band unknown — if it is
    // not in POSITION mode, what it does with a position setpoint is
    // undefined. (2026-08-04: joint 6's config service stopped answering
    // while it still reported position normally.)
    // Back to false 2026-08-04 (later the same night): joint 6 is now
    // excluded from the limits gate via its zero config entry, so nothing
    // in the startup path probes the wedged service any more and the
    // override has nothing to excuse.
    inline constexpr bool kAllowUnverifiedActuators = false;

    // true = skip BOTH startup gates for every joint. Nothing is verified
    // and, critically, the j4/j6 JOINT_LIMIT thresholds are NEVER
    // re-applied — they revert to a degenerate 0/0 band on each power
    // cycle, and the firmware faults outward motion on a 0/0 band. Prefer
    // kAllowUnverifiedActuators, which keeps both gates working for every
    // healthy joint.
    inline constexpr bool kSkipStartupGates = false;

    // true = the loop NEVER stops on following error. Read this next to
    // kStopOnFault above, which is already false: with both set, and the
    // no-motion stops removed on 2026-08-03, the ONLY remaining automatic
    // stop is loss of low-level servoing. Nothing else ends a run — not a
    // fault, not a joint that has stopped following its setpoint. The
    // operator and the robot's own firmware limits are the entire safety
    // margin. kFollowingErrorLimitDeg keeps its value for the telemetry
    // and the stop report; this only removes the stop.
    inline constexpr bool kDisableFollowingErrorStop = false;

    // Consecutive-cycle stop counters; N <= 0 disables one. Non-finite
    // controller output is never integrated (that cycle holds); overrun =
    // measured dt above kOverrunFactor x nominal.
    inline constexpr int kNonFiniteStopCycles = 3;
    inline constexpr int kOverrunStopCycles = 10;
    inline constexpr double kOverrunFactor = 1.5;

    // Following-error guard: stop when any joint's |command - measured|
    // exceeds this, deg — fires before the base's own ~5 deg ejection
    // (resolved-rate-position-integration.md).
    inline constexpr double kFollowingErrorLimitDeg = 3.0;

    // Command shaping, not stops. Null centering ramps in over
    // kNullRampDurationS; integrated setpoints may lead feedback by at most
    // kCommandLeadLimitDeg, bounding how far the setpoint runs away from a
    // joint that is not following. Because 1 deg is well below the 3 deg
    // following-error guard, that guard cannot fire while the limiter is
    // active — so nothing stops a frozen plant by itself (compiled-config.md).
    inline constexpr double kNullRampDurationS = 1.0;
    inline constexpr double kCommandLeadLimitDeg = 1.0;

    // Arrival notice: one printed line the first time the end-effector
    // comes within this distance of a new target, m. Informational only.
    // 1 mm is tight — raise toward 5 mm if the notice comes late or never.
    inline constexpr double kArrivalToleranceM = 0.001;

    // Loop log, written DURING the run by a writer thread (Hardware.h).
    // kLogBufferSeconds sizes only the handoff queue — slack for a disk
    // hiccup, not a retention limit. ~175 KB/s at 500 Hz; prune runs/.
    inline constexpr std::size_t kLogBufferSeconds = 30;
    inline constexpr std::chrono::milliseconds kLogDrainInterval{100};
    inline constexpr const char* kLoopLogPrefix = "loop_log";
} // namespace config
