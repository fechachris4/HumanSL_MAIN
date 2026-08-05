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
    // ConfiguredTool_Link, not the bare flange (EndEffector_Link): the
    // right arm carries a mounted tool, so this is the frame that matches
    // both the Kinova web dashboard's tool_pose and physical reality. See
    // the ConfiguredTool_Link comment in config/GEN3_dual_mounted.urdf for
    // the reading this offset was taken from and when to refresh it.
    inline constexpr const char* kRightEndEffectorFrame = "ConfiguredTool_Link";

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

    // Before any controller or command-integrator state is initialized,
    // stream the Seed pose for this exact grid-aligned interval. This is a
    // low-level POSITION-path handshake, not a motion-response test.
    inline constexpr double kTakeoverHoldS = 0.05;
    inline constexpr std::chrono::microseconds kTakeoverHoldDuration =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::duration<double>{kTakeoverHoldS});
    inline constexpr std::size_t kTakeoverHoldCycles =
        static_cast<std::size_t>(kTakeoverHoldDuration / kCyclePeriod);
    static_assert(kTakeoverHoldDuration == kCyclePeriod * kTakeoverHoldCycles,
                  "takeover hold must be an exact number of cyclic periods");

    // The named pipe the controller reads targets from. Created by Main at
    // startup if missing. Writers (planner_bridge, echo) open/write/close per
    // plan; the reader reopens after every EOF.
    inline constexpr const char* kTargetPipePath = "/tmp/humansl_bridge_targets";

    // This controller's target contract is position-only: every target
    // preserves the orientation captured at takeover. The startup pose is
    // measured at runtime; the arm holds it until the first validated pipe
    // waypoint arrives — there is no compiled terminal target.
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

    // Feedback correction is kKpCartesian * (p_reference - p_current), 1/s.
    // The reference itself is shaped below; the independent per-joint clip
    // remains the final command-rate bound.
    inline constexpr double kKpCartesian = 10.0;

    // Joint-space tracking gain on (q_ref - q_meas), 1/s.
    inline constexpr double kKpJointTracking = 5.0;
    // A trajectory whose first point is farther than this from the measured
    // position (any joint) is rejected at activation — the splice guard.
    inline constexpr double kTrajStartToleranceDeg = 2.0;
    // Joint-space following-error stop: measured vs reference, any joint.
    inline constexpr double kTrajFollowingErrorStopDeg = 8.0;
    // Which reference source the Runner is given. true = follow whole joint
    // trajectories from the pipe (JointTrajectorySource); false = the older
    // Cartesian pose path (PoseTargetSource), kept compiled as the documented
    // fallback. REMOVE THIS SWITCH IN TASK 6b, together with the pose path it
    // selects, once the supervised hardware run has passed.
    inline constexpr bool kUseJointTrajectorySource = true;

    // Terminal-to-terminal Cartesian references use a conservative
    // seventh-order profile.  These are source-side reference limits, not
    // replacements for the per-joint command clip or any safety guard.
    inline constexpr double kProfileMaxSpeedMps = 0.025;
    inline constexpr double kProfileMaxAccelerationMps2 = 0.05;
    inline constexpr double kProfileMaxJerkMps3 = 0.25;
    // Dwell held at each reached target before the queue advances, s. Also
    // reused as the non-arrival timeout: if the arm is parked at a target and
    // has not arrived within this long, the run reports "target NOT reached"
    // and keeps holding (Controller.cpp / Runner.cpp). Shortening this also
    // shortens that timeout.
    inline constexpr double kTargetHoldS = 2.0;

    // DLS damping λ (ReactiveLaw.h). Larger = slower but better conditioned
    // near singularities. Also damps the null-space projector.
    inline constexpr double kDlsLambda = 0.1;

    // Reactive-pose law gains (reactive-pose-port.md). Staging reached:
    // orientation, Kd term, and null-space limit avoidance ON.
    inline constexpr double kKpRotation = 10.0; // 1/s on the rotation-log error
    inline constexpr double kKdPosition = 0.5; // on the linear-velocity error
    inline constexpr double kKdRotation = 0.5; // on the angular-velocity error
    inline constexpr bool kOrientationEnabled = true;
    inline constexpr bool kVelocityTermEnabled = true; // Kd term
    inline constexpr bool kNullSpaceEnabled = true;

    // Published Gen3 7-DoF position concepts, degrees, in Kortex actuator
    // order. Kinova's User Guide, Table 39:
    // https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf
    // Joints 1/3/5/7 are continuous (zero mask and zero limits); the
    // bounded joints are 2/4/6. These model values remain the source for
    // client-side position reasoning because bundled Kortex 2.7.0's
    // KinematicLimits schema has no joint_position_limits field.
    inline constexpr JointVector kJointLowerDeg = {
        0, -128.9, 0, -147.8, 0, -120.3, 0
    };
    inline constexpr JointVector kJointUpperDeg = {
        0, 128.9, 0, 147.8, 0, 120.3, 0
    };
    inline constexpr JointVector kJointBoundedMask = {0, 1, 0, 1, 0, 1, 0};

    // The clip applied to q̇ before integration. Equal to the model limits
    // above by construction, so the two cannot disagree.
    inline constexpr JointVector kQdotLimitDegS = kModelVelocityLimitsDegS;

    // JOINT_LIMIT thresholds re-applied on EVERY connection, because SDK
    // writes do not survive a power cycle (compiled-config.md). Before
    // takeover, EnsureJointLimits reads each HIGH/LOW setting, corrects it
    // if needed, then requires both warning and error values on read-back.
    // Firmware-enforced joint-position thresholds: magnitude in deg, sign
    // applied per HIGH/LOW; 0 = leave that joint alone. The controller owns
    // every bounded joint's 2/4/6 threshold; continuous 1/3/5/7 have none.
    // Warnings are j2's established 130 deg and j4/j6 145/118 deg, inside
    // their documented 147.8/120.3 deg ranges. Errors 140/150/123 deg are
    // outside the documented 128.9/147.8/120.3 deg ranges. Do not widen
    // either firmware threshold; they remain robot-side enforcement.
    inline constexpr JointVector kJointLimitWarnDeg = {0, 130.0, 0, 145.0, 0, 118.0, 0};
    inline constexpr JointVector kJointLimitErrorDeg = {0, 140.0, 0, 150.0, 0, 123.0, 0};

    // Software stops before a bounded joint moves outward across this
    // conservative boundary. It is Table 39's upper magnitude less 2 deg,
    // capped by (never wider than) the configured firmware warning. The
    // zero entries preserve the continuous-joint sentinel.
    inline constexpr double kJointSoftwareLimitMarginDeg = 2.0;
    inline constexpr JointVector kJointSoftwareLimitDeg = {
        0,
        (kJointUpperDeg[1] - kJointSoftwareLimitMarginDeg < kJointLimitWarnDeg[1]
             ? kJointUpperDeg[1] - kJointSoftwareLimitMarginDeg
             : kJointLimitWarnDeg[1]),
        0,
        (kJointUpperDeg[3] - kJointSoftwareLimitMarginDeg < kJointLimitWarnDeg[3]
             ? kJointUpperDeg[3] - kJointSoftwareLimitMarginDeg
             : kJointLimitWarnDeg[3]),
        0,
        (kJointUpperDeg[5] - kJointSoftwareLimitMarginDeg < kJointLimitWarnDeg[5]
             ? kJointUpperDeg[5] - kJointSoftwareLimitMarginDeg
             : kJointLimitWarnDeg[5]),
        0
    };

    // Deadband null-space limit avoidance (ReactiveLaw.h). The objective is
    // exactly zero until a bounded joint's wrapped position comes within
    // kLimitAvoidZoneDeg of its software limit above, then pushes inward at
    // kLimitAvoidGain (1/s) times the excess — at the limit that is
    // gain × zone ≈ 40 deg/s before projection, bounded by the per-joint
    // clip. Replaced midpoint centering 2026-08-05:
    // docs/superpowers/specs/2026-08-05-null-space-limit-avoidance-design.md.
    inline constexpr double kLimitAvoidZoneDeg = 20.0;
    inline constexpr double kLimitAvoidGain = 2.0;

    // Validation runs stop automatically on any live base or actuator fault.
    // Compile-time only, never runtime-settable.
    inline constexpr bool kStopOnFault = true;

    // ---- Guard overrides. All three default false = every guard active. ----
    // Each one trades a protection for the ability to run through a fault.
    // They are compile-time only and echoed into the CSV preamble, so a run
    // recorded with one enabled is identifiable afterwards. Turn them back
    // off the moment the hardware problem they work around is fixed.

    // true = the startup gate accepts an actuator whose configured-limit
    // service does not answer, instead of refusing takeover. The joints
    // that do answer are still read, corrected, and verified. false keeps
    // the required contract: every configured joint's HIGH and LOW warning
    // and error thresholds must be read back before takeover.
    inline constexpr bool kAllowUnverifiedActuators = false;

    // true = skip BOTH startup gates for every joint. Nothing is verified
    // and, critically, the j2/j4/j6 JOINT_LIMIT thresholds are NEVER
    // re-applied — they revert to a degenerate 0/0 band on each power
    // cycle, and the firmware faults outward motion on a 0/0 band. Prefer
    // kAllowUnverifiedActuators, which keeps both gates working for every
    // healthy joint.
    inline constexpr bool kSkipStartupGates = false;

    // true = the loop NEVER stops on following error. Read this next to
    // kStopOnFault above: disabling either removes an independent automatic
    // stop. Loss of low-level servoing, the software joint-boundary guard,
    // and enabled non-finite/overrun counters remain automatic stops.
    // kFollowingErrorLimitDeg keeps its value for telemetry and the stop
    // report; this switch removes only the following-error stop.
    inline constexpr bool kDisableFollowingErrorStop = false;

    // Consecutive-cycle stop counters; N <= 0 disables one. Non-finite
    // controller output is never integrated (that cycle holds); overrun =
    // measured dt above kOverrunFactor x nominal. A stale cyclic
    // acknowledgement stops after 25 samples = 50 ms at 500 Hz: command IDs
    // must advance per actuator even when physical motion is stationary.
    inline constexpr int kNonFiniteStopCycles = 3;
    inline constexpr int kOverrunStopCycles = 10;
    inline constexpr int kStaleFeedbackStopCycles = 25;
    inline constexpr double kOverrunFactor = 1.5;

    // Following-error guard: stop when any joint's |command - measured|
    // exceeds this, deg — fires before the base's own ~5 deg ejection
    // (resolved-rate-position-integration.md).
    inline constexpr double kFollowingErrorLimitDeg = 3.0;

    // Command shaping, not stops. Null centering ramps in over
    // kNullRampDurationS; the lead projection targets at most
    // kCommandLeadLimitDeg from wrapped feedback. On discontinuous feedback,
    // the final per-cycle rate envelope wins, so actual lead can temporarily
    // exceed 1 deg while it recovers. The unchanged 3 deg following-error
    // guard is then the backstop (compiled-config.md).
    inline constexpr double kNullRampDurationS = 1.0;
    inline constexpr double kCommandLeadLimitDeg = 1.0;

    // Periodic operator status line (FormatStatusLine): position/rotation
    // error, the task and null-space velocity norms BEFORE summation, the
    // null-space leak speed, sigma_min, clip-saturation count, and the
    // bounded joints against their software limits. One line every this
    // many seconds; non-positive disables it. Reporting only — printed from
    // the loop thread like the arrival notice, so keep it >= 0.5 s.
    inline constexpr double kStatusPrintPeriodS = 1.0;

    // Arrival notice: one printed line the first time the end-effector
    // comes within this distance of a new target, m. Informational only.
    // 1 mm is tight — raise toward 5 mm if the notice comes late or never.
    inline constexpr double kArrivalToleranceM = 0.001;

    // Arrival settling debounce: the arrival notice fires only after the
    // end-effector holds within kArrivalToleranceM continuously for this long,
    // s. Sized above the ~144 ms measured closed-loop response lag
    // (whole-path-validation.md) so a debounced arrival confirms the physical
    // arm settled, not just the command. Non-positive disables the debounce.
    inline constexpr double kArrivalDwellS = 0.15;
    static_assert(kTargetHoldS > kArrivalDwellS,
                  "the non-arrival timeout must outlast the settling window");

    inline constexpr double kArrivalOrientationToleranceRad = 0.001;

    // Stage 1.6 gate: while false, a 7-field target line (x y z qx qy qz qw)
    // is REJECTED loudly — orientation is never silently dropped. Enabling
    // consumption is a separate reviewed change, blocked on j6 health.
    inline constexpr bool kAcceptOrientationTargets = false;

    // Loop log, written DURING the run by a writer thread (Hardware.h).
    // kLogBufferSeconds sizes only the handoff queue — slack for a disk
    // hiccup, not a retention limit. ~175 KB/s at 500 Hz; prune runs/.
    inline constexpr std::size_t kLogBufferSeconds = 30;
    inline constexpr std::chrono::milliseconds kLogDrainInterval{100};
    inline constexpr const char* kLoopLogPrefix = "loop_log";
} // namespace config
