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
    // Both arms are connectable, one process each: --arm selects which IP
    // this run controls (Main.cpp). The URDF always models both mounted
    // arms; only the selected 7 joints reach the loop and get a Connect.
    inline constexpr const char* kRightRobotIp = "192.168.1.10";
    inline constexpr const char* kLeftRobotIp = "192.168.1.9";
    inline constexpr const char* kRightBaseFrame = "base_link";
    // ConfiguredTool_Link, not the bare flange (EndEffector_Link): the
    // right arm carries a mounted tool, so this is the frame that matches
    // both the Kinova web dashboard's tool_pose and physical reality. See
    // the ConfiguredTool_Link comment in config/GEN3_dual_mounted.urdf for
    // the reading this offset was taken from and when to refresh it.
    inline constexpr const char* kRightEndEffectorFrame = "ConfiguredTool_Link";

    // Assumed pose of whichever arm is NOT the one this process controls,
    // held fixed whenever the 14-joint configuration is assembled — that
    // arm has no connection, no feedback and no command in THIS process, so
    // its mount-frame FK is model-only, not measured. See DualArmKinematics
    // (Kinematics.h) for how the controlled/other split is wired.
    inline constexpr JointVector kLeftNominalRad = {0, 0, 0, 0, 0, 0, 0};
    inline constexpr JointVector kRightNominalRad = {0, 0, 0, 0, 0, 0, 0};
    // ---------------------------------------------------------------
    // The pipeline's reference frame — ONE switch
    // ---------------------------------------------------------------
    //
    // Which frame Cartesian targets are READ in and Cartesian quantities are
    // REPORTED in, across the planner and the controller. Change it here and
    // nothing else in either project changes: the transforms themselves come
    // from the URDF (config/dual_arm_mounting.yaml -> GEN3_dual_mounted.urdf,
    // via Pinocchio), never from a constant in code.
    //
    // Applies to: a goal file with no `frame:` key, a bare `--goal X Y Z`,
    // and printed/diagnostic Cartesian output.
    //
    // Deliberately does NOT apply to two places, because it cannot:
    //   - pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink, which
    //     is permanently the queried arm's OWN base_link/leftbase_link (its
    //     left_arm parameter, not this switch, selects which) — utils.cpp
    //     composes it with DhRootInBaseLink().inverse(), and the GPMP2 arm
    //     model and the SDF must share a frame or every collision check is
    //     silently wrong;
    //   - tools/print_dual_arm_fk, which prints mount AND base side by side
    //     because that comparison is what validates the mounting transform.
    //
    // `mount` is the URDF root: the midpoint of the two arm bases, rigidly
    // attached to both. It is NOT a room frame — it travels with the rig.
    // A room frame belongs ABOVE it as T_room_mount, identity while the rig
    // is bolted to a bench and supplied by motion capture once it is worn.
    // That transform does not exist yet; when it does, it composes in
    // BridgeMain's frame boundary and in DualArmKinematics::MountFromBase,
    // and nothing else needs to move.
    enum class ReferenceFrame { kMount, kRightBase, kLeftBase };
    inline constexpr ReferenceFrame kReferenceFrame = ReferenceFrame::kMount;

    // The names accepted in a goal file's `frame:` key, in enum order.
    inline constexpr const char* kReferenceFrameNames[] = {
        "mount", "right_base", "left_base"
    };

    // Left-arm frame names, for kinematics only. The left chain ends at the
    // bare flange: ConfiguredTool_Link is RIGHT-ARM ONLY, because it encodes
    // the tool physically mounted on the right flange. Left and right tool
    // points are therefore not the same point on the arm — never compare
    // their poses as though they were.
    inline constexpr const char* kLeftBaseFrame = "leftbase_link";
    inline constexpr const char* kLeftEndEffectorFrame = "leftEndEffector_Link";

    // ---------------------------------------------------------------
    // Per-arm bring-up settings — everything --arm selects between
    // ---------------------------------------------------------------
    //
    // One instance per physical arm. `other_arm_nominal_rad` is the OTHER
    // arm's assumed pose (see the comment on kLeftNominalRad above) — for
    // kRightArmConfig that's the left arm's nominal, and vice versa.
    // target_pipe_path and log_prefix are per-arm so a --arm=both run keeps
    // the two arms' trajectory input and telemetry apart; lock_path names
    // the IP it guards, matching ProcessLock's existing per-run contract.
    struct ArmConfig {
        const char* name;
        const char* ip;
        const char* base_frame;
        const char* end_effector_frame;
        JointVector other_arm_nominal_rad;
        const char* target_pipe_path;
        const char* log_prefix;
        const char* lock_path;
    };

    inline constexpr ArmConfig kRightArmConfig{
        "right", kRightRobotIp, kRightBaseFrame, kRightEndEffectorFrame,
        kLeftNominalRad,
        "/tmp/humansl_bridge_targets_right", "loop_log_right",
        "/tmp/basic_control-192.168.1.10.lock"
    };
    inline constexpr ArmConfig kLeftArmConfig{
        "left", kLeftRobotIp, kLeftBaseFrame, kLeftEndEffectorFrame,
        kRightNominalRad,
        "/tmp/humansl_bridge_targets_left", "loop_log_left",
        "/tmp/basic_control-192.168.1.9.lock"
    };

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

    // The named pipe the controller reads targets from is per-arm
    // (ArmConfig::target_pipe_path above). Created by Main at startup if
    // missing. Writers (planner_bridge, echo) open/write/close per plan; the
    // reader reopens after every EOF.

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

    // Null-space nominal-posture guidance (ReactiveLaw.h PostureObjective):
    // a planner's q_nom/qdot_nom supplied through Reference.posture biases
    // the redundant motion as qdot_nom + kPostureGain * wrap(q_nom - q).
    // Guidance, never a competitor: the gain is kept twenty times below
    // kKpCartesian because the DAMPED projector leaks secondary objectives
    // into task space — on 2026-08-05 a null-space gain of 23 balanced the
    // task term and froze the arm 218 mm short of its target. Disabled
    // until a reference source actually publishes posture (slice 2 of the
    // world-frame architecture); like every disabled term it contributes
    // exactly zero.
    inline constexpr double kPostureGain = 0.5; // 1/s on the wrapped error
    inline constexpr bool kPostureEnabled = true;

    // Which channel owns the command while following a planner trajectory.
    // false = joint mode, today's behaviour: the joint tracking law follows
    // q_nom(t) directly. true = pose-primary mode (TrajectoryPoseSource):
    // the reactive Cartesian law tracks FK(q_nom(t)) with twist J·q̇_nom,
    // and q_nom guides the redundancy through the null space instead of
    // owning the command. Compile-time like every staging switch; the CSV
    // preamble records which mode produced a run. First hardware run of
    // pose-primary mode requires its own supervised bring-up.
    inline constexpr bool kTrajectoryPosePrimary = false;

    // Feasibility advisories (Feasibility.h) — graded supervision, slice 4.
    // ADVISORY ONLY: crossing a threshold raises the replan_advised level in
    // the status/CSV after kReplanAdviseCycles consecutive degraded cycles;
    // nothing stops or slows. The stops that do exist are unchanged.
    inline constexpr double kReplanSigmaMin = 0.02;        // σ_min(J) below
    inline constexpr double kReplanJointMarginDeg = 5.0;   // limit margin below
    inline constexpr double kReplanPostureErrorDeg = 15.0; // plan deviation above
    inline constexpr double kReplanPositionErrorM = 0.05;  // tracking error above
    inline constexpr int kReplanAdviseCycles = 250;        // ~0.5 s at 2 ms

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

    // true = the loop never stops on the CARTESIAN following-error rule
    // (|command - measured| against kFollowingErrorLimitDeg). Read this next
    // to kStopOnFault above: disabling either removes an independent
    // automatic stop. Loss of low-level servoing, the software joint-boundary
    // guard, and enabled non-finite/overrun counters remain automatic stops —
    // and so does the joint tracking law's own gate on the wrapped reference
    // error, kTrajFollowingErrorStopDeg, which this switch does NOT touch.
    // Setting that threshold non-positive is the only way to disable it.
    // kFollowingErrorLimitDeg keeps its value for telemetry and the stop
    // report; this switch removes only the Cartesian stop.
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
