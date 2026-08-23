//
// Config.h — JointVector and the one compiled source of truth for
// controller behaviour. Runtime flags select arm/log destination only, so these
// values are the only thing between the operator and the arm: read this
// file before a session. Rationale: ../../docs/decisions/, indexed by
// compiled-config.md.
//

#pragma once

#include <array>
#include <chrono>
#include <cstddef>

// Generated at build time from planning/config/joint_limits.yaml — the one
// authoritative table of physical Kinova joint limits and of the margins the
// planner and controller apply to them. See tools/generate_joint_limits.py.
#include "JointLimits.h"

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
    inline constexpr const char* kRightBaseFrame = "right_base_link";
    // right_tool_link, not the bare flange (right_end_effector_link): the
    // right arm carries a mounted tool, so this is the frame that matches
    // both the Kinova web dashboard's tool_pose and physical reality. See
    // the right_tool_link comment in config/GEN3_dual_mounted.urdf for
    // the reading this offset was taken from and when to refresh it.
    inline constexpr const char* kRightEndEffectorFrame = "right_tool_link";

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
    //     is permanently the queried arm's OWN base frame,
    //     right_base_link or left_base_link (its left_arm parameter, not
    //     this switch, selects which) — utils.cpp
    //     composes it with DhRootInBaseLink().inverse(), and the GPMP2 arm
    //     model and the SDF must share a frame or every collision check is
    //     silently wrong;
    //   - tools/print_dual_arm_fk, which prints mount AND base side by side
    //     because that comparison is what validates the mounting transform.
    //
    // `mount` is the URDF root: the midpoint of the two arm bases, rigidly
    // attached to both. It travels with the rig. `world` is Vicon's fixed
    // laboratory frame and composes above it through the measured T_W_M.
    enum class ReferenceFrame { kMount, kRightBase, kLeftBase, kWorld };
    inline constexpr ReferenceFrame kReferenceFrame = ReferenceFrame::kMount;

    // The names accepted in a goal file's `frame:` key, in enum order.
    inline constexpr const char* kReferenceFrameNames[] = {
        "mount", "right_base", "left_base", "world"
    };

    // Left-arm frame names, for kinematics only. The left chain ends at the
    // bare flange: right_tool_link is RIGHT-ARM ONLY, because it encodes
    // the tool physically mounted on the right flange. Left and right tool
    // points are therefore not the same point on the arm — never compare
    // their poses as though they were.
    inline constexpr const char* kLeftBaseFrame = "left_base_link";
    inline constexpr const char* kLeftEndEffectorFrame = "left_end_effector_link";

    // ---------------------------------------------------------------
    // Per-arm bring-up settings — everything --arm selects between
    // ---------------------------------------------------------------
    //
    // One instance per physical arm. `other_arm_nominal_rad` is the OTHER
    // arm's assumed pose (see the comment on kLeftNominalRad above) — for
    // kRightArmConfig that's the left arm's nominal, and vice versa.
    // The log prefix and lock path are per-arm so a --arm=both run keeps
    // telemetry apart; lock_path names the IP it guards, matching
    // ProcessLock's existing per-run contract.
    struct ArmConfig {
        const char* name;
        const char* ip;
        const char* base_frame;
        const char* end_effector_frame;
        JointVector other_arm_nominal_rad;
        const char* log_prefix;
        const char* lock_path;
    };

    inline constexpr ArmConfig kRightArmConfig{
        "right", kRightRobotIp, kRightBaseFrame, kRightEndEffectorFrame,
        kLeftNominalRad,
        "loop_log_right",
        "/tmp/basic_control-192.168.1.10.lock"
    };
    inline constexpr ArmConfig kLeftArmConfig{
        "left", kLeftRobotIp, kLeftBaseFrame, kLeftEndEffectorFrame,
        kRightNominalRad,
        "loop_log_left",
        "/tmp/basic_control-192.168.1.9.lock"
    };

    // Both right-arm sessions (TCP + UDP, Hardware.h). Timeouts are what
    // the base waits before dropping an idle session/connection.
    inline constexpr const char* kSessionUsername = "admin";
    inline constexpr const char* kSessionPassword = "admin";
    inline constexpr int kSessionInactivityTimeoutMs = 60000;
    inline constexpr int kConnectionInactivityTimeoutMs = 2000;

    // Commanded-speed clip, deg/s. Derived, not authored: the physical
    // hard limits the base reports at startup live in
    // planning/config/joint_limits.yaml, and this is
    // kVelocityControllerFraction of them (JointLimits.h, generated from
    // that yaml at build time). The planner derives its own, stricter,
    // figure from the same table by the same mechanism, so the two cannot
    // drift apart the way the twin hand-written 5% derates did.
    // VerifyKinematicHardLimits still refuses startup if this ever exceeds
    // what the base actually reports.
    inline constexpr JointVector kModelVelocityLimitsDegS =
        limits::kControllerVelocityDegS;

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

    // This controller's target contract is position-only: every target
    // preserves the orientation captured at takeover. The startup pose is
    // measured at runtime; the arm holds it until the first typed planner
    // trajectory arrives — there is no compiled terminal target.
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

    // Null-space attraction toward the planner's per-sample joint posture
    // (ReactiveLaw.h equation 5b) — the redundancy decision the plan was
    // validated with, instead of leaving redundancy ungoverned between the
    // limit-avoidance zones (2026-08-23: two runs walked joint 6 to its
    // outward software boundary while the plan held a 20.7 deg margin).
    //
    // ON since 2026-08-23, after hardware-free closed-loop validation
    // (test_posture_closed_loop: full ArmExecutionCore against an ideal
    // plant): task tracking is unchanged to micrometres at any gain tried
    // (0.5-4.0), and the executed configuration ends 1.4 deg from the
    // planned one at this gain against 9.2 deg with the term off. Gain
    // 1.0 is a deliberate distance from BOTH failure modes on record:
    // far under the 2026-08-05 stall regime (gain 23), and half of
    // kLimitAvoidGain — though priority is structural, not gain-based
    // (avoidance suppresses posture per joint in the law itself). The
    // damped projector's task-space leak stays visible in null_leak_mps.
    // First hardware run remains supervised evidence, not a formality:
    // watch null_leak_mps and the joint-6 margin.
    inline constexpr bool kPostureTrackingEnabled = true;
    inline constexpr double kPostureGain = 1.0; // 1/s on wrapped posture error

    // Published Gen3 7-DoF position limits, degrees, in Kortex actuator
    // order — Kinova's User Guide Table 39. Derived, not authored: they come
    // from planning/config/joint_limits.yaml through the generated
    // JointLimits.h, which is the single authoritative table for the whole
    // repository. Joints 1/3/5/7 are continuous and carry the zero sentinel
    // and a zero mask. These model values remain the source for client-side
    // position reasoning because bundled Kortex 2.7.0's KinematicLimits
    // schema has no joint_position_limits field. The URDF's <limit> tags are
    // NOT a source: they are silent on all four continuous joints and, where
    // they speak, rounded away from Table 39.
    inline constexpr JointVector kJointLowerDeg = limits::kPhysicalLowerDeg;
    inline constexpr JointVector kJointUpperDeg = limits::kPhysicalUpperDeg;
    inline constexpr JointVector kJointBoundedMask = limits::kBoundedMask;

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
    // The WARNING threshold is the physical limit itself, derived from
    // planning/config/joint_limits.yaml like every other limit in the
    // repository (2026-08-20). It used to be three hand-authored numbers —
    // 130/145/118 deg — that followed no rule: j2's sat 1.1 deg OUTSIDE its
    // physical limit while j6's sat 2.3 deg inside, and that 118 was the
    // origin of the planner/controller gap nobody could explain. With the
    // warning ON the physical limit, the whole ladder is ordered by
    // construction: planner 2 deg inside, controller 1 deg inside, firmware
    // warning at the limit, firmware error outside it.
    //
    // EnsureJointLimits still writes these on every connection, and that is
    // the point of keeping it: SDK writes do not survive a power cycle, and
    // this arm's persisted state is a degenerate 0/0 band on the bounded
    // joints, which makes the firmware fault ANY outward motion (2026-08-04
    // incident). The mechanism repairs that; it is not an override of a
    // healthy factory configuration.
    inline constexpr JointVector kJointLimitWarnDeg = limits::kPhysicalUpperDeg;

    // The ERROR threshold is still hand-authored, deliberately, and is NOT
    // changed by this step. It sits outside the physical limit (+11.1/+2.2/
    // +2.7 deg) as the last robot-side protection, after both software stops
    // and the warning have already been passed. Do not widen it.
    inline constexpr JointVector kJointLimitErrorDeg = {0, 140.0, 0, 150.0, 0, 123.0, 0};

    // Software stops before a bounded joint moves outward across this
    // boundary: the physical limit less the controller margin, and nothing
    // else. This used to be Table 39 less a locally-declared 2 deg, capped
    // by min() against the separately-authored firmware warning, which is
    // where joint 6's unexplained 118 deg came from. One margin now, defined
    // beside the physical limits it applies to. The zero entries preserve
    // the continuous-joint sentinel.
    inline constexpr JointVector kJointSoftwareLimitDeg =
        limits::kControllerUpperDeg;

    // The ordering this rests on, all from one physical table:
    //   planner 126.9 / 145.8 / 118.3  (physical - 2 deg)
    //   controller 127.9 / 146.8 / 119.3  (physical - 1 deg)
    //   firmware warning 128.9 / 147.8 / 120.3  (physical)
    //   firmware error 140 / 150 / 123  (outside physical)
    // Each layer stops before the next. No min(), no second margin, no
    // number that is not derived from joint_limits.yaml or explained here.

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

    // true = the loop never stops on the command following-error rule
    // (|command - measured| against kFollowingErrorLimitDeg). Read this next
    // to kStopOnFault above: disabling either removes an independent
    // automatic stop. Loss of low-level servoing, the software joint-boundary
    // guard, and enabled non-finite/overrun counters remain automatic stops.
    inline constexpr bool kDisableFollowingErrorStop = false;

    // Fresh = sample age at cycle start <= this (5 lost frames at 100 Hz;
    // measured p99 age on 2026-08-13 was 9.7 ms, so 50 ms is a real gap).
    inline constexpr double kWorldFreshMaxAgeS = 0.05;
    // Mount-twist low-pass time constant. Five Vicon frames
    // at the nominal 100 Hz smooth finite-difference noise without hiding the
    // value in an undocumented implementation constant. This estimate is
    inline constexpr double kViconMountTwistFilterTauS = 0.05;
    // A gap at or above this resets differentiation; the next fresh frame
    // establishes a pose but deliberately has no inherited pre-gap twist.
    inline constexpr double kViconMountTwistResetGapS = 0.2;
    // Active Cartesian trajectories are cancelled after this much
    // continuously stale world state; fresh recovery captures a new hold
    // and requests a new plan instead of resuming across unknown base motion.
    inline constexpr double kWorldProlongedStaleS = 0.2;

    // Consecutive-cycle stop counters; N <= 0 disables one. Non-finite
    // controller output is never integrated (that cycle holds); overrun =
    // raw measured dt above kOverrunFactor x the fixed control period. A stale cyclic
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

    // Historical terminal-status cadence retained for schema/config
    // compatibility. The 500 Hz loop no longer performs terminal I/O;
    // format-13 telemetry carries the same diagnostic values.
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

    // Handover continuity gate: how far an incoming trajectory's first point
    // may sit from the MEASURED pose at activation. Deliberately its own
    // pair, not the arrival-notice values above: the hold's steady servo
    // offset alone is ~1.2 mm at gravity-loaded postures, and on 2026-08-23
    // a whole session's plans were silently rejected at 1.28/1.16 mm against
    // the 1 mm arrival value — a diagnostic threshold acting as a gate. The
    // gate's real job is the gross case (a plan built from a world that no
    // longer exists); the controller absorbs a few millimetres of reference
    // step exactly as it absorbs any error, and the per-cycle command-lead
    // clamp and speed clips bound what any accepted step can do.
    inline constexpr double kHandoverToleranceM = 0.005;
    inline constexpr double kHandoverOrientationToleranceRad = 0.02;

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
