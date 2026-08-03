//
// Config.h — JointVector and the one compiled source of truth for
// controller behaviour. The only runtime argument is --log.
//

#pragma once

#include <array>
#include <chrono>
#include <cstddef>

// ---------------------------------------------------------------
// The joint-space value type
// ---------------------------------------------------------------
//
// JointVector: one value per joint, in Kortex actuator order: index 0 =
// joint 1 ... index 6 = joint 7 (same ordering as feedback.actuators(i) /
// command.actuators(i)). Fixed size, so "exactly 7 values" is enforced by
// the type at compile time.
//
using JointVector = std::array<double, 7>;

// ---------------------------------------------------------------
// The settings
// ---------------------------------------------------------------

namespace config
{
    // The reference source: WHERE the arm should be (Controller.h explains
    // the architecture — one TrackingController, pluggable sources).
    //   "operator"   — typed / file-edited pose targets (Targets.h
    //                  PoseTargetSource; kUseFixedTarget/kTargetFile below)
    //   "trajectory" — one pre-validated joint trajectory, replayed
    //                  (Trajectory.h TrajectorySource; kTrajectoryFile)
    inline constexpr const char* kReferenceSource = "operator";
    inline constexpr const char* kTargetFile = "";
    inline constexpr const char* kTrajectoryFile = "trajectories/hold_home_10s.csv";
    // Hardware ownership is deliberately right-only. The single runtime URDF
    // models both mounted arms, but only this IP gets a Kortex connection and
    // only the right seven-joint chain reaches the command loop.
    inline constexpr const char* kRightRobotIp = "192.168.1.10";
    inline constexpr const char* kRightBaseFrame = "base_link";
    inline constexpr const char* kRightEndEffectorFrame = "EndEffector_Link";

    // The left arm is model-only for the first integration stage. Its seven
    // joints stay fixed at this explicit nominal configuration whenever the
    // full 14-joint Pinocchio configuration is assembled. No left connection,
    // feedback or command frame exists.
    inline constexpr JointVector kLeftNominalRad = {0, 0, 0, 0, 0, 0, 0};

    // Both right-arm Kortex sessions (TCP + UDP, see Hardware.h) log in with the
    // same credentials; timeouts are what the base waits before dropping an
    // idle session/connection.
    inline constexpr const char* kSessionUsername = "admin";
    inline constexpr const char* kSessionPassword = "admin";
    inline constexpr int kSessionInactivityTimeoutMs = 60000;
    inline constexpr int kConnectionInactivityTimeoutMs = 2000;

    // Right-arm commanded-speed clip, deg/s, joints 1-7 — the controller's
    // one actuator-command limit.
    //
    // TEMPORARY BRING-UP VALUE. 45 deg/s uniform sits WELL BELOW the rated
    // model limit and is not derived from it: the Kinova Gen3 User Guide
    // Table 40 general limits are 79.64 deg/s (joints 1-4) and 69.91 deg/s
    // (joints 5-7), which is what this array held until the 2026-08-03
    // restructure and what qdot-limit-raise.md argues for. Kinova's official
    // URDF separately encodes 1.3963 / 1.2218 rad/s — close to Table 40 but
    // not numerically identical. Installed firmware thresholds and
    // configured soft limits remain separate from all of these.
    //
    // Raising this back toward Table 40 is a deliberate decision, not a
    // cleanup: re-read qdot-limit-raise.md first, because the higher clip is
    // what made the base's own streaming-speed enforcement reachable (see
    // kQdotLimitDegS below).
    inline constexpr JointVector kModelVelocityLimitsDegS = {
        45, 45, 45, 45,
        45, 45, 45
    };

    // Control timing, single source of truth: the nominal period is
    // kControlDtS; everything else (loop grid, frequency, log sizing) is
    // derived. 500 Hz — one Send/Feedback exchange every two milliseconds.
    inline constexpr double kControlDtS = 0.002;
    inline constexpr double kControlFrequencyHz = 1.0 / kControlDtS;
    inline constexpr std::chrono::microseconds kCyclePeriod{
        static_cast<long>(kControlDtS * 1e6)
    };

    // Fixed run target, right-arm base frame, METERS (orientation target is
    // left at the takeover pose, exactly like typing "x y z" used to be).
    //
    // With kUseFixedTarget = true, Main hands this target to the
    // PoseTargetSource and reads NOTHING from the terminal: THE ARM STARTS
    // MOVING TOWARD IT IMMEDIATELY after the takeover. (Before 2026-08-03
    // this was silently broken — the old ReactivePose::Reset overwrote the
    // pre-stored target with the measured pose, so the arm just held; the
    // source now serves it for real.) Check the value
    // against the arm's actual pose before every run — a distant target is
    // commanded at clip speed (see kQdotLimitDegS). false restores the
    // interactive stdin prompt.
    inline constexpr bool kUseFixedTarget = true;
    inline constexpr std::array<double, 3> kFixedTargetM = {0.5562, -0.2731, 0.3987};

    // Cartesian velocity controller (the pose channel, Controller.h):
    //   v_desired = kKpCartesian * (p_desired - p_current)   [m/s]
    // kKpCartesian is 1/s. At the CURRENT 10.0 a 10 cm error commands
    // 1.0 m/s — an aggressive gain, and the per-joint clip (kQdotLimitDegS)
    // is what actually bounds the resulting motion. There is deliberately
    // NO Cartesian velocity/acceleration/workspace limiting
    // (docs/decisions/cartesian-velocity-controller.md) — this gain and the
    // size of the target ARE the speed control.
    inline constexpr double kKpCartesian = 10.0;

    // Damped-least-squares damping λ (ReactiveLaw.h). Larger = slower but better
    // conditioned near singular poses; 0.1 follows Pinocchio's IK example
    // scale for arm-sized Jacobians. Shared by both control laws (and by
    // the reactive law's null-space projector).
    inline constexpr double kDlsLambda = 0.1;

    // Reactive-pose law (Controller.h, the pose-reference channel).
    // The bring-up started at the hardware-proven Kp = 1/s scale, well below
    // the simulation's Kp = 32/s, which saturates the velocity clamp and
    // trips the following-error guard on any few-cm error (the documented
    // 2026-07-17 failure mode: docs/decisions/reactive-control-removal.md).
    // Both P gains now sit at 10/s — a tenth of the simulation value but ten
    // times where bring-up started, so the clamp is reachable on a target
    // more than a few cm away. The position P gain is the shared
    // kKpCartesian above.
    //
    // The term switches exist so each addition can be brought up on its own
    // (docs/decisions/reactive-pose-port.md). Where that staging has reached:
    // orientation and the velocity (Kd) term are ON, null-space centering is
    // still OFF. So the commanded twist is the full PD on the pose error,
    // and no secondary joint-centering objective is projected.
    inline constexpr double kKpRotation = 10.0; // 1/s on the rotation-log error
    inline constexpr double kKdPosition = 0.5; // on the linear-velocity error
    inline constexpr double kKdRotation = 0.5; // on the angular-velocity error
    inline constexpr double kNullGain = 2.0; // 1/s on the centering error
    inline constexpr bool kOrientationEnabled = true;
    inline constexpr bool kVelocityTermEnabled = true; // Kd term
    inline constexpr bool kNullSpaceEnabled = false;

    // Null-space centering targets, deg, Kortex joint order. The Gen3's
    // bounded joints (2, 4, 6) have URDF ranges symmetric about 0, so their
    // midpoints are 0; the continuous joints (1, 3, 5, 7) have no meaningful
    // midpoint and are masked out. CAUTION: this arm has CONFIGURED position
    // soft limits far inside the URDF range (joint 4 near −19.6°, joint 6
    // near +36° — README "Safety"); centering toward 0 can push joint 4
    // toward its configured limit. Confirm the configured limits in the
    // Kinova web dashboard before enabling kNullSpaceEnabled.
    inline constexpr JointVector kNullMidpointDeg = {0, 0, 0, 0, 0, 0, 0};
    inline constexpr JointVector kNullCenteringMask = {0, 1, 0, 1, 0, 1, 0};

    // Cartesian control is expressed in the right base_link frame, so its
    // origin is exactly zero. FLAG ONLY — targets are never rejected or
    // projected.
    inline constexpr std::array<double, 3> kRightBaseOriginControlM = {0, 0, 0};
    inline constexpr double kReachRadiusM = 0.902; // Gen3 7-DOF max reach (Kinova spec)
    inline constexpr double kReachMarginM = 0.05; // near full extension is singular anyway

    // Per-joint clip for the controller's q̇ before integration, deg/s —
    // the program's single speed limit. It is whatever kModelVelocityLimitsDegS
    // holds, currently the temporary 45 deg/s uniform bring-up value, NOT the
    // Table 40 model limit.
    //
    // The base enforces whatever joint speed limits apply to low-level
    // streaming, regardless of what we command: position setpoints stepping
    // faster than an enforced limit are not followed — the joint stands
    // still, tracking error grows, and at ~5 deg the safety kicks the arm
    // out of low-level servoing (WRONG_SERVOING_MODE). Actuator firmware
    // safety thresholds can still bind lower in low-level mode
    // (docs/decisions/qdot-limit-raise.md; until 2026-07-22 this arm was
    // configured at 50 deg/s and the clip was a uniform 45 — the value it
    // has returned to).
    inline constexpr JointVector kQdotLimitDegS = kModelVelocityLimitsDegS;

    // Configured JOINT_LIMIT safety thresholds, re-applied at every startup
    // (Hardware.h, Connect::EnsureJointLimits). Magnitudes in degrees; the
    // sign is applied per HIGH/LOW. 0 means "leave that joint alone".
    //
    // Why re-applied rather than set once: values written through
    // DeviceConfig::SetSafetyConfiguration do NOT survive a robot power
    // cycle (2026-08-03 — written and read back correctly, found back at
    // 0/0 about an hour later), and Kortex 2.7.0 exposes no save/commit
    // RPC. Joint 2's +/-140 / +/-130 DOES persist, so the Kinova web
    // dashboard writes through a different path; setting joints 4 and 6
    // there is the real permanent fix, and this gate is the stopgap that
    // keeps them from ever running with a degenerate empty band.
    //
    // Left at 0: joints 1/3/5/7 are continuous (no position limit applies)
    // and joint 2 already carries persistent values.
    //
    // The magnitudes sit just OUTSIDE the rated model range (j4 +/-147.8,
    // j6 +/-120.3 per Kinova spec) so the software limits stay primary and
    // the firmware is a backstop. Nothing here widens a limit past rated.
    inline constexpr JointVector kJointLimitWarnDeg = {0, 0, 0, 145.0, 0, 118.0, 0};
    inline constexpr JointVector kJointLimitErrorDeg = {0, 0, 0, 150.0, 0, 123.0, 0};

    // Startup fault recovery is UNCONDITIONAL (TrajectoryExecution's
    // pattern): main issues Base::ClearFaults right after connecting — no
    // motion, the same operation as the web dashboard's "Clear faults" —
    // and the readiness gate then runs on a fresh post-clear read. A fault
    // that re-latches is live, not stale, and still refuses the takeover.

    // Fault-stop policy, read directly by the Runner. COMPILE-TIME
    // ONLY — deliberately not settable from any runtime configuration.
    //
    // CURRENTLY false: A LIVE FAULT DOES NOT END THE RUN. This is the
    // 2026-07-20 fault-ignoring experiment left switched on — faults are
    // still decoded, printed on their edges and logged, but the loop keeps
    // commanding. ATTENDED USE ONLY: the operator is the stop. Safety
    // consequences are argued in docs/decisions/qdot-limit-raise.md. Set
    // true to make ClassifyStop end the run on a live fault again.
    inline constexpr bool kStopOnFault = false;

    // Supervisor consecutive-cycle counters (decision 12); N <= 0 disables
    // one. Non-finite controller output is never integrated (that cycle
    // holds); overrun = measured dt above kOverrunFactor x nominal. The
    // saturation stop was removed 2026-07-23: a pinned clamp is normal
    // transit toward a far target (the clamp itself still limits speed).
    inline constexpr int kNonFiniteStopCycles = 3;
    inline constexpr int kOverrunStopCycles = 10;
    inline constexpr double kOverrunFactor = 1.5;

    // Following-error guard: stop when any joint's |command - measured|
    // exceeds this, deg — fires before the base's own ~5 deg ejection.
    // Evidence and window: resolved-rate-position-integration.md.
    inline constexpr double kFollowingErrorLimitDeg = 3.0;

    // Command shaping. Control starts directly from one measured-position
    // seed frame (the 0.5 s takeover-hold window was trimmed 2026-08-03;
    // the loop's guards cover the same ground from the first cycle), with
    // null centering ramped in. Integrated setpoints may lead feedback by
    // at most one degree — a command SHAPING rule, not a stop: it bounds
    // how far the setpoint can run away from a joint that is not following.
    //
    // There is deliberately NO stop keyed on "this joint did not move" or
    // "this feedback value did not change" (removed 2026-08-03). Both the
    // short-window motion-response timeout and the unchanged-acknowledgement
    // stop are now telemetry only: req_j*/cmd_j*/meas_j*, lead_limited_j*
    // and ack_unchanged_j* record the same evidence per cycle without ending
    // the run. CONSEQUENCE: a completely unresponsive arm no longer stops
    // the program by itself. Because this lead limit (1 deg) is well below
    // kFollowingErrorLimitDeg (3 deg), the following-error guard cannot fire
    // while the limiter is active, so the operator and the robot's own
    // firmware limits are the backstops for a frozen plant. Attended use.
    inline constexpr double kNullRampDurationS = 1.0;
    inline constexpr double kCommandLeadLimitDeg = 1.0;

    // Arrival notice (raised in Controller.cpp, printed by the Runner): one
    // line the first time the
    // end-effector comes within this distance of a newly typed target, m.
    // Purely informational — motion and the position hold are unaffected.
    // 1 mm is tight: if the steady-state hold error turns out to hover at
    // this scale, the notice may come late or not at all (raise it back
    // toward 5 mm in that case).
    inline constexpr double kArrivalToleranceM = 0.001;

    // Trajectory playback (kReferenceSource = "trajectory", Trajectory.h).
    //
    // kPlaybackKp: P gain on the wrapped reference-minus-measured error,
    // 1/s. Sized from the 2026-07-31 run evidence (loop_log_20260731_121752):
    // tracking error during ~5 deg/s motion peaked at 0.089 deg, so the
    // correction term contributes < 0.05 deg/s in normal play — feed-forward
    // dominates, the P term only absorbs the (gated, small) start offset and
    // clamp losses. Deliberately far below the null_gain = 10 scale that
    // produced the 2026-07-27 windup incident.
    inline constexpr double kPlaybackKp = 0.5;
    //
    // kStartMismatchLimitDeg: per-joint gate between the measured position
    // and the trajectory's first row, checked BEFORE the takeover (main)
    // and again at Reset (arm moved in between -> permanent hold). 0.2 deg
    // is ~30x the still-state tracking error observed in the same run
    // (0.006 deg) and 15x below the 3 deg following-error stop, so a pass
    // guarantees the initial correction transient is negligible.
    inline constexpr double kStartMismatchLimitDeg = 0.2;
    //
    // Validation gates a trajectory file must pass before a run (see
    // Trajectory.h). Velocity: 90% of the command clip, so the
    // Runner's clamp can never engage on a validated file (the clamp stays
    // as the independent backstop). Acceleration: the Kinova Table 43
    // joint-trajectory limits (config/joint_limits.yaml — 1 rad/s^2 joints
    // 1-4, 10 rad/s^2 joints 5-7).
    inline constexpr double kTrajectoryVelGateFactor = 0.9;
    inline constexpr JointVector kTrajectoryAccelLimitDegS2 = {
        57.3, 57.3, 57.3, 57.3, 572.95, 572.95, 572.95
    };
    // Wrapped position range for the bounded joints 2/4/6 from Kinova User
    // Guide Table 39 (±128.9 / ±147.8 / ±120.3 deg); 0 = continuous joint,
    // no limit. The official URDF uses rounded conservative radian metadata
    // (±2.24 / ±2.57 / ±2.09 rad). This arm's CONFIGURED soft limits can sit
    // far inside either factory description (joint 4 near −19.6°, joint 6
    // near +36° per the README): confirm them in the Kinova web dashboard
    // before a session.
    inline constexpr JointVector kTrajectoryPosLimitDeg = {
        0.0, 128.9, 0.0, 147.8, 0.0, 120.3, 0.0
    };

    // Loop log: the CSV is written DURING the run by a writer thread
    // (Hardware.h, LoopLogWriter), so a killed run keeps everything up to
    // the last
    // drain and the file holds the whole run, however long it is.
    //
    // kLogBufferSeconds only sizes the handoff queue between the loop and
    // that thread — it is slack for a disk hiccup, not a retention limit.
    // 30 s of samples is ~15 MB and 300x the drain interval; if the writer
    // ever falls that far behind, samples are dropped and counted rather
    // than silently overwritten.
    //
    // Cost of keeping everything: ~175 KB/s of CSV at the 500 Hz loop rate,
    // so a 30-minute run is ~300 MB. Prune runs/ rather than shortening the
    // record.
    inline constexpr std::size_t kLogBufferSeconds = 30;
    inline constexpr std::chrono::milliseconds kLogDrainInterval{100};
    inline constexpr const char* kLoopLogPrefix = "loop_log";
} // namespace config
