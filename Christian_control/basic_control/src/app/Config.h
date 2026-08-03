/*
 * Config.h — the one compiled source of truth for controller behaviour.
 * The only runtime argument is --log, which names the output CSV.
 */
#pragma once

#include <array>
#include <chrono>
#include <cstddef>

#include "JointVector.h"

namespace config
{
    // Controller and optional input sources. The current TOML-effective
    // reactive-pose tuning is preserved here.
    inline constexpr const char* kController = "playback";
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

    // Both right-arm Kortex sessions (TCP + UDP, see Connect.h) log in with the
    // same credentials; timeouts are what the base waits before dropping an
    // idle session/connection.
    inline constexpr const char* kSessionUsername = "admin";
    inline constexpr const char* kSessionPassword = "admin";
    inline constexpr int kSessionInactivityTimeoutMs = 60000;
    inline constexpr int kConnectionInactivityTimeoutMs = 2000;

    // Right-arm commanded-speed limits, deg/s, joints 1-7. These are Kinova
    // Gen3 User Guide Table 40's general limits and remain the controller's
    // one actuator-command clip. Kinova's official URDF separately encodes
    // 1.3963 / 1.2218 rad/s; those values are close but not numerically
    // identical to Table 40. Installed firmware and configured soft limits
    // remain separate.
    inline constexpr JointVector kModelVelocityLimitsDegS = {
        79.64, 79.64, 79.64, 79.64,
        69.91, 69.91, 69.91
    };

    // Control timing, single source of truth: the nominal period is
    // kControlDtS; everything else (loop grid, frequency, log sizing) is
    // derived. 500 Hz — one Send/Feedback exchange every two milliseconds.
    inline constexpr double kControlDtS = 0.002;
    inline constexpr double kControlFrequencyHz = 1.0 / kControlDtS;
    inline constexpr std::chrono::microseconds kCyclePeriod{
        static_cast<long>(kControlDtS * 1e6)
    };

    // Cartesian velocity controller (Loop.h):
    //   v_desired = kKpCartesian * (p_desired - p_current)   [m/s]
    // kKpCartesian is 1/s: with 1.0, a 10 cm error commands 0.1 m/s.
    // There is deliberately NO velocity/acceleration/workspace limiting in
    // this version (docs/decisions/cartesian-velocity-controller.md) — the
    // gain and the size of typed targets ARE the speed control.
    inline constexpr double kKpCartesian = 10.0;

    // Damped-least-squares damping λ (Dls.h). Larger = slower but better
    // conditioned near singular poses; 0.1 follows Pinocchio's IK example
    // scale for arm-sized Jacobians. Shared by both control laws (and by
    // the reactive law's null-space projector).
    inline constexpr double kDlsLambda = 0.1;

    // Reactive-pose law (control/ReactiveLaw.h, kController = reactive-pose).
    // Gains start at the hardware-proven resolved-rate scale, NOT the
    // simulation's Kp=32/s — that would saturate the velocity clamp and trip
    // the following-error guard on any few-cm error (the documented
    // 2026-07-17 failure mode: docs/decisions/reactive-control-removal.md).
    // The position P gain is the shared kKpCartesian above. Term switches
    // implement the staged bring-up: P-only first; the velocity (Kd) term
    // and null-space centering exist in the law but default OFF
    // (docs/decisions/reactive-pose-port.md).
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

    // Per-joint clip for the resolved-rate q̇ before integration, deg/s —
    // the program's single speed limit, equal to the model limits above
    // (79.64 deg/s joints 1-4, 69.91 joints 5-7). The base enforces whatever
    // joint speed limits apply to low-level streaming, regardless of what
    // we command:
    // position setpoints stepping faster than an enforced limit are not
    // followed — the joint stands still, tracking error grows, and at
    // ~5 deg the safety kicks the arm out of low-level servoing
    // (WRONG_SERVOING_MODE). The clip uses the model hard limits; actuator
    // firmware safety thresholds can still bind lower in low-level mode
    // (docs/decisions/qdot-limit-raise.md; until 2026-07-22 this arm was
    // configured at 50 deg/s and the clip was a uniform 45).
    inline constexpr JointVector kQdotLimitDegS = kModelVelocityLimitsDegS;

    // Fault-stop policy (safety/Supervisor.h StopPolicy). COMPILE-TIME
    // ONLY — deliberately not settable from any runtime configuration.
    // false = the 2026-07-20 fault-ignoring experiment (attended use only;
    // docs/decisions/qdot-limit-raise.md, safety consequences).
    inline constexpr bool kStopOnFault = true;

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

    // Takeover and command shaping. The low-level session first streams an
    // unchanged measured-position hold, then control begins with null
    // centering ramped in. Integrated setpoints may lead feedback by at most
    // one degree — a command SHAPING rule, not a stop: it bounds how far the
    // setpoint can run away from a joint that is not following.
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
    inline constexpr double kTakeoverHoldS = 0.5;
    inline constexpr double kNullRampDurationS = 1.0;
    inline constexpr double kCommandLeadLimitDeg = 1.0;

    // Arrival notice (Loop.h): the loop prints one line the first time the
    // end-effector comes within this distance of a newly typed target, m.
    // Purely informational — motion and the position hold are unaffected.
    // 1 mm is tight: if the steady-state hold error turns out to hover at
    // this scale, the notice may come late or not at all (raise it back
    // toward 5 mm in that case).
    inline constexpr double kArrivalToleranceM = 0.001;

    // Trajectory playback (controller = "playback", control/TrajectoryPlayback.h).
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
    // control/TrajectoryFile.h). Velocity: 90% of the command clip, so the
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
    // (hardware/Record.h), so a killed run keeps everything up to the last
    // drain and the file holds the whole run, however long it is.
    //
    // kLogBufferSeconds only sizes the handoff queue between the loop and
    // that thread — it is slack for a disk hiccup, not a retention limit.
    // 30 s of samples is ~15 MB and 300x the drain interval; if the writer
    // ever falls that far behind, samples are dropped and counted rather
    // than silently overwritten.
    //
    // Cost of keeping everything: ~350 KB/s of CSV at 1 kHz, so a 30-minute
    // run is ~600 MB. Prune runs/ rather than shortening the record.
    inline constexpr std::size_t kLogBufferSeconds = 30;
    inline constexpr std::chrono::milliseconds kLogDrainInterval{100};
    inline constexpr const char* kLoopLogPrefix = "loop_log";
} // namespace config
