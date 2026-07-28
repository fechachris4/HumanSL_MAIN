/*
 * Config.h — the compiled DEFAULTS for every runtime setting.
 *
 * Gains and thresholds can be overridden at runtime (CLI > TOML > these
 * defaults — app/Options.h, docs/decisions/runtime-config.md). Safety
 * policy cannot: kStopOnFault is compile-time only, and connection
 * parameters and the speed-clip derivation live only here.
 */
#pragma once

#include <array>
#include <chrono>
#include <cstddef>

#include "JointVector.h"

namespace config
{
    // Hardware ownership is deliberately right-only. The single runtime URDF
    // models both mounted arms, but only this IP gets a Kortex connection and
    // only the right seven-joint chain reaches the command loop.
    inline constexpr const char* kRightRobotIp = "192.168.1.10";
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

    // Right-arm commanded-speed limits, deg/s, joints 1-7. These remain the
    // controller's one actuator-command clip. The downloaded dual URDF keeps
    // its own 0.8727 rad/s descriptive model limits; they are not used to
    // overwrite or derive the hardware command clip.
    inline constexpr JointVector kModelVelocityLimitsDegS = {
        79.6, 79.6, 79.6, 79.6,
        69.9, 69.9, 69.9
    };

    // Control timing, single source of truth: the nominal period is
    // kControlDtS; everything else (loop grid, frequency, log sizing) is
    // derived. 1 kHz — the BaseCyclic low-level rate; one Send/Feedback
    // exchange per millisecond.
    inline constexpr double kControlDtS = 0.001;
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
    inline constexpr double kKpCartesian = 1.0;

    // Damped-least-squares damping λ (Dls.h). Larger = slower but better
    // conditioned near singular poses; 0.1 follows Pinocchio's IK example
    // scale for arm-sized Jacobians. Shared by both control laws (and by
    // the reactive law's null-space projector).
    inline constexpr double kDlsLambda = 0.1;

    // Reactive-pose law (control/ReactiveLaw.h, --controller reactive-pose).
    // Gains start at the hardware-proven resolved-rate scale, NOT the
    // simulation's Kp=32/s — that would saturate the velocity clamp and trip
    // the following-error guard on any few-cm error (the documented
    // 2026-07-17 failure mode: docs/decisions/reactive-control-removal.md).
    // The position P gain is the shared kKpCartesian above. Term switches
    // implement the staged bring-up: P-only first; the velocity (Kd) term
    // and null-space centering exist in the law but default OFF
    // (docs/decisions/reactive-pose-port.md).
    inline constexpr double kKpRotation = 1.0; // 1/s on the rotation-log error
    inline constexpr double kKdPosition = 0.3; // on the linear-velocity error
    inline constexpr double kKdRotation = 0.3; // on the angular-velocity error
    inline constexpr double kNullGain = 1.0; // 1/s on the centering error
    inline constexpr bool kOrientationEnabled = true;
    inline constexpr bool kVelocityTermEnabled = false; // Kd term
    inline constexpr bool kNullSpaceEnabled = false;

    // Null-space centering targets, deg, Kortex joint order. The Gen3's
    // bounded joints (2, 4, 6) have URDF ranges symmetric about 0, so their
    // midpoints are 0; the continuous joints (1, 3, 5, 7) have no meaningful
    // midpoint and are masked out. CAUTION: this arm has CONFIGURED position
    // soft limits far inside the URDF range (joint 4 near −19.6°, joint 6
    // near +36° — README "Safety"); centering toward 0 can push joint 4
    // toward its configured limit. Verify with ./query_limits before
    // enabling kNullSpaceEnabled.
    inline constexpr JointVector kNullMidpointDeg = {0, 0, 0, 0, 0, 0, 0};
    inline constexpr JointVector kNullCenteringMask = {0, 1, 0, 1, 0, 1, 0};

    // Reachable-workspace telemetry remains right-base-relative even though
    // targets/FK are now expressed in the dual model's world/common mount
    // frame. This is the right base_link origin in that common frame, derived
    // from the URDF's fixed +1.2085 rad roll followed by (0,-0.16,0) m.
    // FLAG ONLY — targets are never rejected or projected.
    inline constexpr std::array<double, 3> kRightBaseOriginCommonM = {
        0.0, -0.056707588407779945, -0.14961366721317215
    };
    inline constexpr double kReachRadiusM = 0.902; // Gen3 7-DOF max reach (Kinova spec)
    inline constexpr double kReachMarginM = 0.05; // near full extension is singular anyway

    // Per-joint clip for the resolved-rate q̇ before integration, deg/s —
    // the program's single speed limit, equal to the model limits above
    // (79.6 deg/s joints 1-4, 69.9 joints 5-7). The base enforces whatever
    // joint speed limits apply to low-level streaming, regardless of what
    // we command:
    // position setpoints stepping faster than an enforced limit are not
    // followed — the joint stands still, tracking error grows, and at
    // ~5 deg the safety kicks the arm out of low-level servoing
    // (WRONG_SERVOING_MODE). The derivation therefore assumes the base is
    // configured at the model limits — read the active configuration with
    // ./query_limits BEFORE a hardware session
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

    // Arrival notice (Loop.h): the loop prints one line the first time the
    // end-effector comes within this distance of a newly typed target, m.
    // Purely informational — motion and the position hold are unaffected.
    // 1 mm is tight: if the steady-state hold error turns out to hover at
    // this scale, the notice may come late or not at all (raise it back
    // toward 5 mm in that case).
    inline constexpr double kArrivalToleranceM = 0.001;

    // Loop log: preallocated ring buffer, most recent kLogCapacitySeconds
    // kept on very long runs; written to one timestamped CSV after the loop.
    inline constexpr std::size_t kLogCapacitySeconds = 600;
    inline constexpr const char* kLoopLogPrefix = "loop_log";
} // namespace config
