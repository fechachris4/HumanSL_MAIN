/*
 * Config.h — the compiled DEFAULTS for every runtime setting.
 *
 * Gains and thresholds can be overridden at runtime (CLI > TOML > these
 * defaults — app/Options.h, docs/decisions/runtime-config.md). Safety
 * policy cannot: kStopOnFault is compile-time only, and connection
 * parameters and the speed-clip derivation live only here.
 */
#pragma once

#include <chrono>
#include <cstddef>

#include "control/Motion.h" // JointVector

namespace config
{
    // Robot connection. Both Kortex sessions (TCP + UDP, see Connect.h) log
    // in with the same credentials; timeouts are what the base waits before
    // dropping an idle session/connection.
    inline constexpr const char* kRobotIp = "192.168.1.10";
    inline constexpr const char* kSessionUsername = "admin";
    inline constexpr const char* kSessionPassword = "admin";
    inline constexpr int kSessionInactivityTimeoutMs = 60000;
    inline constexpr int kConnectionInactivityTimeoutMs = 2000;

    // Model joint velocity limits, deg/s, joints 1-7 — the authoritative
    // values behind config/GEN3_custom.urdf's <limit velocity="..."> fields
    // (written there as rad/s literals: 1.3893 x4, 1.2200 x3; keep the two
    // in sync by hand — the URDF cannot read this header). Pinocchio takes
    // its velocityLimit straight from the URDF; no code overwrites it.
    // The controller's commanded-speed clip is derived from these below
    // (kQdotLimitDegS) — the ONE place speed limits are set.
    inline constexpr JointVector kModelVelocityLimitsDegS = {
        79.6, 79.6, 79.6, 79.6,
        69.9, 69.9, 69.9
    };

    // Control timing, single source of truth: the nominal period is
    // kControlDtS; everything else (loop grid, frequency, log sizing) is
    // derived. 100 Hz — position streaming does not need the 1 kHz cycle,
    // and 10 ms leaves ample compute headroom.
    inline constexpr double kControlDtS = 0.02;
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
    inline constexpr double kNullGain = 1.0;   // 1/s on the centering error
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

    // The controlled frame, from config/GEN3_custom.urdf. The flange frame:
    // no TCP offset (the gripper TCP sits ~0.12 m further along tool z —
    // see math/Kinematics.cpp's FK cross-check).
    inline constexpr const char* kEndEffectorFrame = "EndEffector_Link";

    // Per-joint clip for the resolved-rate q̇ before integration, deg/s —
    // the program's single speed limit, derived at compile time as
    // kQdotLimitSafetyFactor × the model limits above (≈71.6 deg/s joints
    // 1-4, ≈62.9 joints 5-7). The base enforces whatever joint speed SOFT
    // limits it is CONFIGURED with, regardless of what we command:
    // position setpoints stepping faster than an enforced limit are not
    // followed — the joint stands still, tracking error grows, and at
    // ~5 deg the safety kicks the arm out of low-level servoing
    // (WRONG_SERVOING_MODE). The derivation therefore assumes the base is
    // configured at the model limits — read the active configuration with
    // ./query_limits BEFORE a hardware session
    // (docs/decisions/qdot-limit-raise.md; until 2026-07-22 this arm was
    // configured at 50 deg/s and the clip was a uniform 45).
    inline constexpr double kQdotLimitSafetyFactor = 0.9; // 10% under
    inline constexpr JointVector kQdotLimitDegS = []
    {
        JointVector limits{};
        for (std::size_t i = 0; i < limits.size(); ++i)
            limits[i] = kQdotLimitSafetyFactor * kModelVelocityLimitsDegS[i];
        return limits;
    }();

    // Fault-stop policy (safety/Supervisor.h StopPolicy). COMPILE-TIME
    // ONLY — deliberately not settable from any runtime configuration.
    // false = the 2026-07-20 fault-ignoring experiment (attended use only;
    // docs/decisions/qdot-limit-raise.md, safety consequences).
    inline constexpr bool kStopOnFault = true;

    // Supervisor consecutive-cycle counters (decision 12); N <= 0 disables
    // one. Non-finite controller output is never integrated (that cycle
    // holds); saturation = >= 1 joint pinned at the clamp bound; overrun =
    // measured dt above kOverrunFactor x nominal.
    inline constexpr int kNonFiniteStopCycles = 3;
    inline constexpr int kSaturationStopCycles = 50; // 0.5 s at 100 Hz
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
