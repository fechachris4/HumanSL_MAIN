/*
 * Config.h — the one place to change runtime settings.
 *
 * Controller programs in this repository take no command-line flags;
 * edit these constants and rebuild instead.
 */
#pragma once

#include <chrono>
#include <cstddef>

#include "JointVector.h"

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
    // values behind config/GEN3_custom.urdf's rad/s literals (keep the two
    // in sync by hand): docs/decisions/custom-urdf.md.
    inline constexpr JointVector kModelVelocityLimitsDegS = {
        79.6, 79.6, 79.6, 79.6,
        69.9, 69.9, 69.9
    };

    // Control timing, single source of truth: the nominal period is
    // kControlDtS; everything else (loop grid, frequency, log sizing) is
    // derived. 100 Hz — position streaming does not need the 1 kHz cycle,
    // and 10 ms leaves ample compute headroom.
    inline constexpr double kControlDtS = 0.01;
    inline constexpr double kControlFrequencyHz = 1.0 / kControlDtS;
    inline constexpr std::chrono::microseconds kCyclePeriod{
        static_cast<long>(kControlDtS * 1e6)
    };

    // Cartesian P gain, 1/s: v_desired = kKpCartesian * (p_desired - p_current).
    // Deliberately NO velocity/accel/workspace limiting in this version —
    // docs/decisions/cartesian-velocity-controller.md.
    inline constexpr double kKpCartesian = 1.0;

    // Damped-least-squares damping λ (Dls.h). Larger = slower but better
    // conditioned near singular poses; 0.1 follows Pinocchio's IK example
    // scale for arm-sized Jacobians.
    inline constexpr double kDlsLambda = 0.1;

    // The controlled frame, from config/GEN3_custom.urdf. The flange frame:
    // no TCP offset (the Robotiq 2F-85 gripper TCP sits ~0.12 m further
    // along tool z).
    inline constexpr const char* kEndEffectorFrame = "EndEffector_Link";

    // Per-joint clip on the resolved-rate q̇, deg/s — the program's single
    // speed limit, 10% under the model limits above. The base must be
    // CONFIGURED at the model limits (verify with ./query_limits before a
    // session) — mechanism and history: docs/decisions/qdot-limit-raise.md.
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
    inline constexpr const char* kLoopLogPrefix = "run";
} // namespace config
