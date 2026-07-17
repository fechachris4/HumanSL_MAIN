/*
 * Config.h — the one place to change runtime settings.
 *
 * Controller programs in this repository take no command-line flags;
 * edit these constants and rebuild instead — including which mode
 * main() starts in (see kStartupMode below).
 */
#pragma once

#include "Motion.h" // JointVector, kDefaultSpeedLimits, speeds_within_limits

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

    // What main() does on startup. Edit this constant and rebuild to switch.
    // There is no reactive/Cartesian-servo mode (see AGENTS.md/known-issues.md
    // for why): only relative joint moves or read-only recording.
    enum class StartupMode { kJoints, kRecord };

    inline constexpr StartupMode kStartupMode = StartupMode::kJoints;

    // One-shot relative joint move, used only when kStartupMode == kJoints.
    // deltas: relative degrees, 0 = hold that joint. speeds: deg/s per joint,
    // checked against Motion.h's kDefaultSpeedLimits below at compile time.
    inline constexpr JointVector kJointDeltasDeg = {0.0, 40.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    inline constexpr JointVector kJointSpeedsDegS = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
    static_assert(speeds_within_limits(kJointSpeedsDegS),
                  "kJointSpeedsDegS exceeds kDefaultSpeedLimits (Motion.h) — do not raise the "
                  "limit, lower the speed");

    // Recording mode.
    inline constexpr double kRecordRateHz = 100.0;
    inline constexpr const char* kRecordFile = "joint_angles.csv";

    // Motion mode (per-cycle move log). A timestamp and ".csv" are appended per
    // run — one file per move, so a failed run's evidence is never overwritten.
    inline constexpr const char* kMoveLogPrefix = "move_log";
} // namespace config
