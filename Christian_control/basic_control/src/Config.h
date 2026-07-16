/*
 * Config.h — the one place to change runtime settings.
 *
 * Controller programs in this repository take no command-line flags;
 * edit these constants and rebuild instead. Motion parameters are the
 * exception: they stay in motion.txt (see Motion.h) because their
 * presence/absence selects the mode at runtime.
 */
#pragma once

namespace config
{
    // Robot connection.
    inline constexpr const char* kRobotIp = "192.168.1.10";

    // Recording mode.
    inline constexpr double kRecordRateHz = 100.0;
    inline constexpr const char* kRecordFile = "joint_angles.csv";

    // Motion mode (per-cycle move log). A timestamp and ".csv" are appended per
    // run — one file per move, so a failed run's evidence is never overwritten.
    inline constexpr const char* kMoveLogPrefix = "move_log";

    // Simple joint-position hold. The target is captured from measured joints
    // once, after low-level servoing starts, then remains fixed for the run.
    // Each position setpoint is limited to a tiny lead over that cycle's
    // measurement; this is a command-safety bound, not a mechanical hard limit.
    inline constexpr double kSimpleHoldKp = 0.02;
    inline constexpr double kSimpleHoldMaxCommandLeadDeg = 0.1;
    inline constexpr const char* kSimpleHoldLogPrefix = "simple_hold";

    // Reactive controller target: desired end-effector position in the base
    // frame, meters. To hold the current position: run read-only (no
    // motion.txt), copy the printed EE position here, rebuild.
    inline constexpr double kTargetPosition[3] = {0.9, 0.2, 0.3};

    // Reactive controller gains. KP in 1/s (error -> task velocity), KD unitless
    // (velocity error -> velocity command; keep < 1, it feeds back measured
    // joint rates one cycle delayed and chatters as it approaches 1).
    inline constexpr double kKpPos = 1.5;
    inline constexpr double kKdPos = 0.2;
    inline constexpr double kDlsDamping = 0.05; // DLS lambda

    // Reactive safety clamps. Speed limit per joint, deg/s: conservative first
    // runs; the project ceiling is 45 (arm kicks out of low-level servoing near
    // its 50 deg/s soft limit — see Motion.h). Lead: max |command - measured|
    // per joint, rad; the arm faults out near 5 deg (0.087 rad) of tracking
    // error, so 0.05 rad (~2.9 deg) bounds it well clear of that.
    inline constexpr double kReactiveSpeedLimitDegS = 20.0;
    inline constexpr double kCtrlLeadRad = 0.05;

    // Reactive mode per-cycle trace log (timestamp + ".csv" appended per run).
    inline constexpr const char* kControlLogPrefix = "control_trace";
} // namespace config
