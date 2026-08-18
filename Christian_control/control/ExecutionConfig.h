//
// ExecutionConfig — one immutable snapshot of every constant the per-cycle
// command pipeline consumes: reactive-law gains, velocity and position
// limits, staleness times, arrival tolerances and stop-counter thresholds.
//
// Purpose (Plan 01 Task 2): the controller, reference source and position
// integration are constructed FROM this snapshot and never read config::
// again at runtime, so the extracted execution core depends on values it was
// handed, not on a compiled global. ProductionExecutionConfig() is the ONLY
// mapping from the compiled config:: constants; it must remain the identity
// onto today's values (tests/test_execution_config.cpp pins every member).
//
// Units are explicit per field. Degree-valued limits stay in degrees because
// that is the configured and operator-facing unit (Kortex boundary); the
// null-space activation zone is stored in radians because the law consumes
// it in radians.
//

#pragma once

#include "Config.h"      // JointVector
#include "ReactiveLaw.h" // ReactivePoseGains

struct ExecutionConfig {
    // Reactive pose law: V_task = Kp*poseError + Kd*(V_ref - V_meas),
    // DLS damping and null-space deadband gain (ReactiveLaw.h).
    ReactivePoseGains gains;

    // Per-joint clip applied to qdot before integration, deg/s.
    JointVector velocity_limit_deg_s{};
    // Bounded-joint software position boundary magnitude, deg (signed
    // symmetric +-limit); 0 is the continuous-joint sentinel.
    JointVector software_limit_deg{};
    // Null-space limit-avoidance activation zone below the boundary, rad.
    double limit_avoid_zone_rad = 0.0;

    // Command shaping and the following-error stop threshold, deg.
    // +infinity disables the lead bound (PositionIntegration contract).
    double command_lead_limit_deg = 0.0;
    double following_error_limit_deg = 0.0;

    // World staleness ladder, s: trusted sample age bound, the accumulated
    // stale time that cancels an active trajectory, and the time constant
    // of the held Mount-twist decay exp(-stale/tau) while stale.
    double world_fresh_max_age_s = 0.0;
    double world_prolonged_stale_s = 0.0;
    double mount_twist_filter_tau_s = 0.0;

    // Arrival detection: tolerances, settling debounce and the non-arrival
    // timeout (non-positive dwell disables the debounce; non-positive hold
    // disables the timeout — Arrival.h).
    double arrival_position_tolerance_m = 0.0;
    double arrival_orientation_tolerance_rad = 0.0;
    double arrival_dwell_s = 0.0;
    double target_hold_s = 0.0;

    // Startup ramp of the null-space gain, s (non-positive = full gain
    // immediately, UnitRamp contract).
    double null_ramp_duration_s = 0.0;

    // Generic stop policy consumed by stop resolution and the decision-12
    // counters (a counter <= 0 disables that stop).
    bool stop_on_fault = false;
    bool disable_following_error_stop = false;
    int nonfinite_stop_cycles = 0;
    int overrun_stop_cycles = 0;
};

// The identity mapping from the compiled production constants in Config.h.
// The only place config:: values enter the execution path.
ExecutionConfig ProductionExecutionConfig();

// Throws std::invalid_argument naming the offending field when a value is
// physically meaningless (non-finite gain, non-positive limit, inverted
// staleness ordering, negative tolerance, inexpressible bounded-joint
// magnitude). Construction-time only — never called in the 500 Hz path.
void ValidateExecutionConfig(const ExecutionConfig& config);
