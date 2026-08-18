//
// ExecutionConfig — the production mapping and its validation
// (ExecutionConfig.h).
//

#include "ExecutionConfig.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace
{
    // Same conversion expression Controller.cpp has always used, so the
    // stored double is bitwise identical to the pre-snapshot value.
    constexpr double kDegToRad = M_PI / 180.0;

    [[noreturn]] void Reject(const std::string& field, const std::string& why)
    {
        throw std::invalid_argument("ExecutionConfig." + field + ": " + why);
    }

    void RequireFinite(double value, const char* field)
    {
        if (!std::isfinite(value))
            Reject(field, "must be finite");
    }

    // A feedback gain of the wrong sign inverts the control law; a
    // non-finite one guarantees a non-finite command.
    void RequireFiniteNonNegative(double value, const char* field)
    {
        RequireFinite(value, field);
        if (value < 0.0)
            Reject(field, "must be >= 0");
    }

    void RequireFinitePositive(double value, const char* field)
    {
        RequireFinite(value, field);
        if (value <= 0.0)
            Reject(field, "must be > 0");
    }
} // namespace

ExecutionConfig ProductionExecutionConfig()
{
    ExecutionConfig production;

    // Exactly the gain mapping the controller compiled until now
    // (position_enabled has no config switch and is always on).
    production.gains.kp_position_s_inv = config::kKpCartesian;
    production.gains.kp_rotation_s_inv = config::kKpRotation;
    production.gains.kd_position = config::kKdPosition;
    production.gains.kd_rotation = config::kKdRotation;
    production.gains.limit_avoid_gain_s_inv = config::kLimitAvoidGain;
    production.gains.dls_lambda = config::kDlsLambda;
    production.gains.position_enabled = true;
    production.gains.orientation_enabled = config::kOrientationEnabled;
    production.gains.velocity_enabled = config::kVelocityTermEnabled;
    production.gains.null_space_enabled = config::kNullSpaceEnabled;

    production.velocity_limit_deg_s = config::kQdotLimitDegS;
    production.software_limit_deg = config::kJointSoftwareLimitDeg;
    production.limit_avoid_zone_rad = config::kLimitAvoidZoneDeg * kDegToRad;

    production.command_lead_limit_deg = config::kCommandLeadLimitDeg;
    production.following_error_limit_deg = config::kFollowingErrorLimitDeg;

    production.world_fresh_max_age_s = config::kWorldFreshMaxAgeS;
    production.world_prolonged_stale_s = config::kWorldProlongedStaleS;
    production.mount_twist_filter_tau_s = config::kViconMountTwistFilterTauS;

    production.arrival_position_tolerance_m = config::kArrivalToleranceM;
    production.arrival_orientation_tolerance_rad =
        config::kArrivalOrientationToleranceRad;
    production.arrival_dwell_s = config::kArrivalDwellS;
    production.target_hold_s = config::kTargetHoldS;

    production.null_ramp_duration_s = config::kNullRampDurationS;

    production.stop_on_fault = config::kStopOnFault;
    production.disable_following_error_stop =
        config::kDisableFollowingErrorStop;
    production.nonfinite_stop_cycles = config::kNonFiniteStopCycles;
    production.overrun_stop_cycles = config::kOverrunStopCycles;
    return production;
}

void ValidateExecutionConfig(const ExecutionConfig& config)
{
    RequireFiniteNonNegative(config.gains.kp_position_s_inv,
                             "gains.kp_position_s_inv");
    RequireFiniteNonNegative(config.gains.kp_rotation_s_inv,
                             "gains.kp_rotation_s_inv");
    RequireFiniteNonNegative(config.gains.kd_position, "gains.kd_position");
    RequireFiniteNonNegative(config.gains.kd_rotation, "gains.kd_rotation");
    RequireFiniteNonNegative(config.gains.limit_avoid_gain_s_inv,
                             "gains.limit_avoid_gain_s_inv");
    // lambda > 0 is what guarantees JJ^T + lambda^2*I is invertible at a
    // singular pose — the damped inverse's boundedness argument.
    RequireFinitePositive(config.gains.dls_lambda, "gains.dls_lambda");

    for (int i = 0; i < 7; ++i) {
        // A negative bound makes std::clamp(lo > hi) undefined; a zero
        // bound silently freezes a joint the law believes it is moving.
        RequireFinitePositive(config.velocity_limit_deg_s[i],
                              "velocity_limit_deg_s");
        // 0 is the continuous-joint sentinel; the boundary check and the
        // null-space deadband wrap positions to signed (-180, 180] deg, so
        // a magnitude >= 180 deg can never be crossed and would silently
        // disable the guard while claiming to enforce it.
        RequireFinite(config.software_limit_deg[i], "software_limit_deg");
        if (config.software_limit_deg[i] < 0.0 ||
            config.software_limit_deg[i] >= 180.0)
            Reject("software_limit_deg",
                   "must be 0 (continuous joint) or in (0, 180) deg");
    }

    RequireFiniteNonNegative(config.limit_avoid_zone_rad,
                             "limit_avoid_zone_rad");

    // +infinity is the documented "lead bound disabled" value; zero would
    // pin the command to feedback and negative is meaningless.
    if (std::isnan(config.command_lead_limit_deg) ||
        config.command_lead_limit_deg <= 0.0)
        Reject("command_lead_limit_deg", "must be > 0 (+inf disables)");

    RequireFinitePositive(config.following_error_limit_deg,
                          "following_error_limit_deg");

    RequireFinitePositive(config.world_fresh_max_age_s,
                          "world_fresh_max_age_s");
    // A sample may be up to world_fresh_max_age_s old while still trusted;
    // a prolonged-stale threshold below that inverts the staleness ladder.
    RequireFinite(config.world_prolonged_stale_s, "world_prolonged_stale_s");
    if (config.world_prolonged_stale_s < config.world_fresh_max_age_s)
        Reject("world_prolonged_stale_s",
               "must be >= world_fresh_max_age_s");
    // The held-twist decay is exp(-stale/tau): tau <= 0 is undefined.
    RequireFinitePositive(config.mount_twist_filter_tau_s,
                          "mount_twist_filter_tau_s");

    RequireFiniteNonNegative(config.arrival_position_tolerance_m,
                             "arrival_position_tolerance_m");
    RequireFiniteNonNegative(config.arrival_orientation_tolerance_rad,
                             "arrival_orientation_tolerance_rad");
    // Non-positive dwell/hold are the documented "disabled" values
    // (Arrival.h), so only finiteness is required of the dwell itself.
    RequireFinite(config.arrival_dwell_s, "arrival_dwell_s");
    RequireFinite(config.target_hold_s, "target_hold_s");
    // Mirror of Config.h's static_assert: an ENABLED non-arrival timeout
    // must outlast the settling window or no target can ever be confirmed.
    if (config.target_hold_s > 0.0 &&
        config.target_hold_s <= config.arrival_dwell_s)
        Reject("target_hold_s",
               "an enabled timeout must outlast arrival_dwell_s");

    RequireFinite(config.null_ramp_duration_s, "null_ramp_duration_s");
    // stop_on_fault / disable_following_error_stop are policy booleans and
    // the stop counters use <= 0 as their documented "disabled" value —
    // no numeric constraint applies to them.
}
