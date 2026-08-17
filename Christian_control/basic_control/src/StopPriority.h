//
// StopPriority — the following-error stop fact and the pure precedence for
// one completed cyclic feedback sample.
//
// This has no Kortex or Eigen dependency so the safety-critical combinations
// can be tested without connecting to hardware. The Runner maps feedback to
// these five facts after it has transmitted the safe holding frame.
//

#pragma once

#include <array>
#include <cmath>

// The ONE definition of the following-error stop condition (2026-08-17
// consolidation): the reply position is shifted by whole turns to within
// +-180 deg of the integrated command (actuator feedback wraps at [0, 360)
// while the command is continuous) before the same-unit
// |command - measured| comparison. Both consumers delegate here — the
// takeover phase through Safety.cpp's LoopLogSample overload and the normal
// loop through ArmExecutionCore::ResolveStop — so the two phases cannot
// drift apart. Whether the stop is ENABLED stays each caller's own config
// read (config::kDisableFollowingErrorStop / ExecutionConfig's snapshot of
// it, equal in production by ProductionExecutionConfig()).
inline bool FollowingErrorExceeded(
    const std::array<double, 7>& commanded_deg,
    const std::array<double, 7>& reply_position_deg,
    double limit_deg)
{
    for (std::size_t i = 0; i < commanded_deg.size(); ++i) {
        const double measured_shifted =
            commanded_deg[i] +
            std::remainder(reply_position_deg[i] - commanded_deg[i], 360.0);
        if (std::abs(measured_shifted - commanded_deg[i]) > limit_deg)
            return true;
    }
    return false;
}

enum class StopPriorityReason {
    kNone,
    kRobotFault,
    kFollowingError,
    kLeftLowLevel,
    kJointLimitWarning,
    kStaleFeedback
};

struct StopPriorityDecision {
    StopPriorityReason reason = StopPriorityReason::kNone;
    bool live_fault_observed = false;
};

// Unconditional live state always wins. A live fault is always recorded, but
// only wins the stop reason when the compile-time policy enables fault stops.
// A held-frame joint warning and then stale acknowledgement win only when no
// unconditional state (and no enabled fault stop) is present.
inline constexpr StopPriorityDecision ResolveStopPriority(
    bool following_error, bool live_fault, bool left_low_level,
    bool joint_limit_warning, bool stop_on_fault, bool stale_feedback = false)
{
    if (following_error)
        return {StopPriorityReason::kFollowingError, live_fault};
    if (left_low_level)
        return {StopPriorityReason::kLeftLowLevel, live_fault};
    if (live_fault && stop_on_fault)
        return {StopPriorityReason::kRobotFault, true};
    if (joint_limit_warning)
        return {StopPriorityReason::kJointLimitWarning, live_fault};
    if (stale_feedback)
        return {StopPriorityReason::kStaleFeedback, live_fault};
    return {StopPriorityReason::kNone, live_fault};
}
