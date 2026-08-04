//
// StopPriority — pure precedence for one completed cyclic feedback sample.
//
// This has no Kortex or Eigen dependency so the safety-critical combinations
// can be tested without connecting to hardware. The Runner maps feedback to
// these five facts after it has transmitted the safe holding frame.
//

#pragma once

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
