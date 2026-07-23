//
// Supervisor: stop classification and the pre-takeover readiness gate —
// the safety policy, separate from the controller it supervises.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_SUPERVISOR_H
#define HUMANSL_MASTERS_PROJECT_2025_SUPERVISOR_H

#include <cstdint>
#include <optional>
#include <ostream>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include "hardware/Record.h" // LoopLogSample

namespace k_api = Kinova::Api;

// Why the loop ended. kUserStop (Ctrl+C) is the only successful outcome —
// every other reason is a failure and exits nonzero. kFollowingError: a
// joint's command-measurement gap exceeded the configured limit (the arm
// stopped following the integrated command — fault, stall, or limit).
// kInternalError: an exception that is neither a Kortex error nor a
// runtime_error escaped the cycle — caught by the loop's catch-all so the
// servoing restore still runs. The last three are the decision-12
// consecutive-cycle counters: non-finite controller output (held, never
// integrated), a joint pinned at the velocity clamp, and cycle overruns.
enum class LoopStop {
    kUserStop,
    kRobotFault,
    kFollowingError,
    kLeftLowLevel,
    kCommunication,
    kInternalError,
    kNonFiniteCommand,
    kSaturation,
    kOverrun
};

// The loop's outcome. faults_observed is true if any LIVE fault signal (an
// actuator fault bit, or a base fault other than the latched JOINT_FAULT
// summary) was seen during the run — even while a fault-ignoring policy
// kept the loop running. Exit 0 requires a clean operator stop AND no
// observed faults: ignored faults taint the exit code.
struct LoopResult {
    LoopStop reason;
    bool faults_observed;
};

// The Runner's stop policy. stop_on_fault is COMPILE-TIME ONLY (F2,
// approved 2026-07-22): its value comes from config::kStopOnFault and no
// CLI flag or TOML key may set it. false reproduces the 2026-07-20
// fault-ignoring experiment — live fault bits do not stop the loop (bank
// changes still print, every cycle's banks are logged, observed faults
// still force a nonzero exit) and the Runner announces the policy loudly
// before takeover.
struct StopPolicy {
    bool stop_on_fault = true;

    // Consecutive-cycle stop counters (decision 12); N <= 0 disables one.
    int nonfinite_stop_cycles = 3;   // non-finite controller output (held)
    int saturation_stop_cycles = 50; // >= 1 joint at the clamp bound
    int overrun_stop_cycles = 10;    // dt above overrun_factor x nominal
    double overrun_factor = 1.5;
};

// Consecutive-cycle counters, updated by the Runner every cycle (reset to
// zero on a healthy cycle); overrun_total is the whole-run tally reported
// after the loop.
struct CycleCounters {
    int nonfinite = 0;
    int saturated = 0;
    int overrun = 0;
    long overrun_total = 0;
};

// The decision-12 stop check, run AFTER ClassifyStop (the guard and live
// faults keep priority).
std::optional<LoopStop> ClassifyCounters(const CycleCounters& counters,
                                         const StopPolicy& policy);

// The base's latched JOINT_FAULT summary bit — alone it is a stale
// historical aggregate, not a live interlock (fault-handling-hardening.md).
inline constexpr std::uint32_t kJointFaultBit =
    k_api::Base::SafetyIdentifier::JOINT_FAULT;

// Live-fault stop policy (no printing — loop-safe). The base's latched
// JOINT_FAULT summary bit alone does NOT stop the loop. The following-error
// guard is checked FIRST so no fault-ignoring policy can mask it.
bool ClassifyStop(const LoopLogSample& s, double following_error_limit_deg,
                  LoopStop& reason);

// Pre-takeover readiness check on a standalone feedback frame (read BEFORE
// the servoing-mode switch): prints the arm state and decoded fault banks to
// `out`; returns false on any LIVE fault (actuator fault bit, or base fault
// other than the latched JOINT_FAULT summary, which alone is only noted).
bool RobotReadyForTakeover(const k_api::BaseCyclic::Feedback& feedback, std::ostream& out);

#endif // HUMANSL_MASTERS_PROJECT_2025_SUPERVISOR_H
