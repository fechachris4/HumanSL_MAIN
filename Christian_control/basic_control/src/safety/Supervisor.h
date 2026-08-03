//
// Supervisor: stop classification and the pre-takeover readiness gate —
// the safety policy, separate from the controller it supervises.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_SUPERVISOR_H
#define HUMANSL_MASTERS_PROJECT_2025_SUPERVISOR_H

#include <cstdint>
#include <array>
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
// servoing restore still runs. The last two are the decision-12
// consecutive-cycle counters: non-finite controller output (held, never
// integrated) and cycle overruns.
enum class LoopStop {
    kUserStop,
    kRobotFault,
    kFollowingError,
    kLeftLowLevel,
    kCommunication,
    kInternalError,
    kNonFiniteCommand,
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
    // When the run ended, seconds on the loop's steady clock since control
    // began, and how many control cycles ran. Both come from the last logged
    // sample, so they line up with time_s / cycle in the run CSV.
    double stop_t_s = 0.0;
    long cycles = 0;
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
    // There is deliberately NO saturation stop: a pinned velocity clamp is
    // normal transit behavior for far targets (removed 2026-07-23; the
    // clamp itself still bounds every joint's speed).
    int nonfinite_stop_cycles = 3;   // non-finite controller output (held)
    int overrun_stop_cycles = 10;    // dt above overrun_factor x nominal
    double overrun_factor = 1.5;
};

// Each actuator feedback carries the command ID of the most recently
// processed cyclic command. A healthy stream advances every joint's
// acknowledgement each cycle; an ID that stays put while command frames keep
// going out is evidence of stale or non-advancing downstream feedback.
//
// TELEMETRY ONLY (2026-08-03). This used to stop the run after N unchanged
// cycles — a short-window "this feedback value did not change" exit. It now
// only counts, and the Runner logs the counters (ack_unchanged_j*); nothing
// here ends a run. Genuine communication failure still surfaces where it
// always did: a throwing Refresh (kCommunication).
//
// The counter is the number of CONSECUTIVE cycles this joint's
// command-acknowledgement ID has repeated; 0 means it advanced this cycle.
class FeedbackFreshnessMonitor
{
public:
    void Update(const std::array<std::uint32_t, 7>& command_ack);
    void Reset();

    const std::array<int, 7>& unchanged_cycles() const { return unchanged_cycles_; }

private:
    bool initialized_ = false;
    std::array<std::uint32_t, 7> previous_{};
    std::array<int, 7> unchanged_cycles_{};
};

// Consecutive-cycle counters, updated by the Runner every cycle (reset to
// zero on a healthy cycle); overrun_total is the whole-run tally reported
// after the loop.
struct CycleCounters {
    int nonfinite = 0;
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
// `out`; returns false on any LIVE fault (actuator fault bit, base fault
// other than the latched JOINT_FAULT summary, or the arm state itself
// reporting ARMSTATE_IN_FAULT — the summary is only noted when nothing
// live contradicts it).
bool RobotReadyForTakeover(const k_api::BaseCyclic::Feedback& feedback, std::ostream& out);

#endif // HUMANSL_MASTERS_PROJECT_2025_SUPERVISOR_H
