//
// Safety — the guards that can stop the arm, and the reports that explain
// why. Stop classification and the pre-takeover readiness gate, fault-bank
// decoding and the stop/fault-change reports, and the RAII servoing guard
// that restores SINGLE_LEVEL on every exit path.
//

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <ostream>
#include <string>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include "Config.h"
#include "Hardware.h"

// ---------------------------------------------------------------
// Supervisor — stop classification and the readiness gate
// ---------------------------------------------------------------

//
// Supervisor: stop classification and the pre-takeover readiness gate —
// the safety policy, separate from the controller it supervises.
//

namespace k_api = Kinova::Api;

// Why the loop ended. kUserStop (Ctrl+C) is the only successful outcome —
// every other reason is a failure and exits nonzero. kFollowingError: a
// joint's command-measurement gap exceeded the configured limit (the arm
// stopped following the integrated command — fault, stall, or limit).
// kInternalError: an exception that is neither a Kortex error nor a
// runtime_error escaped the cycle — caught by the loop's catch-all so the
// servoing restore still runs. kJointLimitWarning holds the last safe full
// command frame before a bounded joint's outward warning crossing can be
// transmitted. The last two are the decision-12 consecutive-cycle counters:
// non-finite controller output (held, never integrated) and cycle overruns.
enum class LoopStop {
    kUserStop,
    kRobotFault,
    kFollowingError,
    kLeftLowLevel,
    kCommunication,
    kInternalError,
    kJointLimitWarning,
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

    const std::array<int, 7>& unchanged_cycles() const { return unchanged_cycles_; }

private:
    bool initialized_ = false;
    std::array<std::uint32_t, 7> previous_{};
    std::array<int, 7> unchanged_cycles_{};
};

// Consecutive-cycle counters, updated by the Runner every cycle (reset to
// zero on a healthy cycle) and compared directly against the Config.h
// thresholds there (decision 12; the checks run AFTER ClassifyStop so the
// guard and live faults keep priority). overrun_total is the whole-run
// tally reported after the loop.
struct CycleCounters {
    int nonfinite = 0;
    int overrun = 0;
    long overrun_total = 0;
};

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

// ---------------------------------------------------------------
// FaultReport — fault-bank decoding and reports
// ---------------------------------------------------------------

//
// FaultReport: decoding of Kortex fault banks and the human-readable stop /
// fault-change reports. Printing only — no policy, no robot I/O; never
// called from inside the cycle except edge-triggered (see Loop.cpp).
//




// "16 (JOINT_FAULT)"-style decoding of a safety bank bitmask.
std::string DecodeBaseBank(std::uint32_t bank);
std::string DecodeActuatorBank(std::uint32_t bank);

// EVERY fault-bank change prints immediately, decoded, once per change
// (not per cycle — a persisting fault stays silent after its edge).
// Bounded: after kMaxFaultChangePrints events the loop stops printing
// (the CSV still has every cycle's banks). Visibility only — whether a
// fault STOPS the loop is config::kStopOnFault's call.
inline constexpr int kMaxFaultChangePrints = 20;

// Stable machine-readable token for a stop reason ("user_stop",
// "following_error", ...) — the run CSV's exit trailer, so offline tooling
// can name the exit without parsing the prose report.
std::string StopReasonName(LoopStop reason);

void PrintStopReport(LoopStop reason, const LoopLogSample& s, long cycle,
                     double following_error_limit_deg,
                     int joint_limit_warning_joint);

void PrintFaultChange(const LoopLogSample& s, long cycle,
                      const std::array<std::uint32_t, 7>& prev_joint_banks,
                      std::uint32_t prev_base_bank);

// ---------------------------------------------------------------
// ServoingGuard — RAII ownership of the servoing mode
// ---------------------------------------------------------------

//
// ServoingGuard: RAII ownership of the base's servoing mode — the ONLY code
// allowed to call SetServoingMode. Constructing enters LOW_LEVEL_SERVOING
// (from here until Restore WE are the controller and must stream, or at
// least seed, commands); explicit Restore returns to SINGLE_LEVEL and the
// destructor retries by unwinding on every exit path if that call failed.
//



class ServoingGuard
{
public:
    // Throws (Kortex) if the mode switch fails.
    explicit ServoingGuard(k_api::Base::BaseClient* base);

    // Guarded restore: the restore is a network call that can fail if the
    // link died — on failure it warns the operator instead of throwing
    // (a throwing destructor would abort the program).
    ~ServoingGuard();

    // Restore immediately after cyclic streaming ends, report the exact
    // Kortex failure when possible, and wait for the mode transition to
    // settle. The destructor retries as a final RAII backstop if this fails.
    bool Restore(std::ostream& out) noexcept;

    ServoingGuard(const ServoingGuard&) = delete;
    ServoingGuard& operator=(const ServoingGuard&) = delete;

private:
    k_api::Base::BaseClient* base_;
    bool restored_ = false;
};
