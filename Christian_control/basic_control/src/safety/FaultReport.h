//
// FaultReport: decoding of Kortex fault banks and the human-readable stop /
// fault-change reports. Printing only — no policy, no robot I/O; never
// called from inside the cycle except edge-triggered (see Loop.cpp).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_FAULTREPORT_H
#define HUMANSL_MASTERS_PROJECT_2025_FAULTREPORT_H

#include <array>
#include <cstdint>
#include <string>

#include "safety/Supervisor.h" // LoopStop
#include "hardware/Record.h"   // LoopLogSample

// "16 (JOINT_FAULT)"-style decoding of a safety bank bitmask.
std::string DecodeBaseBank(std::uint32_t bank);
std::string DecodeActuatorBank(std::uint32_t bank);

// EVERY fault-bank change prints immediately, decoded, once per change
// (not per cycle — a persisting fault stays silent after its edge).
// Bounded: after kMaxFaultChangePrints events the loop stops printing
// (the CSV still has every cycle's banks). Visibility only — whether a
// fault STOPS the loop is StopPolicy's call.
inline constexpr int kMaxFaultChangePrints = 20;

// Stable machine-readable token for a stop reason ("user_stop",
// "following_error", ...) — the run CSV's exit trailer, so offline tooling
// can name the exit without parsing the prose report.
std::string StopReasonName(LoopStop reason);

void PrintStopReport(LoopStop reason, const LoopLogSample& s, long cycle,
                     double following_error_limit_deg);

void PrintFaultChange(const LoopLogSample& s, long cycle,
                      const std::array<std::uint32_t, 7>& prev_joint_banks,
                      std::uint32_t prev_base_bank);

#endif // HUMANSL_MASTERS_PROJECT_2025_FAULTREPORT_H
