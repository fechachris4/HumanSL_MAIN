//
// Record: fixed-rate logging loop — read joints, timestamp, append to CSV.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_RECORD_H
#define HUMANSL_MASTERS_PROJECT_2025_RECORD_H

#include <array>
#include <atomic>
#include <cstdint>
#include <ostream>
#include <string>

#include <BaseCyclicClientRpc.h>

#include "Motion.h" // JointVector

namespace k_api = Kinova::Api;

// Reads all joint angles at rate_hz and appends one CSV row per cycle to
// `csv` ("time_s,dt_s,j1..j7"). Prints a heartbeat line to stdout once per
// second. Runs until `stop` becomes true (e.g. set by a Ctrl+C handler).
// Read-only: never moves the arm. Returns the number of samples recorded.
long record_joint_angles(k_api::BaseCyclic::BaseCyclicClient* base_cyclic, std::ostream& csv,
                         double rate_hz, const std::atomic<bool>& stop);

// One CSV row of the per-cycle move log: everything needed to reconstruct
// why a move failed after the fact — whether the robot stopped tracking,
// hit a torque/current wall, faulted, or left low-level servoing.
struct MoveLogSample {
    double t_s = 0.0;                            // since motion start
    double dt_s = 0.0;                           // since previous cycle
    JointVector commanded_deg{};                 // our setpoints (unwrapped)
    JointVector measured_deg{};                  // feedback, shifted next to the command
    JointVector error_deg{};                     // commanded - measured
    JointVector velocity_deg_s{};                // feedback
    JointVector torque_nm{};                     // feedback
    JointVector current_a{};                     // motor current, feedback
    std::array<std::uint32_t, 7> fault_bank{};   // per-actuator fault bits
    std::array<std::uint32_t, 7> warning_bank{}; // per-actuator warning bits
    std::uint32_t arm_state = 0;                 // Common::ArmState (drops out of
                                                 // SERVOING_LOW_LEVEL if we lose control)
    std::uint32_t base_fault_bank = 0;
    std::uint32_t base_warning_bank = 0;
    bool refresh_ok = false; // false on the row logged when a
                             // Refresh (send+receive) call failed
};

// CSV formatting for the per-cycle move log written by move_joints_relative.
// Kept here so all CSV formatting lives in Record, even though Motion drives
// the loop.
void write_move_log_header(std::ostream& csv);
void write_move_log_row(std::ostream& csv, const MoveLogSample& sample);

// "<prefix>_YYYY-MM-DD_HH-MM-SS.csv" in local time — one file per run, so a
// failed move's log is never overwritten by the next attempt.
std::string timestamped_csv_name(const std::string& prefix);

// Runs scripts/plot_move.py on a finished move log: tracking stats in the
// terminal, PNG plots saved next to the CSV. Best-effort — if python3, its
// libraries, or the script are missing it prints why and returns; the move's
// result is unaffected. Close/flush the log stream before calling this.
void plot_move_log(const std::string& csv_path);

#endif // HUMANSL_MASTERS_PROJECT_2025_RECORD_H
