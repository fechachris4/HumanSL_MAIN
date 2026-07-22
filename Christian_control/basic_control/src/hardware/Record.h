//
// Record: preallocated in-memory log for the cyclic loop, written to CSV
// once the loop has ended. Nothing here runs inside the loop except push(),
// which is allocation- and I/O-free.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_RECORD_H
#define HUMANSL_MASTERS_PROJECT_2025_RECORD_H

#include <array>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include "JointVector.h"

// One cycle of the loop: everything needed to reconstruct afterwards what
// was asked (p_desired), what the controller computed (commanded joint
// velocities, current end-effector position), and what the robot did
// (measured joint state, torques, faults). The Cartesian error is not
// stored — it is exactly p_desired - p_current, computed offline.
struct LoopLogSample {
    double t_s = 0.0;  // since loop start
    double dt_s = 0.0; // since previous cycle
    double p_desired_m[3] = {0, 0, 0}; // operator target (base frame)
    double p_current_m[3] = {0, 0, 0}; // FK of this cycle's measured q
    JointVector commanded_deg{};   // integrated position command (sent)
    JointVector commanded_velocity_deg_s{}; // clipped q̇ fed to the integrator
    JointVector measured_deg{};     // feedback shifted within ±180° of the
                                    // command (FillSample) — same axis as
                                    // commanded_deg for plots, but ambiguous
                                    // by whole turns once the gap is large
    JointVector measured_raw_deg{}; // feedback exactly as reported, [0, 360)
    JointVector velocity_deg_s{};  // measured
    JointVector torque_nm{};
    std::array<std::uint32_t, 7> fault_bank{}; // per-actuator fault bits
    std::uint32_t arm_state = 0;
    std::uint32_t base_fault_bank = 0;
    bool refresh_ok = false; // false on the row logged when Refresh failed
};

// Fixed-capacity ring buffer, fully allocated in the constructor. push()
// overwrites the oldest sample once full, so a very long run keeps the most
// recent kCapacity samples. WriteCsv emits rows oldest-first.
class LoopLog
{
public:
    explicit LoopLog(std::size_t capacity);

    void push(const LoopLogSample& sample); // alloc-free, loop-safe
    void WriteCsv(std::ostream& csv) const;

    std::size_t size() const;          // samples currently held
    std::size_t total_pushed() const;  // samples ever pushed (>= size)

private:
    std::vector<LoopLogSample> samples_;
    std::size_t next_ = 0;         // ring write position
    std::size_t total_pushed_ = 0;
};

// "<prefix>_YYYYMMDD_HHMMSS.csv" in local time — one file per run, so a
// failed run's log is never overwritten by the next attempt.
std::string timestamped_csv_name(const std::string& prefix);

#endif // HUMANSL_MASTERS_PROJECT_2025_RECORD_H
