//
// Record: preallocated in-memory log for the cyclic loop, written to CSV
// once the loop has ended. Nothing here runs inside the loop except push(),
// which is allocation- and I/O-free.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_RECORD_H
#define HUMANSL_MASTERS_PROJECT_2025_RECORD_H

#include <array>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

#include "JointVector.h"

// One cycle of the loop: everything needed to reconstruct afterwards what
// was asked (p_desired), what the controller computed (commanded joint
// velocities, current end-effector position), and what the robot did
// (measured joint state, torques, faults). The Cartesian error is not
// stored — it is exactly p_desired - p_current, computed offline.
//
// CSV column order (log_format = 2; WriteCsv is the authority):
//   time_s, dt_s, pd_x..z, p_x..z, cmd_j1..7, cmdvel_j1..7, meas_j1..7,
//   measraw_j1..7, vel_j1..7, torque_j1..7, fault_j1..7, arm_state,
//   base_fault, refresh_ok, sigma_min, rot_error_rad, t_send_s, t_recv_s,
//   quat_x, quat_y, quat_z, quat_w, pd_beyond_reach        (69 columns)
// New columns are appended so older tooling's column indices stay valid.
//
// Timestamp semantics (all from one steady_clock, seconds since t_start):
//   time_s   — cycle start, when this cycle's feedback was consumed
//   dt_s     — measured gap to the previous cycle start
//   t_send_s — immediately before this cycle's cyclic.Send()
//   t_recv_s — immediately after Send() returned its feedback reply
// So t_send_s - time_s is the cycle's compute time and t_recv_s - t_send_s
// is the UDP exchange round-trip.
//
// Cross-exchange row semantics — rows mix two exchanges: in row i,
// p_current (FK) and the controller inputs come from the feedback RECEIVED
// in row i-1 (sampled near row i-1's t_recv_s), while meas_j*/vel/torque/
// fault fields come from row i's own Send reply (sampled near row i's
// t_recv_s). Offline analysis must match signals by these timestamps,
// never by row index.
struct LoopLogSample {
    double t_s = 0.0;  // since loop start
    double dt_s = 0.0; // since previous cycle
    double t_send_s = 0.0; // just before cyclic.Send
    double t_recv_s = 0.0; // just after Send returned
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
    double sigma_min =       // smallest singular value of the task Jacobian
        std::numeric_limits<double>::quiet_NaN(); // (NaN: no task Jacobian)
    double rot_error_rad =   // rotation-log error norm (reactive-pose law)
        std::numeric_limits<double>::quiet_NaN(); // (NaN: law has no
                                                  // orientation task)
    double tool_quat_xyzw[4] = { // measured tool orientation, base frame,
        std::numeric_limits<double>::quiet_NaN(), // Hamilton, w >= 0
        std::numeric_limits<double>::quiet_NaN(), // (ControllerStatus::
        std::numeric_limits<double>::quiet_NaN(), //  tool_quat; NaN when the
        std::numeric_limits<double>::quiet_NaN()}; // law has no tool frame)
    bool pd_beyond_reach = false; // |p_desired| outside the reach sphere
                                  // (config::kReachRadiusM - kReachMarginM)
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

// "<runs_root>/YYYY-MM-DD" in local time — today's session folder. The
// caller creates it (main.cpp, before takeover) and puts the run CSV inside.
std::string dated_run_dir(const std::string& runs_root);

#endif // HUMANSL_MASTERS_PROJECT_2025_RECORD_H
