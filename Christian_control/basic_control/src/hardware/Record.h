//
// Record: preallocated sample queue for the cyclic loop, drained to CSV by
// a writer thread WHILE the loop runs. Nothing here runs inside the loop
// except push(), which is allocation-, lock- and I/O-free.
//
// Why streaming rather than one write at the end: the log used to be held
// entirely in RAM until the loop returned, so any exit that skipped
// destructors — SIGKILL, the IDE stop button, a debugger detach, abort()
// via std::terminate — left a zero-byte CSV and no evidence whatsoever.
// 16 of the first 60 runs recorded under that design are empty files.
// Rows now reach the kernel every kLogDrainInterval, so a killed run keeps
// everything up to the last drain and the file always ends on a row
// boundary.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_RECORD_H
#define HUMANSL_MASTERS_PROJECT_2025_RECORD_H

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>
#include <thread>
#include <vector>

#include "JointVector.h"

// One cycle of the loop: everything needed to reconstruct afterwards what
// was asked (p_desired), what the controller computed (commanded joint
// velocities, current end-effector position), and what the robot did
// (measured joint state, torques, faults). The Cartesian error is not
// stored — it is exactly p_desired - p_current, computed offline.
//
// CSV column order (log_format = 5; WriteCsvRow is the authority):
//   time_s, dt_s, pd_x..z, p_x..z, cmd_j1..7, cmdvel_j1..7, meas_j1..7,
//   measraw_j1..7, vel_j1..7, torque_j1..7, fault_j1..7, arm_state,
//   base_fault, refresh_ok, sigma_min, rot_error_rad, t_send_s, t_recv_s,
//   quat_x, quat_y, quat_z, quat_w, pd_beyond_reach,
//   ref_j1..7, playback_t_s, playback_state,
//   command_frame_id, feedback_frame_id, command_ack_j1..7,
//   status_flags_j1..7, jitter_us_j1..7,
//   cycle, req_j1..7, reqvel_j1..7, lead_limited_j1..7,
//   ack_unchanged_j1..7                                   (130 columns)
// Format 4 appended cyclic frame/actuator acknowledgement diagnostics after
// format 3's columns. Format 5 (2026-08-03) drops the two columns that only
// named the removed no-motion/stale-feedback stops
// (stale_feedback_joint, no_response_joint) and appends the
// requested-vs-sent telemetry below. Every format-3 name and index is still
// valid; format-4 tooling that read the two dropped columns by NAME keeps
// working, by index does not.
//
// Requested vs sent vs measured — the three quantities and their units:
//   reqvel_j*  deg/s  controller output BEFORE the per-joint speed clamp
//   cmdvel_j*  deg/s  velocity actually realised by the integrated command
//   req_j*     deg    integrated position command BEFORE the lead limiter
//   cmd_j*     deg    position actually written into the cyclic message
//   meas_j*    deg    position returned by the robot in the Send reply
// So req_j* - cmd_j* is exactly what the lead limiter removed this cycle
// (nonzero only where lead_limited_j* is 1), and cmd_j* - meas_j* is the
// same-unit tracking error the following-error guard tests.
//
// ack_unchanged_j* is a COUNT OF CONSECUTIVE CYCLES this joint's
// command-acknowledgement ID repeated (0 = advanced this cycle). It is
// evidence about the feedback stream, not about acceptance: the Kortex 2.7.0
// BaseCyclic ActuatorFeedback exposes command_id (fixed32) as the ID of the
// last command the actuator PROCESSED — it does not report whether a
// setpoint was accepted or acted upon, so nothing here may be read that way.
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
    double p_desired_m[3] = {0, 0, 0}; // right-arm base frame
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
    double tool_quat_xyzw[4] = { // measured tool orientation, right base,
        std::numeric_limits<double>::quiet_NaN(), // Hamilton, w >= 0
        std::numeric_limits<double>::quiet_NaN(), // (ControllerStatus::
        std::numeric_limits<double>::quiet_NaN(), //  tool_quat; NaN when the
        std::numeric_limits<double>::quiet_NaN()}; // law has no tool frame)
    bool pd_beyond_reach = false; // target outside right-base-relative sphere

    // Trajectory playback (log_format 3): the per-cycle reference the
    // integrated command should land on, the playback clock, and the
    // playback state (0 none, 1 playing, 2 done, 3 refused). NaN/0 for
    // non-playback controllers.
    JointVector ref_deg{};
    double playback_t_s = std::numeric_limits<double>::quiet_NaN();
    int playback_state = 0;

    // Cyclic freshness evidence (log_format 4). command_frame_id is what
    // this process sent; feedback_frame_id and actuator_command_ack are what
    // Kortex returned. Per-actuator status/jitter distinguish a stale
    // downstream device from a healthy but mechanically stationary joint.
    std::uint32_t command_frame_id = 0;
    std::uint32_t feedback_frame_id = 0;
    std::array<std::uint32_t, 7> actuator_command_ack{};
    std::array<std::uint32_t, 7> actuator_status_flags{};
    std::array<std::uint32_t, 7> actuator_jitter_us{};

    // Requested-vs-sent telemetry (log_format 5). See the column note above.
    long cycle = 0; // 0 during the takeover hold, then 1.. per control cycle
    JointVector requested_deg{};            // before the command-lead limiter
    JointVector requested_velocity_deg_s{}; // before the per-joint speed clamp
    std::array<bool, 7> lead_limited{};     // limiter changed this setpoint
    std::array<int, 7> ack_unchanged_cycles{}; // consecutive repeated ack IDs
};

// Single-producer / single-consumer queue over a fixed-capacity ring, fully
// allocated in the constructor. The control loop is the only producer
// (push); a LoopLogWriter thread is the only consumer (Drain).
//
// The producer never overwrites a slot the consumer has not released, so no
// sample is ever read while it is being written — the race a lossy ring
// would have. The price is that when the consumer falls a whole buffer
// behind, push() drops the sample rather than the oldest one and counts it
// in dropped(). At the shipped sizing (kLogBufferSeconds of slack against a
// kLogDrainInterval drain) the consumer would have to stall for hundreds of
// drains for that to happen; dropped() > 0 means the disk, not the loop.
class LoopLog
{
public:
    explicit LoopLog(std::size_t capacity);

    // Producer side — the cyclic loop. Allocation-, lock- and I/O-free.
    void push(const LoopLogSample& sample);

    // Consumer side — the writer thread. Copies every sample published
    // since the previous call into `out` (resized to fit; reserve to
    // capacity() once to keep it allocation-free) and returns the count.
    std::size_t Drain(std::vector<LoopLogSample>& out);

    std::size_t capacity() const;
    std::size_t total_pushed() const; // samples ever offered by the loop
    std::size_t dropped() const;      // offered but refused (queue full)

private:
    std::vector<LoopLogSample> samples_;
    std::atomic<std::size_t> head_{0}; // published by the producer
    std::atomic<std::size_t> tail_{0}; // released by the consumer
    std::size_t dropped_ = 0;          // producer only; read after the loop
};

// Column header and one data row — the authority for log_format = 5. Both
// rely on the stream's default formatting (six significant digits), which
// is what every existing run log and every parsing script assumes.
void WriteCsvHeader(std::ostream& csv);
void WriteCsvRow(std::ostream& csv, const LoopLogSample& s);

// Drains a LoopLog to CSV on its own thread while the loop runs. The loop
// thread never touches the file; this thread never touches the robot.
//
// Construction writes the column header and flushes, so the file is already
// self-describing before the takeover. Each drain formats whole rows into
// one buffer, writes it, and flushes — the bytes are in the kernel's hands
// from that moment, and a kill can only ever cut the file at a row
// boundary, never mid-row.
class LoopLogWriter
{
public:
    // `log` and `csv` must outlive the writer, and nothing else may write
    // to `csv` until Stop() has returned.
    LoopLogWriter(LoopLog& log, std::ostream& csv,
                  std::chrono::milliseconds interval);
    ~LoopLogWriter(); // Stop(), swallowing exceptions

    LoopLogWriter(const LoopLogWriter&) = delete;
    LoopLogWriter& operator=(const LoopLogWriter&) = delete;

    // Stops the thread, drains what the loop left behind, flushes.
    // Idempotent. Call it only once the producer has stopped pushing (i.e.
    // after RunControlLoop has returned), so the final drain is final.
    void Stop();

    std::size_t rows_written() const; // valid once Stop() has returned

private:
    void Run();
    void DrainOnce();

    LoopLog& log_;
    std::ostream& csv_;
    std::chrono::milliseconds interval_;
    std::atomic<bool> stop_{false};
    std::vector<LoopLogSample> staging_; // reused every drain
    std::size_t rows_written_ = 0;       // writer thread only until joined
    std::thread thread_;
};

// "<prefix>_YYYYMMDD_HHMMSS.csv" in local time — one file per run, so a
// failed run's log is never overwritten by the next attempt.
std::string timestamped_csv_name(const std::string& prefix);

// "<runs_root>/YYYY-MM-DD" in local time — today's session folder. The
// caller creates it (main.cpp, before takeover) and puts the run CSV inside.
std::string dated_run_dir(const std::string& runs_root);

#endif // HUMANSL_MASTERS_PROJECT_2025_RECORD_H
