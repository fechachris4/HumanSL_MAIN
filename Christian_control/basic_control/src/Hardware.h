//
// Hardware — everything that talks to the robot or records what it did:
// the two Kortex sessions and their startup gates, the cyclic exchange,
// standalone feedback reads, and the run-log sample/queue/CSV writer.
//

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <ostream>
#include <string>
#include <thread>
#include <vector>
#include <ActuatorConfig.pb.h> // SafetyIdentifierBankA, used by EnsureJointLimits
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ControlConfigClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <ITransportClient.h>
#include <RouterClient.h>
#include <SessionManager.h>

#include "Config.h"

// ---------------------------------------------------------------
// Connect — the Kortex sessions
// ---------------------------------------------------------------

//
// Connect: the two Kortex sessions to the arm, opened on construction and
// closed on destruction (RAII), on every exit path including exceptions.
//




namespace k_api = Kinova::Api;

// Opens two sessions to the arm on construction, cleans up on destruction:
//  - TCP (port 10000): configuration + high-level commands  -> base()
//  - UDP (port 10001): 1 kHz low-level cyclic streaming     -> base_cyclic()
//
// Throws std::runtime_error if the arm cannot be reached. If opening the
// second channel fails, the first is closed properly before the exception
// leaves the constructor (C++ destroys fully-constructed members on the way
// out), so no session is ever left open on the robot.
class Connect
{
public:
    explicit Connect(const std::string& ip_address);

    k_api::Base::BaseClient* base()
    {
        return base_.get();
    }
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic()
    {
        return base_cyclic_.get();
    }

    // The TCP (configuration) router, for building additional service
    // clients outside this class — read-only diagnostics tools use this.
    //
    // CAUTION: each Kortex service client registers a notification callback
    // on its router, and the router rejects a SECOND registration for the
    // same service ("notification callback is already registered in the
    // client router"). Connect already owns DeviceConfig and ControlConfig,
    // so build only services it does NOT own here (for example,
    // DeviceManager) and reach the owned one through its gate methods.
    k_api::RouterClient* tcp_router()
    {
        return tcp_.router.get();
    }

    // The DeviceConfig service this connection already owns — safety
    // thresholds, device-level configuration. Shared rather than rebuilt,
    // see the caution on tcp_router().
    k_api::DeviceConfig::DeviceConfigClient* device_config()
    {
        return device_config_.get();
    }

    // The ControlConfig service this connection already owns — see the
    // caution on tcp_router(). VerifyKinematicHardLimits() uses it
    // internally; read-only diagnostics reach it here instead of building
    // a second one.
    k_api::ControlConfig::ControlConfigClient* control_config()
    {
        return control_config_.get();
    }

    // The configured JOINT_LIMIT safety thresholds
    // (config::kJointLimitWarnDeg / kJointLimitErrorDeg): read, correct if
    // they do not match, re-read to verify. Writes no motion. Exists
    // because these thresholds do not survive a robot power cycle and a
    // degenerate 0/0 band makes the firmware fault any outward motion —
    // see the Config.h comment for the full history. Only joints with a
    // non-zero config entry are touched.
    bool EnsureJointLimits(std::ostream& out);

    // READ-ONLY pre-takeover gate. KinematicLimits in the bundled Kortex
    // 2.7.0 schema exposes only speed/acceleration/twist fields, not live
    // joint positions. Require exactly seven finite positive live hard joint
    // speeds and require config::kQdotLimitDegS not to exceed them.
    bool VerifyKinematicHardLimits(std::ostream& out);

private:
    // One connected transport + router + logged-in session. Members are
    // declared socket -> router -> session, so destruction (reverse order)
    // logs out first and closes the socket last.
    struct Channel {
        // Takes ownership of an unconnected transport (TCP or UDP — that is
        // the only difference between the two channels), connects it, and
        // logs in. `label` names the channel in error messages.
        Channel(std::unique_ptr<k_api::ITransportClient> transport_in,
                const std::string& ip_address, unsigned port, const char* label);
        ~Channel();

        std::unique_ptr<k_api::ITransportClient> transport;
        std::unique_ptr<k_api::RouterClient> router;
        std::unique_ptr<k_api::SessionManager> session;
    };

    Channel tcp_; // port 10000: configuration + high-level commands
    Channel udp_; // port 10001: real-time cyclic channel

    std::unique_ptr<k_api::Base::BaseClient> base_;
    std::unique_ptr<k_api::DeviceConfig::DeviceConfigClient> device_config_;
    std::unique_ptr<k_api::ControlConfig::ControlConfigClient> control_config_;
    std::unique_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic_;
};

// ---------------------------------------------------------------
// CyclicSession — the per-cycle command frame
// ---------------------------------------------------------------

//
// Cyclic: the UDP cyclic exchange — owns the BaseCyclic command frame and
// its frame/command-id stamping. SENDS COMMANDS TO THE ARM.
//





namespace k_api = Kinova::Api;

// One session's cyclic command state.
//
// Seed(): the program's single standalone feedback read inside a command
// loop (via read_feedback), plus initialization of the command frame's 7
// actuator slots. Call once, AFTER the servoing-mode switch.
//
// Send(): the one exchange per cycle — write `setpoints_deg` (degrees, any
// winding; wrapped to [0, 360) on the way out) into the frame, stamp the
// ids the base uses to reject stale packets, send, and return the same
// cycle's feedback. For actuators in POSITION control mode (the default).
class CyclicSession
{
public:
    explicit CyclicSession(k_api::BaseCyclic::BaseCyclicClient* base_cyclic);

    k_api::BaseCyclic::Feedback Seed();
    k_api::BaseCyclic::Feedback Send(const JointVector& setpoints_deg);
    std::uint32_t last_command_frame_id() const { return command_.frame_id(); }

private:
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic_;
    k_api::BaseCyclic::Command command_;
};

// ---------------------------------------------------------------
// Measurement — the standalone feedback read
// ---------------------------------------------------------------

//
// Measure: read sensor data from the arm (no motion commands).
//



namespace k_api = Kinova::Api;

// THE robot state reader: the program's single standalone RefreshFeedback;
// the cyclic loop instead reuses Refresh(command)'s reply —
// docs/decisions/single-loop-controller.md ("single reader").
k_api::BaseCyclic::Feedback read_feedback(k_api::BaseCyclic::BaseCyclicClient* base_cyclic);

// ---------------------------------------------------------------
// Recording — the run log and its writer
// ---------------------------------------------------------------

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




// One cycle of the loop: everything needed to reconstruct afterwards what
// was asked (p_desired), what the controller computed (commanded joint
// velocities, current end-effector position), and what the robot did
// (measured joint state, torques, faults). The Cartesian error is not
// stored — it is exactly p_desired - p_current, computed offline.
//
// CSV column order (log_format = 13; WriteCsvRow is the authority):
//   time_s, dt_s, pd_x..z, p_x..z, cmd_j1..7, cmdvel_j1..7, meas_j1..7,
//   measraw_j1..7, vel_j1..7, torque_j1..7, fault_j1..7, arm_state,
//   base_fault, refresh_ok, sigma_min, rot_error_rad, t_send_s, t_recv_s,
//   quat_x, quat_y, quat_z, quat_w,
//   command_frame_id, feedback_frame_id, command_ack_j1..7,
//   status_flags_j1..7, jitter_us_j1..7,
//   cycle, req_j1..7, reqvel_j1..7, lead_limited_j1..7,
//   ack_unchanged_j1..7, taskvel_j1..7, nullvel_j1..7,
//   null_leak_mps,
//   vicon_seq, vicon_frame, vicon_latency_s, vicon_age_s,
//   vicon_<seg>_{x_m,y_m,z_m,qx,qy,qz,qw,valid} for each of
//   mount, leftbase, rightbase, leftee, rightee,
//   vicon_frame_rate_hz, vicon_mount_vx_mps..vz_mps,
//   vicon_mount_wx_radps..wz_radps, vicon_mount_twist_valid,
//   cart_traj_activated..cart_reference_time_s,
//   cart_ref_world_{pose,twist}, cart_meas_world_{pose,twist},
//   world_fresh, world_mount_twist_valid                 (225 columns)
// Format 4 appended cyclic frame/actuator acknowledgement diagnostics after
// format 3's columns. Format 5 (2026-08-03) drops the two columns that only
// named the removed no-motion/stale-feedback stops
// (stale_feedback_joint, no_response_joint) and appends the
// requested-vs-sent telemetry below. Every format-3 name and index is still
// valid; format-4 tooling that read the two dropped columns by NAME keeps
// working, by index does not. Format 6 (2026-08-04) removes the nine
// trajectory-playback columns (ref_j1..7, playback_t_s, playback_state)
// with the trajectory playback feature. Format 7 (2026-08-04) removes the
// informational reach-screen column; tooling should use column names rather
// than positional indexes. Format 8 (2026-08-05) appends the reactive-law
// decomposition — taskvel_j*/nullvel_j* (deg/s, the two law terms BEFORE
// summation and before the speed clamp) and null_leak_mps (the linear speed
// of J·q̇_null, the end-effector velocity the damped null-space projector
// leaks into task space; NaN on rows where no law ran). Added after the
// 2026-08-05 stall, where the summed telemetry could not show two large
// cancelling terms. Format 9 (2026-08-05) appends the joint-trajectory edges
// — traj_activated/traj_rejected/traj_complete are 1 only on the single cycle
// that accepted, refused, or finished a trajectory, and traj_start_error_deg
// is the worst-joint splice distance carried by a rejection row. It also
// appends the joint tracking law's own following-error evidence:
// joint_follow_error_deg is the worst joint's WRAPPED reference error every
// tracking cycle, and joint_follow_stop is 1 on a row whose error passed
// config::kTrajFollowingErrorStopDeg — the row that stopped the loop.
// Format 10 (2026-08-13) appends the world-pose observation: the Vicon
// sample contract (vicon_seq/vicon_frame/vicon_latency_s/vicon_age_s) and
// all five segments' raw poses in the VICON-WORLD frame, metres,
// quaternion x,y,z,w — named vicon_<seg>_* because they are segment poses,
// NOT calibrated mount/base poses (mountseg_T_mount does not exist yet).
// Observed and logged only; no control law reads them at this format. See
// the field-group comment inside LoopLogSample for the ZOH/absence rules.
// Format 11 (2026-08-13, world-hold slice) appends the hold evidence:
// hold_state (0 inactive / 1 engaged / 2 frozen / 3 latched-off),
// world_err_m and world_err_rot_rad (the UNRAMPED world-frame error the
// divergence latch watches; NaN when the hold is not the active
// reference), hold_ramp (0..1 authority ramp; NaN when not holding), and
// hold_reanchor_count (blackout recoveries this run). The Mount sample
// then fed the retired world-hold path — the first format in which a vicon_*
// input reached a control law; the vicon_* columns record exactly the
// sample the controller used that cycle (one slot read per cycle).
// Format 12 (2026-08-16, Mount-twist observation slice) appends Vicon's
// source frame rate and the filtered Mount segment twist in VICON-WORLD
// axes (linear m/s, angular rad/s). Repeated vicon_seq rows deliberately
// repeat the held estimate; validity says whether it belongs to that source
// frame. In format 12 this estimate was observation-only. Format 13 wires it
// into measured world twist and replaces the retired joint-trajectory
// and world-hold telemetry with the sole world-Cartesian reference path's
// identity, splice edges, reference/measured pose and twist, and freshness.
//
// Requested vs sent vs measured — the three quantities and their units:
//   reqvel_j*  deg/s  controller output BEFORE the per-joint speed clamp
//   cmdvel_j*  deg/s  velocity actually realised by the integrated command
//   req_j*     deg    integrated position command BEFORE lead/rate constraints
//   cmd_j*     deg    position actually written into the cyclic message
//   meas_j*    deg    position returned by the robot in the Send reply
// So req_j* - cmd_j* is the combined effect of the lead projection and final
// rate envelope this cycle. lead_limited_j* records that the lead projection
// was active, even if the rate envelope deferred recovery and req_j* equals
// cmd_j*. cmd_j* - meas_j* is the same-unit tracking error the
// following-error guard tests.
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
    double p_desired_m[3] = {0, 0, 0}; // Vicon world W
    double p_current_m[3] = {0, 0, 0}; // Vicon world W
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
    long cycle = 0; // 1.. per control cycle (0 = before control began)
    JointVector requested_deg{};            // pre lead/rate candidate
    JointVector requested_velocity_deg_s{}; // before the per-joint speed clamp
    std::array<bool, 7> lead_limited{};     // lead constraint was active
    std::array<int, 7> ack_unchanged_cycles{}; // consecutive repeated ack IDs

    // Reactive-law decomposition (log_format 8). See the column note above.
    JointVector qdot_task_deg_s{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    JointVector qdot_null_deg_s{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    double null_leak_m_s = std::numeric_limits<double>::quiet_NaN();

    // World-pose observation (log_format 10). Copied verbatim from the
    // BasePoseSample the loop read this cycle (src/BasePose.h — Vicon-world
    // frame, metres, quaternions x,y,z,w); OBSERVED AND LOGGED ONLY, no
    // control law reads any of it in this format. vicon_sequence repeats
    // across ~5 consecutive rows at the 100 Hz Vicon / 500 Hz loop ratio —
    // that is zero-order hold, detected by the unchanged sequence, with
    // vicon_age_s growing. All-NaN + sequence 0 means no Vicon source (or
    // no sample yet): absence is never a plausible zero. Kept as flat
    // arrays so Hardware.h does not include BasePose.h; Runner.cpp copies
    // field-for-field and the log-schema test pins the layout.
    std::uint64_t vicon_sequence = 0;      // 0 = no sample has ever arrived
    std::uint32_t vicon_frame_number = 0;  // Vicon's own frame counter
    double vicon_latency_s =               // SDK-reported total latency
        std::numeric_limits<double>::quiet_NaN();
    double vicon_age_s =                   // now − sample receive time,
        std::numeric_limits<double>::quiet_NaN(); // steady_clock seconds.
    // "now" is the CYCLE START (t_now, the same instant time_s is stamped
    // from), while the acquisition thread can publish mid-cycle — so a
    // frame that lands between cycle start and the row fill shows a small
    // NEGATIVE age (bounded by the cycle's compute+exchange time; −0.6 ms
    // observed on the 2026-08-13 validation run). Deliberately not
    // clamped: a clamp would hide the very timing evidence this column
    // exists to record.
    double vicon_seg_pos_m[5][3] = {       // Mount, LeftBase, RightBase,
        {std::numeric_limits<double>::quiet_NaN(),  // LeftEE, RightEE —
         std::numeric_limits<double>::quiet_NaN(),  // BasePose.h order
         std::numeric_limits<double>::quiet_NaN()},
        {std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()},
        {std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()},
        {std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()},
        {std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()}};
    double vicon_seg_quat_xyzw[5][4] = {
        {std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()},
        {std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()},
        {std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()},
        {std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()},
        {std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::quiet_NaN()}};
    bool vicon_seg_valid[5] = {false, false, false, false, false};

    // Mount-twist observation (log_format 12). Copied from the same single
    // BasePoseSample read as the pose fields above, so frame, pose and twist
    // cannot tear across Vicon publications. World axes; NaN + false means
    // unavailable. A repeated vicon_sequence is intentional zero-order hold.
    double vicon_frame_rate_hz =
        std::numeric_limits<double>::quiet_NaN();
    double vicon_mount_linear_world_m_s[3] = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    double vicon_mount_angular_world_rad_s[3] = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    bool vicon_mount_twist_valid = false;

    // Sole world-Cartesian path evidence (log_format 13). Reference and
    // measurement are from the exact cycle that generated requested qdot.
    bool cart_traj_activated = false;
    bool cart_traj_rejected = false;
    bool cart_traj_complete = false;
    bool cart_traj_cancelled = false;
    bool cart_replan_requested = false;
    double cart_start_position_error_m = 0.0;
    double cart_start_orientation_error_rad = 0.0;
    std::uint64_t cart_trajectory_id = 0;
    std::uint64_t cart_planner_vicon_sequence = 0;
    double cart_reference_time_s = 0.0;
    double cart_ref_position_world_m[3] = {0.0, 0.0, 0.0};
    double cart_ref_quat_world_xyzw[4] = {0.0, 0.0, 0.0, 1.0};
    double cart_ref_linear_world_m_s[3] = {0.0, 0.0, 0.0};
    double cart_ref_angular_world_rad_s[3] = {0.0, 0.0, 0.0};
    double cart_measured_position_world_m[3] = {0.0, 0.0, 0.0};
    double cart_measured_quat_world_xyzw[4] = {0.0, 0.0, 0.0, 1.0};
    double cart_measured_linear_world_m_s[3] = {0.0, 0.0, 0.0};
    double cart_measured_angular_world_rad_s[3] = {0.0, 0.0, 0.0};
    bool world_fresh = false;
    bool world_mount_twist_valid = false;
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

// Column header and one data row — the authority for log_format = 12. Both
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
