//
// Loop: the resolved-rate Cartesian controller — MOVES THE ARM toward the
// operator's desired end-effector position by integrating clipped joint
// velocities into a POSITION command streamed at 100 Hz.
//
//   e       = p_desired - p(q_measured)
//   v_d     = Kp * e
//   q̇_raw   = Jpᵀ (Jp Jpᵀ + λ² I₃)⁻¹ v_d     (damped least squares, Dls.h)
//   q̇_i     = clamp(q̇_raw_i, ±q̇_max_i)       per joint, independently
//   q_cmd  += q̇_clipped · dt                  (dt = measured, clamped)
//
// Actuators stay in their default POSITION control mode: the actuator's
// position servo rejects gravity implicitly. Low-level VELOCITY mode was
// tried and abandoned — the inner velocity loop has no gravity
// compensation and gravity-loaded joints drift/ignore commands (run log
// 2026-07-20 + Kinova kortex issues #42/#93/#156); see
// docs/decisions/resolved-rate-position-integration.md.
//
// The critical distinction: q_measured (fresh feedback, used for FK and
// the Jacobian every cycle) vs q_command (persistent integrator state,
// what is sent). q_command = q_measured happens ONLY at startup — resetting
// it from feedback every cycle would break the integration. Every run
// seeds afresh, so a restart can never inherit stale integrator state.
//
// One RefreshFeedback to seed; afterwards Refresh(command) is the only
// per-cycle exchange (its returned feedback drives the next iteration).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_LOOP_H
#define HUMANSL_MASTERS_PROJECT_2025_LOOP_H

#include <atomic>
#include <chrono>
#include <ostream>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include "control/Motion.h"
#include "control/Target.h"
#include "hardware/Record.h"
#include "math/Kinematics.h"
#include "Dynamics.h"

namespace k_api = Kinova::Api;

// Why the loop ended. kUserStop (Ctrl+C) is the only successful outcome —
// every other reason is a failure and exits nonzero. kFollowingError: a
// joint's command-measurement gap exceeded the configured limit (the arm
// stopped following the integrated command — fault, stall, or limit).
// kInternalError: an exception that is neither a Kortex error nor a
// runtime_error escaped the cycle — caught by the loop's catch-all so the
// servoing restore still runs.
enum class LoopStop {
    kUserStop,
    kRobotFault,
    kFollowingError,
    kLeftLowLevel,
    kCommunication,
    kInternalError
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

// Pre-takeover readiness check on a standalone feedback frame (read BEFORE
// the servoing-mode switch): prints the arm state and decoded fault banks to
// `out`; returns false on any LIVE fault (actuator fault bit, or base fault
// other than the latched JOINT_FAULT summary, which alone is only noted).
bool RobotReadyForTakeover(const k_api::BaseCyclic::Feedback& feedback, std::ostream& out);

// The controller. Enters LOW_LEVEL_SERVOING, reads one feedback frame
// (AFTER the mode switch — the round trip lets the base finish entering),
// seeds q_command = q_measured and p_desired = p(q_measured), sends one
// unchanged holding frame, then once per `period` on a fixed sleep_until
// grid:
//
//   use the previous exchange's feedback -> following-error guard
//   (any |q_command - q_measured| > following_error_limit_deg stops the
//   loop, even under a fault-ignoring experiment policy) -> fault/state
//   check ->
//   q_measured (deg -> rad at this boundary) -> FK + translational
//   Jacobian from the SAME q -> snapshot p_desired -> v_d = Kp e ->
//   damped least squares -> per-joint clamp to ±qdot_limit_deg_s[i] ->
//   q_command += q̇_clipped · dt (dt = measured elapsed cycle time,
//   clamped to [0, 2 · nominal] so a scheduler stall cannot integrate one
//   large position jump) -> rad -> deg -> send_positions (the one
//   exchange, returns next feedback) -> push one sample into `log` ->
//   sleep until the next grid slot.
//
// Inside the loop there is no per-cycle printing, allocation, or file I/O.
// Known exceptions, all edge-triggered and bounded: the small heap vector
// convertJointAnglesToConfig returns (see the decision record); the
// arrival notice — one line the first time ‖p_desired - p_current‖ drops
// under arrival_tolerance_m for a NEWLY typed target (TargetStore
// sequence change; the seeded hold target never prints); and fault
// visibility — one decoded old -> new line whenever a fault bank CHANGES,
// capped per run (Loop.cpp kMaxFaultChangePrints; the CSV keeps every
// cycle's banks). Shutdown, on EVERY exit path (stop, fault,
// exception): stop updating q_command — in POSITION mode the arm holds the
// last commanded setpoint — print the decoded stop report, and restore
// SINGLE_LEVEL_SERVOING (guarded; a failed restore warns and cannot
// overwrite the recorded stop reason). Exceptions of ANY type are caught
// (unknown ones as kInternalError) so no exception can skip the restore.
LoopResult RunResolvedRateLoop(k_api::Base::BaseClient* base,
                             k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                             Dynamics& dynamics, TargetStore& targets, LoopLog& log,
                             const std::atomic<bool>& stop, std::chrono::microseconds period,
                             double kp, double dls_lambda,
                             const JointVector& qdot_limit_deg_s,
                             double following_error_limit_deg,
                             double arrival_tolerance_m,
                             const std::string& ee_frame_name);

#endif // HUMANSL_MASTERS_PROJECT_2025_LOOP_H
