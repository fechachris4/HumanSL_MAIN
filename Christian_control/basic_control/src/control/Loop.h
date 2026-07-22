//
// Loop: the resolved-rate Cartesian controller — MOVES THE ARM by integrating
// clipped joint velocities into a cyclically streamed POSITION command.
// Design, math, and the q_measured/q_command state distinction:
// docs/decisions/resolved-rate-position-integration.md
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

// The controller: takeover, cyclic resolved-rate loop, decoded stop report,
// and a guarded SINGLE_LEVEL_SERVOING restore on EVERY exit path (exceptions
// of any type included — unknown ones stop as kInternalError). Cycle order
// and loop I/O rules: resolved-rate-position-integration.md ("Cycle order").
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
