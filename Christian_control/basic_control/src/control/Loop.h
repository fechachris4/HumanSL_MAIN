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

#include "control/Target.h"
#include "hardware/Record.h"
#include "math/Kinematics.h"
#include "JointVector.h"
#include "safety/Supervisor.h" // LoopStop, LoopResult, RobotReadyForTakeover
#include "Dynamics.h"

namespace k_api = Kinova::Api;

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
