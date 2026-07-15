//
// Controller: task-space control of the end-effector.
// Read-only error report, plus the reactive servo loop (MOVES THE ARM).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_CONTROLLER_H
#define HUMANSL_MASTERS_PROJECT_2025_CONTROLLER_H

#include <atomic>
#include <ostream>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include "Dynamics.h"

namespace k_api = Kinova::Api;

// Everything the control law needs each cycle, in the base frame, meters.
struct ControllerState
{
    Eigen::Vector3d current;  // end-effector position from FK
    Eigen::Vector3d target;   // from config::kTargetPosition
    Eigen::Vector3d error;    // target - current
};

// One "sense" step: read the joints once, run forward kinematics, and
// compare the end-effector position to the target. Read-only.
ControllerState controller_error(Dynamics& dynamics,
                                 k_api::BaseCyclic::BaseCyclicClient* base_cyclic);

// Calls controller_error once and prints target / current / error, plus the
// current EE orientation as a quaternion (copy it into Config.h to hold it).
void report_position_error(Dynamics& dynamics,
                           k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                           std::ostream& out);

// MOVES THE ARM: reactive task-space servo to the Config.h target pose
// (position + orientation) at 1 kHz until `stop` (Ctrl+C), then freezes at
// the measured position. Per cycle: FK + Jacobian -> PD task twist ->
// damped-least-squares joint rates -> optional null-space centering
// (config::kUseNullspaceCentering, off by default) -> speed clip ->
// integrate setpoints -> lead clamp -> stream via BaseCyclic. One CSV trace
// row per cycle goes to `log` (see ControlTraceSample in Record.h).
// Returns false on fault/communication error, true on clean Ctrl+C stop.
bool run_reactive_control(k_api::Base::BaseClient* base,
                          k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          Dynamics& dynamics,
                          const std::atomic<bool>& stop,
                          std::ostream* log);

#endif //HUMANSL_MASTERS_PROJECT_2025_CONTROLLER_H
