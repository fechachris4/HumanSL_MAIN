//
// Runner: the cyclic control loop — timing, clamping, safety hooks, logging
// and the hardware exchange. Controllers plug in via control/Controller.h,
// actuation strategies via actuation/Actuation.h. The Runner owns the cycle
// ORDER; it contains no control math.
//
// Takeover sequence (decision 9, 2026-07-22 — each step exactly once, in
// this order):
//   T1  the readiness gate has passed (RobotReadyForTakeover) — hard
//       precondition, asserted via the robot_ready argument
//   T2  ServoingGuard construction -> base enters LOW_LEVEL_SERVOING
//   T3  CyclicSession::Seed — the one standalone read, AFTER the mode
//       switch (the round trip lets the base finish entering; commanding
//       earlier fails with WRONG_SERVOING_MODE)
//   T4  Actuation::Prepare(seed state) — the ONLY seed of command state
//       from measurement
//   T5  Controller::Reset(seed state) — before any DesiredVelocity
//   T6  one unchanged holding frame; its reply is cycle 1's input
//
// Per cycle: dt (measured, clamped; nominal on cycle 0) -> RobotState from
// the previous exchange's reply -> Controller::DesiredVelocity -> per-joint
// clamp to ±qdot_limit_deg_s -> Actuation::Apply -> CyclicSession::Send ->
// log sample -> edge-triggered fault prints -> ClassifyStop -> sleep_until
// grid. Loop I/O rules as documented in
// docs/decisions/resolved-rate-position-integration.md ("Cycle order").
//
// Teardown, on EVERY exit path (exceptions of any type included):
//   D1  Actuation::Restore (guarded internally)
//   D2  decoded stop report
//   D3  ServoingGuard destructor restores SINGLE_LEVEL by unwinding —
//       cannot be skipped, cannot overwrite the recorded stop reason
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_RUNNER_H
#define HUMANSL_MASTERS_PROJECT_2025_RUNNER_H

#include <atomic>
#include <chrono>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include "actuation/Actuation.h"
#include "control/Controller.h"
#include "hardware/Record.h"
#include "safety/Supervisor.h"
#include "JointVector.h"

namespace k_api = Kinova::Api;

LoopResult RunControlLoop(k_api::Base::BaseClient* base,
                          k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          Controller& controller, Actuation& actuation,
                          LoopLog& log, const std::atomic<bool>& stop,
                          std::chrono::microseconds period,
                          const JointVector& qdot_limit_deg_s,
                          double following_error_limit_deg,
                          bool robot_ready);

#endif // HUMANSL_MASTERS_PROJECT_2025_RUNNER_H
