//
// Runner — the cyclic control loop: timing, the Kortex takeover and
// exchange, safety-fact decoding, logging and teardown. Since the Plan 01
// extraction it owns NO per-cycle control mathematics: the command
// pipeline (measurement -> world-freshness classification -> reference ->
// law -> non-finite hold -> clamp -> integration) lives in
// ArmExecutionCore (ExecutionCore.h); this file assembles the core's
// input from Kortex feedback and the wait-free Vicon slot, transmits the
// command frame the core returns, and hands the reply's decoded health
// facts back for the stop verdict.
//
// Takeover sequence — each step exactly once, in this order:
//   T1  the readiness gate has passed (RobotReadyForTakeover) — a hard
//       precondition, enforced by the robot_ready check that throws
//   T2  ServoingGuard construction -> base enters LOW_LEVEL_SERVOING
//   T3  CyclicSession::Seed — the one standalone read, AFTER the mode
//       switch (commanding earlier fails with WRONG_SERVOING_MODE)
//   T4  stream the fixed Seed measured-position command on the normal grid
//       for config::kTakeoverHoldS; log and safety-classify every reply
//   T5  seed the execution core from the final hold reply (its first
//       measurement and its integrator seed), then send one
//       measured-position frame whose reply is cycle 1's input
//   T6  normal control
//
// Per cycle: measure dt at cycle start -> assemble ArmExecutionInput
// (previous exchange's joint feedback in degrees; one wait-free world
// slot read as a RAW WorldSample) -> ArmExecutionCore::Step (the frozen
// pipeline order, through position integration) -> publish the typed planning
// request if the core raised the replan edge (the edge is core evidence,
// the slot publish is adapter I/O) -> CyclicSession::Send the returned
// command frame -> log sample -> acknowledgement-freshness update ->
// decode the reply's health facts -> ArmExecutionCore::ResolveStop
// (send-then-resolve: the stop verdict never precedes the send) ->
// sleep_until grid.
//
// No stop is keyed on physical motion or a stationary position value. A
// per-actuator cyclic command acknowledgement that fails to advance for the
// configured 25 completed replies does stop, because it evidences a stalled
// downstream cyclic-feedback path.
//
// Teardown, on EVERY exit path (exceptions of any type included):
//   D1  explicit ServoingGuard restore to SINGLE_LEVEL + settling wait;
//       the destructor retries by unwinding if the explicit call failed
//   D2  decoded stop report
//

#pragma once

#include <atomic>
#include <chrono>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include "Config.h"
#include "ExecutionCore.h"
#include "GoalSocket.h"
#include "Hardware.h"
#include "PlanningRequest.h"
#include "PlanningRequestSlot.h"
#include "Safety.h"
#include "State.h"

class BasePoseSlot; // src/BasePose.h — only Runner.cpp needs the definition

// MOVES THE ARM. Runs the sequence above until `stop`, a guard trip, or a
// communication failure; restores single-level servoing on every exit path.
// `core` must be freshly constructed (not yet seeded): T5 performs its one
// Seed. `following_error_limit_deg` paces the takeover hold's guard and the
// stop report; it must equal the core's configured limit (both come from
// config::kFollowingErrorLimitDeg in production).
LoopResult RunControlLoop(Kinova::Api::Base::BaseClient* base,
                          Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          ArmExecutionCore& core,
                          LoopLog& log, const std::atomic<bool>& stop,
                          std::chrono::microseconds period,
                          double following_error_limit_deg, bool robot_ready,
                          PlanningArm planning_arm,
                          PlanningRequestSlot* planning_requests,
                          // The loop samples this wait-free slot once per
                          // cycle. Its coherent Mount pose/twist feeds both
                          // measurement and telemetry. nullptr is supported
                          // by hardware-free tests and yields awaiting-world
                          // zero-error hold behaviour.
                          BasePoseSlot* base_pose = nullptr,
                          GoalCommandSlot* live_goals = nullptr);
