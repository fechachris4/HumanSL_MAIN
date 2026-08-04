//
// Runner — the cyclic control loop: timing, clamping, safety hooks,
// logging and the hardware exchange. It owns the cycle ORDER and contains
// no control math (that is Controller.h) and no program setup (Main.cpp).
//
// Takeover sequence — each step exactly once, in this order:
//   T1  the readiness gate has passed (RobotReadyForTakeover) — a hard
//       precondition, enforced by the robot_ready check that throws
//   T2  ServoingGuard construction -> base enters LOW_LEVEL_SERVOING
//   T3  CyclicSession::Seed — the one standalone read, AFTER the mode
//       switch (commanding earlier fails with WRONG_SERVOING_MODE)
//   T4  one measured-position holding frame; its reply seeds the actuation
//       and the controller (the only command-state measurement seed), then
//       one more holding frame whose reply is cycle 1's input
//   T5  normal control
//
// Per cycle: dt (measured, clamped; nominal on cycle 0) -> RobotState from
// the previous exchange -> PoseTargetSource::Get ->
// TrackingController::DesiredVelocity -> per-joint clamp to
// ±qdot_limit_deg_s -> PositionIntegration::Apply (command lead plus the
// joint-warning hold) -> CyclicSession::Send -> log sample -> edge-triggered fault prints ->
// acknowledgement-freshness update -> live-state/joint-boundary/stale-feedback
// priority -> the counter stops -> sleep_until grid.
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

#include <algorithm>
#include <atomic>
#include <chrono>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include "Actuation.h"
#include "Config.h"
#include "Controller.h"
#include "Hardware.h"
#include "Safety.h"
#include "State.h"
#include "Targets.h"

// MOVES THE ARM. Runs the sequence above until `stop`, a guard trip, or a
// communication failure; restores single-level servoing on every exit path.
LoopResult RunControlLoop(Kinova::Api::Base::BaseClient* base,
                          Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          PoseTargetSource& reference,
                          TrackingController& controller,
                          PositionIntegration& actuation,
                          LoopLog& log, const std::atomic<bool>& stop,
                          std::chrono::microseconds period,
                          const JointVector& qdot_limit_deg_s,
                          double following_error_limit_deg, bool robot_ready);
