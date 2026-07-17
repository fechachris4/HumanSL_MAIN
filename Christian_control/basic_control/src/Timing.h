//
// Timing: latency benchmarks for the control loop.
// How long does it take to get feedback, run a full command cycle,
// and do our own dynamics math? All numbers in microseconds.
//
// Each benchmark writes its stats to `out` (e.g. an open std::ofstream),
// so the terminal stays clean.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_TIMING_H
#define HUMANSL_MASTERS_PROJECT_2025_TIMING_H

#include <ostream>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include "Dynamics.h"

namespace k_api = Kinova::Api;

// 1. Feedback round-trip: RefreshFeedback() -> joint angles in hand.
//    Read-only, the arm never moves.
void time_feedback_roundtrip(k_api::BaseCyclic::BaseCyclicClient* base_cyclic, std::ostream& out,
                             int cycles = 1000);

// 2. Full control cycle: Refresh(command) -> feedback back.
//    Switches to LOW_LEVEL_SERVOING and commands "hold current position"
//    every cycle, so the arm stays still. Restores SINGLE_LEVEL after.
void time_control_cycle(k_api::Base::BaseClient* base,
                        k_api::BaseCyclic::BaseCyclicClient* base_cyclic, std::ostream& out,
                        int cycles = 1000);

// 3. Compute budget: one gravity-torque solve (Pinocchio) per cycle.
//    Pure math, no robot communication.
void time_dynamics_solve(Dynamics& dynamics, const Eigen::VectorXd& q_pin, std::ostream& out,
                         int cycles = 1000);

#endif // HUMANSL_MASTERS_PROJECT_2025_TIMING_H
