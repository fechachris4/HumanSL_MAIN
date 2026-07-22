//
// Motion: cyclic command primitives for BaseCyclic — all 7 joints at once.
// SENDS COMMANDS TO THE ARM — see Safety in README.md before use.
//
// Previous move executor removed 2026-07-20 (checkpoint c47525b3) —
// docs/decisions/one-shot-reduction.md.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_MOTION_H
#define HUMANSL_MASTERS_PROJECT_2025_MOTION_H

#include <array>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

namespace k_api = Kinova::Api;

// One value per joint, in Kortex actuator order: index 0 = joint 1 ... index 6
// = joint 7 (same ordering as feedback.actuators(i) / command.actuators(i)).
// Fixed size, so "exactly 7 values" is enforced by the type at compile time.
using JointVector = std::array<double, 7>;

// One cyclic exchange: write `angles` (degrees, any winding — wrapped to
// [0, 360) on the way out) into the frame, stamp the ids the base uses to
// reject stale packets, send, and return the same cycle's feedback.
// For actuators in POSITION control mode (the default).
k_api::BaseCyclic::Feedback send_positions(k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                                           k_api::BaseCyclic::Command& command,
                                           const JointVector& angles);

// Switch the base into LOW_LEVEL_SERVOING: from here until the restore, WE
// are the controller and must stream (or at least seed) position commands.
// Throws (Kortex) if the mode switch fails.
void enter_low_level_servoing(k_api::Base::BaseClient* base);

// Restore SINGLE_LEVEL_SERVOING — the single restore point for every exit
// path. The restore itself is a network call: on failure (link died) it
// warns the operator instead of throwing, and returns false. Safe to call
// from cleanup paths.
bool restore_single_level_servoing(k_api::Base::BaseClient* base);

#endif // HUMANSL_MASTERS_PROJECT_2025_MOTION_H
