//
// Cyclic: the UDP cyclic exchange — owns the BaseCyclic command frame and
// its frame/command-id stamping. SENDS COMMANDS TO THE ARM.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_CYCLIC_H
#define HUMANSL_MASTERS_PROJECT_2025_CYCLIC_H

#include <BaseCyclicClientRpc.h>

#include "JointVector.h"

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

private:
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic_;
    k_api::BaseCyclic::Command command_;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_CYCLIC_H
