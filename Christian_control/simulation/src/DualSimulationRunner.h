//
// DualSimulationRunner — the single deterministic coordinator of the
// dual-arm execution twin (Plan 02 Task 5). One thread, fixed order; its
// Step() reads top-to-bottom as the frozen hardware cycle:
//
//   build both ArmExecutionInputs from the PREVIOUS exchange reply and
//   the same tick time / same ideal Mount WorldSample (per-arm joint
//   fields, shared world — the input struct carries per-arm measurements,
//   so one literally shared instance would feed the left core the right
//   arm's joints)
//   -> right core Step -> left core Step   (both commands computed
//                                           before physics advances)
//   -> backend Exchange                     (the "transmit": one 2 ms
//                                           tick of held controls)
//   -> right ResolveStop -> left ResolveStop (one-shot, against the reply
//                                           to THIS cycle's send — the
//                                           frozen send-then-resolve
//                                           invariant)
//   -> merge into the shared stop latch -> return the cycle snapshot.
//
// Ideal sensing (this plan): the raw WorldSample is built here from the
// prescribed Mount state — valid, pose/twist exact, sequence = tick + 1
// (sequence 0 means "never arrived", State.h:61, so the first tick must
// present 1 to be classifiable as fresh), age 0. The CORE owns freshness
// classification, ZOH and stale-twist decay; this type never classifies.
// AdapterHealth is constant all-false: the ideal adapter honestly has no
// fault banks, no servoing mode and no acknowledgement clock to report.
//
// One prescription per tick, pose and twist together — one
// SampleMountMotion(t) call (MountMotion.h) evaluated once in Step(): the
// pose goes to the plant and to the sample, the twist to the sample only,
// because the Mount is a world-welded mocap body with no velocity the
// physics can see. They are never two independently supplied numbers,
// because nothing could then stop a run from telling the cores the Mount
// is moving while the plant holds it still. With the default kStatic
// motion the pose is the configured anchor and the twist is its exact
// derivative: zero.
//
// Seeding: each core is seeded from the backend's CONTINUOUS degrees, not
// the wrapped wire (the home keyframe holds joint 4 at -130 deg, which
// the wrapped wire reports as 230 deg; the backend converts command
// degrees to ctrl by direct deg -> rad with no unwrap, so a wrapped seed
// would command joint 4 to 4.01 rad against a +/-2.62 rad range).
// Per-cycle Step inputs and ResolveStop replies stay wrapped, exactly as
// the hardware wire delivers them.
//
// The hold: the trajectory mailboxes stay empty, so each core's
// CartesianReferenceSource performs its kAwaitingWorld -> kHolding
// transition on the first fresh WorldSample and holds the measured TCP
// world pose from that cycle. No new hold concept exists here.
//
// Runtime guards (accepted hazard questionnaire — API-ordering only,
// matching the core's own seeded_ pattern): Step before Start, Start
// twice without Reset, and Step after the shared stop latched all throw
// std::logic_error. A tick after a stop verdict would produce
// plausible-looking evidence from a run the stop ladder already
// condemned; in this offline rig the loud stop is the informative
// outcome. No numeric or state re-checks: non-finite inputs and diverged
// state are owned per-substep by MujocoBackend, non-finite commands by
// the core (validate once).
//
// Telemetry: the returned DualSimulationCycle IS the immutable snapshot,
// assembled at this boundary and returned by value. No observers, no
// callbacks; SimMain decides what to print.
//
// Evidence class for anything computed against this rig: simulation
// (generic position servos — not a Kinova servo model).
//

#pragma once

#include <cstdint>
#include <optional>

#include "CartesianTrajectoryMailbox.h"
#include "Dynamics.h"
#include "ExecutionCore.h"
#include "Kinematics.h"
#include "MujocoBackend.h"
#include "SimulationConfig.h"

// One completed control cycle, by value: both cores' full evidence, both
// stop verdicts, the exchange feedback the NEXT cycle will consume, and
// the merged latch state.
struct DualSimulationCycle {
    ArmExecutionResult right;
    ArmExecutionResult left;
    ExecutionStopDecision right_stop;
    ExecutionStopDecision left_stop;
    SimArmFeedback next_feedback;
    bool shared_stop = false;
};

class DualSimulationRunner {
public:
    // Validates the SimulationConfig once (fields no other constructor
    // owns); the backend and the cores validate their own inputs at their
    // own construction. Throws std::invalid_argument / std::runtime_error
    // out of those constructors on a meaningless configuration or model.
    explicit DualSimulationRunner(const SimulationConfig& config);

    DualSimulationRunner(const DualSimulationRunner&) = delete;
    DualSimulationRunner& operator=(const DualSimulationRunner&) = delete;

    // Backend Seed (home keyframe + prescribed Mount) then both core
    // Seeds from the continuous feedback. Returns the seed feedback.
    // Throws std::logic_error on Start without an intervening Reset.
    SimArmFeedback Start();

    // One 0.002 s control cycle in the frozen order above. Throws
    // std::logic_error before Start or after the shared stop latched.
    DualSimulationCycle Step();

    // Re-emplaces both cores (ArmExecutionCore::Seed is one-shot by
    // contract, so reset means reconstruct, not re-seed), clears the
    // latch and the tick counter. A fresh Start is required before the
    // next Step.
    void Reset();

    long tick() const { return tick_; }
    bool started() const { return started_; }
    bool stopped() const { return stop_latched_; }
    // Read-only plant access for validation and telemetry (sim truth is
    // evaluation-only, contract section 4 — never fed to the cores).
    const MujocoBackend& Backend() const { return backend_; }

private:
    SimulationConfig config_;

    // Production kinematic authority: one Pinocchio model, one
    // DualArmKinematics per controlled arm (constructed exactly as the
    // parity test constructs them against the same URDF).
    Dynamics dynamics_;
    DualArmKinematics kinematics_right_;
    DualArmKinematics kinematics_left_;

    // Deliberately empty: no trajectory is ever published in Plan 02.
    CartesianTrajectoryMailbox mailbox_right_;
    CartesianTrajectoryMailbox mailbox_left_;

    MujocoBackend backend_;

    // In std::optional so Reset() can re-emplace fresh, unseeded cores.
    std::optional<ArmExecutionCore> core_right_;
    std::optional<ArmExecutionCore> core_left_;

    // The previous exchange reply = the next cycle's measurement.
    SimArmFeedback feedback_{};
    // The tick counter is also the ideal-sensing sequence owner:
    // WorldSample sequence = tick + 1 (one owner per concern).
    long tick_ = 0;
    bool started_ = false;
    bool stop_latched_ = false;
};
