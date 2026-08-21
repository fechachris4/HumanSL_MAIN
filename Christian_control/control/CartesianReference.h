#pragma once

#include <memory>

#include "CartesianTrajectoryMailbox.h"
#include "State.h"

enum class CartesianReferenceState {
    kAwaitingWorld,
    kHolding,
    kTracking,
};

struct ExecutionConfig;

// Concrete, allocation-free-per-cycle source for startup hold, active WORLD
// trajectory interpolation, and final WORLD hold.
class CartesianReferenceSource
{
public:
    // The activation-continuity tolerances and the prolonged-stale
    // threshold are copied from the snapshot at construction; no config::
    // value is read again at runtime.
    CartesianReferenceSource(CartesianTrajectoryMailbox& mailbox,
                             const ExecutionConfig& config);
    // Production convenience: identical to injecting
    // ProductionExecutionConfig() (the runner is rewired to explicit
    // injection in Plan 01 Task 4).
    explicit CartesianReferenceSource(CartesianTrajectoryMailbox& mailbox);

    // `world_stale_elapsed_s` is the consecutive control time (s) since the
    // last fresh world sample: this cycle's dt already added while stale,
    // exactly zero on a fresh cycle. ArmExecutionCore owns that clock
    // (ExecutionCore.h — one clock, one owner; 2026-08-17 consolidation of
    // the former duplicate here); this policy only compares it against the
    // prolonged-stale threshold. `dt_s` is the fixed control step and advances
    // the trajectory clock.
    PoseReference Get(const RobotState& state,
                      const MeasuredCartesianState& measured,
                      double dt_s,
                      double world_stale_elapsed_s,
                      ControllerStatus& status);

    CartesianReferenceState state() const noexcept { return state_; }

private:
    PoseReference HoldingReference() const;
    PoseReference TrackingReference(double time_s) const;
    bool TryActivate(std::unique_ptr<WorldCartesianTrajectory> incoming,
                     const RobotState& state,
                     const MeasuredCartesianState& measured,
                     ControllerStatus& status);

    CartesianTrajectoryMailbox& mailbox_;
    // Immutable after construction (ExecutionConfig snapshot values).
    double arrival_position_tolerance_m_ = 0.0;
    double arrival_orientation_tolerance_rad_ = 0.0;
    double world_prolonged_stale_s_ = 0.0;
    CartesianReferenceState state_ = CartesianReferenceState::kAwaitingWorld;
    std::unique_ptr<WorldCartesianTrajectory> active_;
    CartesianPose hold_world_;
    std::uint64_t hold_trajectory_id_ = 0;
    std::uint64_t hold_planner_vicon_sequence_ = 0;
    double hold_reference_time_s_ = 0.0;
    bool hold_arrival_eligible_ = false;
    double trajectory_time_s_ = 0.0;
    bool complete_reported_ = false;
    bool previous_world_fresh_ = false;
    bool needs_replan_on_recovery_ = false;
    bool prolonged_stale_handled_ = false;
    std::uint64_t minimum_planner_vicon_sequence_ = 0;
};
