//
// Actuation: converts a clamped desired joint velocity into this cycle's
// per-joint setpoints, owning whatever persistent command state that
// requires (PositionIntegration: the q_command integrator).
//
// Interface asymmetry (F5, approved 2026-07-22): Prepare and Restore MAY do
// hardware I/O (mode/config changes, so implementations may hold hardware
// handles); Apply must be pure — no I/O, no blocking, no allocation.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_ACTUATION_H
#define HUMANSL_MASTERS_PROJECT_2025_ACTUATION_H

#include <optional>
#include <array>

#include <Eigen/Dense>

#include "JointVector.h"
#include "control/Controller.h" // RobotState

class Actuation
{
public:
    // What this cycle's Apply did, for the run record. `requested_deg` is the
    // setpoint the controller's velocity would have produced on its own, in
    // degrees, BEFORE any constraint this actuation applies; `lead_limited`
    // marks the joints where the written setpoint differs from it. Together
    // with the setpoints themselves they give requested-vs-sent per joint.
    struct ApplyStatus {
        JointVector requested_deg{};
        std::array<bool, 7> lead_limited{};
    };

    virtual ~Actuation() = default;

    // At takeover: after the servoing-mode switch and the seed read, BEFORE
    // the first Apply. Seeds internal command state from `state`.
    virtual void Prepare(const RobotState& state) = 0;

    // Per cycle, pure: fill `setpoints_deg` (degrees) from the
    // ALREADY-CLAMPED desired joint velocity and current measurement.
    // `setpoint_velocity_deg_s` reports the velocity actually applied after
    // constraints; ApplyStatus identifies active command-lead constraints.
    virtual ApplyStatus
    Apply(const Eigen::Matrix<double, 7, 1>& qdot_clamped_rad_s,
          const RobotState& measured_state, double dt_s,
          JointVector& setpoints_deg,
          JointVector& setpoint_velocity_deg_s) = 0;

    // The actuation's own tracking-guard signal: per-joint error (deg) the
    // Supervisor compares against the following-error limit, or nullopt if
    // this actuation cannot provide one — an actuation without a tracking
    // guard must not be run (the Runner will refuse to start on nullopt).
    virtual std::optional<JointVector>
    TrackingErrorDeg(const RobotState& state) const = 0;

    // At teardown, on every exit path, BEFORE the ServoingGuard restores
    // SINGLE_LEVEL. May do hardware I/O; must not throw (guard internally).
    // A future VelocityStreaming does its explicit zero-velocity frame and
    // the actuator POSITION-control-mode restore here, in that order (per
    // cartesian-velocity-controller.md's shutdown record) — and is BLOCKED
    // on (a) defining its TrackingErrorDeg and (b) re-reviewing
    // resolved-rate-position-integration.md's no-gravity-compensation
    // evidence before it may exist at all.
    virtual void Restore() = 0;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_ACTUATION_H
