//
// Actuation — turning a clamped joint velocity into this cycle's position
// setpoints, and owning the persistent command state that requires.
//
// Lifecycle asymmetry, which the Runner relies on: Prepare and Restore MAY
// do hardware I/O (mode changes, so an actuation may hold hardware
// handles); Apply must be pure — no I/O, no blocking, no allocation,
// because it runs inside the cycle.
//

#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <optional>

#include <Eigen/Dense>

#include "Config.h"
#include "State.h"

// The dt Apply may integrate over: the measured elapsed cycle time, but
// never more than twice the nominal period — a scheduler stall must not
// integrate into one large position jump (the base faults on those).
inline double ClampedCycleDt(double measured_dt_s, double nominal_dt_s)
{
    return std::min(measured_dt_s, 2.0 * nominal_dt_s);
}

//
// The actuation: integrate the clamped q̇ into a persistent position
// command streamed in verified POSITION control mode, with an optional
// command-to-measurement lead bound.
//
class PositionIntegration
{
public:
    // What this cycle's Apply did, for the run record. `requested_deg` is
    // the setpoint the controller's velocity alone would have produced,
    // BEFORE any constraint here; `lead_limited` marks the joints where the
    // written setpoint differs from it. With the setpoints themselves they
    // give requested-vs-sent per joint.
    struct ApplyStatus {
        JointVector requested_deg{};
        std::array<bool, 7> lead_limited{};
    };

    explicit PositionIntegration(double command_lead_limit_deg =
                                     std::numeric_limits<double>::infinity());

    // At takeover, after the servoing-mode switch and the seed read, BEFORE
    // the first Apply: q_command = q_measured — the ONLY time command state
    // is seeded from measurement. The Runner must call this before the first
    // Apply; the initial member value is zero and is not a safe substitute.
    // MAY do hardware I/O.
    void Prepare(const RobotState& state);

    // Per cycle, PURE — no I/O, no blocking, no allocation. Proposes
    // q_command += q̇_clamped · dt from the ALREADY-CLAMPED desired velocity,
    // then bounds its lead over the wrapped measurement when configured.
    // setpoint_velocity_deg_s reports the applied step, not the requested
    // qdot, because lead limiting may shorten it.
    ApplyStatus Apply(const Eigen::Matrix<double, 7, 1>& qdot_clamped_rad_s,
                      const RobotState& measured_state, double dt_s,
                      JointVector& setpoints_deg,
                      JointVector& setpoint_velocity_deg_s);

    // The tracking-guard signal the stop classifier compares against the
    // following-error limit: |q_command − q_measured| per joint, the
    // measurement shifted by whole turns to within ±180° of the command.
    // nullopt means no guard is available, and the Runner refuses to start.
    std::optional<JointVector> TrackingErrorDeg(const RobotState& state) const;

    // At teardown, on every exit path, BEFORE the ServoingGuard restores
    // SINGLE_LEVEL. MAY do hardware I/O; must not throw. Currently a no-op:
    // in POSITION mode the arm holds the last commanded setpoint.
    void Restore();

private:
    double command_lead_limit_rad_;
    Eigen::Matrix<double, 7, 1> q_command_rad_ =
        Eigen::Matrix<double, 7, 1>::Zero();
};
