//
// Actuation — turning a clamped joint velocity into this cycle's position
// setpoints, and owning the persistent command state that requires.
//
// PositionIntegration owns only persistent command state: Prepare seeds it
// outside the cycle, and Apply must be pure — no I/O, no blocking, no
// allocation — because it runs inside the cycle.
//

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

#include <Eigen/Dense>

#include "Config.h"
#include "State.h"

// The dt Apply may integrate over: the measured elapsed cycle time, but
// never more than twice the nominal period — a scheduler stall must not
// integrate into one large position jump (the base faults on those).
// Non-positive or non-finite inputs fail safe to zero rather than reversing
// or poisoning an integration step.
inline double ClampedCycleDt(double measured_dt_s, double nominal_dt_s)
{
    if (!std::isfinite(measured_dt_s) || !std::isfinite(nominal_dt_s) ||
        measured_dt_s <= 0.0 || nominal_dt_s <= 0.0)
        return 0.0;
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
    // lead bound was active. A final rate envelope can defer lead recovery
    // after discontinuous feedback, so this flag may be true even when the
    // written setpoint stays at the requested value. With the setpoints
    // themselves they give requested-vs-sent per joint. If a bounded joint
    // would move farther outward past its configured warning threshold,
    // `joint_limit_warning_joint` names it and every setpoint holds the
    // prior command for the Runner to transmit and stop on.
    struct ApplyStatus {
        JointVector requested_deg{};
        std::array<bool, 7> lead_limited{};
        std::optional<int> joint_limit_warning_joint;
    };

    explicit PositionIntegration(double command_lead_limit_deg =
                                     std::numeric_limits<double>::infinity());

    // At takeover, after the servoing-mode switch and the seed read, BEFORE
    // the first Apply: q_command = q_measured — the ONLY time command state
    // is seeded from measurement. The Runner must call this before the first
    // Apply; the initial member value is zero and is not a safe substitute.
    // This state seed performs no hardware I/O.
    void Prepare(const RobotState& state);

    // Per cycle, PURE — no I/O, no blocking, no allocation. Proposes
    // q_command += q̇_clamped · dt from the ALREADY-CLAMPED desired velocity,
    // then bounds its lead over the wrapped measurement when configured. A
    // final per-cycle rate envelope always wins: after discontinuous feedback
    // lead recovery may temporarily remain beyond the configured lead bound,
    // leaving the existing following-error guard as the backstop. Before any
    // state commits, a bounded joint moving farther outward past its firmware
    // warning threshold holds the complete command frame. A direct
    // non-positive or non-finite dt is treated as zero for every result.
    // setpoint_velocity_deg_s reports the applied step after both constraints,
    // not the requested qdot.
    ApplyStatus Apply(const Eigen::Matrix<double, 7, 1>& qdot_clamped_rad_s,
                      const RobotState& measured_state, double dt_s,
                      JointVector& setpoints_deg,
                      JointVector& setpoint_velocity_deg_s);

private:
    double command_lead_limit_rad_;
    Eigen::Matrix<double, 7, 1> q_command_rad_ =
        Eigen::Matrix<double, 7, 1>::Zero();
};
