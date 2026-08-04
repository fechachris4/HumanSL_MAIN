//
// Actuation — implementation of PositionIntegration (Actuation.h).
//

#include <cmath>
#include <tuple>

#include "Actuation.h"

namespace
{
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;
    // The controller works in radians; Kortex commands and the log use
    // degrees, so conversion happens only at this boundary.
    constexpr double kRadToDeg = 180.0 / M_PI;
} // namespace

PositionIntegration::PositionIntegration(double command_lead_limit_deg)
    // Store the safety limit in radians too, so every calculation inside the
    // controller uses one unit.
    : command_lead_limit_rad_(command_lead_limit_deg / kRadToDeg)
{}

void PositionIntegration::Prepare(const RobotState& state)
{
    // Start by commanding exactly where the robot really is. This prevents a
    // jump when low-level position control first takes over.
    q_command_rad_ = state.q_rad;
}

PositionIntegration::ApplyStatus PositionIntegration::Apply(
    const Eigen::Matrix<double, 7, 1>& qdot_clamped_rad_s,
    const RobotState& measured_state, double dt_s, JointVector& setpoints_deg,
    JointVector& setpoint_velocity_deg_s)
{
    ApplyStatus status;
    // Do the same small calculation independently for joints 1 through 7.
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        // `previous` is the last position we asked this joint to hold.
        const double previous = q_command_rad_[i];
        // Take one small step from that last command using the already-safe,
        // speed-limited joint velocity supplied by the Runner.
        const double proposed = previous + qdot_clamped_rad_s[i] * dt_s;
        // Record the unconstrained proposal before the lead limiter can
        // change it — this is the "requested" half of the run record.
        status.requested_deg[i] = proposed * kRadToDeg;
        // Feedback wraps at one turn. Shift it to the turn nearest the new
        // proposal before bounding command lead, so 359/1 deg is handled as
        // a 2 deg separation rather than a 358 deg jump.
        const double measured_near =
            proposed + std::remainder(measured_state.q_rad[i] - proposed,
                                      2.0 * M_PI);
        const double lead = proposed - measured_near;
        double lead_bounded_candidate = proposed;
        if (std::isfinite(command_lead_limit_rad_) &&
            std::abs(lead) > command_lead_limit_rad_)
        {
            // Keep the command on the same angular turn as this feedback,
            // then move only as far as the permitted command lead.
            lead_bounded_candidate = measured_near +
                std::copysign(command_lead_limit_rad_, lead);
            status.lead_limited[i] = true;
        }

        // A feedback discontinuity can make the lead-bound candidate jump in
        // the opposite direction. The final command must still move no more
        // than this cycle's already-clamped integration step, so recovery of
        // the configured lead may take later cycles.
        const double max_step_rad = std::abs(qdot_clamped_rad_s[i] * dt_s);
        q_command_rad_[i] = std::clamp(lead_bounded_candidate,
                                       previous - max_step_rad,
                                       previous + max_step_rad);

        // Derive log velocity from the command actually applied above, after
        // both the lead candidate and the final rate envelope.
        setpoint_velocity_deg_s[i] =
            dt_s > 0.0 ? (q_command_rad_[i] - previous) * kRadToDeg / dt_s : 0.0;
        setpoints_deg[i] = q_command_rad_[i] * kRadToDeg;
    }
    return status;
}
