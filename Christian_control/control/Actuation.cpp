//
// Actuation — implementation of PositionIntegration (Actuation.h).
//

#include <cmath>
#include <tuple>

#include "Actuation.h"

#include "ExecutionConfig.h"

namespace
{
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;
    // The controller works in radians; Kortex commands and the log use
    // degrees, so conversion happens only at this boundary.
    constexpr double kRadToDeg = 180.0 / M_PI;
} // namespace

JointVelocityClampResult ClampJointVelocity(
    const Eigen::Matrix<double, 7, 1>& requested_rad_s,
    const JointVector& limits_deg_s)
{
    JointVelocityClampResult result;
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        const double requested_deg_s = requested_rad_s[i] * kRadToDeg;
        const double clamped_deg_s = std::clamp(requested_deg_s,
                                                -limits_deg_s[i],
                                                limits_deg_s[i]);
        result.qdot_rad_s[i] = clamped_deg_s / kRadToDeg;
        result.saturated[i] = clamped_deg_s != requested_deg_s;
    }
    return result;
}

PositionIntegration::PositionIntegration(const ExecutionConfig& config)
    // Store the safety limit in radians too, so every calculation inside the
    // controller uses one unit.
    : command_lead_limit_rad_(config.command_lead_limit_deg / kRadToDeg),
      software_limit_deg_(config.software_limit_deg)
{}

PositionIntegration::PositionIntegration(double command_lead_limit_deg)
    : command_lead_limit_rad_(command_lead_limit_deg / kRadToDeg),
      // Construction-time snapshot: the production boundaries, so the
      // whole-frame hold behaves exactly as with the production config.
      software_limit_deg_(ProductionExecutionConfig().software_limit_deg)
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
    // The core supplies the fixed positive control step, but Apply is a public
    // pure seam: direct invalid input must hold rather than reverse or poison
    // the persistent command. Use this one effective value everywhere below.
    const double effective_dt_s = std::isfinite(dt_s) && dt_s > 0.0 ? dt_s : 0.0;
    // Calculate all candidates before committing any of them: one bounded
    // joint crossing its software position boundary must hold the whole
    // command frame.
    Eigen::Matrix<double, 7, 1> next_q_command_rad;
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        // `previous` is the last position we asked this joint to hold.
        const double previous = q_command_rad_[i];
        // Take one small step from that last command using the already-safe,
        // speed-limited joint velocity supplied by the Runner.
        const double requested_step_rad = effective_dt_s > 0.0
            ? qdot_clamped_rad_s[i] * effective_dt_s : 0.0;
        const double proposed = previous + requested_step_rad;
        // Record the unconstrained proposal before the lead and final-rate
        // constraints can change it — this is the "requested" run record.
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
        const double max_step_rad = std::abs(requested_step_rad);
        next_q_command_rad[i] = std::clamp(lead_bounded_candidate,
                                           previous - max_step_rad,
                                           previous + max_step_rad);
    }

    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        const double software_limit_deg = software_limit_deg_[i];
        if (software_limit_deg <= 0.0)
            continue; // continuous joint: no bounded position boundary

        // Kortex reports bounded positions in [0, 360), while the firmware
        // thresholds are signed. Keep the candidate on the nearest turn to
        // the previous sent command, so 355 -> 356 is -5 -> -4, not a jump.
        const double previous_signed_deg =
            std::remainder(q_command_rad_[i] * kRadToDeg, 360.0);
        const double step_signed_deg = std::remainder(
            (next_q_command_rad[i] - q_command_rad_[i]) * kRadToDeg, 360.0);
        const double candidate_signed_deg = previous_signed_deg + step_signed_deg;
        const bool farther_outward =
            (candidate_signed_deg > software_limit_deg && step_signed_deg > 0.0) ||
            (candidate_signed_deg < -software_limit_deg && step_signed_deg < 0.0);
        if (!farther_outward)
            continue;

        status.joint_limit_warning_joint = i;
        for (int j = 0; j < NUM_JOINTS; ++j)
        {
            setpoints_deg[j] = q_command_rad_[j] * kRadToDeg;
            setpoint_velocity_deg_s[j] = 0.0;
        }
        return status;
    }

    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        const double previous = q_command_rad_[i];
        q_command_rad_[i] = next_q_command_rad[i];
        // Derive log velocity from the command actually applied above, after
        // both the lead candidate and the final rate envelope.
        setpoint_velocity_deg_s[i] =
            effective_dt_s > 0.0
                ? (q_command_rad_[i] - previous) * kRadToDeg / effective_dt_s
                : 0.0;
        setpoints_deg[i] = q_command_rad_[i] * kRadToDeg;
    }
    return status;
}
