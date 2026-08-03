//
// PositionIntegration: the implemented actuation — integrate the clamped q̇
// into a persistent position command streamed in verified POSITION control
// mode, with an optional command-to-measurement lead bound:
// docs/decisions/resolved-rate-position-integration.md.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_POSITIONINTEGRATION_H
#define HUMANSL_MASTERS_PROJECT_2025_POSITIONINTEGRATION_H

#include <limits>

#include "actuation/Actuation.h"

class PositionIntegration : public Actuation
{
public:
    explicit PositionIntegration(double command_lead_limit_deg =
                                     std::numeric_limits<double>::infinity());

    // q_command = q_measured — the ONLY time command state is seeded from
    // measurement (resolved-rate-position-integration.md, "state distinction").
    // The Runner must call this before the first Apply: the initial member
    // value is zero and is not a safe substitute for robot feedback.
    void Prepare(const RobotState& state) override;

    // Propose q_command += q̇_clamped · dt, then bound its lead over the
    // wrapped measurement when configured; setpoints are q_command in deg.
    // setpoint_velocity_deg_s reports the applied command step, not the
    // requested qdot, because lead limiting may shorten the step.
    ApplyStatus Apply(const Eigen::Matrix<double, 7, 1>& qdot_clamped_rad_s,
                      const RobotState& measured_state, double dt_s,
                      JointVector& setpoints_deg,
                      JointVector& setpoint_velocity_deg_s) override;

    // |q_command − q_measured| per joint, the measurement shifted by whole
    // turns to within ±180° of the command (same convention as FillSample).
    std::optional<JointVector> TrackingErrorDeg(const RobotState& state) const override;

    // No-op: in POSITION mode the arm holds the last commanded setpoint.
    void Restore() override;

private:
    double command_lead_limit_rad_;
    Eigen::Matrix<double, 7, 1> q_command_rad_ =
        Eigen::Matrix<double, 7, 1>::Zero();
};

#endif // HUMANSL_MASTERS_PROJECT_2025_POSITIONINTEGRATION_H
