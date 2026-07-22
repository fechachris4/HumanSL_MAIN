//
// PositionIntegration: the implemented actuation — integrate the clamped q̇
// into a persistent position command streamed in the actuators' default
// POSITION control mode: docs/decisions/resolved-rate-position-integration.md.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_POSITIONINTEGRATION_H
#define HUMANSL_MASTERS_PROJECT_2025_POSITIONINTEGRATION_H

#include "actuation/Actuation.h"

class PositionIntegration : public Actuation
{
public:
    // q_command = q_measured — the ONLY time command state is seeded from
    // measurement (resolved-rate-position-integration.md, "state distinction").
    void Prepare(const RobotState& state) override;

    // q_command += q̇_clamped · dt; setpoints are q_command in degrees.
    void Apply(const Eigen::Matrix<double, 7, 1>& qdot_clamped_rad_s,
               double dt_s, JointVector& setpoints_deg,
               JointVector& setpoint_velocity_deg_s) override;

    // |q_command − q_measured| per joint, the measurement shifted by whole
    // turns to within ±180° of the command (same convention as FillSample).
    std::optional<JointVector> TrackingErrorDeg(const RobotState& state) const override;

    // No-op: in POSITION mode the arm holds the last commanded setpoint.
    void Restore() override;

private:
    Eigen::Matrix<double, 7, 1> q_command_rad_ =
        Eigen::Matrix<double, 7, 1>::Zero();
};

#endif // HUMANSL_MASTERS_PROJECT_2025_POSITIONINTEGRATION_H
