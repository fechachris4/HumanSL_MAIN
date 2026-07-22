//
// PositionIntegration: integrate clamped q̇ into a position command.
//

#include "actuation/PositionIntegration.h"

#include <cmath>
#include <tuple>

namespace
{
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;
    constexpr double kRadToDeg = 180.0 / M_PI;
} // namespace

void PositionIntegration::Prepare(const RobotState& state)
{
    q_command_rad_ = state.q_rad;
}

void PositionIntegration::Apply(const Eigen::Matrix<double, 7, 1>& qdot_clamped_rad_s,
                                double dt_s, JointVector& setpoints_deg,
                                JointVector& setpoint_velocity_deg_s)
{
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        q_command_rad_[i] += qdot_clamped_rad_s[i] * dt_s;
        setpoint_velocity_deg_s[i] = qdot_clamped_rad_s[i] * kRadToDeg;
        setpoints_deg[i] = q_command_rad_[i] * kRadToDeg;
    }
}

std::optional<JointVector> PositionIntegration::TrackingErrorDeg(const RobotState& state) const
{
    JointVector error_deg{};
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        const double command_deg = q_command_rad_[i] * kRadToDeg;
        const double measured_deg = state.q_rad[i] * kRadToDeg;
        error_deg[i] = std::abs(std::remainder(measured_deg - command_deg, 360.0));
    }
    return error_deg;
}

void PositionIntegration::Restore()
{
    // Nothing to undo: the position servo holds the last commanded setpoint.
}
