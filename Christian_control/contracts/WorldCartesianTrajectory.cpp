#include "WorldCartesianTrajectory.h"

#include <cmath>

namespace {
constexpr double kStartTimeToleranceS = 1e-12;
constexpr double kQuaternionNormTolerance = 1e-3;
constexpr double kTerminalTwistTolerance = 1e-12;

std::string PointName(std::size_t index)
{
    return "point " + std::to_string(index);
}
}  // namespace

std::optional<std::string> ValidateWorldCartesianTrajectory(
    const WorldCartesianTrajectory& trajectory)
{
    if (trajectory.points.size() < 2)
        return "trajectory needs at least 2 points";
    if (std::abs(trajectory.points.front().t_from_start_s) >
        kStartTimeToleranceS)
        return "first point must be at t_from_start_s == 0";

    for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
        const WorldCartesianTrajectoryPoint& point = trajectory.points[index];
        if (!std::isfinite(point.t_from_start_s) ||
            !point.position_world_m.allFinite() ||
            !point.orientation_world.coeffs().allFinite() ||
            !point.linear_velocity_world_m_s.allFinite() ||
            !point.angular_velocity_world_rad_s.allFinite())
            return PointName(index) + " has non-finite values";
        if (index > 0 &&
            point.t_from_start_s <=
                trajectory.points[index - 1].t_from_start_s)
            return PointName(index) + " time is not strictly increasing";
        if (std::abs(point.orientation_world.norm() - 1.0) >
            kQuaternionNormTolerance)
            return PointName(index) + " quaternion is not unit length";
        if (index + 1 < trajectory.points.size() && point.arrival_eligible)
            return PointName(index) + " is arrival eligible before the end";
    }

    const WorldCartesianTrajectoryPoint& last = trajectory.points.back();
    if (!last.arrival_eligible)
        return "final point must be arrival eligible";
    if (last.linear_velocity_world_m_s.cwiseAbs().maxCoeff() >
            kTerminalTwistTolerance ||
        last.angular_velocity_world_rad_s.cwiseAbs().maxCoeff() >
            kTerminalTwistTolerance)
        return "final point reference twist must be zero";
    return std::nullopt;
}
