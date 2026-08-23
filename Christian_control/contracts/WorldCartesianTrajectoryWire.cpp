#include "WorldCartesianTrajectoryWire.h"

#include <iomanip>
#include <sstream>
#include <stdexcept>

std::string FormatWorldCartesianTrajectoryBlock(
    const WorldCartesianTrajectory& trajectory)
{
    if (const std::optional<std::string> error =
            ValidateWorldCartesianTrajectory(trajectory))
        throw std::invalid_argument(*error);

    std::ostringstream block;
    block << std::setprecision(17);
    block << "CART_TRAJ_BEGIN 1 " << trajectory.trajectory_id << " "
          << trajectory.planner_vicon_sequence << " WORLD "
          << trajectory.points.size() << "\n";
    for (const WorldCartesianTrajectoryPoint& point : trajectory.points) {
        block << point.t_from_start_s;
        for (const double value : point.position_world_m)
            block << " " << value;
        block << " " << point.orientation_world.x()
              << " " << point.orientation_world.y()
              << " " << point.orientation_world.z()
              << " " << point.orientation_world.w();
        for (const double value : point.linear_velocity_world_m_s)
            block << " " << value;
        for (const double value : point.angular_velocity_world_rad_s)
            block << " " << value;
        block << " " << (point.arrival_eligible ? 1 : 0) << "\n";
    }
    block << "CART_TRAJ_END\n";
    return block.str();
}
