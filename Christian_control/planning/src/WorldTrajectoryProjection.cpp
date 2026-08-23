#include "WorldTrajectoryProjection.h"

#include <cmath>
#include <optional>
#include <stdexcept>

#include "PinocchioKinematicsAdapter.h"

WorldCartesianTrajectory ProjectWorldTrajectory(
    const PlannerModel& model,
    const Eigen::Isometry3d& world_T_mount,
    const std::vector<gtsam::Vector>& position_rad,
    const std::vector<gtsam::Vector>& velocity_rad_s,
    double total_time_s,
    std::uint64_t trajectory_id,
    std::uint64_t planner_vicon_sequence)
{
    if (position_rad.size() < 2)
        throw std::invalid_argument("trajectory needs at least two states");
    if (position_rad.size() != velocity_rad_s.size())
        throw std::invalid_argument("position/velocity state counts differ");
    if (!std::isfinite(total_time_s) || total_time_s <= 0.0)
        throw std::invalid_argument(
            "trajectory duration must be finite and positive");
    for (std::size_t index = 0; index < position_rad.size(); ++index) {
        if (position_rad[index].size() != 7 ||
            velocity_rad_s[index].size() != 7)
            throw std::invalid_argument("state " + std::to_string(index) +
                                        " is not seven-dimensional");
        if (!position_rad[index].allFinite() ||
            !velocity_rad_s[index].allFinite())
            throw std::invalid_argument("state " + std::to_string(index) +
                                        " contains non-finite values");
    }

    if (!world_T_mount.matrix().allFinite())
        throw std::invalid_argument("world_T_mount must be finite");
    const Eigen::Isometry3d world_T_base =
        world_T_mount *
        pinocchio_kinematics_adapter::MountFromBase(model.left_arm);

    WorldCartesianTrajectory trajectory;
    trajectory.trajectory_id = trajectory_id;
    trajectory.planner_vicon_sequence = planner_vicon_sequence;
    trajectory.points.reserve(position_rad.size());
    const double dt_s =
        total_time_s / static_cast<double>(position_rad.size() - 1);

    for (std::size_t index = 0; index < position_rad.size(); ++index) {
        const Eigen::Matrix<double, 7, 1> q(position_rad[index]);
        const Eigen::Matrix<double, 7, 1> qdot(velocity_rad_s[index]);
        const auto base =
            pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
                q, model.end_effector_frame, model.left_arm);

        WorldCartesianTrajectoryPoint point;
        point.t_from_start_s = static_cast<double>(index) * dt_s;
        point.position_world_m = world_T_base * base.position;
        point.orientation_world =
            Eigen::Quaterniond(world_T_base.linear() * base.rotation).normalized();
        if (!trajectory.points.empty() &&
            trajectory.points.back().orientation_world.coeffs().dot(
                point.orientation_world.coeffs()) < 0.0)
            point.orientation_world.coeffs() *= -1.0;

        if (index + 1 < position_rad.size()) {
            point.linear_velocity_world_m_s =
                world_T_base.linear() * base.jacobian.topRows<3>() * qdot;
            point.angular_velocity_world_rad_s =
                world_T_base.linear() * base.jacobian.bottomRows<3>() * qdot;
        } else {
            point.linear_velocity_world_m_s.setZero();
            point.angular_velocity_world_rad_s.setZero();
            point.arrival_eligible = true;
        }
        trajectory.points.push_back(point);
    }

    if (const std::optional<std::string> error =
            ValidateWorldCartesianTrajectory(trajectory))
        throw std::invalid_argument("projected trajectory is invalid: " + *error);
    return trajectory;
}
