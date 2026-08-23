// WorldCartesianTrajectory — the typed planner/controller contract for a timed
// end-effector reference expressed entirely in Vicon world.
//
// Units: seconds, metres, unit quaternion x y z w, metres/second and
// radians/second. Validation is independent of GPMP2 and the robot runtime.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

struct WorldCartesianTrajectoryPoint {
    double t_from_start_s = 0.0;
    Eigen::Vector3d position_world_m = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_world = Eigen::Quaterniond::Identity();
    Eigen::Vector3d linear_velocity_world_m_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_world_rad_s = Eigen::Vector3d::Zero();
    bool arrival_eligible = false;
};

struct WorldCartesianTrajectory {
    std::uint64_t trajectory_id = 0;
    std::uint64_t planner_vicon_sequence = 0;
    std::vector<WorldCartesianTrajectoryPoint> points;

    // Controller-runtime reclamation link. It is not part of the trajectory
    // semantics; the cyclic consumer uses it to return ownership to the
    // non-real-time planner thread without allocating or freeing memory.
    WorldCartesianTrajectory* reclamation_next = nullptr;
};

// nullopt means valid. The final point is the only arrival-eligible point
// and must carry exactly a stationary reference (within numerical epsilon).
std::optional<std::string> ValidateWorldCartesianTrajectory(
    const WorldCartesianTrajectory& trajectory);
