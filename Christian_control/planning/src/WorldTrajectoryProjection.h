#pragma once

#include <cstdint>
#include <vector>

#include <gtsam/base/Vector.h>

#include "PlannerModel.h"
#include "WorldCartesianTrajectory.h"

// THE planner's only mount->world conversion. Projects every state of the
// final, validated and time-scaled GPMP2 joint trajectory through Pinocchio
// FK and J qdot — evaluated in `mount`, the planner's internal frame — then
// carries each pose and twist into Vicon `world` with the immutable
// snapshot `world_T_mount`, because the controller's wire contract
// (WorldCartesianTrajectory) is world-frame. No decimation or independent
// SE(3) interpolation is performed. The terminal published reference is
// explicitly stationary and is the only arrival-eligible sample.
WorldCartesianTrajectory ProjectWorldTrajectory(
    const PlannerModel& model,
    const Eigen::Isometry3d& world_T_mount,
    const std::vector<gtsam::Vector>& position_rad,
    const std::vector<gtsam::Vector>& velocity_rad_s,
    double total_time_s,
    std::uint64_t trajectory_id,
    std::uint64_t planner_vicon_sequence);
