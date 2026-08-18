#pragma once

#include <cstdint>
#include <vector>

#include <gtsam/base/Vector.h>

#include "PlannerModel.h"
#include "WorldCartesianTrajectory.h"

// Projects every state of the final, validated and time-scaled GPMP2 joint
// trajectory through Pinocchio FK and J_W qdot. No decimation or independent
// SE(3) interpolation is performed. The terminal published reference is
// explicitly stationary and is the only arrival-eligible sample.
WorldCartesianTrajectory ProjectWorldTrajectory(
    const PlannerModel& model,
    const std::vector<gtsam::Vector>& position_rad,
    const std::vector<gtsam::Vector>& velocity_rad_s,
    double total_time_s,
    std::uint64_t trajectory_id,
    std::uint64_t planner_vicon_sequence);
