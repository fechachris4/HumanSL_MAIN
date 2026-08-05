#pragma once
#include <string>
#include "PlannerModel.h"
#include "WorldSdf.h"

struct PlanRequest {
    Eigen::Matrix<double, 7, 1> q_start_rad;  // Kortex order
    Eigen::Vector3d goal_position_m;          // base_link
    std::optional<AxisAlignedBox> obstacle;
};

struct PlanOutcome {
    bool ok = false;
    std::string error;                 // set when !ok
    TrajectoryResult result;           // trajectory_pos: radians, Kortex order
    double final_goal_error_m = 0.0;   // FK(last waypoint) vs requested goal
    double total_time_sec = 0.0;       // planned duration the states span
};

// joint_limits_yaml: TrajectoryGeneration/config/joint_limits.yaml.
// Goal orientation = tool orientation at q_start (the controller is
// position-only and preserves takeover orientation; the pose prior is soft).
PlanOutcome SolveToPosition(const PlannerModel& model, const PlanRequest& request,
                            const std::string& joint_limits_yaml);
