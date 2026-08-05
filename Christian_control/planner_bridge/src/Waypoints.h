#pragma once
#include <optional>
#include <string>
#include <vector>
#include "PlannerModel.h"

// Controller-side acceptance rules (Config.h:160 kJointLimitWarnDeg):
// joints 2/4/6 within ±130/145/118 deg; 1/3/5/7 continuous. Returns an
// error description, or nullopt when every support state passes.
std::optional<std::string> ValidateJointPath(
    const std::vector<gtsam::Vector>& trajectory_pos);

// Cartesian tool positions of the support states, thinned to at most
// max_count points at least min_spacing_m apart. The final point is
// always included; the first (current position) is always dropped.
std::vector<Eigen::Vector3d> SampleCartesianWaypoints(
    const PlannerModel& model, const std::vector<gtsam::Vector>& trajectory_pos,
    std::size_t max_count = 8, double min_spacing_m = 0.05);

// "x y z" with 6 decimals — the exact grammar ParsePoseTarget accepts.
std::string FormatTargetLine(const Eigen::Vector3d& position_m);
