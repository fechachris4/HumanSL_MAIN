#pragma once
#include <array>
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include "PlannerModel.h"

// Controller-side acceptance rules: joints 2/4/6 within the controller's
// effective software stop, config::kJointSoftwareLimitDeg (Config.h:168 —
// upper Table-39 magnitude less a 2 deg margin, capped by the firmware
// warn threshold; NOT the wider warn limit alone); 1/3/5/7 continuous.
// Returns an error description, or nullopt when every support state
// passes.
std::optional<std::string> ValidateJointPath(
    const std::vector<gtsam::Vector>& trajectory_pos);

// The joints-2/4/6 limits ValidateJointPath enforces, degrees, indexed
// like config::JointVector (index 1/3/5; the rest are the continuous-joint
// zero sentinel). Exposed read-only so tests can pin these literals against
// config::kJointSoftwareLimitDeg without bridge_core depending on
// basic_control's Config.h.
const std::array<double, 7>& ValidationLimitsDeg();

// Cartesian tool positions of the support states, thinned to at most
// max_count points at least min_spacing_m apart. The final point is
// always included; the first (current position) is always dropped.
// When `rotations_xyzw` is non-null, it is cleared and filled with the
// tool orientation (ToolPoseInBaseLink(model, q).rotation(), as xyzw)
// at each kept support state, 1:1 with the returned waypoints — including
// entries evicted by the goal-proximity eviction below.
std::vector<Eigen::Vector3d> SampleCartesianWaypoints(
    const PlannerModel& model, const std::vector<gtsam::Vector>& trajectory_pos,
    std::size_t max_count = 8, double min_spacing_m = 0.05,
    std::vector<Eigen::Quaterniond>* rotations_xyzw = nullptr);

// "x y z" with 6 decimals — the exact grammar ParsePoseTarget accepts.
std::string FormatTargetLine(const Eigen::Vector3d& position_m);

// "x y z qx qy qz qw", 6 decimals, xyzw — the orientation-carrying form.
std::string FormatTargetLine(const Eigen::Vector3d& position_m,
                             const Eigen::Quaterniond& rotation_xyzw);
