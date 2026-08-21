#pragma once
#include <optional>
#include <string>
#include <vector>
#include "PlannerModel.h"

// Controller-side acceptance: would the controller's software stop refuse
// any state of this plan?
//
// Both halves of that question come from one place, config::limits
// (generated from planning/config/joint_limits.yaml, which matches the
// URDF). Which joints are bounded is kBoundedMask; where their stops are
// is kControllerLowerDeg/kControllerUpperDeg, the physical limit less the
// controller margin. Nothing is restated here, so this check cannot drift
// from the limits the controller actually enforces — which is exactly what
// happened while it kept its own copy (2026-08-21).
//
// Continuous joints 1/3/5/7 have no mechanical stop and are NOT checked
// here at all — no absolute angular bound, and no wrap into a principal
// range either. Equivalent configurations differing by whole revolutions
// are the same configuration, and this function must not pick between
// them. Continuity across the path is enforced where it belongs: PathIk
// measures every step with std::remainder against the previous solved
// sample and reports maximum_joint_step_rad, and ValidatePlannedPath
// reports each joint's speed and acceleration against its own limit. An
// unnecessary full revolution therefore shows up as a velocity,
// acceleration or joint-step finding, never as a position-limit rejection.
//
// Returns an error description, or nullopt when every support state passes.
std::optional<std::string> ValidateJointPath(
    const std::vector<gtsam::Vector>& trajectory_pos);
