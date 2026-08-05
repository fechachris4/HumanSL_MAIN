#pragma once
#include <array>
#include <optional>
#include <string>
#include <vector>
#include "PlannerModel.h"

// Controller-side acceptance rules: joints 2/4/6 within the controller's
// effective software stop, config::kJointSoftwareLimitDeg (Config.h:168 —
// upper Table-39 magnitude less a 2 deg margin, capped by the firmware
// warn threshold; NOT the wider warn limit alone); 1/3/5/7 continuous but
// still bounded at ±360 deg, matching the controller's own ingest gate
// (Targets.cpp's kContinuousJointBoundDeg) — a sanity check against the
// optimizer settling a solution a full revolution off, not a real joint
// limit. Returns an error description, or nullopt when every support
// state passes.
std::optional<std::string> ValidateJointPath(
    const std::vector<gtsam::Vector>& trajectory_pos);

// The joints-2/4/6 limits ValidateJointPath enforces, degrees, indexed
// like config::JointVector (index 1/3/5; the rest are the continuous-joint
// zero sentinel). Exposed read-only so tests can pin these literals against
// config::kJointSoftwareLimitDeg without bridge_core depending on
// basic_control's Config.h.
const std::array<double, 7>& ValidationLimitsDeg();
