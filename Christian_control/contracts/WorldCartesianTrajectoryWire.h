// Offline preview-only text adapter for WorldCartesianTrajectory.
// Production controller code must use the typed contract directly.

#pragma once

#include <string>

#include "WorldCartesianTrajectory.h"

std::string FormatWorldCartesianTrajectoryBlock(
    const WorldCartesianTrajectory& trajectory);
