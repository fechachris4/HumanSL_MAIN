// Offline preview-only text adapter for WorldCartesianTrajectory.
// Production controller code must use the typed contract directly.

#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "WorldCartesianTrajectory.h"

std::string FormatWorldCartesianTrajectoryBlock(
    const WorldCartesianTrajectory& trajectory);

class WorldCartesianTrajectoryAccumulator
{
public:
    std::optional<WorldCartesianTrajectory> Feed(const std::string& line,
                                                 std::string& error);
    bool Collecting() const noexcept { return collecting_; }

private:
    void Reset() noexcept;

    bool collecting_ = false;
    std::size_t expected_points_ = 0;
    WorldCartesianTrajectory pending_;
};
