// Concrete non-real-time planner worker for the controller process.

#pragma once

#include <atomic>
#include "CartesianTrajectoryMailbox.h"
#include "PlanningRequestSlot.h"
#include "PlannerRuntime.h"

void RunInProcessPlanner(
    PlanningRequestSlot& requests,
    CartesianTrajectoryMailbox& trajectories,
    const PlannerRuntimeConfig& planner_config,
    const std::atomic<bool>& stop);
