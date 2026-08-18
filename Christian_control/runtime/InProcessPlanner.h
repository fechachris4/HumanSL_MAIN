// Concrete non-real-time planner worker for the controller process.

#pragma once

#include <atomic>
#include <iosfwd>

#include "CartesianTrajectoryMailbox.h"
#include "PlanningRequestSlot.h"
#include "PlannerRuntime.h"

using PlannerSolveFunction = PlannerSolveResult (*) (
    const PlanningRequest&, const PlannerRuntimeConfig&, std::ostream&);

void RunInProcessPlanner(
    PlanningRequestSlot& requests,
    CartesianTrajectoryMailbox& trajectories,
    const PlannerRuntimeConfig& planner_config,
    const std::atomic<bool>& stop,
    PlannerSolveFunction solve);
