#include "InProcessPlanner.h"

#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

namespace {
constexpr auto kPlannerPoll = std::chrono::milliseconds(20);
}

void RunInProcessPlanner(
    PlanningRequestSlot& requests,
    CartesianTrajectoryMailbox& trajectories,
    const PlannerRuntimeConfig& planner_config,
    const std::atomic<bool>& stop,
    PlannerSolveFunction solve)
{
    while (!stop.load(std::memory_order_relaxed)) {
        PlanningRequest request;
        if (!requests.TakeLatest(request)) {
            std::this_thread::sleep_for(kPlannerPoll);
            continue;
        }

        if (const std::optional<std::string> error =
                ValidatePlanningRequest(request)) {
            std::cerr << "planning request rejected: " << *error << "\n";
            continue;
        }

        std::ostringstream diagnostics;
        PlannerSolveResult result;
        try {
            result = solve(request, planner_config, diagnostics);
        } catch (const std::exception& error) {
            diagnostics << "planner worker exception: " << error.what() << "\n";
        } catch (...) {
            diagnostics << "planner worker exception: unknown exception\n";
        }
        if (!diagnostics.str().empty())
            std::cerr << diagnostics.str();

        if (stop.load(std::memory_order_relaxed))
            return;
        if (IsExecutable(result.status) && result.trajectory)
            trajectories.Publish(std::move(result.trajectory));
    }
}
