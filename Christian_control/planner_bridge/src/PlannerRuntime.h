// Typed planner entry points shared by the standalone preview and the
// in-process controller worker. Planner work remains outside the 500 Hz loop.

#pragma once

#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "PlanningRequest.h"
#include "WorldCartesianTrajectory.h"

struct PlannerRuntimeConfig {
    std::string goal_file;
    std::string planner_config_file;
    std::string joint_limits_file;
    std::string right_dh_file;
    std::string left_dh_file;
    std::string runs_root;
};

struct PlannerSolveResult {
    int exit_code = 1;
    std::unique_ptr<WorldCartesianTrajectory> trajectory;
};

// Parses the planner command-line contract, solves and projects one plan, and
// returns the validated typed trajectory. No output serialization occurs here.
PlannerSolveResult SolveWorldTrajectory(
    const std::vector<std::string>& args,
    std::ostream& diagnostics);

// Builds the equivalent explicit planner arguments from one typed controller
// snapshot and returns the planner-owned result without a process boundary.
PlannerSolveResult SolveWorldTrajectoryForRequest(
    const PlanningRequest& request,
    const PlannerRuntimeConfig& config,
    std::ostream& diagnostics);
