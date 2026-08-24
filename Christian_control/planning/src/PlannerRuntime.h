// Typed planner entry points shared by the standalone preview and the
// in-process controller worker. Planner work remains outside the 500 Hz loop.

#pragma once

#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "PlanningRequest.h"
#include "PlanValidationReport.h"
#include "WorldCartesianTrajectory.h"

struct PlannerRuntimeConfig {
    std::string planner_config_file;
    std::string joint_limits_file;
    std::string right_dh_file;
    std::string left_dh_file;
    std::string runs_root;
    // Empty disables per-attempt artifacts. run_session.sh supplies its
    // session-local plans/ directory for live current-planner sessions.
    std::string planning_artifacts_root;
};

struct PlannerSolveResult {
    int exit_code = 1;
    PlanStatus status = PlanStatus::kFailed;
    std::string failure_reason;
    std::unique_ptr<WorldCartesianTrajectory> trajectory;
};

// Parses the planner command-line contract, solves one plan (mount-internal,
// see PathFrames.h) and projects it to the world-frame controller contract.
// No output serialization occurs here.
PlannerSolveResult SolvePlan(
    const std::vector<std::string>& args,
    std::ostream& diagnostics);

// Builds the equivalent explicit planner arguments from one typed controller
// snapshot and returns the planner-owned result without a process boundary.
PlannerSolveResult SolvePlanForRequest(
    const PlanningRequest& request,
    const PlannerRuntimeConfig& config,
    std::ostream& diagnostics);
