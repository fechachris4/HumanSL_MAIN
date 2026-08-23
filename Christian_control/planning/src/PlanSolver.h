#pragma once
#include <optional>
#include <string>
#include "PlannerConfig.h"
#include "PlannerModel.h"
#include "CartesianPath.h"
#include "PathIk.h"
#include "PlanValidationReport.h"
#include "ValidatePlan.h"

// The physical and effective operating limits the plan was actually solved
// against, carried with the outcome so artifacts state both bands.
struct PlanJointLimits {
    Eigen::Matrix<double, 7, 1> lower_rad = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> upper_rad = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> hardware_velocity_rad_s = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> effective_velocity_rad_s = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> hardware_acceleration_rad_s2 = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> effective_acceleration_rad_s2 = Eigen::Matrix<double, 7, 1>::Zero();
};

struct PlanRequest {
    Eigen::Matrix<double, 7, 1> q_start_rad;  // Kortex order
    std::optional<Eigen::Matrix<double, 7, 1>> qdot_start_rad_s;
    Eigen::Vector3d goal_position_m;          // metres, `mount`
    // Orientation the tool should hold AT the goal, in `mount` (declared
    // frames are converted at the boundary, PathFrames.h). Unset means "inherit whatever orientation the arm happens
    // to have at q_start", which is the historical behaviour and is a trap:
    // it makes a goal's feasibility depend on where the arm was parked
    // beforehand, which is neither requested nor controllable. Measured
    // 2026-08-06: the same left-arm goal position solved to 1.34 mm from one
    // start and was unreachable from another, purely because of the
    // orientation it inherited. Set it explicitly to make the request mean
    // the same thing every run.
    std::optional<Eigen::Matrix3d> goal_rotation;
};

struct PlanOutcome {
    PlanStatus status = PlanStatus::kFailed;
    std::string failure_reason;
    std::optional<TrajectoryResult> trajectory;
    PlanValidationReport validation;
    double final_goal_error_m = 0.0;   // FK(last waypoint) vs requested goal
    double total_time_sec = 0.0;       // planned duration the states span
    std::optional<TerminalIkCandidate> terminal_candidate;
    PlanJointLimits joint_limits;
};

// joint_limits_yaml: planner_bridge/config/joint_limits.yaml.
// config: the run's tuning, from planner_bridge/config/planner.yaml — plan
// pacing and every factor-graph weight (PlannerConfig.h). It is a parameter
// rather than part of PlanRequest because it is ambient policy for the run,
// not part of the question "where should the arm go".
// Goal orientation = tool orientation at q_start when the request omits one;
// the selected terminal IK candidate is enforced by an exact joint equality.
PlanOutcome SolveToPosition(const PlannerModel& model, const PlanRequest& request,
                            const std::string& joint_limits_yaml,
                            const PlannerConfig& config);

// ---------------------------------------------------------------
// Cartesian path following
// ---------------------------------------------------------------

// What a traced-path plan produced. Separate from PlanOutcome because a
// path plan answers a different question: not "how close to the goal" but
// "how faithfully was the shape traced", and by what margin it may be run.
struct PathPlanOutcome {
    PlanStatus status = PlanStatus::kFailed;
    std::string failure_reason;
    std::optional<TrajectoryResult> trajectory;
    PlanValidationReport validation;
    std::optional<TerminalIkCandidate> terminal_candidate;
    double total_time_sec = 0.0;
    int time_scaling_passes = 0;  // how many alpha iterations were needed
    bool time_scaling_settled = true;
    // Continuation-IK diagnostics for the traced path. Short unresolved gaps
    // use interpolated initial guesses; every path pose prior keeps its full
    // configured strength and the final validator judges the result.
    double maximum_joint_step_rad = 0.0;
    double closure_drift_rad = 0.0;
    std::size_t ik_unresolved_samples = 0;
    std::size_t ik_interpolated_samples = 0;
    // When the constrained task phase begins in trajectory time (the
    // approach before it is free), so a per-time figure can shade which
    // stretch of the trajectory the traced path actually governs.
    double task_start_time_s = 0.0;
    PlanJointLimits joint_limits;
    // The continuation walk in full, per sample. Kept because a FAILED walk
    // is exactly the case worth looking at, and the counters above cannot
    // say WHICH samples failed or where they sit on the path.
    PathIkResult ik_walk;
};

// Plans a trajectory that traces `task_path`, then densely validates the
// fresh optimizer result.
//
// If dynamic limits fail, a bounded duration re-solve rebuilds timed
// waypoints, initial values and the optimizer result from scratch, then
// validates that new result. The validated artifact is therefore the fresh
// solve that would be sent.
//
// Time scaling is applied ONLY for dynamic-limit failures. A geometric,
// collision or fidelity failure is not something slowing down can fix, and
// retrying it would just burn solves while the report said the same thing.
PathPlanOutcome SolveAlongPath(const PlannerModel& model,
                               const CartesianPath& task_path,
                               const Eigen::Matrix<double, 7, 1>& q_start_rad,
                               const std::optional<Eigen::Matrix<double, 7, 1>>&
                                   qdot_start_rad_s,
                               const std::string& joint_limits_yaml,
                               const PlannerConfig& config);
