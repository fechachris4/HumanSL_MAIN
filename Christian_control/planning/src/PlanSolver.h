#pragma once
#include <string>
#include "PlannerConfig.h"
#include "PlannerModel.h"
#include "CartesianPath.h"
#include "PathIk.h"
#include "PathValidationReport.h"
#include "TrajectoryInitiation.h"  // InitSource
#include "MountSdf.h"

// The operating limits the plan was actually solved against: the table
// values with the planner's margin already applied, not the raw numbers in
// joint_limits.yaml. Carried out with the outcome so a diagnostic dump can
// draw the joint trajectory against the band the solver really used, rather
// than re-deriving it from the file and getting the margin wrong.
struct PlanJointLimits {
    Eigen::Matrix<double, 7, 1> lower_rad = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> upper_rad = Eigen::Matrix<double, 7, 1>::Zero();
};

struct PlanRequest {
    Eigen::Matrix<double, 7, 1> q_start_rad;  // Kortex order
    Eigen::Vector3d goal_position_m;          // metres, `mount`
    std::optional<AxisAlignedBox> obstacle;
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
    bool ok = false;
    std::string error;                 // set when !ok
    TrajectoryResult result;           // trajectory_pos: radians, Kortex order
    double final_goal_error_m = 0.0;   // FK(last waypoint) vs requested goal
    double total_time_sec = 0.0;       // planned duration the states span
    // How the optimiser's starting sketch was built, and how far the IK
    // that built it landed from the requested pose. A plan is NOT refused
    // for a poor initialisation — the optimiser has its own goal term and
    // final_goal_error_m above says what it actually achieved — but the
    // operator is told, because a kNearMiss or kHeldStart plan deserves
    // more scrutiny than a kSolvedIk one. The two errors also say WHICH
    // half of the pose the IK could not reach: position or orientation.
    InitSource init_source = InitSource::kSolvedIk;
    double init_position_error_m = 0.0;
    double init_orientation_error_rad = 0.0;
    PlanJointLimits joint_limits;
};

// joint_limits_yaml: planner_bridge/config/joint_limits.yaml.
// config: the run's tuning, from planner_bridge/config/planner.yaml — plan
// pacing and every factor-graph weight (PlannerConfig.h). It is a parameter
// rather than part of PlanRequest because it is ambient policy for the run,
// not part of the question "where should the arm go".
// Goal orientation = tool orientation at q_start (the controller is
// position-only and preserves takeover orientation; the pose prior is soft).
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
    bool ok = false;
    std::string error;            // set when !ok
    TrajectoryResult result;      // FINAL dense q/qdot, already time-scaled
    PathValidationReport report;  // measured on the final dense internal path
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

// Plans a trajectory that traces `task_path`, then emits, reconstructs and
// validates it.
//
// The loop is: solve -> emit -> reconstruct -> validate. If the DYNAMIC
// limits fail, the trajectory is uniformly time-scaled and re-emitted, and
// the whole measurement repeats on the new block, so the validated artefact
// is always the one that would be sent. Bounded to a few passes.
//
// Time scaling is applied ONLY for dynamic-limit failures. A geometric,
// collision or fidelity failure is not something slowing down can fix, and
// retrying it would just burn solves while the report said the same thing.
PathPlanOutcome SolveAlongPath(const PlannerModel& model,
                               const CartesianPath& task_path,
                               const Eigen::Matrix<double, 7, 1>& q_start_rad,
                               const std::optional<AxisAlignedBox>& obstacle,
                               const std::string& joint_limits_yaml,
                               const PlannerConfig& config,
                               const ValidationInputs& validation_template);
