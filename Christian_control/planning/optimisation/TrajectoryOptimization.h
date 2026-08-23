#pragma once

#include <optional>

#include "utils.h"
#include <unordered_map>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Symbol.h> 

#include <gtsam/linear/NoiseModel.h>

#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPArm.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/obstacle/SelfCollisionArm.h>

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/JointLimitFactorVector.h>
#include <gpmp2/kinematics/VelocityLimitFactorVector.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePositionArm.h>

#include <gpmp2/planner/TrajUtils.h>

#include "JerkPenaltyFactor.h"
#include "OptimisationWaypoint.h"
#include "../src/MountSdf.h"

// THE one place gpmp2's workspace-pose error ordering is written down.
//
// gpmp2::GaussianPriorWorkspacePose's error is `des_pose.logmap(actual)`,
// which gtsam orders [rotation; translation] — the reverse of the
// [translation; rotation] most people assume. Getting it backwards is
// silent: the sigmas are still six plausible numbers and the solve still
// converges, just weighting the wrong axes. Every pose prior in this file
// is built here so the ordering exists once, and test_pose_noise_ordering
// perturbs one rotational and one translational axis independently to hold
// it honest.
gtsam::SharedNoiseModel PoseNoiseModel(const Eigen::Vector3d& rotation_sigma_rad,
                                       const Eigen::Vector3d& translation_sigma_m);

// The factor-graph weights optimizeJointTrajectory builds its graph from.
// These were function-local constants until planner_bridge's
// config/planner.yaml gained the ability to set them; the defaults below
// are those original values, so a caller that passes no tuning gets exactly
// the previous behaviour. planner_bridge's test_planner_config holds the
// checked-in YAML to these defaults, so file and code cannot drift apart.
//
// Every `sigma` is a gtsam noise-model sigma: SMALLER means a STIFFER
// constraint, which is the opposite of what "tolerance" would suggest.
struct OptimizerTuning {
    // Metres from an obstacle surface at which its cost switches on.
    double preferred_clearance_m = 0.10;
    // Base collision sigma retained in planner config; callers pass it
    // explicitly to the separate scene/self collision channels.
    double collision_sigma = 0.0005;
    // Scales the GP prior covariance Qc — the smoothness knob. Larger is
    // freer to deviate from constant velocity.
    double qc_scale = 1.0;
    // Levenberg-Marquardt iteration ceiling.
    int max_iterations = 1000;
};

class OptimizeTrajectory {
private:

public:

    OptimizeTrajectory();

    // tuning: the graph weights (see OptimizerTuning). Defaulted so the
    // other overloads below, and any caller that does not tune, are
    // unaffected.
    TrajectoryResult optimizeJointTrajectory(
        const gpmp2::ArmModel& arm_model,
        const std::vector<NamedObstacleField>& obstacle_fields,
        const gtsam::Values& init_values,
        const gtsam::Vector& start_config,
        const std::optional<gtsam::Vector>& start_vel,
        const std::optional<gtsam::Vector>& terminal_config,
        const JointLimits& pos_limits,
        const JointLimits& vel_limits,
        const size_t total_time_step,
        const double total_time_sec,
        const OptimizerTuning& tuning,
        const double scene_collision_sigma,
        const double self_collision_sigma,
        const double target_dt = 0.001
    );

    // Path following: one support state per entry of `waypoints`, each
    // either carrying a pose prior or free (OptimisationWaypoint.h).
    //
    // Replaces an earlier signature whose `target_pose_only` flag chose
    // between "constrain the last waypoint" and "constrain all of them",
    // with tube-task-specific y/z tolerances baked in. Neither expresses
    // what a traced path needs — a FREE approach phase followed by a
    // CONSTRAINED task phase — so the per-waypoint optional replaces both.
    // That old signature had no callers, so nothing had to be preserved.
    //
    // `waypoints.size()` must equal the number of support states in
    // `init_values` (x0..xN and v0..vN, so N+1 of each), and times must be
    // strictly increasing from zero.
    //
    // zero_velocity_indices: task-entry and endpoint support states pinned to
    // rest, so the arm comes to a stop before tracing rather than blending an
    // arbitrary approach direction into the path tangent. State zero is
    // governed by the measured start equalities instead.
    TrajectoryResult optimizeTaskTrajectory(
        const gpmp2::ArmModel& arm_model,
        const std::vector<NamedObstacleField>& obstacle_fields,
        const gtsam::Values& init_values,
        const std::vector<OptimisationWaypoint>& waypoints,
        const gtsam::Vector& start_config,
        const std::optional<gtsam::Vector>& start_vel,
        const std::optional<gtsam::Vector>& terminal_config,
        const std::vector<size_t>& zero_velocity_indices,
        const JointLimits& pos_limits,
        const JointLimits& vel_limits,
        const double total_time_sec,
        const OptimizerTuning& tuning,
        const double scene_collision_sigma,
        const double self_collision_sigma,
        const double target_dt = 0.001
    );

    std::pair<std::vector<gtsam::Vector>, std::vector<gtsam::Vector>> densifyTrajectory(
        const gtsam::Values& optimized_values,
        const gtsam::SharedNoiseModel& Qc_model,
        double delta_t,
        double total_time_sec,
        double target_dt);

};
