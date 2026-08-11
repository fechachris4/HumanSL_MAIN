#include "PlanSolver.h"
#include <algorithm>
#include <cmath>
#include <sstream>

#include "GenerateTrajectory.h"
#include "PathAssembly.h"
#include "PathIk.h"
#include "ReconstructBlock.h"
#include "TrajectoryEmit.h"
#include "TrajectoryOptimization.h"
#include "ValidatePath.h"
#include "TrajectoryInitiation.h"

PlanOutcome SolveToPosition(const PlannerModel& model, const PlanRequest& request,
                            const std::string& joint_limits_yaml,
                            const PlannerConfig& config)
{
    PlanOutcome outcome;
    try
    {
        const auto [pos_limits, vel_limits] = createJointLimits(joint_limits_yaml);
        const gtsam::Pose3 start_pose = ToolPoseInMount(model, request.q_start_rad);
        // An explicitly requested orientation wins; otherwise inherit the
        // start pose's, as this has always done (PlanRequest::goal_rotation
        // explains why inheriting is a trap and BridgeMain says so on every
        // run that relies on it).
        const gtsam::Rot3 goal_rotation =
            request.goal_rotation ? gtsam::Rot3(*request.goal_rotation)
                                  : start_pose.rotation();
        const gtsam::Pose3 goal_pose(goal_rotation,
                                     gtsam::Point3(request.goal_position_m));
        // Pacing: the goal distance at the configured speed, never shorter
        // than the configured floor. The emitted block carries these times
        // and the controller follows them, so this is what sets how fast
        // the arm actually moves (config/planner.yaml, motion.*).
        const double distance_m =
            (request.goal_position_m - start_pose.translation()).norm();
        const size_t total_time_step = static_cast<size_t>(config.motion.waypoints);
        const double total_time_sec = std::max(
            config.motion.min_duration_s, distance_m / config.motion.nominal_speed_mps);
        outcome.total_time_sec = total_time_sec;

        // The initializer's IK must target the same arm/frame this model's
        // DH table was generated for — defaulting it to the right arm here
        // is what produced the constant 120 mm (tool-vs-flange) goal error
        // on the first left-arm runs (2026-08-06).
        InitializeTrajectory initializer(model.dh, model.end_effector_frame,
                                         model.left_arm);
        const TrajectoryInit init = initializer.initJointTrajectoryFromTarget(
            gtsam::Vector(request.q_start_rad), goal_pose, model.base_pose,
            total_time_step);
        outcome.init_source = init.source;
        outcome.init_position_error_m = init.position_error_m;
        outcome.init_orientation_error_rad = init.orientation_error_rad;
        const auto sdf = MakeWorldSdf(request.obstacle);
        outcome.result = optimizeJointTrajectory(
            *model.arm_model, sdf, init.values, goal_pose,
            gtsam::Vector(request.q_start_rad), pos_limits, vel_limits,
            total_time_step, total_time_sec, config.optimizer);
        if (outcome.result.trajectory_pos.empty())
        {
            outcome.error = "optimizer returned an empty trajectory";
            return outcome;
        }
        Eigen::Matrix<double, 7, 1> q_final(outcome.result.trajectory_pos.back());
        outcome.final_goal_error_m =
            (ToolPositionInMount(model, q_final) - request.goal_position_m).norm();
        outcome.ok = true;
    }
    catch (const std::exception& exception)
    {
        outcome.error = exception.what();
    }
    return outcome;
}

// ---------------------------------------------------------------
// Cartesian path following
// ---------------------------------------------------------------

namespace {

// Uniform time scaling: the factor by which the trajectory must be slowed
// so no joint exceeds its velocity or acceleration limit. Geometry is
// untouched — the arm follows the same joint path, just later.
//
//   alpha = max(1, max|qdot|/qdot_max, sqrt(max|qddot|/qddot_max))
//
// Velocity scales as 1/alpha and acceleration as 1/alpha^2, which is why
// the acceleration term takes a square root: slowing by alpha reduces
// acceleration by alpha squared.
double TimeScalingFactor(const PathValidationReport& report,
                         double velocity_limit_rad_s,
                         double acceleration_limit_rad_s2) {
    double alpha = 1.0;
    if (velocity_limit_rad_s > 0.0)
        alpha = std::max(alpha, report.max_joint_velocity_rad_s / velocity_limit_rad_s);
    if (acceleration_limit_rad_s2 > 0.0)
        alpha = std::max(alpha, std::sqrt(report.max_joint_acceleration_rad_s2 /
                                          acceleration_limit_rad_s2));
    return alpha;
}

std::string DescribeSdf(const std::optional<AxisAlignedBox>& obstacle) {
    std::ostringstream text;
    const GridBounds bounds = WorldGridBounds();
    text << "arm-workspace grid x [" << bounds.min_m.x() << ", " << bounds.max_m.x()
         << "] y [" << bounds.min_m.y() << ", " << bounds.max_m.y() << "] z ["
         << bounds.min_m.z() << ", " << bounds.max_m.z() << "] m; "
         << (obstacle ? "1 operator box" : "no obstacles")
         << "; NOT modelled: the wearer, the torso, the other arm";
    return text.str();
}

}  // namespace

PathPlanOutcome SolveAlongPath(const PlannerModel& model,
                               const CartesianPath& task_path,
                               const Eigen::Matrix<double, 7, 1>& q_start_rad,
                               const std::optional<AxisAlignedBox>& obstacle,
                               const std::string& joint_limits_yaml,
                               const PlannerConfig& config,
                               const ValidationInputs& validation_template) {
    PathPlanOutcome outcome;
    try {
        const auto [pos_limits, vel_limits] = createJointLimits(joint_limits_yaml);

        // ---- 1. continuation IK along the requested path ---------------
        // Converge far tighter than the pass/fail requirement so the SEED
        // is not itself the limiting error: the solver's own default stops
        // at 20 mm, which for a traced path measures where it gave up
        // rather than where the arm can reach (measured 2026-08-07).
        PathIkArm arm;
        arm.base_transform = model.base_pose.matrix();
        arm.end_effector_frame = model.end_effector_frame;
        arm.left_arm = model.left_arm;

        analytical_ik::IKTolerance tolerance;
        const double target_m = config.path_following.maximum_planning_error_m;
        tolerance.converge_position_m = target_m * 0.1;
        tolerance.converge_orientation_rad =
            config.path_following.maximum_orientation_error_rad * 0.1;
        tolerance.accept_position_m = target_m;
        tolerance.accept_orientation_rad =
            config.path_following.maximum_orientation_error_rad;

        // The run's recorded seed. Without this the walk would still draw
        // from IKSeeding's compiled default and planner.yaml's seed would
        // silently do nothing — the exact "I changed it and nothing
        // happened" failure the strict config exists to prevent.
        analytical_ik::IKSeeding ik_seeding;
        ik_seeding.seed = config.effective_ik_seed;
        const PathIkResult walk = SolvePathIk(task_path, arm, q_start_rad, tolerance,
                                              /*closed=*/true, /*attempts=*/40,
                                              ik_seeding);
        outcome.maximum_joint_step_rad = walk.maximum_joint_step_rad;
        outcome.closure_drift_rad = walk.closure_drift_rad;
        if (!walk.success) {
            outcome.error =
                "the requested path is not reachable: IK failed at sample " +
                std::to_string(walk.failed_sample.value_or(0)) + " of " +
                std::to_string(task_path.samples.size()) +
                " (run probe_path_reachability for the per-sample report)";
            return outcome;
        }

        std::vector<Eigen::Matrix<double, 7, 1>> task_configurations;
        task_configurations.reserve(walk.samples.size());
        for (const PathIkSample& sample : walk.samples)
            task_configurations.push_back(sample.configuration);

        // ---- 2. assemble the free approach + constrained task path ------
        ApproachPacing pacing;
        pacing.velocity_fraction = config.path_following.approach_velocity_fraction;
        pacing.minimum_duration_s = config.path_following.approach_min_duration_s;
        pacing.waypoints = config.path_following.approach_waypoints;

        JointVelocityLimits joint_velocity_limits;
        for (int j = 0; j < 7; ++j) joint_velocity_limits(j) = vel_limits.upper(j);

        const AssembledPath assembled = AssembleCirclePlan(
            task_path, task_configurations, q_start_rad, joint_velocity_limits,
            pacing,
            Eigen::Vector3d::Constant(config.path_following.rotation_prior_sigma_rad),
            Eigen::Vector3d::Constant(config.path_following.position_prior_sigma_m));

        gtsam::Values init_values;
        const std::size_t states = assembled.waypoints.size();
        for (std::size_t i = 0; i < states; ++i) {
            init_values.insert(gtsam::Symbol('x', i),
                               gtsam::Vector(assembled.initial_configurations[i]));
            // Average velocity across the step, so the initial guess is at
            // least self-consistent in position and velocity.
            gtsam::Vector velocity = gtsam::Vector::Zero(7);
            if (i + 1 < states) {
                const double dt = assembled.waypoints[i + 1].time_s -
                                  assembled.waypoints[i].time_s;
                if (dt > 0.0)
                    velocity = gtsam::Vector((assembled.initial_configurations[i + 1] -
                                              assembled.initial_configurations[i]) / dt);
            }
            init_values.insert(gtsam::Symbol('v', i), velocity);
        }

        const auto sdf = MakeWorldSdf(obstacle);
        const std::string sdf_contents = DescribeSdf(obstacle);

        // ---- 3. solve -------------------------------------------------
        OptimizeTrajectory optimizer;
        double duration_s = assembled.total_duration_s;
        TrajectoryResult solved = optimizer.optimizeTaskTrajectory(
            *model.arm_model, sdf, init_values, assembled.waypoints,
            gtsam::Vector(q_start_rad), assembled.zero_velocity_indices,
            pos_limits, vel_limits, duration_s, config.optimizer);
        if (solved.trajectory_pos.empty()) {
            outcome.error = "optimizer returned an empty trajectory";
            return outcome;
        }

        // ---- 4. emit -> reconstruct -> validate, scaling if needed -----
        // Time scaling changes the emitted block, so the block is rebuilt
        // and re-measured on every pass: what is validated is always what
        // would be sent, never a proxy for it.
        constexpr int kMaxScalingPasses = 3;
        ValidationInputs inputs = validation_template;
        inputs.desired_task_path = &task_path;
        inputs.measured_start = q_start_rad;
        inputs.validation_dt_s = config.path_following.validation_dt_s;
        inputs.maximum_planning_error_m = config.path_following.maximum_planning_error_m;
        inputs.maximum_orientation_error_rad =
            config.path_following.maximum_orientation_error_rad;
        for (int j = 0; j < 7; ++j) {
            inputs.joint_velocity_limits_rad_s(j) = vel_limits.upper(j);
            // No acceleration table exists in joint_limits.yaml; derive a
            // conservative bound from velocity (reach the limit in ~0.5 s)
            // rather than skipping the check entirely.
            inputs.joint_acceleration_limits_rad_s2(j) = vel_limits.upper(j) * 2.0;
        }

        // Where the traced phase begins, as a FRACTION of the trajectory.
        // Uniform time scaling stretches everything equally, so the
        // fraction is invariant while the absolute time is not — computing
        // it once and multiplying is correct on every pass, whereas
        // recomputing an absolute time against a stale duration is not.
        const double task_start_fraction =
            assembled.waypoints[assembled.task_start_index].time_s /
            assembled.total_duration_s;

        for (int pass = 0; pass < kMaxScalingPasses; ++pass) {
            outcome.time_scaling_passes = pass + 1;
            inputs.task_start_time_s = duration_s * task_start_fraction;

            const std::string block = FormatTrajectoryBlock(
                solved.trajectory_pos, solved.trajectory_vel, duration_s);
            const ReconstructedTrajectory reconstructed =
                ReconstructEmittedBlock(block, inputs.validation_dt_s);
            if (!reconstructed.parsed) {
                outcome.error = "emitted block did not parse: " + reconstructed.error;
                return outcome;
            }
            const auto sample_at = MakeReconstructionSampler(block);
            const PathValidationReport report = ValidatePlannedPath(
                model, reconstructed, solved.trajectory_pos, duration_s, sample_at,
                sdf, sdf_contents, inputs, /*optimiser_converged=*/true);

            outcome.emitted_block = block;
            outcome.report = report;
            outcome.total_time_sec = duration_s;

            if (report.dynamic_limits_valid) break;

            // ONLY dynamic failures are retried. Slowing down cannot fix a
            // trajectory that traces the wrong shape or clips an obstacle,
            // and pretending otherwise would burn solves while the report
            // said the same thing each time.
            const double alpha = TimeScalingFactor(
                report, inputs.joint_velocity_limits_rad_s.minCoeff(),
                inputs.joint_acceleration_limits_rad_s2.minCoeff());
            if (!(alpha > 1.0)) break;  // limits failed for another reason
            if (pass + 1 == kMaxScalingPasses) {
                outcome.time_scaling_settled = false;
                break;
            }
            // Uniform scaling: same joint path, later. Velocities scale by
            // 1/alpha because the same displacement now takes alpha times
            // as long.
            duration_s *= alpha;
            for (auto& velocity : solved.trajectory_vel) velocity /= alpha;
        }

        outcome.ok = true;
    } catch (const std::exception& exception) {
        outcome.error = exception.what();
    }
    return outcome;
}
