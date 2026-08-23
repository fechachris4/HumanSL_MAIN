#include "PlanSolver.h"
#include <algorithm>
#include <cmath>

#include "GenerateTrajectory.h"
#include "MountSdf.h"
#include "PathAssembly.h"
#include "PathIk.h"
#include "TrajectoryOptimization.h"
#include "ValidatePlan.h"
#include "TrajectoryInitiation.h"

PlanOutcome SolveToPosition(const PlannerModel& model, const PlanRequest& request,
                            const std::string& joint_limits_yaml,
                            const PlannerConfig& config)
{
    PlanOutcome outcome;
    try
    {
        const PlannerJointLimits limits = createJointLimits(joint_limits_yaml);
        const JointLimits& pos_limits = limits.position_rad;
        const JointLimits& vel_limits = limits.effective_velocity_rad_s;
        for (int joint = 0; joint < 7; ++joint) {
            outcome.joint_limits.lower_rad(joint) = pos_limits.lower(joint);
            outcome.joint_limits.upper_rad(joint) = pos_limits.upper(joint);
            outcome.joint_limits.hardware_velocity_rad_s(joint) = limits.hardware_velocity_rad_s.upper(joint);
            outcome.joint_limits.effective_velocity_rad_s(joint) = vel_limits.upper(joint);
            outcome.joint_limits.hardware_acceleration_rad_s2(joint) = limits.hardware_acceleration_rad_s2.upper(joint);
            outcome.joint_limits.effective_acceleration_rad_s2(joint) = limits.effective_acceleration_rad_s2.upper(joint);
        }
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
        const auto obstacle_fields = MakeNamedObstacleFields(
            MountGridGeometry(), model, config.scene);
        std::optional<gtsam::Vector> start_vel;
        if (request.qdot_start_rad_s)
            start_vel = gtsam::Vector(*request.qdot_start_rad_s);
        TrajectoryResult solved = optimizeJointTrajectory(
            *model.arm_model, obstacle_fields, init.values, goal_pose,
            gtsam::Vector(request.q_start_rad), start_vel,
            pos_limits, vel_limits,
            total_time_step, total_time_sec, config.optimizer);
        if (solved.trajectory_pos.empty())
        {
            outcome.failure_reason = "optimizer returned an empty trajectory";
            return outcome;
        }
        PlanValidationInputs inputs;
        inputs.measured_q_rad = request.q_start_rad;
        inputs.measured_qdot_rad_s = request.qdot_start_rad_s;
        inputs.position_lower_rad = limits.position_rad.lower;
        inputs.position_upper_rad = limits.position_rad.upper;
        inputs.effective_velocity_rad_s = limits.effective_velocity_rad_s.upper;
        inputs.effective_acceleration_rad_s2 = limits.effective_acceleration_rad_s2.upper;
        inputs.obstacle_fields = &obstacle_fields;
        inputs.minimum_clearance_m = config.minimum_clearance_m;
        inputs.requested_terminal_mount = goal_pose;
        inputs.candidate_terminal_mount = goal_pose;
        inputs.intended_status = PlanStatus::kReached;
        inputs.validation_dt_s = config.path_following.validation_dt_s;
        outcome.validation = ValidatePlan(model, solved, total_time_sec, inputs);
        outcome.final_goal_error_m =
            outcome.validation.requested_terminal_position_error_m;
        if (outcome.validation.executable) {
            outcome.status = PlanStatus::kReached;
            outcome.trajectory = std::move(solved);
        } else {
            outcome.failure_reason = outcome.validation.failure_reason;
        }
    }
    catch (const std::exception& exception)
    {
        outcome.failure_reason = exception.what();
    }
    return outcome;
}

// ---------------------------------------------------------------
// Cartesian path following
// ---------------------------------------------------------------

PathPlanOutcome SolveAlongPath(const PlannerModel& model,
                               const CartesianPath& task_path,
                               const Eigen::Matrix<double, 7, 1>& q_start_rad,
                               const std::optional<Eigen::Matrix<double, 7, 1>>&
                                   qdot_start_rad_s,
                               const std::string& joint_limits_yaml,
                               const PlannerConfig& config) {
    PathPlanOutcome outcome;
    try {
        const PlannerJointLimits limits = createJointLimits(joint_limits_yaml);
        const JointLimits& pos_limits = limits.position_rad;
        const JointLimits& vel_limits = limits.effective_velocity_rad_s;
        for (int joint = 0; joint < 7; ++joint) {
            outcome.joint_limits.lower_rad(joint) = pos_limits.lower(joint);
            outcome.joint_limits.upper_rad(joint) = pos_limits.upper(joint);
            outcome.joint_limits.hardware_velocity_rad_s(joint) = limits.hardware_velocity_rad_s.upper(joint);
            outcome.joint_limits.effective_velocity_rad_s(joint) = vel_limits.upper(joint);
            outcome.joint_limits.hardware_acceleration_rad_s2(joint) = limits.hardware_acceleration_rad_s2.upper(joint);
            outcome.joint_limits.effective_acceleration_rad_s2(joint) = limits.effective_acceleration_rad_s2.upper(joint);
        }

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

        PathIkJointLimits ik_limits;
        for (int joint = 0; joint < 7; ++joint) {
            ik_limits.lower_rad(joint) = pos_limits.lower(joint);
            ik_limits.upper_rad(joint) = pos_limits.upper(joint);
        }
        const PathIkResult walk =
            SolvePathIk(task_path, arm, q_start_rad, ik_limits, tolerance,
                        /*closed=*/true, config.effective_ik_seed);
        outcome.ik_walk = walk;
        outcome.maximum_joint_step_rad = walk.maximum_joint_step_rad;
        outcome.closure_drift_rad = walk.closure_drift_rad;
        outcome.ik_unresolved_samples = walk.unresolved_samples;
        outcome.ik_interpolated_samples = walk.interpolated_samples;
        if (!walk.success) {
            // The one initialization failure left: the path's entry pose.
            // Failed intermediate anchors are dropped and interpolated over;
            // nothing can interpolate toward an unknown entry.
            outcome.failure_reason =
                "path IK initialization failed: the path entry pose could "
                "not be solved from the measured start configuration within "
                "the bounded search";
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

        const std::size_t states = assembled.waypoints.size();
        const GridGeometry grid = MountGridGeometry();
        const auto obstacle_fields = MakeNamedObstacleFields(
            grid, model, config.scene);
        std::optional<gtsam::Vector> start_vel;
        if (qdot_start_rad_s)
            start_vel = gtsam::Vector(*qdot_start_rad_s);
        // ---- 3. solve and validate, rebuilding every duration attempt ---
        // Time scaling changes qdot and timestamps, so the dense view is
        // rebuilt and re-measured on every pass. No wire decimation sits
        // between the final GPMP2 artefact and this validation.
        constexpr int kMaxScalingPasses = 3;
        const double base_duration_s = assembled.total_duration_s;
        // Where the traced phase begins, as a FRACTION of the base trajectory.
        // Each longer-duration attempt rescales waypoint times and validates
        // the newly solved trajectory against the corresponding task window.
        const double task_start_fraction =
            assembled.waypoints[assembled.task_start_index].time_s /
            assembled.total_duration_s;

        double duration_s = base_duration_s;
        for (int pass = 0; pass < kMaxScalingPasses; ++pass) {
            outcome.time_scaling_passes = pass + 1;
            const double task_start_time_s = duration_s * task_start_fraction;
            outcome.task_start_time_s = task_start_time_s;

            const double time_scale = duration_s / base_duration_s;
            std::vector<OptimisationWaypoint> attempt_waypoints =
                assembled.waypoints;
            for (OptimisationWaypoint& waypoint : attempt_waypoints)
                waypoint.time_s *= time_scale;
            gtsam::Values init_values;
            for (std::size_t i = 0; i < states; ++i) {
                init_values.insert(
                    gtsam::Symbol('x', i),
                    gtsam::Vector(assembled.initial_configurations[i]));
                gtsam::Vector velocity = gtsam::Vector::Zero(7);
                if (i + 1 < states) {
                    const double dt = attempt_waypoints[i + 1].time_s -
                                      attempt_waypoints[i].time_s;
                    if (dt > 0.0)
                        velocity = gtsam::Vector(
                            (assembled.initial_configurations[i + 1] -
                             assembled.initial_configurations[i]) /
                            dt);
                }
                init_values.insert(gtsam::Symbol('v', i), velocity);
            }
            OptimizeTrajectory optimizer;
            TrajectoryResult solved = optimizer.optimizeTaskTrajectory(
                *model.arm_model, obstacle_fields, init_values, attempt_waypoints,
                gtsam::Vector(q_start_rad), start_vel,
                assembled.zero_velocity_indices, pos_limits, vel_limits,
                duration_s, config.optimizer);
            if (solved.trajectory_pos.empty()) {
                outcome.failure_reason = "optimizer returned an empty trajectory";
                return outcome;
            }

            PlanValidationInputs plan_inputs;
            plan_inputs.measured_q_rad = q_start_rad;
            plan_inputs.measured_qdot_rad_s = qdot_start_rad_s;
            plan_inputs.position_lower_rad = limits.position_rad.lower;
            plan_inputs.position_upper_rad = limits.position_rad.upper;
            plan_inputs.effective_velocity_rad_s = limits.effective_velocity_rad_s.upper;
            plan_inputs.effective_acceleration_rad_s2 = limits.effective_acceleration_rad_s2.upper;
            plan_inputs.obstacle_fields = &obstacle_fields;
            plan_inputs.minimum_clearance_m = config.minimum_clearance_m;
            plan_inputs.requested_terminal_mount =
                gtsam::Pose3(gtsam::Rot3(task_path.samples.back().pose.linear()),
                             gtsam::Point3(task_path.samples.back().pose.translation()));
            plan_inputs.candidate_terminal_mount = plan_inputs.requested_terminal_mount;
            plan_inputs.intended_status = PlanStatus::kReached;
            plan_inputs.desired_task_path = &task_path;
            plan_inputs.task_start_time_s = task_start_time_s;
            plan_inputs.validation_dt_s = config.path_following.validation_dt_s;
            const PlanValidationReport report =
                ValidatePlan(model, solved, duration_s, plan_inputs);

            outcome.validation = report;
            outcome.total_time_sec = duration_s;

            // Fully inside both dynamic limits: nothing left to repair.
            // (An unset velocity limit reads as an infinite ratio and an
            // unconfigured acceleration table as 0, so this keeps the old
            // pass/skip semantics.)
            if (report.disposition == CandidateDisposition::kExecutable) {
                outcome.status = PlanStatus::kReached;
                outcome.trajectory = std::move(solved);
                break;
            }
            if (report.disposition != CandidateDisposition::kNeedsLongerDuration) {
                outcome.failure_reason = report.failure_reason;
                break;
            }

            // ONLY dynamic failures are retried. Slowing down cannot fix a
            // trajectory that traces the wrong shape or clips an obstacle,
            // and pretending otherwise would burn solves while the report
            // said the same thing each time.
            const double alpha = std::max(report.max_velocity_ratio,
                                         std::sqrt(report.max_acceleration_ratio));
            if (!(alpha > 1.0)) break;  // limits failed for another reason
            if (pass + 1 == kMaxScalingPasses) {
                outcome.time_scaling_settled = false;
                outcome.failure_reason = report.failure_reason;
                break;
            }
            // Select a longer duration; the next loop iteration rebuilds the
            // timed waypoints, initial values and optimizer result from scratch.
            duration_s *= alpha;
        }

        if (outcome.status == PlanStatus::kFailed && outcome.failure_reason.empty())
            outcome.failure_reason = "no executable traced trajectory";
    } catch (const std::exception& exception) {
        outcome.failure_reason = exception.what();
    }
    return outcome;
}
