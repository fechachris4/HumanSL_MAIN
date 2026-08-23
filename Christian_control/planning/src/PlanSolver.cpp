#include "PlanSolver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#include "MountSdf.h"
#include "PathAssembly.h"
#include "PathIk.h"
#include "TrajectoryOptimization.h"
#include "ValidatePlan.h"

namespace
{

    constexpr int kMaximumDurationAttempts = 3;

    struct RouteSolution {
        TrajectoryResult trajectory;
        PlanValidationReport validation;
        TerminalIkCandidate terminal;
        PathIkResult ik_walk;
        bool has_ik_walk = false;
        RouteHypothesis route = RouteHypothesis::kNormal;
        double duration_s = 0.0;
        double task_start_time_s = 0.0;
        std::size_t evidence_index = 0;
    };

    struct DurationSearchResult {
        std::optional<RouteSolution> executable;
        PlanValidationReport last_validation;
        double last_duration_s = 0.0;
    };

    template <typename Solve, typename Validate>
    DurationSearchResult
    SolveDurationCandidate(const TerminalIkCandidate& terminal, std::size_t terminal_branch,
                           RouteHypothesis route, double base_duration_s, double scene_sigma,
                           Solve solve, Validate validate, std::vector<CandidateEvidence>& attempts,
                           std::string& failure_reason)
    {
        DurationSearchResult result;
        double duration_s = base_duration_s;
        for (int attempt = 1; attempt <= kMaximumDurationAttempts; ++attempt) {
            result.last_duration_s = duration_s;
            const auto solve_start = std::chrono::steady_clock::now();
            TrajectoryResult trajectory;
            try {
                trajectory = solve(duration_s);
            } catch (const std::exception& exception) {
                CandidateEvidence evidence;
                evidence.terminal_kind = PlanStatus::kReached;
                evidence.terminal_branch = terminal_branch;
                evidence.route = route;
                evidence.duration_attempt = attempt;
                evidence.duration_s = duration_s;
                evidence.scene_collision_sigma = scene_sigma;
                evidence.solve_time_s =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - solve_start)
                        .count();
                evidence.disposition = "optimizer_exception";
                attempts.push_back(std::move(evidence));
                failure_reason = exception.what();
                return result;
            }
            const auto solve_end = std::chrono::steady_clock::now();
            trajectory.optimization_duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(solve_end - solve_start);

            CandidateEvidence evidence;
            evidence.terminal_kind = PlanStatus::kReached;
            evidence.terminal_branch = terminal_branch;
            evidence.route = route;
            evidence.duration_attempt = attempt;
            evidence.duration_s = duration_s;
            evidence.scene_collision_sigma = scene_sigma;
            evidence.solve_time_s = std::chrono::duration<double>(solve_end - solve_start).count();
            if (trajectory.trajectory_pos.empty()) {
                evidence.disposition = "empty_trajectory";
                attempts.push_back(std::move(evidence));
                failure_reason = "optimizer returned an empty trajectory";
                return result;
            }

            result.last_validation = validate(trajectory, duration_s);
            evidence.validation = result.last_validation;
            evidence.disposition = result.last_validation.executable
                                       ? "executable"
                                       : result.last_validation.failure_reason;
            if (result.last_validation.disposition == CandidateDisposition::kNeedsLongerDuration &&
                attempt == kMaximumDurationAttempts)
                evidence.disposition = "dynamic_attempts_exhausted";
            attempts.push_back(std::move(evidence));
            const std::size_t evidence_index = attempts.size() - 1;

            if (result.last_validation.executable) {
                RouteSolution solution;
                solution.trajectory = std::move(trajectory);
                solution.validation = result.last_validation;
                solution.terminal = terminal;
                solution.route = route;
                solution.duration_s = duration_s;
                solution.evidence_index = evidence_index;
                result.executable = std::move(solution);
                failure_reason.clear();
                return result;
            }
            failure_reason = result.last_validation.failure_reason;
            if (result.last_validation.disposition != CandidateDisposition::kNeedsLongerDuration)
                return result;
            if (attempt == kMaximumDurationAttempts) {
                failure_reason = "dynamic_attempts_exhausted";
                return result;
            }
            const double alpha = std::max(result.last_validation.max_velocity_ratio,
                                          std::sqrt(result.last_validation.max_acceleration_ratio));
            if (!(alpha > 1.0))
                return result;
            duration_s *= alpha;
        }
        return result;
    }

    void RecordSeedFailure(std::size_t terminal_branch, RouteHypothesis route, double scene_sigma,
                           std::vector<CandidateEvidence>& attempts, std::string& failure_reason)
    {
        CandidateEvidence evidence;
        evidence.terminal_kind = PlanStatus::kReached;
        evidence.terminal_branch = terminal_branch;
        evidence.route = route;
        evidence.duration_attempt = 0;
        evidence.scene_collision_sigma = scene_sigma;
        evidence.disposition = "route_seed_ik_failure";
        attempts.push_back(std::move(evidence));
        failure_reason = "route_seed_ik_failure";
    }

    void CopyJointLimits(const PlannerJointLimits& source, PlanJointLimits& target)
    {
        for (int joint = 0; joint < 7; ++joint) {
            target.lower_rad(joint) = source.position_rad.lower(joint);
            target.upper_rad(joint) = source.position_rad.upper(joint);
            target.hardware_velocity_rad_s(joint) = source.hardware_velocity_rad_s.upper(joint);
            target.effective_velocity_rad_s(joint) = source.effective_velocity_rad_s.upper(joint);
            target.hardware_acceleration_rad_s2(joint) =
                source.hardware_acceleration_rad_s2.upper(joint);
            target.effective_acceleration_rad_s2(joint) =
                source.effective_acceleration_rad_s2.upper(joint);
        }
    }

    PlanValidationInputs ValidationInputs(
        const Eigen::Matrix<double, 7, 1>& q_start_rad,
        const std::optional<Eigen::Matrix<double, 7, 1>>& qdot_start_rad_s,
        const PlannerJointLimits& limits, const std::vector<NamedObstacleField>& obstacle_fields,
        double minimum_clearance_m, const gtsam::Pose3& terminal_mount, double validation_dt_s)
    {
        PlanValidationInputs inputs;
        inputs.measured_q_rad = q_start_rad;
        inputs.measured_qdot_rad_s = qdot_start_rad_s;
        inputs.position_lower_rad = limits.position_rad.lower;
        inputs.position_upper_rad = limits.position_rad.upper;
        inputs.effective_velocity_rad_s = limits.effective_velocity_rad_s.upper;
        inputs.effective_acceleration_rad_s2 = limits.effective_acceleration_rad_s2.upper;
        inputs.obstacle_fields = &obstacle_fields;
        inputs.minimum_clearance_m = minimum_clearance_m;
        inputs.requested_terminal_mount = terminal_mount;
        inputs.candidate_terminal_mount = terminal_mount;
        inputs.intended_status = PlanStatus::kReached;
        inputs.validation_dt_s = validation_dt_s;
        return inputs;
    }

    bool
    StartVelocityExceedsLimit(const std::optional<Eigen::Matrix<double, 7, 1>>& qdot_start_rad_s,
                              const PlannerJointLimits& limits)
    {
        return qdot_start_rad_s && ((qdot_start_rad_s->cwiseAbs().array() >
                                     limits.effective_velocity_rad_s.upper.array())
                                        .any());
    }

    bool WithinPlannerLimits(const Eigen::Matrix<double, 7, 1>& q, const PathIkJointLimits& limits)
    {
        if (!q.allFinite())
            return false;
        for (int joint = 0; joint < 7; ++joint) {
            const bool continuous =
                limits.lower_rad(joint) < -1e10 || limits.upper_rad(joint) > 1e10;
            if (!continuous &&
                (q(joint) < limits.lower_rad(joint) || q(joint) > limits.upper_rad(joint)))
                return false;
        }
        return true;
    }

    std::optional<JointConfiguration>
    SolveBypassMidpointIk(const PathIkArm& arm, const PathIkJointLimits& limits,
                          const PlannerModel& model, const PlanValidationReport& collision,
                          const Eigen::Vector3d& displacement_mount)
    {
        const gtsam::Pose3 collision_pose =
            ToolPoseInMount(model, collision.first_scene_violation_q);
        Eigen::Isometry3d target(collision_pose.matrix());
        target.translation() += displacement_mount;

        analytical_ik::IKTolerance tolerance;
        tolerance.converge_position_m = 0.001;
        tolerance.accept_position_m = 0.001;
        tolerance.converge_orientation_rad = 0.01;
        tolerance.accept_orientation_rad = 0.01;
        JointConfiguration seed = collision.first_scene_violation_q;
        const auto result = analytical_ik::AnalyticalIKSolver::solveBestIK(
            target.matrix(), arm.base_transform, seed, 1, arm.end_effector_frame, arm.left_arm,
            tolerance);
        if (!result.attempted || !result.is_valid ||
            result.position_error_m > tolerance.accept_position_m ||
            result.orientation_error_rad > tolerance.accept_orientation_rad ||
            !WithinPlannerLimits(result.joint_angles, limits))
            return std::nullopt;
        return result.joint_angles;
    }

    double CappedClearance(const PlanValidationReport& validation, double preferred_clearance_m)
    {
        return std::min(validation.minimum_scene_clearance_m, preferred_clearance_m);
    }

    bool BetterPointRoute(const RouteSolution& candidate, const RouteSolution& incumbent,
                          double preferred_clearance_m)
    {
        const double candidate_clearance =
            CappedClearance(candidate.validation, preferred_clearance_m);
        const double incumbent_clearance =
            CappedClearance(incumbent.validation, preferred_clearance_m);
        if (candidate_clearance != incumbent_clearance)
            return candidate_clearance > incumbent_clearance;
        if (candidate.duration_s != incumbent.duration_s)
            return candidate.duration_s < incumbent.duration_s;
        if (candidate.validation.integrated_joint_travel_rad !=
            incumbent.validation.integrated_joint_travel_rad)
            return candidate.validation.integrated_joint_travel_rad <
                   incumbent.validation.integrated_joint_travel_rad;
        return static_cast<int>(candidate.route) < static_cast<int>(incumbent.route);
    }

    bool BetterTraceRoute(const RouteSolution& candidate, const RouteSolution& incumbent,
                          double preferred_clearance_m)
    {
        if (candidate.validation.trace_rms_position_m != incumbent.validation.trace_rms_position_m)
            return candidate.validation.trace_rms_position_m <
                   incumbent.validation.trace_rms_position_m;
        if (candidate.validation.trace_max_position_m != incumbent.validation.trace_max_position_m)
            return candidate.validation.trace_max_position_m <
                   incumbent.validation.trace_max_position_m;
        const double candidate_clearance =
            CappedClearance(candidate.validation, preferred_clearance_m);
        const double incumbent_clearance =
            CappedClearance(incumbent.validation, preferred_clearance_m);
        if (candidate_clearance != incumbent_clearance)
            return candidate_clearance > incumbent_clearance;
        return static_cast<int>(candidate.route) < static_cast<int>(incumbent.route);
    }

    Eigen::Vector3d BypassDisplacement(const PlanValidationReport& collision,
                                       double preferred_clearance_m, RouteHypothesis route)
    {
        const double lift = preferred_clearance_m - collision.first_scene_violation_clearance_m;
        const Eigen::Vector3d normal = collision.first_scene_violation_normal_mount.normalized();
        const Eigen::Vector3d tangent = DeterministicTangent(normal);
        const double tangent_sign = route == RouteHypothesis::kPositiveBypass ? 1.0 : -1.0;
        return lift * normal + tangent_sign * lift * tangent;
    }

} // namespace

const char* RouteHypothesisName(RouteHypothesis route)
{
    switch (route) {
    case RouteHypothesis::kNormal:
        return "normal";
    case RouteHypothesis::kPositiveBypass:
        return "positive_bypass";
    case RouteHypothesis::kNegativeBypass:
        return "negative_bypass";
    }
    return "unknown";
}

PlanOutcome SolveToPosition(const PlannerModel& model, const PlanRequest& request,
                            const std::string& joint_limits_yaml, const PlannerConfig& config)
{
    PlanOutcome outcome;
    try {
        const PlannerJointLimits limits = createJointLimits(joint_limits_yaml);
        CopyJointLimits(limits, outcome.joint_limits);
        if (StartVelocityExceedsLimit(request.qdot_start_rad_s, limits)) {
            outcome.failure_reason = "start_velocity_over_effective_limit";
            return outcome;
        }

        const JointLimits& pos_limits = limits.position_rad;
        const JointLimits& vel_limits = limits.effective_velocity_rad_s;
        const gtsam::Pose3 start_pose = ToolPoseInMount(model, request.q_start_rad);
        const gtsam::Rot3 goal_rotation =
            request.goal_rotation ? gtsam::Rot3(*request.goal_rotation) : start_pose.rotation();
        const gtsam::Pose3 goal_pose(goal_rotation, gtsam::Point3(request.goal_position_m));
        const double distance_m = (request.goal_position_m - start_pose.translation()).norm();
        const std::size_t support_intervals = static_cast<std::size_t>(config.motion.waypoints);
        const double base_duration_s =
            std::max(config.motion.min_duration_s, distance_m / config.motion.nominal_speed_mps);
        outcome.total_time_sec = base_duration_s;

        PathIkArm arm;
        arm.base_transform = model.base_pose.matrix();
        arm.end_effector_frame = model.end_effector_frame;
        arm.left_arm = model.left_arm;
        PathIkJointLimits ik_limits;
        ik_limits.lower_rad = pos_limits.lower;
        ik_limits.upper_rad = pos_limits.upper;
        const auto terminal_candidates =
            SolveTerminalIkCandidates(arm, Eigen::Isometry3d(goal_pose.matrix()),
                                      request.q_start_rad, ik_limits, config.effective_ik_seed);
        if (terminal_candidates.empty()) {
            outcome.failure_reason = "terminal IK produced no legal candidate";
            return outcome;
        }

        const auto obstacle_fields =
            MakeNamedObstacleFields(MountGridGeometry(), model, config.scene);
        std::optional<gtsam::Vector> start_vel;
        if (request.qdot_start_rad_s)
            start_vel = gtsam::Vector(*request.qdot_start_rad_s);
        OptimizeTrajectory optimizer;

        for (std::size_t branch = 0; branch < terminal_candidates.size(); ++branch) {
            const TerminalIkCandidate& terminal = terminal_candidates[branch];
            const auto solve_route = [&](RouteHypothesis route,
                                         const std::vector<JointConfiguration>& seed) {
                const double scene_sigma = route == RouteHypothesis::kNormal
                                               ? config.optimizer.collision_sigma
                                               : 0.1 * config.optimizer.collision_sigma;
                const auto solve = [&](double duration_s) {
                    gtsam::Values initial_values;
                    const double dt = duration_s / static_cast<double>(support_intervals);
                    for (std::size_t i = 0; i <= support_intervals; ++i) {
                        initial_values.insert(gtsam::Symbol('x', i), gtsam::Vector(seed[i]));
                        const gtsam::Vector velocity =
                            i == support_intervals ? gtsam::Vector::Zero(7)
                                                   : gtsam::Vector((seed[i + 1] - seed[i]) / dt);
                        initial_values.insert(gtsam::Symbol('v', i), velocity);
                    }
                    return optimizer.optimizeJointTrajectory(
                        *model.arm_model, obstacle_fields, initial_values,
                        gtsam::Vector(request.q_start_rad), start_vel,
                        gtsam::Vector(terminal.configuration), pos_limits, vel_limits,
                        support_intervals, duration_s, config.optimizer, scene_sigma,
                        config.optimizer.collision_sigma);
                };
                const auto validate = [&](const TrajectoryResult& trajectory, double duration_s) {
                    const PlanValidationInputs inputs =
                        ValidationInputs(request.q_start_rad, request.qdot_start_rad_s, limits,
                                         obstacle_fields, config.minimum_clearance_m, goal_pose,
                                         config.path_following.validation_dt_s);
                    return ValidatePlan(model, trajectory, duration_s, inputs);
                };
                return SolveDurationCandidate(terminal, branch, route, base_duration_s, scene_sigma,
                                              solve, validate, outcome.candidate_attempts,
                                              outcome.failure_reason);
            };

            const auto normal_seed =
                PointBypassSeed(request.q_start_rad, terminal.configuration, support_intervals + 1);
            auto normal = solve_route(RouteHypothesis::kNormal, normal_seed);
            const PlanValidationReport& normal_report = normal.last_validation;
            const double normal_duration_s = normal.last_duration_s;
            std::optional<RouteSolution> branch_winner = std::move(normal.executable);

            if (!branch_winner && normal_report.has_first_scene_violation) {
                if (normal_report.first_scene_violation_time_s <= 0.0) {
                    outcome.failure_reason = "prohibited_start_collision";
                    return outcome;
                }
                if (normal_report.first_scene_violation_time_s < normal_duration_s) {
                    const double fraction =
                        normal_report.first_scene_violation_time_s / normal_duration_s;
                    for (const RouteHypothesis route :
                         {RouteHypothesis::kPositiveBypass, RouteHypothesis::kNegativeBypass}) {
                        const auto midpoint = SolveBypassMidpointIk(
                            arm, ik_limits, model, normal_report,
                            BypassDisplacement(normal_report,
                                               config.optimizer.preferred_clearance_m, route));
                        if (!midpoint) {
                            RecordSeedFailure(branch, route, 0.1 * config.optimizer.collision_sigma,
                                              outcome.candidate_attempts, outcome.failure_reason);
                            continue;
                        }
                        const auto bypass_seed = PointBypassSeed(
                            request.q_start_rad, terminal.configuration, support_intervals + 1,
                            std::make_pair(fraction, *midpoint));
                        auto solved = solve_route(route, bypass_seed);
                        if (solved.executable &&
                            (!branch_winner ||
                             BetterPointRoute(*solved.executable, *branch_winner,
                                              config.optimizer.preferred_clearance_m)))
                            branch_winner = std::move(solved.executable);
                    }
                }
            }

            if (branch_winner) {
                outcome.status = PlanStatus::kReached;
                outcome.trajectory = std::move(branch_winner->trajectory);
                outcome.validation = branch_winner->validation;
                outcome.final_goal_error_m =
                    branch_winner->validation.requested_terminal_position_error_m;
                outcome.total_time_sec = branch_winner->duration_s;
                outcome.terminal_candidate = branch_winner->terminal;
                outcome.selected_candidate_attempt = branch_winner->evidence_index;
                outcome.failure_reason.clear();
                break;
            }
        }

        if (!outcome.trajectory && outcome.failure_reason.empty())
            outcome.failure_reason = "no executable point trajectory";
    } catch (const std::exception& exception) {
        outcome.failure_reason = exception.what();
    }
    return outcome;
}

PathPlanOutcome SolveAlongPath(const PlannerModel& model, const CartesianPath& task_path,
                               const Eigen::Matrix<double, 7, 1>& q_start_rad,
                               const std::optional<Eigen::Matrix<double, 7, 1>>& qdot_start_rad_s,
                               const std::string& joint_limits_yaml, const PlannerConfig& config)
{
    PathPlanOutcome outcome;
    try {
        const PlannerJointLimits limits = createJointLimits(joint_limits_yaml);
        CopyJointLimits(limits, outcome.joint_limits);
        if (StartVelocityExceedsLimit(qdot_start_rad_s, limits)) {
            outcome.failure_reason = "start_velocity_over_effective_limit";
            return outcome;
        }
        const JointLimits& pos_limits = limits.position_rad;
        const JointLimits& vel_limits = limits.effective_velocity_rad_s;

        PathIkArm arm;
        arm.base_transform = model.base_pose.matrix();
        arm.end_effector_frame = model.end_effector_frame;
        arm.left_arm = model.left_arm;
        PathIkJointLimits ik_limits;
        ik_limits.lower_rad = pos_limits.lower;
        ik_limits.upper_rad = pos_limits.upper;

        const auto terminal_candidates = SolveTerminalIkCandidates(
            arm, task_path.samples.back().pose, q_start_rad, ik_limits, config.effective_ik_seed);
        if (terminal_candidates.empty()) {
            outcome.failure_reason = "terminal IK produced no legal candidate";
            return outcome;
        }

        analytical_ik::IKTolerance tolerance;
        const double target_m = config.path_following.maximum_planning_error_m;
        tolerance.converge_position_m = target_m * 0.1;
        tolerance.converge_orientation_rad =
            config.path_following.maximum_orientation_error_rad * 0.1;
        tolerance.accept_position_m = target_m;
        tolerance.accept_orientation_rad = config.path_following.maximum_orientation_error_rad;

        const PathIkResult normal_walk =
            SolvePathIk(task_path, arm, q_start_rad, ik_limits, tolerance,
                        /*closed=*/true, config.effective_ik_seed);
        outcome.ik_walk = normal_walk;
        if (!normal_walk.success) {
            for (std::size_t branch = 0; branch < terminal_candidates.size(); ++branch) {
                CandidateEvidence evidence;
                evidence.terminal_kind = PlanStatus::kReached;
                evidence.terminal_branch = branch;
                evidence.route = RouteHypothesis::kNormal;
                evidence.duration_attempt = 0;
                evidence.scene_collision_sigma = config.optimizer.collision_sigma;
                evidence.disposition = "route_seed_ik_failure";
                outcome.candidate_attempts.push_back(std::move(evidence));
            }
            outcome.failure_reason = "path IK initialization failed: the path entry pose could "
                                     "not be solved from the measured start configuration within "
                                     "the bounded search";
            return outcome;
        }

        const auto configurations_from_walk = [](const PathIkResult& walk) {
            std::vector<JointConfiguration> configurations;
            configurations.reserve(walk.samples.size());
            for (const PathIkSample& sample : walk.samples)
                configurations.push_back(sample.configuration);
            return configurations;
        };

        ApproachPacing pacing;
        pacing.velocity_fraction = config.path_following.approach_velocity_fraction;
        pacing.minimum_duration_s = config.path_following.approach_min_duration_s;
        pacing.waypoints = config.path_following.approach_waypoints;
        JointVelocityLimits joint_velocity_limits;
        for (int joint = 0; joint < 7; ++joint)
            joint_velocity_limits(joint) = vel_limits.upper(joint);
        const Eigen::Vector3d rotation_sigma =
            Eigen::Vector3d::Constant(config.path_following.rotation_prior_sigma_rad);
        const Eigen::Vector3d position_sigma =
            Eigen::Vector3d::Constant(config.path_following.position_prior_sigma_m);
        const AssembledPath normal_assembled =
            AssembleCirclePlan(task_path, configurations_from_walk(normal_walk), q_start_rad,
                               joint_velocity_limits, pacing, rotation_sigma, position_sigma);

        const auto obstacle_fields =
            MakeNamedObstacleFields(MountGridGeometry(), model, config.scene);
        std::optional<gtsam::Vector> start_vel;
        if (qdot_start_rad_s)
            start_vel = gtsam::Vector(*qdot_start_rad_s);
        OptimizeTrajectory optimizer;

        const auto update_walk_summary = [&](const PathIkResult& walk) {
            outcome.maximum_joint_step_rad = walk.maximum_joint_step_rad;
            outcome.closure_drift_rad = walk.closure_drift_rad;
            outcome.ik_unresolved_samples = walk.unresolved_samples;
            outcome.ik_interpolated_samples = walk.interpolated_samples;
        };
        update_walk_summary(normal_walk);

        for (std::size_t branch = 0; branch < terminal_candidates.size(); ++branch) {
            const TerminalIkCandidate& terminal = terminal_candidates[branch];
            const auto solve_route = [&](RouteHypothesis route, const AssembledPath& assembled,
                                         const PathIkResult& seed_walk) {
                const double scene_sigma = route == RouteHypothesis::kNormal
                                               ? config.optimizer.collision_sigma
                                               : 0.1 * config.optimizer.collision_sigma;
                const double base_duration_s = assembled.total_duration_s;
                const double task_start_fraction =
                    assembled.waypoints[assembled.task_start_index].time_s / base_duration_s;
                const auto solve = [&](double duration_s) {
                    const double time_scale = duration_s / base_duration_s;
                    std::vector<OptimisationWaypoint> attempt_waypoints = assembled.waypoints;
                    for (OptimisationWaypoint& waypoint : attempt_waypoints)
                        waypoint.time_s *= time_scale;

                    gtsam::Values initial_values;
                    const std::size_t states = attempt_waypoints.size();
                    for (std::size_t i = 0; i < states; ++i) {
                        const JointConfiguration& q = i + 1 == states
                                                          ? terminal.configuration
                                                          : assembled.initial_configurations[i];
                        initial_values.insert(gtsam::Symbol('x', i), gtsam::Vector(q));
                        gtsam::Vector velocity = gtsam::Vector::Zero(7);
                        if (i + 1 < states) {
                            const double dt =
                                attempt_waypoints[i + 1].time_s - attempt_waypoints[i].time_s;
                            const JointConfiguration& next_q =
                                i + 2 == states ? terminal.configuration
                                                : assembled.initial_configurations[i + 1];
                            velocity = gtsam::Vector((next_q - q) / dt);
                        }
                        initial_values.insert(gtsam::Symbol('v', i), velocity);
                    }
                    return optimizer.optimizeTaskTrajectory(
                        *model.arm_model, obstacle_fields, initial_values, attempt_waypoints,
                        gtsam::Vector(q_start_rad), start_vel,
                        gtsam::Vector(terminal.configuration), assembled.zero_velocity_indices,
                        pos_limits, vel_limits, duration_s, config.optimizer, scene_sigma,
                        config.optimizer.collision_sigma);
                };
                const auto validate = [&](const TrajectoryResult& trajectory, double duration_s) {
                    const double task_start_time_s = duration_s * task_start_fraction;
                    const gtsam::Pose3 terminal_pose(
                        gtsam::Rot3(task_path.samples.back().pose.linear()),
                        gtsam::Point3(task_path.samples.back().pose.translation()));
                    PlanValidationInputs inputs =
                        ValidationInputs(q_start_rad, qdot_start_rad_s, limits, obstacle_fields,
                                         config.minimum_clearance_m, terminal_pose,
                                         config.path_following.validation_dt_s);
                    inputs.desired_task_path = &task_path;
                    inputs.task_start_time_s = task_start_time_s;
                    return ValidatePlan(model, trajectory, duration_s, inputs);
                };
                auto result = SolveDurationCandidate(
                    terminal, branch, route, base_duration_s, scene_sigma, solve, validate,
                    outcome.candidate_attempts, outcome.failure_reason);
                if (result.executable) {
                    result.executable->ik_walk = seed_walk;
                    result.executable->has_ik_walk = true;
                    result.executable->task_start_time_s =
                        result.executable->duration_s * task_start_fraction;
                }
                return result;
            };

            auto normal = solve_route(RouteHypothesis::kNormal, normal_assembled, normal_walk);
            const PlanValidationReport& normal_report = normal.last_validation;
            const double normal_duration_s = normal.last_duration_s;
            const double normal_task_start_time_s =
                normal_duration_s *
                normal_assembled.waypoints[normal_assembled.task_start_index].time_s /
                normal_assembled.total_duration_s;
            std::optional<RouteSolution> branch_winner = std::move(normal.executable);

            if (!branch_winner && normal_report.has_first_scene_violation) {
                if (normal_report.first_scene_violation_time_s <= 0.0) {
                    outcome.failure_reason = "prohibited_start_collision";
                    return outcome;
                }
                if (normal_report.first_scene_violation_time_s < normal_duration_s) {
                    for (const RouteHypothesis route :
                         {RouteHypothesis::kPositiveBypass, RouteHypothesis::kNegativeBypass}) {
                        const Eigen::Vector3d displacement = BypassDisplacement(
                            normal_report, config.optimizer.preferred_clearance_m, route);
                        AssembledPath bypass_assembled;
                        PathIkResult bypass_walk = normal_walk;
                        if (normal_report.first_scene_violation_time_s < normal_task_start_time_s) {
                            const auto midpoint = SolveBypassMidpointIk(
                                arm, ik_limits, model, normal_report, displacement);
                            if (!midpoint) {
                                RecordSeedFailure(
                                    branch, route, 0.1 * config.optimizer.collision_sigma,
                                    outcome.candidate_attempts, outcome.failure_reason);
                                continue;
                            }
                            bypass_assembled = normal_assembled;
                            const double fraction = normal_report.first_scene_violation_time_s /
                                                    normal_task_start_time_s;
                            const auto approach_seed = PointBypassSeed(
                                q_start_rad,
                                normal_assembled
                                    .initial_configurations[normal_assembled.task_start_index],
                                normal_assembled.task_start_index + 1,
                                std::make_pair(fraction, *midpoint));
                            for (std::size_t i = 0; i <= normal_assembled.task_start_index; ++i)
                                bypass_assembled.initial_configurations[i] = approach_seed[i];
                        } else {
                            const double task_span_s = normal_duration_s - normal_task_start_time_s;
                            const double parameter_u = (normal_report.first_scene_violation_time_s -
                                                        normal_task_start_time_s) /
                                                       task_span_s;
                            const CartesianPath offset_path = TraceBypassSeed(
                                task_path, parameter_u, displacement, kPathIkAnchorStride);
                            bypass_walk =
                                SolvePathIk(offset_path, arm, q_start_rad, ik_limits, tolerance,
                                            /*closed=*/true, config.effective_ik_seed);
                            if (!bypass_walk.success) {
                                RecordSeedFailure(
                                    branch, route, 0.1 * config.optimizer.collision_sigma,
                                    outcome.candidate_attempts, outcome.failure_reason);
                                continue;
                            }
                            bypass_assembled = AssembleCirclePlan(
                                task_path, configurations_from_walk(bypass_walk), q_start_rad,
                                joint_velocity_limits, pacing, rotation_sigma, position_sigma);
                        }

                        auto solved = solve_route(route, bypass_assembled, bypass_walk);
                        if (solved.executable &&
                            (!branch_winner ||
                             BetterTraceRoute(*solved.executable, *branch_winner,
                                              config.optimizer.preferred_clearance_m)))
                            branch_winner = std::move(solved.executable);
                    }
                }
            }

            if (branch_winner) {
                outcome.status = PlanStatus::kReached;
                outcome.trajectory = std::move(branch_winner->trajectory);
                outcome.validation = branch_winner->validation;
                outcome.terminal_candidate = branch_winner->terminal;
                outcome.selected_candidate_attempt = branch_winner->evidence_index;
                outcome.total_time_sec = branch_winner->duration_s;
                outcome.task_start_time_s = branch_winner->task_start_time_s;
                if (branch_winner->has_ik_walk) {
                    outcome.ik_walk = branch_winner->ik_walk;
                    update_walk_summary(branch_winner->ik_walk);
                }
                outcome.failure_reason.clear();
                break;
            }
        }

        if (!outcome.trajectory && outcome.failure_reason.empty())
            outcome.failure_reason = "no executable traced trajectory";
    } catch (const std::exception& exception) {
        outcome.failure_reason = exception.what();
    }
    return outcome;
}
