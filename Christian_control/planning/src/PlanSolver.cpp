#include "PlanSolver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "MountSdf.h"
#include "PathAssembly.h"
#include "PathIk.h"
#include "TrajectoryOptimization.h"
#include "ValidatePlan.h"

// The pipeline, both entry points:
//
//   request -> terminal IK seeds (few, diverse) -> one GPMP2 solve per seed
//           -> best geometrically valid solution -> retime for dynamics
//           -> final full validation -> trajectory (or an honest failure)
//
// GPMP2 decides WHERE the arm goes; retiming decides HOW FAST. A failed
// stage reports why it failed — it never invents a new planning problem
// to rescue itself.

namespace
{

    // How many distinct terminal joint postures are tried, each with ONE
    // optimiser solve. Diversity, not volume: a redundant 7-DoF arm has a
    // handful of meaningfully different solution families for one pose, and
    // SolveTerminalIkCandidates already ranks the best distinct ones first.
    constexpr std::size_t kMaxTerminalSeeds = 4;

    // Safety margin applied on top of the exact retiming factor, so the
    // revalidated ratios land visibly below 1 rather than on the boundary.
    constexpr double kRetimeMargin = 1.02;

    struct SearchTerminal {
        TerminalIkCandidate ik;
        gtsam::Pose3 candidate_pose;
        double requested_position_shortfall_m = 0.0;
        double requested_orientation_shortfall_rad = 0.0;
        int orientation_tier = 1;
    };

    // The best geometrically valid candidate so far, carried between the
    // per-seed solve loop and the retime/final-validation stage.
    struct SelectedCandidate {
        TrajectoryResult trajectory;
        PlanValidationReport validation;
        SearchTerminal terminal;
        std::size_t terminal_branch = 0;
        double duration_s = 0.0;
        std::size_t evidence_index = 0;
    };

    CandidateEvidence MakeEvidence(const SearchTerminal& terminal,
                                   std::size_t terminal_branch,
                                   double duration_s, double scene_sigma,
                                   const gtsam::Pose3& requested_pose)
    {
        CandidateEvidence evidence;
        evidence.terminal_kind = PlanStatus::kReached;
        evidence.target_position_mount_m = requested_pose.translation();
        evidence.target_orientation_mount =
            Eigen::Quaterniond(requested_pose.rotation().matrix());
        evidence.terminal_branch = terminal_branch;
        evidence.terminal_ik_stream_id = terminal.ik.stream_id;
        evidence.terminal_ik_attempt_count = kTerminalIkAttemptsPerStream;
        evidence.terminal_ik_attempt_index = terminal.ik.attempt_index;
        evidence.terminal_ik_position_residual_m = terminal.ik.position_residual_m;
        evidence.terminal_ik_orientation_residual_rad = terminal.ik.orientation_residual_rad;
        evidence.terminal_ik_legal = terminal.ik.planner_limit_legal;
        evidence.terminal_ik_exact = terminal.ik.exact;
        evidence.requested_position_shortfall_m =
            terminal.requested_position_shortfall_m;
        evidence.requested_orientation_shortfall_rad =
            terminal.requested_orientation_shortfall_rad;
        evidence.orientation_tier = terminal.orientation_tier;
        evidence.duration_s = duration_s;
        evidence.scene_collision_sigma = scene_sigma;
        return evidence;
    }

    void CopyOptimizerEvidence(const TrajectoryResult& trajectory,
                               CandidateEvidence& evidence)
    {
        evidence.optimizer_iterations = trajectory.optimizer_iterations;
        evidence.optimizer_max_iterations = trajectory.optimizer_max_iterations;
        evidence.optimizer_converged = trajectory.optimizer_converged;
        evidence.optimizer_termination = trajectory.optimizer_termination;
        evidence.optimizer_start_total_cost = trajectory.start_error;
        evidence.optimizer_final_total_cost = trajectory.final_error;
        evidence.optimizer_final_factor_costs.insert(trajectory.final_costs.begin(),
                                                     trajectory.final_costs.end());
    }

    // ---- validation facts -------------------------------------------------
    //
    // The validator reports facts; these two predicates are the ONLY places
    // those facts are turned into decisions. Raw GTSAM graph error is an
    // optimiser objective, not a physical statement — it is recorded in the
    // evidence rows as a diagnostic and gates nothing.

    // Everything retiming cannot fix. A candidate failing any of these is
    // rejected outright: slowing down cannot repair a shape, a collision,
    // a joint-limit violation or a task-error miss.
    bool GeometricallyValid(const PlanValidationReport& report)
    {
        return report.disposition != CandidateDisposition::kInvalid &&
               report.finite && report.start_valid && report.scene_valid &&
               report.self_collision_valid && report.joint_limits_valid &&
               report.task_valid;
    }

    // The final physical acceptance: geometry AND dynamics.
    bool PhysicallyExecutable(const PlanValidationReport& report)
    {
        return GeometricallyValid(report) && report.executable;
    }

    std::string GeometryFailureReason(const PlanValidationReport& report)
    {
        if (!report.finite) return "trajectory_not_finite";
        if (!report.start_valid) return "start_state_mismatch";
        if (!report.scene_valid) return "scene_clearance_violation";
        if (!report.self_collision_valid) return "self_collision";
        if (!report.joint_limits_valid) return "joint_limit_violation";
        if (!report.task_valid) return "task_error_above_tolerance";
        if (!report.failure_reason.empty()) return report.failure_reason;
        return "not_executable";
    }

    // ---- retiming ---------------------------------------------------------
    //
    // Uniform time scaling: the factor by which the trajectory must be
    // slowed so no joint exceeds its velocity or acceleration limit.
    // Geometry is untouched — the arm follows the same joint path, later.
    //
    //   alpha = max(1, max|qdot|/qdot_max, sqrt(max|qddot|/qddot_max))
    //
    // Velocity scales as 1/alpha and acceleration as 1/alpha^2, which is
    // why the acceleration term takes a square root.
    double RetimeScale(const PlanValidationReport& report)
    {
        const double alpha = std::max(
            {1.0, report.max_velocity_ratio,
             std::sqrt(std::max(0.0, report.max_acceleration_ratio))});
        return alpha > 1.0 ? alpha * kRetimeMargin : 1.0;
    }

    void ApplyRetiming(TrajectoryResult& trajectory, double alpha,
                       double& duration_s)
    {
        if (alpha <= 1.0) return;
        duration_s *= alpha;
        for (auto& velocity : trajectory.trajectory_vel) velocity /= alpha;
    }

    // ---- shared boundary helpers -------------------------------------------

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

    // The measured start is a fact, not a request. There is no prohibited
    // start position: a start below the configured clearance floor relaxes
    // the floor to its own clearance for the whole solve (2026-08-23: two
    // sessions deadlocked on a start refusal until the arm was hand-moved),
    // so the plan may leave the region but never come closer than it
    // started, and a clean start keeps the configured floor unchanged.
    double ClearanceFloorFromStart(
        const PlannerModel& model,
        const Eigen::Matrix<double, 7, 1>& q_start_rad,
        const std::vector<NamedObstacleField>& obstacle_fields,
        double minimum_clearance_m)
    {
        return std::min(minimum_clearance_m,
                        MeasureConfigurationClearance(
                            model, q_start_rad, obstacle_fields,
                            minimum_clearance_m)
                            .minimum_scene_clearance_m);
    }

    double OrientationError(const gtsam::Pose3& requested,
                            const gtsam::Pose3& actual)
    {
        return gtsam::Rot3::Logmap(
                   requested.rotation().between(actual.rotation()))
            .norm();
    }

    SearchTerminal MakeSearchTerminal(const PlannerModel& model,
                                      const TerminalIkCandidate& ik,
                                      const gtsam::Pose3& requested_pose)
    {
        SearchTerminal terminal;
        terminal.ik = ik;
        terminal.candidate_pose = ToolPoseInMount(model, ik.configuration);
        terminal.requested_position_shortfall_m =
            (terminal.candidate_pose.translation() -
             requested_pose.translation())
                .norm();
        terminal.requested_orientation_shortfall_rad =
            OrientationError(requested_pose, terminal.candidate_pose);
        terminal.orientation_tier =
            terminal.requested_orientation_shortfall_rad <= 0.01 ? 1 : 2;
        return terminal;
    }

    void RecordTerminalIkSummaries(
        const PlannerModel& model,
        const std::vector<TerminalIkCandidate>& attempts,
        const gtsam::Pose3& requested_pose,
        std::vector<CandidateEvidence>& evidence_rows)
    {
        const auto rank = [](const TerminalIkCandidate& value) {
            return std::make_tuple(
                !value.planner_limit_legal, !value.exact,
                value.orientation_residual_rad,
                value.position_residual_m, value.attempt_index);
        };
        for (std::uint64_t stream = 0; stream < kTerminalIkSeedStreams; ++stream) {
            const TerminalIkCandidate* best = nullptr;
            std::size_t attempt_count = 0;
            for (const TerminalIkCandidate& candidate : attempts) {
                if (candidate.stream_id != stream)
                    continue;
                ++attempt_count;
                if (!best || rank(candidate) < rank(*best))
                    best = &candidate;
            }
            if (!best)
                continue;
            const SearchTerminal terminal =
                MakeSearchTerminal(model, *best, requested_pose);
            CandidateEvidence evidence =
                MakeEvidence(terminal, 0, 0.0, 0.0, requested_pose);
            evidence.stage = "terminal_ik";
            evidence.terminal_ik_attempt_count = attempt_count;
            if (!best->planner_limit_legal)
                evidence.disposition = "planner_limit_illegal";
            else if (best->exact)
                evidence.disposition = "exact_pose_candidate";
            else
                evidence.disposition = "near_miss_candidate";
            evidence_rows.push_back(std::move(evidence));
        }
    }

    // The few diverse terminal postures the seed loop draws from, best
    // posture first (SolveTerminalIkCandidates ranks distinct solution
    // families by joint headroom and displacement).
    std::vector<SearchTerminal> GenerateTerminalSeeds(
        const PlannerModel& model, const PathIkArm& arm,
        const PathIkJointLimits& limits,
        const Eigen::Matrix<double, 7, 1>& measured_q,
        const gtsam::Pose3& requested_pose, std::uint64_t effective_seed,
        std::size_t max_seeds,
        std::vector<CandidateEvidence>& evidence_rows)
    {
        std::vector<TerminalIkCandidate> attempts;
        const auto retained = SolveTerminalIkCandidates(
            arm, Eigen::Isometry3d(requested_pose.matrix()), measured_q,
            limits, effective_seed, max_seeds, &attempts);
        RecordTerminalIkSummaries(model, attempts, requested_pose, evidence_rows);
        std::vector<SearchTerminal> terminals;
        for (const TerminalIkCandidate& ik : retained)
            terminals.push_back(MakeSearchTerminal(model, ik, requested_pose));
        return terminals;
    }

    // Straight joint-space interpolation from the measured start to the
    // terminal posture: GPMP2's initial guess for a point goal.
    std::vector<Eigen::Matrix<double, 7, 1>> InterpolatedJointSeed(
        const Eigen::Matrix<double, 7, 1>& start,
        const Eigen::Matrix<double, 7, 1>& terminal, std::size_t count)
    {
        std::vector<Eigen::Matrix<double, 7, 1>> seed;
        seed.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const double w = count > 1
                                 ? static_cast<double>(i) /
                                       static_cast<double>(count - 1)
                                 : 1.0;
            seed.push_back(start + w * (terminal - start));
        }
        return seed;
    }

    // Retime the selected candidate and run the one final full validation.
    // Returns true when the retimed trajectory is physically executable;
    // the winner's evidence row is updated either way.
    template <typename Revalidate>
    bool FinishSelected(SelectedCandidate& selected, Revalidate revalidate,
                        std::vector<CandidateEvidence>& attempts,
                        std::string& failure_reason)
    {
        const double alpha = RetimeScale(selected.validation);
        ApplyRetiming(selected.trajectory, alpha, selected.duration_s);
        selected.validation =
            revalidate(selected.trajectory, selected.duration_s,
                       selected.terminal_branch);
        CandidateEvidence& row = attempts[selected.evidence_index];
        row.duration_s = selected.duration_s;
        row.validation = selected.validation;
        if (PhysicallyExecutable(selected.validation)) {
            row.disposition = "executable_selected";
            failure_reason.clear();
            return true;
        }
        row.disposition = selected.validation.executable
                              ? GeometryFailureReason(selected.validation)
                              : selected.validation.failure_reason;
        failure_reason = row.disposition;
        return false;
    }

} // namespace

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
        const std::size_t support_intervals = static_cast<std::size_t>(config.motion.waypoints);
        PathIkArm arm;
        arm.base_transform = model.base_pose.matrix();
        arm.end_effector_frame = model.end_effector_frame;
        arm.left_arm = model.left_arm;
        PathIkJointLimits ik_limits;
        ik_limits.lower_rad = pos_limits.lower;
        ik_limits.upper_rad = pos_limits.upper;
        const auto obstacle_fields =
            MakeNamedObstacleFields(MountGridGeometry(), model, config.scene);
        const double clearance_floor_m = ClearanceFloorFromStart(
            model, request.q_start_rad, obstacle_fields,
            config.minimum_clearance_m);
        std::optional<gtsam::Vector> start_vel;
        if (request.qdot_start_rad_s)
            start_vel = gtsam::Vector(*request.qdot_start_rad_s);
        OptimizeTrajectory optimizer;

        const std::size_t max_seeds = std::min(
            kMaxTerminalSeeds,
            static_cast<std::size_t>(std::max(1, config.max_restart_attempts)));
        const auto terminals = GenerateTerminalSeeds(
            model, arm, ik_limits, request.q_start_rad, goal_pose,
            config.effective_ik_seed, max_seeds, outcome.candidate_attempts);

        const double distance_m =
            (goal_pose.translation() - start_pose.translation()).norm();
        const double duration_s = std::max(
            config.motion.min_duration_s,
            distance_m / config.motion.nominal_speed_mps);

        const auto validate = [&](const TrajectoryResult& trajectory,
                                  double trajectory_duration_s,
                                  std::size_t branch) {
            const SearchTerminal& candidate = terminals[branch];
            PlanValidationInputs inputs =
                ValidationInputs(request.q_start_rad, request.qdot_start_rad_s, limits,
                                 obstacle_fields, clearance_floor_m, goal_pose,
                                 config.path_following.validation_dt_s);
            inputs.candidate_terminal_mount = candidate.candidate_pose;
            return ValidatePlan(model, trajectory, trajectory_duration_s, inputs);
        };

        // One solve per seed, best geometrically valid solution wins.
        // Ranking is physical: smallest requested-goal error, then the
        // optimiser's own final cost as a tie-break diagnostic.
        std::optional<SelectedCandidate> selected;
        for (std::size_t branch = 0; branch < terminals.size(); ++branch) {
            const SearchTerminal& candidate = terminals[branch];
            const auto solve_start = std::chrono::steady_clock::now();
            CandidateEvidence evidence = MakeEvidence(
                candidate, branch, duration_s,
                config.optimizer.collision_sigma, goal_pose);
            TrajectoryResult trajectory;
            try {
                const auto seed = InterpolatedJointSeed(
                    request.q_start_rad, candidate.ik.configuration,
                    support_intervals + 1);
                gtsam::Values initial_values;
                const double dt = duration_s / static_cast<double>(support_intervals);
                for (std::size_t i = 0; i <= support_intervals; ++i) {
                    initial_values.insert(gtsam::Symbol('x', i), gtsam::Vector(seed[i]));
                    const gtsam::Vector velocity =
                        i == support_intervals ? gtsam::Vector::Zero(7)
                                               : gtsam::Vector((seed[i + 1] - seed[i]) / dt);
                    initial_values.insert(gtsam::Symbol('v', i), velocity);
                }
                trajectory = optimizer.optimizeJointTrajectory(
                    *model.arm_model, obstacle_fields, initial_values,
                    gtsam::Vector(request.q_start_rad), start_vel,
                    gtsam::Vector(candidate.ik.configuration), pos_limits, vel_limits,
                    support_intervals, duration_s, config.optimizer,
                    config.optimizer.collision_sigma, config.optimizer.collision_sigma);
            } catch (const std::exception& exception) {
                evidence.solve_time_s =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                  solve_start)
                        .count();
                evidence.disposition = "optimizer_exception";
                outcome.candidate_attempts.push_back(std::move(evidence));
                outcome.failure_reason = exception.what();
                continue;
            }
            const auto solve_end = std::chrono::steady_clock::now();
            trajectory.optimization_duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(solve_end - solve_start);
            evidence.solve_time_s =
                std::chrono::duration<double>(solve_end - solve_start).count();
            if (trajectory.trajectory_pos.empty()) {
                evidence.disposition = "empty_trajectory";
                outcome.candidate_attempts.push_back(std::move(evidence));
                outcome.failure_reason = "optimizer returned an empty trajectory";
                continue;
            }
            CopyOptimizerEvidence(trajectory, evidence);
            const PlanValidationReport report = validate(trajectory, duration_s, branch);
            evidence.validation = report;
            evidence.capped_clearance_m =
                std::min(report.minimum_scene_clearance_m,
                         config.optimizer.preferred_clearance_m);
            const bool valid = GeometricallyValid(report);
            evidence.disposition =
                valid ? "geometry_valid" : GeometryFailureReason(report);
            outcome.candidate_attempts.push_back(std::move(evidence));
            if (!valid) {
                outcome.failure_reason = GeometryFailureReason(report);
                continue;
            }
            const bool better =
                !selected ||
                report.requested_terminal_position_error_m <
                    selected->validation.requested_terminal_position_error_m ||
                (report.requested_terminal_position_error_m ==
                     selected->validation.requested_terminal_position_error_m &&
                 trajectory.final_error < selected->trajectory.final_error);
            if (better) {
                SelectedCandidate candidate_solution;
                candidate_solution.trajectory = std::move(trajectory);
                candidate_solution.validation = report;
                candidate_solution.terminal = candidate;
                candidate_solution.terminal_branch = branch;
                candidate_solution.duration_s = duration_s;
                candidate_solution.evidence_index =
                    outcome.candidate_attempts.size() - 1;
                selected = std::move(candidate_solution);
            }
        }

        if (selected &&
            FinishSelected(*selected, validate, outcome.candidate_attempts,
                           outcome.failure_reason)) {
            outcome.status = PlanStatus::kReached;
            outcome.trajectory = std::move(selected->trajectory);
            outcome.validation = selected->validation;
            outcome.total_time_sec = selected->duration_s;
            outcome.terminal_candidate = selected->terminal.ik;
            outcome.selected_candidate_attempt = selected->evidence_index;
            outcome.final_goal_error_m =
                selected->validation.requested_terminal_position_error_m;
        } else {
            outcome.status = PlanStatus::kFailed;
            outcome.trajectory.reset();
            if (outcome.failure_reason.empty())
                outcome.failure_reason = terminals.empty()
                                             ? "goal_ik_failure"
                                             : "no executable point trajectory";
        }
    } catch (const std::exception& exception) {
        outcome.failure_reason = exception.what();
        outcome.status = PlanStatus::kFailed;
        outcome.trajectory.reset();
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

        if (task_path.samples.empty())
            throw std::invalid_argument("traced request contains no path samples");
        const gtsam::Pose3 requested_terminal(
            gtsam::Rot3(task_path.samples.back().pose.linear()),
            gtsam::Point3(task_path.samples.back().pose.translation()));

        PathIkArm arm;
        arm.base_transform = model.base_pose.matrix();
        arm.end_effector_frame = model.end_effector_frame;
        arm.left_arm = model.left_arm;
        PathIkJointLimits ik_limits;
        ik_limits.lower_rad = pos_limits.lower;
        ik_limits.upper_rad = pos_limits.upper;

        const auto obstacle_fields =
            MakeNamedObstacleFields(MountGridGeometry(), model, config.scene);
        const double clearance_floor_m = ClearanceFloorFromStart(
            model, q_start_rad, obstacle_fields, config.minimum_clearance_m);

        const std::size_t max_seeds = std::min(
            kMaxTerminalSeeds,
            static_cast<std::size_t>(std::max(1, config.max_restart_attempts)));
        const auto terminals = GenerateTerminalSeeds(
            model, arm, ik_limits, q_start_rad, requested_terminal,
            config.effective_ik_seed, max_seeds, outcome.candidate_attempts);

        analytical_ik::IKTolerance tolerance;
        const double target_m = config.path_following.maximum_planning_error_m;
        tolerance.converge_position_m = target_m * 0.1;
        tolerance.converge_orientation_rad =
            config.path_following.maximum_orientation_error_rad * 0.1;
        tolerance.accept_position_m = target_m;
        tolerance.accept_orientation_rad = config.path_following.maximum_orientation_error_rad;

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

        // Per-seed candidate: its own continuation-IK walk (a fresh
        // deterministic IKSeeding stream per seed, so each draw is
        // reproducible) and the assembled approach+task problem built from
        // it. A failed walk records its evidence and moves to the next
        // seed — path IK generates initial guesses; it does not decide
        // planner success.
        struct PathCandidate {
            PathIkResult walk;
            AssembledPath assembled;
        };
        std::vector<std::optional<PathCandidate>> candidates(terminals.size());
        for (std::size_t branch = 0; branch < terminals.size(); ++branch) {
            const std::uint64_t walk_seed =
                analytical_ik::IKSeeding{config.effective_ik_seed, branch}.Mixed();
            PathIkResult walk = SolvePathIk(task_path, arm, q_start_rad, ik_limits,
                                            tolerance, /*closed=*/true, walk_seed);
            outcome.ik_walk = walk;
            update_walk_summary(walk);
            if (!walk.success) {
                CandidateEvidence evidence = MakeEvidence(
                    terminals[branch], branch, 0.0,
                    config.optimizer.collision_sigma, requested_terminal);
                evidence.stage = "path_ik";
                evidence.disposition = "path_ik_failure";
                outcome.candidate_attempts.push_back(std::move(evidence));
                outcome.failure_reason = "path_ik_failure";
                continue;
            }
            AssembledPath assembled = AssembleCirclePlan(
                task_path, configurations_from_walk(walk), q_start_rad,
                joint_velocity_limits, pacing, rotation_sigma, position_sigma);
            candidates[branch] = PathCandidate{std::move(walk), std::move(assembled)};
        }

        const auto validate = [&](const TrajectoryResult& trajectory,
                                  double trajectory_duration_s,
                                  std::size_t branch) {
            const PathCandidate& entry = *candidates[branch];
            const double task_start_fraction =
                entry.assembled.waypoints[entry.assembled.task_start_index].time_s /
                entry.assembled.total_duration_s;
            PlanValidationInputs inputs =
                ValidationInputs(q_start_rad, qdot_start_rad_s, limits, obstacle_fields,
                                 clearance_floor_m, requested_terminal,
                                 config.path_following.validation_dt_s);
            inputs.candidate_terminal_mount = terminals[branch].candidate_pose;
            inputs.desired_task_path = &task_path;
            inputs.task_start_time_s = trajectory_duration_s * task_start_fraction;
            inputs.path_position_tolerance_m =
                config.path_following.maximum_planning_error_m;
            return ValidatePlan(model, trajectory, trajectory_duration_s, inputs);
        };

        // One solve per seed at the assembly's own duration. The terminal
        // is NOT pinned by a joint equality: the walk is obstacle-unaware,
        // so when the scene cost pushes the on-path states' redundant
        // joints away from the walk's terminal configuration, an equality
        // there demands the gap be repaid inside the final support interval,
        // producing an acceleration peak no slowing can reduce (measured
        // 2026-08-24: joint 2, ratio 2.2-3.3 invariant from 11 s to 223 s;
        // without the equality the same request plans at ratio 0.9). The
        // final state keeps its circle pose prior, its rest-velocity
        // constraint, and the walk end as its initial value.
        std::optional<SelectedCandidate> selected;
        for (std::size_t branch = 0; branch < terminals.size(); ++branch) {
            if (!candidates[branch])
                continue;
            const PathCandidate& entry = *candidates[branch];
            const double duration_s = entry.assembled.total_duration_s;
            const auto solve_start = std::chrono::steady_clock::now();
            CandidateEvidence evidence = MakeEvidence(
                terminals[branch], branch, duration_s,
                config.optimizer.collision_sigma, requested_terminal);
            TrajectoryResult trajectory;
            try {
                const PathIkSample& walk_end = entry.walk.samples.back();
                const JointConfiguration& terminal_configuration =
                    walk_end.solved ? walk_end.configuration
                                    : terminals[branch].ik.configuration;
                gtsam::Values initial_values;
                const std::size_t states = entry.assembled.waypoints.size();
                for (std::size_t i = 0; i < states; ++i) {
                    const JointConfiguration& q =
                        i + 1 == states ? terminal_configuration
                                        : entry.assembled.initial_configurations[i];
                    initial_values.insert(gtsam::Symbol('x', i), gtsam::Vector(q));
                    gtsam::Vector velocity = gtsam::Vector::Zero(7);
                    if (i + 1 < states) {
                        const double dt = entry.assembled.waypoints[i + 1].time_s -
                                          entry.assembled.waypoints[i].time_s;
                        const JointConfiguration& next_q =
                            i + 2 == states
                                ? terminal_configuration
                                : entry.assembled.initial_configurations[i + 1];
                        velocity = gtsam::Vector((next_q - q) / dt);
                    }
                    initial_values.insert(gtsam::Symbol('v', i), velocity);
                }
                trajectory = optimizer.optimizeTaskTrajectory(
                    *model.arm_model, obstacle_fields, initial_values,
                    entry.assembled.waypoints, gtsam::Vector(q_start_rad), start_vel,
                    /*terminal_config=*/std::nullopt,
                    entry.assembled.zero_velocity_indices, pos_limits, vel_limits,
                    duration_s, config.optimizer, config.optimizer.collision_sigma,
                    config.optimizer.collision_sigma);
            } catch (const std::exception& exception) {
                evidence.solve_time_s =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                  solve_start)
                        .count();
                evidence.disposition = "optimizer_exception";
                outcome.candidate_attempts.push_back(std::move(evidence));
                outcome.failure_reason = exception.what();
                continue;
            }
            const auto solve_end = std::chrono::steady_clock::now();
            trajectory.optimization_duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(solve_end - solve_start);
            evidence.solve_time_s =
                std::chrono::duration<double>(solve_end - solve_start).count();
            if (trajectory.trajectory_pos.empty()) {
                evidence.disposition = "empty_trajectory";
                outcome.candidate_attempts.push_back(std::move(evidence));
                outcome.failure_reason = "optimizer returned an empty trajectory";
                continue;
            }
            CopyOptimizerEvidence(trajectory, evidence);
            const PlanValidationReport report = validate(trajectory, duration_s, branch);
            evidence.validation = report;
            evidence.capped_clearance_m =
                std::min(report.minimum_scene_clearance_m,
                         config.optimizer.preferred_clearance_m);
            const bool valid = GeometricallyValid(report);
            evidence.disposition =
                valid ? "geometry_valid" : GeometryFailureReason(report);
            outcome.candidate_attempts.push_back(std::move(evidence));
            if (!valid) {
                outcome.failure_reason = GeometryFailureReason(report);
                continue;
            }
            // Ranking is physical: tightest dense executed-path fidelity,
            // then the optimiser's own final cost as a tie-break.
            const bool better =
                !selected ||
                report.trace_dense_max_position_m <
                    selected->validation.trace_dense_max_position_m ||
                (report.trace_dense_max_position_m ==
                     selected->validation.trace_dense_max_position_m &&
                 trajectory.final_error < selected->trajectory.final_error);
            if (better) {
                SelectedCandidate candidate_solution;
                candidate_solution.trajectory = std::move(trajectory);
                candidate_solution.validation = report;
                candidate_solution.terminal = terminals[branch];
                candidate_solution.terminal_branch = branch;
                candidate_solution.duration_s = duration_s;
                candidate_solution.evidence_index =
                    outcome.candidate_attempts.size() - 1;
                selected = std::move(candidate_solution);
            }
        }

        if (selected &&
            FinishSelected(*selected, validate, outcome.candidate_attempts,
                           outcome.failure_reason)) {
            const PathCandidate& winner = *candidates[selected->terminal_branch];
            const double task_start_fraction =
                winner.assembled.waypoints[winner.assembled.task_start_index].time_s /
                winner.assembled.total_duration_s;
            outcome.status = PlanStatus::kReached;
            outcome.trajectory = std::move(selected->trajectory);
            outcome.validation = selected->validation;
            outcome.total_time_sec = selected->duration_s;
            outcome.terminal_candidate = selected->terminal.ik;
            outcome.selected_candidate_attempt = selected->evidence_index;
            outcome.task_start_time_s =
                selected->duration_s * task_start_fraction;
            outcome.ik_walk = winner.walk;
            update_walk_summary(winner.walk);
        } else {
            outcome.status = PlanStatus::kFailed;
            outcome.trajectory.reset();
            if (outcome.failure_reason.empty())
                outcome.failure_reason = terminals.empty()
                                             ? "goal_ik_failure"
                                             : "no executable traced trajectory";
        }
    } catch (const std::exception& exception) {
        outcome.failure_reason = exception.what();
        outcome.status = PlanStatus::kFailed;
        outcome.trajectory.reset();
    }
    return outcome;
}
