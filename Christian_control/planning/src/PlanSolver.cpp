#include "PlanSolver.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <tuple>
#include <utility>

#include "MountSdf.h"
#include "PathAssembly.h"
#include "PathIk.h"
#include "TrajectoryOptimization.h"
#include "ValidatePlan.h"

namespace
{

    constexpr int kMaximumDurationAttempts = 3;

    struct TerminalTarget {
        gtsam::Pose3 pose;
        std::string source = "requested";
        std::size_t ordinal = 0;
        double fraction = 1.0;
        std::string blocker_id;
    };

    struct SearchTerminal {
        TerminalIkCandidate ik;
        TerminalTarget target;
        gtsam::Pose3 candidate_pose;
        PlanStatus kind = PlanStatus::kReached;
        double requested_position_shortfall_m = 0.0;
        double requested_orientation_shortfall_rad = 0.0;
        int orientation_tier = 1;
    };

    struct RouteSolution {
        TrajectoryResult trajectory;
        PlanValidationReport validation;
        SearchTerminal terminal;
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

    template <typename Outcome>
    void ApplyRouteSolution(RouteSolution& solution, Outcome& outcome)
    {
        outcome.status = solution.terminal.kind;
        outcome.trajectory = std::move(solution.trajectory);
        outcome.validation = solution.validation;
        outcome.total_time_sec = solution.duration_s;
        outcome.terminal_candidate = solution.terminal.ik;
        outcome.selected_candidate_attempt = solution.evidence_index;
        outcome.candidate_attempts[solution.evidence_index].disposition =
            "best_validated_bounded_candidate";
        outcome.failure_reason.clear();
    }

    CandidateEvidence MakeEvidence(const SearchTerminal& terminal,
                                   std::size_t terminal_branch,
                                   RouteHypothesis route, int duration_attempt,
                                   double duration_s, double scene_sigma)
    {
        CandidateEvidence evidence;
        evidence.terminal_kind = terminal.kind;
        evidence.target_source = terminal.target.source;
        evidence.target_ordinal = terminal.target.ordinal;
        evidence.target_fraction = terminal.target.fraction;
        evidence.target_position_mount_m = terminal.target.pose.translation();
        evidence.target_orientation_mount =
            Eigen::Quaterniond(terminal.target.pose.rotation().matrix());
        evidence.blocker_id = terminal.target.blocker_id;
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
        evidence.route = route;
        evidence.duration_attempt = duration_attempt;
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

    template <typename Solve, typename Validate>
    DurationSearchResult
    SolveDurationCandidate(const SearchTerminal& terminal, std::size_t terminal_branch,
                           RouteHypothesis route, double base_duration_s, double scene_sigma,
                           double preferred_clearance_m, Solve solve, Validate validate,
                           std::vector<CandidateEvidence>& attempts, std::string& failure_reason)
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
                CandidateEvidence evidence = MakeEvidence(
                    terminal, terminal_branch, route, attempt, duration_s, scene_sigma);
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

            CandidateEvidence evidence = MakeEvidence(
                terminal, terminal_branch, route, attempt, duration_s, scene_sigma);
            evidence.solve_time_s = std::chrono::duration<double>(solve_end - solve_start).count();
            if (trajectory.trajectory_pos.empty()) {
                evidence.disposition = "empty_trajectory";
                attempts.push_back(std::move(evidence));
                failure_reason = "optimizer returned an empty trajectory";
                return result;
            }
            CopyOptimizerEvidence(trajectory, evidence);

            result.last_validation = validate(trajectory, duration_s);
            evidence.validation = result.last_validation;
            evidence.capped_clearance_m =
                std::min(result.last_validation.minimum_scene_clearance_m,
                         preferred_clearance_m);
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

    void RecordSeedFailure(const SearchTerminal& terminal, std::size_t terminal_branch,
                           RouteHypothesis route, double scene_sigma,
                           std::vector<CandidateEvidence>& attempts, std::string& failure_reason)
    {
        CandidateEvidence evidence =
            MakeEvidence(terminal, terminal_branch, route, 0, 0.0, scene_sigma);
        evidence.stage = "route_seed";
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

    template <typename Outcome>
    bool MeasuredStartCollision(
        const PlannerModel& model,
        const Eigen::Matrix<double, 7, 1>& q_start_rad,
        const std::vector<NamedObstacleField>& obstacle_fields,
        double minimum_clearance_m, Outcome& outcome)
    {
        outcome.validation = MeasureConfigurationClearance(
            model, q_start_rad, obstacle_fields, minimum_clearance_m);
        if (!outcome.validation.scene_valid) {
            const SceneViolationEvidence& violation =
                outcome.validation.first_scene_violations.front();
            outcome.failure_reason =
                "prohibited_start_collision object=" + violation.object_id +
                " sphere=" + std::to_string(violation.sphere_index);
            return true;
        }
        if (!outcome.validation.self_collision_valid) {
            outcome.failure_reason =
                "prohibited_start_self_collision spheres=" +
                std::to_string(outcome.validation.worst_self_first_sphere) +
                "/" +
                std::to_string(outcome.validation.worst_self_second_sphere);
            return true;
        }
        return false;
    }

    std::optional<JointConfiguration>
    SolveBypassMidpointIk(const PathIkArm& arm, const PathIkJointLimits& limits,
                          const PlannerModel& model, const SceneViolationEvidence& collision,
                          const Eigen::Vector3d& displacement_mount,
                          std::uint64_t effective_ik_seed)
    {
        const gtsam::Pose3 collision_pose =
            ToolPoseInMount(model, collision.q);
        Eigen::Isometry3d target(collision_pose.matrix());
        target.translation() += displacement_mount;

        const auto candidates = SolveTerminalIkCandidates(
            arm, target, collision.q, limits,
            effective_ik_seed, 1);
        if (candidates.empty())
            return std::nullopt;
        return candidates.front().configuration;
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

    Eigen::Vector3d BypassDisplacement(const SceneViolationEvidence& collision,
                                       double preferred_clearance_m, RouteHypothesis route)
    {
        const double lift = preferred_clearance_m - collision.clearance_m;
        const Eigen::Vector3d normal = collision.outward_normal_mount.normalized();
        const Eigen::Vector3d tangent = DeterministicTangent(normal);
        const double tangent_sign = route == RouteHypothesis::kPositiveBypass ? 1.0 : -1.0;
        return lift * normal + tangent_sign * lift * tangent;
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
                                      const TerminalTarget& target,
                                      const gtsam::Pose3& requested_pose,
                                      PlanStatus kind)
    {
        SearchTerminal terminal;
        terminal.ik = ik;
        terminal.target = target;
        terminal.candidate_pose = ToolPoseInMount(model, ik.configuration);
        terminal.kind = kind;
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
        const TerminalTarget& target, const gtsam::Pose3& requested_pose,
        PlanStatus kind,
        const std::vector<NamedObstacleField>* obstacle_fields,
        double minimum_clearance_m,
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
            const TerminalIkCandidate& ik = *best;
            const SearchTerminal terminal = MakeSearchTerminal(
                model, ik, target, requested_pose, kind);
            CandidateEvidence evidence = MakeEvidence(
                terminal, 0, RouteHypothesis::kNormal, 0, 0.0, 0.0);
            evidence.stage = "terminal_ik";
            evidence.terminal_ik_attempt_count = attempt_count;
            if (!ik.planner_limit_legal)
                evidence.disposition = "planner_limit_illegal";
            else if (ik.exact)
                evidence.disposition = "exact_pose_candidate";
            else
                evidence.disposition = "near_miss_candidate";
            if (obstacle_fields && ik.planner_limit_legal) {
                evidence.validation = MeasureConfigurationClearance(
                    model, ik.configuration, *obstacle_fields,
                    minimum_clearance_m);
                if (!evidence.validation.scene_valid)
                    evidence.disposition = "terminal_scene_clearance";
                else if (!evidence.validation.self_collision_valid)
                    evidence.disposition = "terminal_self_clearance";
            }
            evidence_rows.push_back(std::move(evidence));
        }
    }

    std::vector<SearchTerminal> GenerateExactTerminals(
        const PlannerModel& model, const PathIkArm& arm,
        const PathIkJointLimits& limits,
        const Eigen::Matrix<double, 7, 1>& measured_q,
        const gtsam::Pose3& requested_pose, std::uint64_t effective_seed,
        std::vector<CandidateEvidence>& evidence_rows)
    {
        TerminalTarget target;
        target.pose = requested_pose;
        std::vector<TerminalIkCandidate> attempts;
        const auto retained = SolveTerminalIkCandidates(
            arm, Eigen::Isometry3d(requested_pose.matrix()), measured_q,
            limits, effective_seed, 3, &attempts);
        RecordTerminalIkSummaries(
            model, attempts, target, requested_pose, PlanStatus::kReached,
            nullptr, 0.0, evidence_rows);
        std::vector<SearchTerminal> terminals;
        for (const TerminalIkCandidate& ik : retained)
            terminals.push_back(MakeSearchTerminal(
                model, ik, target, requested_pose, PlanStatus::kReached));
        return terminals;
    }

    void AppendDistinctBlockers(const PlanValidationReport& report,
                                std::vector<SceneViolationEvidence>& blockers)
    {
        for (const SceneViolationEvidence& violation :
             report.first_scene_violations) {
            const bool seen = std::any_of(
                blockers.begin(), blockers.end(),
                [&](const SceneViolationEvidence& blocker) {
                    return blocker.object_id == violation.object_id;
                });
            if (!seen)
                blockers.push_back(violation);
        }
    }

    bool SameTarget(const TerminalTarget& first, const TerminalTarget& second)
    {
        return (first.pose.translation() - second.pose.translation()).norm() <=
                   1e-6 &&
               OrientationError(first.pose, second.pose) <= 1e-9;
    }

    std::vector<TerminalTarget> GenerateShortenedTargets(
        const PlannerModel& model, const gtsam::Pose3& start_pose,
        const gtsam::Pose3& requested_pose,
        const std::vector<SceneViolationEvidence>& blockers,
        double minimum_clearance_m)
    {
        std::vector<TerminalTarget> targets;
        std::size_t ordinal = 0;
        for (int numerator = 16; numerator >= 1; --numerator) {
            const double fraction = static_cast<double>(numerator) / 17.0;
            TerminalTarget target;
            target.pose = gtsam::Pose3(
                requested_pose.rotation(),
                gtsam::Point3(start_pose.translation() +
                              fraction * (requested_pose.translation() -
                                          start_pose.translation())));
            target.source = "line_fraction";
            target.ordinal = ordinal++;
            target.fraction = fraction;
            targets.push_back(std::move(target));
        }
        for (const SceneViolationEvidence& blocker : blockers) {
            const Eigen::Vector3d violating_position =
                ToolPoseInMount(model, blocker.q).translation();
            const Eigen::Vector3d normal =
                blocker.outward_normal_mount.normalized();
            const Eigen::Vector3d tangent = DeterministicTangent(normal);
            const double lift =
                std::max(0.0, minimum_clearance_m - blocker.clearance_m);
            const std::array<std::pair<std::string, Eigen::Vector3d>, 3>
                offsets{{{"outward_projection", (lift * normal).eval()},
                         {"positive_tangent", (lift * (normal + tangent)).eval()},
                         {"negative_tangent", (lift * (normal - tangent)).eval()}}};
            for (const auto& source_and_offset : offsets) {
                TerminalTarget target;
                target.pose = gtsam::Pose3(
                    requested_pose.rotation(),
                    gtsam::Point3(violating_position +
                                  source_and_offset.second));
                target.source = source_and_offset.first;
                target.ordinal = ordinal++;
                target.fraction = 0.0;
                target.blocker_id = blocker.object_id;
                if (std::none_of(targets.begin(), targets.end(),
                                 [&](const TerminalTarget& retained) {
                                     return SameTarget(retained, target);
                                 }))
                    targets.push_back(std::move(target));
            }
        }
        return targets;
    }

    std::vector<SearchTerminal> GenerateShortenedTerminals(
        const PlannerModel& model, const PathIkArm& arm,
        const PathIkJointLimits& limits,
        const Eigen::Matrix<double, 7, 1>& measured_q,
        const gtsam::Pose3& start_pose, const gtsam::Pose3& requested_pose,
        const std::vector<SceneViolationEvidence>& blockers,
        const std::vector<NamedObstacleField>& obstacle_fields,
        double minimum_clearance_m,
        std::uint64_t effective_seed,
        std::vector<CandidateEvidence>& evidence_rows)
    {
        const auto targets = GenerateShortenedTargets(
            model, start_pose, requested_pose, blockers, minimum_clearance_m);
        std::vector<SearchTerminal> exact_orientation;
        std::vector<SearchTerminal> relaxed_orientation;
        for (const TerminalTarget& target : targets) {
            std::vector<TerminalIkCandidate> attempts;
            (void)SolveTerminalIkCandidates(
                arm, Eigen::Isometry3d(target.pose.matrix()), measured_q,
                limits, effective_seed, 3, &attempts);
            for (const TerminalIkCandidate& ik : attempts) {
                if (!ik.planner_limit_legal)
                    continue;
                const PlanValidationReport terminal_clearance =
                    MeasureConfigurationClearance(
                        model, ik.configuration, obstacle_fields,
                        minimum_clearance_m);
                if (!terminal_clearance.scene_valid)
                    continue;
                if (!terminal_clearance.self_collision_valid)
                    continue;
                SearchTerminal terminal = MakeSearchTerminal(
                    model, ik, target, requested_pose,
                    PlanStatus::kGoalBlocked);
                if (terminal.orientation_tier == 1)
                    exact_orientation.push_back(std::move(terminal));
                else
                    relaxed_orientation.push_back(std::move(terminal));
            }
            RecordTerminalIkSummaries(
                model, attempts, target, requested_pose,
                PlanStatus::kGoalBlocked, &obstacle_fields,
                minimum_clearance_m, evidence_rows);
        }

        std::vector<SearchTerminal>& pool =
            exact_orientation.empty() ? relaxed_orientation : exact_orientation;
        std::sort(pool.begin(), pool.end(), [](const SearchTerminal& first,
                                               const SearchTerminal& second) {
            if (first.orientation_tier != second.orientation_tier)
                return first.orientation_tier < second.orientation_tier;
            if (first.orientation_tier == 2 &&
                first.requested_orientation_shortfall_rad !=
                    second.requested_orientation_shortfall_rad)
                return first.requested_orientation_shortfall_rad <
                       second.requested_orientation_shortfall_rad;
            if (first.requested_position_shortfall_m !=
                second.requested_position_shortfall_m)
                return first.requested_position_shortfall_m <
                       second.requested_position_shortfall_m;
            if (first.target.ordinal != second.target.ordinal)
                return first.target.ordinal < second.target.ordinal;
            if (first.ik.stream_id != second.ik.stream_id)
                return first.ik.stream_id < second.ik.stream_id;
            return first.ik.attempt_index < second.ik.attempt_index;
        });

        std::vector<SearchTerminal> retained;
        for (SearchTerminal& candidate : pool) {
            const bool duplicate = std::any_of(
                retained.begin(), retained.end(),
                [&](const SearchTerminal& existing) {
                    Eigen::Matrix<double, 7, 1> difference =
                        candidate.ik.configuration - existing.ik.configuration;
                    for (int joint = 0; joint < 7; ++joint)
                        difference(joint) =
                            std::remainder(difference(joint), 2.0 * M_PI);
                    return difference.cwiseAbs().maxCoeff() <= 1e-3;
                });
            if (duplicate)
                continue;
            retained.push_back(std::move(candidate));
            if (retained.size() == 3)
                break;
        }
        return retained;
    }

    CartesianPath TranslateTraceSeedToTerminal(
        const CartesianPath& requested_path,
        const gtsam::Pose3& candidate_terminal)
    {
        CartesianPath seed_path = requested_path;
        if (seed_path.samples.empty())
            return seed_path;
        const Eigen::Vector3d displacement =
            candidate_terminal.translation() -
            requested_path.samples.back().pose.translation();
        for (std::size_t index = 0; index < seed_path.samples.size(); ++index) {
            const double u = seed_path.samples.size() > 1
                                 ? static_cast<double>(index) /
                                       static_cast<double>(seed_path.samples.size() - 1)
                                 : 1.0;
            seed_path.samples[index].pose.translation() += u * displacement;
        }
        return seed_path;
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
        if (MeasuredStartCollision(
                model, request.q_start_rad, obstacle_fields,
                config.minimum_clearance_m, outcome)) {
            return outcome;
        }
        std::optional<gtsam::Vector> start_vel;
        if (request.qdot_start_rad_s)
            start_vel = gtsam::Vector(*request.qdot_start_rad_s);
        OptimizeTrajectory optimizer;

        std::vector<SceneViolationEvidence> exact_blockers;
        const auto search_phase = [&](const std::vector<SearchTerminal>& terminals,
                                      bool collect_blockers)
            -> std::optional<RouteSolution> {
          for (std::size_t branch = 0; branch < terminals.size(); ++branch) {
            const SearchTerminal& terminal = terminals[branch];
            const auto solve_route = [&](RouteHypothesis route,
                                         const std::vector<JointConfiguration>& seed) {
                const double scene_sigma = route == RouteHypothesis::kNormal
                                               ? config.optimizer.collision_sigma
                                               : 0.1 * config.optimizer.collision_sigma;
                const double distance_m =
                    (terminal.candidate_pose.translation() -
                     start_pose.translation())
                        .norm();
                const double base_duration_s = std::max(
                    config.motion.min_duration_s,
                    distance_m / config.motion.nominal_speed_mps);
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
                        gtsam::Vector(terminal.ik.configuration), pos_limits, vel_limits,
                        support_intervals, duration_s, config.optimizer, scene_sigma,
                        config.optimizer.collision_sigma);
                };
                const auto validate = [&](const TrajectoryResult& trajectory, double duration_s) {
                    PlanValidationInputs inputs =
                        ValidationInputs(request.q_start_rad, request.qdot_start_rad_s, limits,
                                         obstacle_fields, config.minimum_clearance_m, goal_pose,
                                         config.path_following.validation_dt_s);
                    inputs.candidate_terminal_mount = terminal.candidate_pose;
                    inputs.intended_status = terminal.kind;
                    return ValidatePlan(model, trajectory, duration_s, inputs);
                };
                auto solved = SolveDurationCandidate(
                    terminal, branch, route, base_duration_s, scene_sigma,
                    config.optimizer.preferred_clearance_m, solve, validate,
                    outcome.candidate_attempts, outcome.failure_reason);
                if (collect_blockers)
                    AppendDistinctBlockers(solved.last_validation,
                                           exact_blockers);
                return solved;
            };

            const auto normal_seed =
                PointBypassSeed(request.q_start_rad, terminal.ik.configuration,
                                support_intervals + 1);
            auto normal = solve_route(RouteHypothesis::kNormal, normal_seed);
            const PlanValidationReport& normal_report = normal.last_validation;
            const double normal_duration_s = normal.last_duration_s;
            std::optional<RouteSolution> branch_winner = std::move(normal.executable);

            if (!branch_winner && !normal_report.first_scene_violations.empty()) {
                const SceneViolationEvidence& first =
                    normal_report.first_scene_violations.front();
                if (first.time_s < normal_duration_s) {
                    const double fraction =
                        first.time_s / normal_duration_s;
                    for (const RouteHypothesis route :
                         {RouteHypothesis::kPositiveBypass, RouteHypothesis::kNegativeBypass}) {
                        const auto midpoint = SolveBypassMidpointIk(
                            arm, ik_limits, model, first,
                            BypassDisplacement(first,
                                               config.optimizer.preferred_clearance_m, route),
                            config.effective_ik_seed);
                        if (!midpoint) {
                            RecordSeedFailure(
                                terminal, branch, route,
                                0.1 * config.optimizer.collision_sigma,
                                outcome.candidate_attempts, outcome.failure_reason);
                            continue;
                        }
                        const auto bypass_seed = PointBypassSeed(
                            request.q_start_rad, terminal.ik.configuration,
                            support_intervals + 1,
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

            if (branch_winner)
                return branch_winner;
          }
          return std::nullopt;
        };

        const auto exact_terminals = GenerateExactTerminals(
            model, arm, ik_limits, request.q_start_rad, goal_pose,
            config.effective_ik_seed, outcome.candidate_attempts);
        std::optional<RouteSolution> selected =
            search_phase(exact_terminals, /*collect_blockers=*/true);
        if (!selected) {
            const auto shortened_terminals = GenerateShortenedTerminals(
                model, arm, ik_limits, request.q_start_rad, start_pose,
                goal_pose, exact_blockers, obstacle_fields,
                config.minimum_clearance_m,
                config.effective_ik_seed, outcome.candidate_attempts);
            selected = search_phase(shortened_terminals,
                                    /*collect_blockers=*/false);
        }

        if (selected) {
            ApplyRouteSolution(*selected, outcome);
            outcome.final_goal_error_m =
                selected->validation.requested_terminal_position_error_m;
        } else {
            outcome.status = PlanStatus::kFailed;
            outcome.trajectory.reset();
            if (outcome.failure_reason.empty())
                outcome.failure_reason = "no executable bounded point trajectory";
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
        const gtsam::Pose3 start_pose = ToolPoseInMount(model, q_start_rad);
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
        if (MeasuredStartCollision(
                model, q_start_rad, obstacle_fields,
                config.minimum_clearance_m, outcome)) {
            return outcome;
        }

        std::vector<SceneViolationEvidence> exact_blockers;
        const auto exact_terminals = GenerateExactTerminals(
            model, arm, ik_limits, q_start_rad, requested_terminal,
            config.effective_ik_seed, outcome.candidate_attempts);

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
        const auto search_phase = [&](const std::vector<SearchTerminal>& terminal_candidates,
                                      bool collect_blockers)
            -> std::optional<RouteSolution> {
          for (std::size_t branch = 0; branch < terminal_candidates.size(); ++branch) {
            const SearchTerminal& terminal = terminal_candidates[branch];
            const CartesianPath seed_path =
                terminal.kind == PlanStatus::kReached
                    ? task_path
                    : TranslateTraceSeedToTerminal(task_path,
                                                   terminal.candidate_pose);
            const PathIkResult normal_walk = SolvePathIk(
                seed_path, arm, q_start_rad, ik_limits, tolerance,
                /*closed=*/terminal.kind == PlanStatus::kReached,
                config.effective_ik_seed);
            outcome.ik_walk = normal_walk;
            update_walk_summary(normal_walk);
            if (!normal_walk.success) {
                RecordSeedFailure(
                    terminal, branch, RouteHypothesis::kNormal,
                    config.optimizer.collision_sigma,
                    outcome.candidate_attempts, outcome.failure_reason);
                continue;
            }
            const AssembledPath normal_assembled = AssembleCirclePlan(
                task_path, configurations_from_walk(normal_walk), q_start_rad,
                joint_velocity_limits, pacing, rotation_sigma, position_sigma);
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
                                                          ? terminal.ik.configuration
                                                          : assembled.initial_configurations[i];
                        initial_values.insert(gtsam::Symbol('x', i), gtsam::Vector(q));
                        gtsam::Vector velocity = gtsam::Vector::Zero(7);
                        if (i + 1 < states) {
                            const double dt =
                                attempt_waypoints[i + 1].time_s - attempt_waypoints[i].time_s;
                            const JointConfiguration& next_q =
                                i + 2 == states ? terminal.ik.configuration
                                                : assembled.initial_configurations[i + 1];
                            velocity = gtsam::Vector((next_q - q) / dt);
                        }
                        initial_values.insert(gtsam::Symbol('v', i), velocity);
                    }
                    return optimizer.optimizeTaskTrajectory(
                        *model.arm_model, obstacle_fields, initial_values, attempt_waypoints,
                        gtsam::Vector(q_start_rad), start_vel,
                        gtsam::Vector(terminal.ik.configuration), assembled.zero_velocity_indices,
                        pos_limits, vel_limits, duration_s, config.optimizer, scene_sigma,
                        config.optimizer.collision_sigma);
                };
                const auto validate = [&](const TrajectoryResult& trajectory, double duration_s) {
                    const double task_start_time_s = duration_s * task_start_fraction;
                    PlanValidationInputs inputs =
                        ValidationInputs(q_start_rad, qdot_start_rad_s, limits, obstacle_fields,
                                         config.minimum_clearance_m, requested_terminal,
                                         config.path_following.validation_dt_s);
                    inputs.candidate_terminal_mount = terminal.candidate_pose;
                    inputs.intended_status = terminal.kind;
                    inputs.desired_task_path = &task_path;
                    inputs.task_start_time_s = task_start_time_s;
                    return ValidatePlan(model, trajectory, duration_s, inputs);
                };
                auto result = SolveDurationCandidate(
                    terminal, branch, route, base_duration_s, scene_sigma,
                    config.optimizer.preferred_clearance_m, solve, validate,
                    outcome.candidate_attempts, outcome.failure_reason);
                if (collect_blockers)
                    AppendDistinctBlockers(result.last_validation,
                                           exact_blockers);
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

            if (!branch_winner && !normal_report.first_scene_violations.empty()) {
                const SceneViolationEvidence& first =
                    normal_report.first_scene_violations.front();
                if (first.time_s < normal_duration_s) {
                    for (const RouteHypothesis route :
                         {RouteHypothesis::kPositiveBypass, RouteHypothesis::kNegativeBypass}) {
                        const Eigen::Vector3d displacement = BypassDisplacement(
                            first, config.optimizer.preferred_clearance_m, route);
                        AssembledPath bypass_assembled;
                        PathIkResult bypass_walk = normal_walk;
                        if (first.time_s < normal_task_start_time_s) {
                            const auto midpoint = SolveBypassMidpointIk(
                                arm, ik_limits, model, first, displacement,
                                config.effective_ik_seed);
                            if (!midpoint) {
                                RecordSeedFailure(
                                    terminal, branch, route,
                                    0.1 * config.optimizer.collision_sigma,
                                    outcome.candidate_attempts, outcome.failure_reason);
                                continue;
                            }
                            bypass_assembled = normal_assembled;
                            const double fraction = first.time_s /
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
                            const double parameter_u = (first.time_s -
                                                        normal_task_start_time_s) /
                                                       task_span_s;
                            const CartesianPath offset_path = TraceBypassSeed(
                                seed_path, parameter_u, displacement,
                                kPathIkAnchorStride);
                            bypass_walk =
                                SolvePathIk(offset_path, arm, q_start_rad, ik_limits, tolerance,
                                            /*closed=*/terminal.kind == PlanStatus::kReached,
                                            config.effective_ik_seed);
                            if (!bypass_walk.success) {
                                RecordSeedFailure(
                                    terminal, branch, route,
                                    0.1 * config.optimizer.collision_sigma,
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

            if (branch_winner)
                return branch_winner;
          }
          return std::nullopt;
        };

        std::optional<RouteSolution> selected =
            search_phase(exact_terminals, /*collect_blockers=*/true);
        if (!selected) {
            const auto shortened_terminals = GenerateShortenedTerminals(
                model, arm, ik_limits, q_start_rad, start_pose,
                requested_terminal, exact_blockers, obstacle_fields,
                config.minimum_clearance_m,
                config.effective_ik_seed,
                outcome.candidate_attempts);
            selected = search_phase(shortened_terminals,
                                    /*collect_blockers=*/false);
        }

        if (selected) {
            ApplyRouteSolution(*selected, outcome);
            outcome.task_start_time_s = selected->task_start_time_s;
            if (selected->has_ik_walk) {
                outcome.ik_walk = selected->ik_walk;
                update_walk_summary(selected->ik_walk);
            }
        }

        if (!outcome.trajectory && outcome.failure_reason.empty())
            outcome.failure_reason = "no executable bounded traced trajectory";
    } catch (const std::exception& exception) {
        outcome.failure_reason = exception.what();
        outcome.status = PlanStatus::kFailed;
        outcome.trajectory.reset();
    }
    return outcome;
}
