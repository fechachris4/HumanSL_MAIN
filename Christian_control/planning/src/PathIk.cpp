#include "PathIk.h"

#include <algorithm>
#include <array>
#include <limits>
#include <random>
#include <cmath>
#include <stdexcept>

namespace {

constexpr std::array<double, kPathIkAlternativeSeedCount> kNullspaceStepsRad =
    {0.05, -0.05, 0.10, -0.10};

bool IsContinuous(const PathIkJointLimits& limits, int joint) {
    return limits.lower_rad(joint) < -1e10 || limits.upper_rad(joint) > 1e10;
}

bool WithinLimits(const Eigen::Matrix<double, 7, 1>& q,
                  const PathIkJointLimits& limits) {
    if (!q.allFinite()) return false;
    for (int joint = 0; joint < 7; ++joint)
        if (!IsContinuous(limits, joint) &&
            (q(joint) < limits.lower_rad(joint) ||
             q(joint) > limits.upper_rad(joint)))
            return false;
    return true;
}

Eigen::Matrix<double, 7, 1> CanonicalSeed(
    const Eigen::Matrix<double, 7, 1>& seed,
    const PathIkJointLimits& limits) {
    if (!limits.lower_rad.allFinite() || !limits.upper_rad.allFinite() ||
        (limits.lower_rad.array() >= limits.upper_rad.array()).any())
        throw std::invalid_argument("PathIk received invalid planner joint limits");
    Eigen::Matrix<double, 7, 1> canonical = seed;
    for (int joint = 0; joint < 7; ++joint)
        if (IsContinuous(limits, joint))
            canonical(joint) = std::remainder(canonical(joint), 2.0 * M_PI);
    if (!WithinLimits(canonical, limits))
        throw std::invalid_argument(
            "PathIk start seed is outside the planner joint limits");
    return canonical;
}

Eigen::Matrix<double, 7, 1> JointDifference(
    const Eigen::Matrix<double, 7, 1>& before,
    const Eigen::Matrix<double, 7, 1>& after,
    const PathIkJointLimits& limits) {
    Eigen::Matrix<double, 7, 1> difference = after - before;
    for (int joint = 0; joint < 7; ++joint)
        if (IsContinuous(limits, joint))
            difference(joint) = std::remainder(difference(joint), 2.0 * M_PI);
    return difference;
}

Eigen::Matrix<double, 7, 1> NullspaceDirection(
    const PathIkArm& arm, const Eigen::Matrix<double, 7, 1>& q) {
    const auto pose_jacobian =
        pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
            q, arm.end_effector_frame, arm.left_arm);
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(
        pose_jacobian.jacobian, Eigen::ComputeFullV);
    Eigen::Matrix<double, 7, 1> direction = svd.matrixV().col(6);
    if (!direction.allFinite() || direction.norm() < 1e-12)
        return Eigen::Matrix<double, 7, 1>::Zero();
    direction.normalize();
    Eigen::Index largest = 0;
    direction.cwiseAbs().maxCoeff(&largest);
    return direction(largest) < 0.0 ? -direction : direction;
}

Eigen::Matrix<double, 7, 1> PerturbWithinLimits(
    const Eigen::Matrix<double, 7, 1>& q,
    const Eigen::Matrix<double, 7, 1>& direction,
    double step_rad, const PathIkJointLimits& limits) {
    double scale = 1.0;
    for (int joint = 0; joint < 7; ++joint) {
        if (IsContinuous(limits, joint)) continue;
        const double delta = step_rad * direction(joint);
        if (delta > 0.0)
            scale = std::min(scale, (limits.upper_rad(joint) - q(joint)) / delta);
        else if (delta < 0.0)
            scale = std::min(scale, (limits.lower_rad(joint) - q(joint)) / delta);
    }
    return q + std::max(0.0, scale) * step_rad * direction;
}

Eigen::Matrix<double, 7, 1> Interpolate(
    const Eigen::Matrix<double, 7, 1>& before,
    const Eigen::Matrix<double, 7, 1>& after, double fraction,
    const PathIkJointLimits& limits) {
    return before + fraction * JointDifference(before, after, limits);
}

}  // namespace

PathIkResult SolvePathIk(const CartesianPath& path, const PathIkArm& arm,
                         const Eigen::Matrix<double, 7, 1>& seed,
                         const PathIkJointLimits& limits,
                         const analytical_ik::IKTolerance& tolerance,
                         bool closed, std::uint64_t random_seed) {
    PathIkResult result;
    const std::size_t count = path.samples.size();
    result.samples.resize(count);
    if (count == 0) return result;

    const Eigen::Matrix<double, 7, 1> canonical_start =
        CanonicalSeed(seed, limits);

    // Anchor selection: the entry, then every stride-th sample, plus an
    // open path's final sample (a closed path's last sample repeats its
    // first, so anchoring it would solve the entry twice).
    std::vector<std::size_t> anchors;
    for (std::size_t index = 0; index < count; index += kPathIkAnchorStride)
        anchors.push_back(index);
    if (!closed && anchors.back() != count - 1) anchors.push_back(count - 1);

    // Deterministic multi-start driver: reproducing a failed plan must
    // replay the identical attempt sequence, so the generator is seeded
    // from the run's configuration, never from time.
    std::mt19937_64 generator(random_seed);
    std::uniform_real_distribution<double> perturbation(-0.35, 0.35);

    // The bounded solve for one anchor: continuation seed, structured
    // null-space perturbations, then random multi-start, first hit wins.
    const auto solve_anchor = [&](const Eigen::Matrix4d& target,
                                  const Eigen::Matrix<double, 7, 1>& walk_seed,
                                  PathIkSample& sample) {
        bool converged_outside_limits = false;
        int attempts = 0;
        double best_failed_residual_m = std::numeric_limits<double>::infinity();
        const auto attempt = [&](const Eigen::Matrix<double, 7, 1>& try_seed) {
            ++attempts;
            Eigen::Matrix<double, 7, 1> mutable_seed = try_seed;
            const auto candidate =
                analytical_ik::AnalyticalIKSolver::solveBestIK(
                    target, arm.base_transform, mutable_seed, 1,
                    arm.end_effector_frame, arm.left_arm, tolerance);
            // On failure the sample keeps the CLOSEST attempt seen, so the
            // diagnostics say how near the bounded search ever got.
            if (candidate.attempted && !sample.solved &&
                candidate.position_error_m < best_failed_residual_m) {
                best_failed_residual_m = candidate.position_error_m;
                sample.configuration = candidate.joint_angles;
                sample.position_residual_m = candidate.position_error_m;
                sample.orientation_residual_rad = candidate.orientation_error_rad;
            }
            if (candidate.is_valid &&
                !WithinLimits(candidate.joint_angles, limits)) {
                converged_outside_limits = true;
                return false;
            }
            if (!candidate.is_valid) return false;
            sample.configuration = candidate.joint_angles;
            sample.position_residual_m = candidate.position_error_m;
            sample.orientation_residual_rad = candidate.orientation_error_rad;
            sample.solved = true;
            return true;
        };
        const auto budget_left = [&] {
            return attempts < kMaxAnchorIkAttempts;
        };

        if (attempt(walk_seed)) return;
        const Eigen::Matrix<double, 7, 1> direction =
            NullspaceDirection(arm, walk_seed);
        for (double step_rad : kNullspaceStepsRad) {
            if (!budget_left()) break;
            if (attempt(PerturbWithinLimits(walk_seed, direction, step_rad,
                                            limits)))
                return;
        }
        while (budget_left()) {
            Eigen::Matrix<double, 7, 1> random_seed_q = walk_seed;
            for (int joint = 0; joint < 7; ++joint)
                random_seed_q(joint) += perturbation(generator);
            for (int joint = 0; joint < 7; ++joint) {
                if (IsContinuous(limits, joint)) continue;
                random_seed_q(joint) =
                    std::clamp(random_seed_q(joint), limits.lower_rad(joint),
                               limits.upper_rad(joint));
            }
            if (attempt(random_seed_q)) return;
        }
        sample.failure = converged_outside_limits
                             ? PathIkFailure::kJointLimits
                             : PathIkFailure::kNoConvergence;
    };

    // ---- 1. solve the anchors, continuation-seeded ----------------------
    std::vector<std::size_t> solved_anchors;
    Eigen::Matrix<double, 7, 1> continuation = canonical_start;
    for (const std::size_t index : anchors) {
        PathIkSample& sample = result.samples[index];
        solve_anchor(path.samples[index].pose.matrix(), continuation, sample);
        if (sample.solved) {
            // Angle continuity: a continuous joint's solution is equivalent
            // under whole revolutions, and the solver reports whichever
            // winding its internal range produced. Rewind each continuous
            // joint to the revolution nearest the previous solved sample so
            // the walk — and the optimiser's initial guess built from it —
            // never carries a spurious ±360° step (measured 2026-08-23: a
            // 351° joint-3 step between adjacent circle samples became the
            // acceleration spike duration repair stretched tenfold).
            // Bounded joints stay untouched: their accepted solutions are
            // already inside the physical stops, and rewinding could not be.
            for (int joint = 0; joint < 7; ++joint) {
                if (!IsContinuous(limits, joint)) continue;
                sample.configuration(joint) =
                    continuation(joint) +
                    std::remainder(
                        sample.configuration(joint) - continuation(joint),
                        2.0 * M_PI);
            }
            solved_anchors.push_back(index);
            continuation = sample.configuration;
        } else {
            ++result.unresolved_samples;
        }
    }
    // Longest run of consecutive failed anchors, for the diagnostics.
    std::size_t run = 0;
    for (const std::size_t index : anchors) {
        run = result.samples[index].solved ? 0 : run + 1;
        result.maximum_unresolved_run =
            std::max(result.maximum_unresolved_run, run);
    }

    // Only the entry anchor is required: interpolation can bridge any
    // interior gap, but nothing can interpolate toward an unknown entry.
    if (solved_anchors.empty() || solved_anchors.front() != 0) {
        result.success = false;
        return result;
    }
    result.success = true;

    // ---- 2. interpolate everything between solved anchors ---------------
    const auto fill_between = [&](std::size_t from, std::size_t to,
                                  std::size_t span) {
        // span = number of samples stepped from `from` to `to` (wrapping).
        for (std::size_t offset = 1; offset < span; ++offset) {
            const std::size_t index = (from + offset) % count;
            if (result.samples[index].solved) continue;
            const double fraction =
                static_cast<double>(offset) / static_cast<double>(span);
            result.samples[index].configuration =
                Interpolate(result.samples[from].configuration,
                            result.samples[to].configuration, fraction, limits);
            result.samples[index].interpolated = true;
            ++result.interpolated_samples;
        }
    };
    for (std::size_t pair = 0; pair + 1 < solved_anchors.size(); ++pair)
        fill_between(solved_anchors[pair], solved_anchors[pair + 1],
                     solved_anchors[pair + 1] - solved_anchors[pair]);
    const std::size_t last = solved_anchors.back();
    if (closed) {
        // Wrap the seam back to the entry so a closed path stays closed.
        fill_between(last, 0, count - last);
    } else {
        // Hold the last solved configuration over an unsolved open tail.
        for (std::size_t index = last + 1; index < count; ++index) {
            if (result.samples[index].solved) continue;
            result.samples[index].configuration =
                result.samples[last].configuration;
            result.samples[index].interpolated = true;
            ++result.interpolated_samples;
        }
    }

    // ---- 3. the walk's own quality numbers ------------------------------
    for (std::size_t index = 1; index < result.samples.size(); ++index)
        result.maximum_joint_step_rad = std::max(
            result.maximum_joint_step_rad,
            JointDifference(result.samples[index - 1].configuration,
                            result.samples[index].configuration, limits)
                .cwiseAbs()
                .maxCoeff());
    if (closed && result.samples.size() >= 2)
        result.closure_drift_rad =
            JointDifference(result.samples.front().configuration,
                            result.samples.back().configuration, limits)
                .cwiseAbs()
                .maxCoeff();
    return result;
}

std::vector<TerminalIkCandidate> SolveTerminalIkCandidates(
    const PathIkArm& arm, const Eigen::Isometry3d& target,
    const Eigen::Matrix<double, 7, 1>& measured_q,
    const PathIkJointLimits& limits, std::uint64_t effective_seed,
    std::size_t max_candidates,
    std::vector<TerminalIkCandidate>* attempted_candidates) {
    std::vector<TerminalIkCandidate> candidates;
    if (max_candidates == 0) return candidates;

    const Eigen::Matrix<double, 7, 1> canonical_measured =
        CanonicalSeed(measured_q, limits);
    analytical_ik::IKTolerance tolerance;
    tolerance.converge_position_m = 0.001;
    tolerance.accept_position_m = 0.001;
    tolerance.converge_orientation_rad = 0.01;
    tolerance.accept_orientation_rad = 0.01;

    std::vector<TerminalIkCandidate> pool;
    pool.reserve(kTerminalIkSeedStreams * kTerminalIkAttemptsPerStream);
    if (attempted_candidates)
        attempted_candidates->reserve(
            attempted_candidates->size() +
            kTerminalIkSeedStreams * kTerminalIkAttemptsPerStream);
    for (std::uint64_t stream = 0; stream < kTerminalIkSeedStreams; ++stream) {
        const auto solutions = analytical_ik::AnalyticalIKSolver::solveIK(
            target.matrix(), arm.base_transform, canonical_measured,
            kTerminalIkAttemptsPerStream,
            arm.end_effector_frame, arm.left_arm, tolerance,
            analytical_ik::IKSeeding{effective_seed, stream});
        for (const auto& solution : solutions) {
            if (!solution.attempted || !solution.joint_angles.allFinite() ||
                !std::isfinite(solution.position_error_m) ||
                !std::isfinite(solution.orientation_error_rad))
                continue;
            const Eigen::Matrix<double, 7, 1> configuration =
                canonical_measured +
                JointDifference(canonical_measured, solution.joint_angles,
                                limits);
            const bool legal = WithinLimits(configuration, limits);
            const bool exact = legal && solution.position_error_m <= 0.001 &&
                               solution.orientation_error_rad <= 0.01;
            TerminalIkCandidate candidate{
                configuration, stream, solution.attempt_index,
                solution.position_error_m, solution.orientation_error_rad,
                legal, exact};
            if (attempted_candidates)
                attempted_candidates->push_back(candidate);
            if (exact)
                pool.push_back(candidate);
        }
    }

    const auto displacement = [&](const TerminalIkCandidate& candidate) {
        return JointDifference(canonical_measured, candidate.configuration,
                               limits)
            .norm();
    };
    std::sort(pool.begin(), pool.end(), [&](const auto& first, const auto& second) {
        const double first_distance = displacement(first);
        const double second_distance = displacement(second);
        if (first_distance != second_distance)
            return first_distance < second_distance;
        if (first.stream_id != second.stream_id)
            return first.stream_id < second.stream_id;
        return first.attempt_index < second.attempt_index;
    });

    for (const auto& candidate : pool) {
        const bool duplicate = std::any_of(
            candidates.begin(), candidates.end(), [&](const auto& retained) {
                return JointDifference(retained.configuration,
                                       candidate.configuration, limits)
                           .cwiseAbs()
                           .maxCoeff() <= 1e-3;
            });
        if (duplicate) continue;
        candidates.push_back(candidate);
        if (candidates.size() == max_candidates) break;
    }
    return candidates;
}
