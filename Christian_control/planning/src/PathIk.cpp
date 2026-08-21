#include "PathIk.h"

#include <algorithm>
#include <array>
#include <chrono>
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
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(kAnchorIkTimeBudgetS);
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
            return attempts < kMaxAnchorIkAttempts &&
                   std::chrono::steady_clock::now() < deadline;
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
