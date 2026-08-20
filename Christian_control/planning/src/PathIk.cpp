#include "PathIk.h"

#include <algorithm>
#include <array>
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

void FillGap(PathIkResult& result, std::size_t first, std::size_t length,
             std::size_t before, std::size_t after,
             const PathIkJointLimits& limits) {
    const std::size_t count = result.samples.size();
    for (std::size_t offset = 0; offset < length; ++offset) {
        const std::size_t index = (first + offset) % count;
        const double fraction = static_cast<double>(offset + 1) /
                                static_cast<double>(length + 1);
        result.samples[index].configuration = Interpolate(
            result.samples[before].configuration,
            result.samples[after].configuration, fraction, limits);
        ++result.interpolated_samples;
    }
}

void ResolveGaps(PathIkResult& result, const PathIkJointLimits& limits,
                 bool closed) {
    result.success = true;
    const std::size_t count = result.samples.size();
    std::size_t anchor = 0;
    std::size_t logical = 0;
    if (closed) {
        const auto first_solved = std::find_if(
            result.samples.begin(), result.samples.end(),
            [](const PathIkSample& sample) { return sample.solved; });
        if (first_solved == result.samples.end()) {
            result.unresolved_samples = count;
            result.maximum_unresolved_run = count;
            result.success = false;
            return;
        }
        anchor = static_cast<std::size_t>(
            std::distance(result.samples.begin(), first_solved));
        logical = 1;  // anchor itself is solved
    }
    const auto index_at = [closed, anchor, count](std::size_t position) {
        return closed ? (anchor + position) % count : position;
    };
    while (logical < count) {
        if (result.samples[index_at(logical)].solved) {
            ++logical;
            continue;
        }
        const std::size_t first = logical;
        while (logical < count && !result.samples[index_at(logical)].solved)
            ++logical;
        const std::size_t length = logical - first;
        result.unresolved_samples += length;
        result.maximum_unresolved_run =
            std::max(result.maximum_unresolved_run, length);
        const bool two_sided = closed || (first > 0 && logical < count);
        if (!two_sided || length > kMaxInterpolatedPathIkGapSamples) {
            result.success = false;
            continue;
        }
        FillGap(result, index_at(first), length, index_at(first - 1),
                index_at(logical), limits);
    }
}

}  // namespace

PathIkResult SolvePathIk(const CartesianPath& path, const PathIkArm& arm,
                         const Eigen::Matrix<double, 7, 1>& seed,
                         const PathIkJointLimits& limits,
                         const analytical_ik::IKTolerance& tolerance,
                         bool closed) {
    PathIkResult result;
    result.samples.reserve(path.samples.size());
    if (path.samples.empty()) return result;

    Eigen::Matrix<double, 7, 1> current_seed = CanonicalSeed(seed, limits);
    for (const PathSample& path_sample : path.samples) {
        const Eigen::Matrix4d target = path_sample.pose.matrix();
        analytical_ik::IKSolution best =
            analytical_ik::AnalyticalIKSolver::solveBestIK(
                target, arm.base_transform, current_seed, 1,
                arm.end_effector_frame, arm.left_arm, tolerance);
        bool solved = best.is_valid && WithinLimits(best.joint_angles, limits);

        if (!solved) {
            const Eigen::Matrix<double, 7, 1> direction =
                NullspaceDirection(arm, current_seed);
            for (double step_rad : kNullspaceStepsRad) {
                const Eigen::Matrix<double, 7, 1> alternative_seed =
                    PerturbWithinLimits(current_seed, direction, step_rad, limits);
                const auto candidate =
                    analytical_ik::AnalyticalIKSolver::solveBestIK(
                        target, arm.base_transform, alternative_seed, 1,
                        arm.end_effector_frame, arm.left_arm, tolerance);
                if (!candidate.is_valid ||
                    !WithinLimits(candidate.joint_angles, limits))
                    continue;
                best = candidate;
                solved = true;
                break;
            }
        }

        PathIkSample sample;
        sample.solved = solved;
        if (best.attempted) {
            sample.configuration = best.joint_angles;
            sample.position_residual_m = best.position_error_m;
            sample.orientation_residual_rad = best.orientation_error_rad;
        }
        result.samples.push_back(sample);
        if (solved) current_seed = best.joint_angles;
    }

    ResolveGaps(result, limits, closed);
    if (result.success) {
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
    }
    return result;
}
