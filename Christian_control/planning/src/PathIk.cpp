#include "PathIk.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {

constexpr std::array<double, kPathIkAlternativeSeedCount> kNullspaceStepsRad =
    {0.05, -0.05, 0.10, -0.10};

// Radians from the nearest bounded joint's limit. Continuous joints
// (1/3/5/7 on a Gen3) carry ±1e20 sentinels and are skipped: they have no
// stop to be near. Negative when a joint is already outside its range.
double JointLimitMargin(const Eigen::Matrix<double, 7, 1>& q) {
    double worst = std::numeric_limits<double>::infinity();
    bool any_bounded = false;
    for (int j = 0; j < 7; ++j) {
        const double lo = analytical_ik::KinovaGen3Params::JOINT_LIMITS_LOWER[j];
        const double hi = analytical_ik::KinovaGen3Params::JOINT_LIMITS_UPPER[j];
        if (lo < -1e10 || hi > 1e10) continue;
        any_bounded = true;
        // Principal value: the solver may return an angle wound past ±pi,
        // and the limit is on the physical joint position.
        const double angle = std::remainder(q(j), 2.0 * M_PI);
        worst = std::min(worst, std::min(angle - lo, hi - angle));
    }
    return any_bounded ? worst : 0.0;
}

// Smallest singular value of the 6x7 task Jacobian — how far this
// configuration is from a singularity.
double Manipulability(const Eigen::Matrix<double, 7, 1>& q, const PathIkArm& arm) {
    const auto pose_jacobian =
        pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
            q, arm.end_effector_frame, arm.left_arm);
    return pose_jacobian.jacobian.jacobiSvd().singularValues().tail<1>()(0);
}

// The right-singular vector associated with the smallest singular value is
// the one-dimensional redundant direction of this 7-DoF/6-DoF task.
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
    // SVD signs are arbitrary. Fix one sign so the alternative sequence is
    // reproducible; both signs are still explicitly tried below.
    for (int joint = 0; joint < 7; ++joint) {
        if (std::abs(direction(joint)) > 1e-12) {
            if (direction(joint) < 0.0) direction = -direction;
            break;
        }
    }
    return direction;
}

Eigen::Matrix<double, 7, 1> BoundedNullspacePerturbation(
    const Eigen::Matrix<double, 7, 1>& q,
    const Eigen::Matrix<double, 7, 1>& direction,
    double step_rad) {
    Eigen::Matrix<double, 7, 1> bounded_base = q;
    for (int joint = 0; joint < 7; ++joint) {
        const double lower = analytical_ik::KinovaGen3Params::JOINT_LIMITS_LOWER[joint];
        const double upper = analytical_ik::KinovaGen3Params::JOINT_LIMITS_UPPER[joint];
        if (lower < -1e10 || upper > 1e10) continue;
        bounded_base(joint) = std::max(lower, std::min(upper, bounded_base(joint)));
    }
    double scale = 0.95;  // leave a small interior margin at bounded joints
    for (int joint = 0; joint < 7; ++joint) {
        const double lower = analytical_ik::KinovaGen3Params::JOINT_LIMITS_LOWER[joint];
        const double upper = analytical_ik::KinovaGen3Params::JOINT_LIMITS_UPPER[joint];
        if (lower < -1e10 || upper > 1e10) continue;
        const double delta = step_rad * direction(joint);
        if (delta > 0.0)
            scale = std::min(scale, (upper - bounded_base(joint)) / delta);
        else if (delta < 0.0)
            scale = std::min(scale, (lower - bounded_base(joint)) / delta);
    }
    scale = std::max(0.0, scale);
    return bounded_base + scale * step_rad * direction;
}

double InterpolateJoint(double before, double after, double fraction, int joint) {
    const double lower = analytical_ik::KinovaGen3Params::JOINT_LIMITS_LOWER[joint];
    const double upper = analytical_ik::KinovaGen3Params::JOINT_LIMITS_UPPER[joint];
    if (lower < -1e10 || upper > 1e10) {
        // Continuous Gen3 joints take the shortest angular arc rather than
        // the arithmetic path across a 2*pi seam.
        return before + fraction * std::remainder(after - before, 2.0 * M_PI);
    }
    return before + fraction * (after - before);
}

Eigen::Matrix<double, 7, 1> InterpolateConfiguration(
    const Eigen::Matrix<double, 7, 1>& before,
    const Eigen::Matrix<double, 7, 1>& after,
    double fraction) {
    Eigen::Matrix<double, 7, 1> configuration;
    for (int joint = 0; joint < 7; ++joint)
        configuration(joint) = InterpolateJoint(before(joint), after(joint),
                                                fraction, joint);
    return configuration;
}

void ResolveUnresolvedSamples(PathIkResult& result, const PathIkArm& arm) {
    result.success = true;
    result.unresolved_samples = 0;
    result.interpolated_samples = 0;
    result.maximum_unresolved_run = 0;

    std::size_t index = 0;
    while (index < result.samples.size()) {
        if (result.samples[index].solved) {
            ++index;
            continue;
        }
        const std::size_t first = index;
        while (index < result.samples.size() && !result.samples[index].solved)
            ++index;
        const std::size_t last_exclusive = index;
        const std::size_t length = last_exclusive - first;
        result.unresolved_samples += length;
        result.maximum_unresolved_run =
            std::max(result.maximum_unresolved_run, length);
        if (first == 0 || last_exclusive == result.samples.size() ||
            length > kMaxInterpolatedPathIkGapSamples) {
            result.success = false;
            continue;
        }

        const auto& before = result.samples[first - 1].configuration;
        const auto& after = result.samples[last_exclusive].configuration;
        for (std::size_t sample_index = first; sample_index < last_exclusive;
             ++sample_index) {
            const double fraction =
                static_cast<double>(sample_index - first + 1) /
                static_cast<double>(last_exclusive - first + 1);
            PathIkSample& sample = result.samples[sample_index];
            sample.configuration =
                InterpolateConfiguration(before, after, fraction);
            sample.minimum_joint_limit_margin_rad =
                JointLimitMargin(sample.configuration);
            sample.minimum_manipulability = Manipulability(sample.configuration, arm);
            sample.seed_interpolated = true;
            ++result.interpolated_samples;
        }
    }
    if (result.unresolved_samples == 0)
        result.success = true;
}

}  // namespace

PathIkResult SolvePathIk(const CartesianPath& path, const PathIkArm& arm,
                         const Eigen::Matrix<double, 7, 1>& seed,
                         const analytical_ik::IKTolerance& tolerance,
                         bool closed) {
    PathIkResult result;
    result.samples.reserve(path.samples.size());
    if (path.samples.empty()) return result;

    Eigen::Matrix<double, 7, 1> current_seed = seed;
    bool have_previous = false;
    Eigen::Matrix<double, 7, 1> previous_solved = seed;

    for (std::size_t i = 0; i < path.samples.size(); ++i) {
        const Eigen::Matrix4d target = path.samples[i].pose.matrix();

        // One deterministic continuation attempt. Passing one attempt means
        // AnalyticalIKSolver uses exactly the provided seed and does not draw
        // its own random alternatives.
        analytical_ik::IKSolution best =
            analytical_ik::AnalyticalIKSolver::solveBestIK(
                target, arm.base_transform, current_seed, 1,
                arm.end_effector_frame, arm.left_arm, tolerance);

        PathIkSample sample;
        if (!best.is_valid) {
            const Eigen::Matrix<double, 7, 1> direction =
                NullspaceDirection(arm, current_seed);
            for (int alternative = 0;
                 alternative < kPathIkAlternativeSeedCount; ++alternative) {
                const Eigen::Matrix<double, 7, 1> alternative_seed =
                    BoundedNullspacePerturbation(
                        current_seed, direction,
                        kNullspaceStepsRad[static_cast<std::size_t>(alternative)]);
                ++sample.alternative_seed_attempts;
                const auto candidate =
                    analytical_ik::AnalyticalIKSolver::solveBestIK(
                        target, arm.base_transform, alternative_seed, 1,
                        arm.end_effector_frame, arm.left_arm, tolerance);
                if (candidate.is_valid) {
                    best = candidate;
                    break;
                }
                if (candidate.attempted &&
                    candidate.position_error_m + candidate.orientation_error_rad <
                        best.position_error_m + best.orientation_error_rad)
                    best = candidate;
            }
        }

        sample.solved = best.is_valid;
        if (best.attempted) {
            sample.configuration = best.joint_angles;
            sample.position_residual_m = best.position_error_m;
            sample.orientation_residual_rad = best.orientation_error_rad;
            sample.minimum_joint_limit_margin_rad = JointLimitMargin(best.joint_angles);
            sample.minimum_manipulability = Manipulability(best.joint_angles, arm);
        }
        result.samples.push_back(sample);

        if (best.is_valid) {
            if (have_previous) {
                const double step =
                    (best.joint_angles - previous_solved).cwiseAbs().maxCoeff();
                result.maximum_joint_step_rad =
                    std::max(result.maximum_joint_step_rad, step);
            }
            // Continuation: the next sample walks from here. A failed sample
            // leaves the seed alone and is resolved after the forward pass.
            current_seed = best.joint_angles;
            previous_solved = best.joint_angles;
            have_previous = true;
        } else if (!result.failed_sample) {
            result.failed_sample = i;
        }
    }

    ResolveUnresolvedSamples(result, arm);

    // Closure drift: the tool pose returns to the start exactly, but the
    // redundant elbow need not. A large drift means a second lap would not
    // repeat the first.
    if (closed && result.samples.size() >= 2 && result.samples.front().solved &&
        result.samples.back().solved) {
        result.closure_drift_rad = (result.samples.back().configuration -
                                    result.samples.front().configuration)
                                       .cwiseAbs()
                                       .maxCoeff();
    }
    return result;
}
