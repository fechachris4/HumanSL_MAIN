#include "PathIk.h"

#include <algorithm>
#include <cmath>

namespace {

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

// How good a STARTING configuration is. Deliberately not IK residual alone:
// a configuration can reach the first pose exactly and still be a bad place
// to begin, because it sits against a joint stop or near a singularity and
// the continuation walk runs out of room a few samples later. Lower is
// better.
double FirstSampleScore(const Eigen::Matrix<double, 7, 1>& q,
                        const Eigen::Matrix<double, 7, 1>& measured,
                        const PathIkArm& arm) {
    // Travel from where the arm actually is — a solution on the far side of
    // the workspace is reachable but means a long approach.
    const double travel = (q - measured).norm();
    const double margin = JointLimitMargin(q);
    const double sigma_min = Manipulability(q, arm);
    // Penalties, not vetoes: a configuration close to a limit or a
    // singularity is ranked worse but still usable if nothing better
    // exists. Refusing outright would hand the whole problem back
    // (CLAUDE.md, graceful degradation).
    const double limit_penalty = margin > 0.0 ? 1.0 / (margin + 0.05) : 100.0;
    const double singularity_penalty =
        sigma_min > 1e-9 ? 0.02 / (sigma_min + 1e-3) : 100.0;
    return travel + limit_penalty + singularity_penalty;
}

}  // namespace

PathIkResult SolvePathIk(const CartesianPath& path, const PathIkArm& arm,
                         const Eigen::Matrix<double, 7, 1>& seed,
                         const analytical_ik::IKTolerance& tolerance,
                         bool closed, int attempts_per_sample,
                         const analytical_ik::IKSeeding& seeding) {
    PathIkResult result;
    result.samples.reserve(path.samples.size());
    if (path.samples.empty()) return result;

    Eigen::Matrix<double, 7, 1> current_seed = seed;
    bool have_previous = false;
    Eigen::Matrix<double, 7, 1> previous_solved = seed;

    for (std::size_t i = 0; i < path.samples.size(); ++i) {
        const Eigen::Matrix4d target = path.samples[i].pose.matrix();

        analytical_ik::IKSolution best;
        if (i == 0) {
            // No predecessor: try several times and rank on more than
            // residual. solveBestIK already draws its own random seeds
            // internally, so repeated calls explore genuinely different
            // branches rather than repeating one answer.
            double best_score = std::numeric_limits<double>::infinity();
            for (int attempt = 0; attempt < 8; ++attempt) {
                // A distinct stream per restart: the eight attempts must
                // explore different branches, but the same eight must recur
                // on the next run of the same request.
                analytical_ik::IKSeeding attempt_seeding = seeding;
                attempt_seeding.stream = seeding.stream * 1000 +
                                         static_cast<std::uint64_t>(attempt);
                const auto candidate = analytical_ik::AnalyticalIKSolver::solveBestIK(
                    target, arm.base_transform, current_seed, attempts_per_sample,
                    arm.end_effector_frame, arm.left_arm, tolerance, attempt_seeding);
                if (!candidate.attempted) continue;
                if (!candidate.is_valid && best.is_valid) continue;
                const double score =
                    FirstSampleScore(candidate.joint_angles, seed, arm);
                // A valid solution always beats an invalid one, whatever
                // the score; among equals, the better score wins.
                if ((candidate.is_valid && !best.is_valid) || score < best_score) {
                    best = candidate;
                    best_score = score;
                }
            }
        } else {
            // One stream per path sample, so sample 5 draws the same
            // restarts every run regardless of what samples 0-4 needed.
            analytical_ik::IKSeeding sample_seeding = seeding;
            sample_seeding.stream =
                seeding.stream * 1000 + 500 + static_cast<std::uint64_t>(i);
            best = analytical_ik::AnalyticalIKSolver::solveBestIK(
                target, arm.base_transform, current_seed, attempts_per_sample,
                arm.end_effector_frame, arm.left_arm, tolerance, sample_seeding);
        }

        PathIkSample sample;
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
            // Continuation: the next sample walks from here. A FAILED
            // sample deliberately leaves the seed alone rather than
            // poisoning the rest of the path with a bad configuration.
            current_seed = best.joint_angles;
            previous_solved = best.joint_angles;
            have_previous = true;
        } else if (!result.failed_sample) {
            result.failed_sample = i;
        }
    }

    result.success = !result.failed_sample.has_value();

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
