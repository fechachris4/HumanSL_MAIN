#ifndef ANALYTICAL_IK_H
#define ANALYTICAL_IK_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <random>
#include <cstdint>
#include <string>
#include <chrono>

#include "Config.h"
#include "PinocchioKinematicsAdapter.h"

namespace analytical_ik {


// Kinova Gen3 joint limits. FK/Jacobian used to come from the DH table that
// lived here too; that's gone now — forwardKinematics()/computeJacobian()
// evaluate Pinocchio against the canonical URDF instead (see
// PinocchioKinematicsAdapter.h).
struct KinovaGen3Params {
    // Joint limits (radians) - using original limits to avoid issues
    static constexpr double JOINT_LIMITS_LOWER[7] = {-1e20, -2.2515, -1e20, -2.5807, -1e20, -2.0996, -1e20};
    static constexpr double JOINT_LIMITS_UPPER[7] = {1e20, 2.2515, 1e20, 2.5807, 1e20, 2.0996, 1e20};
};

// How close the solver must get before it stops, and how close it must be
// to call the result valid.
//
// The defaults are the values this solver has always used, so any caller
// that does not pass one is unaffected. They are LOOSE on purpose: the
// solver's original job was to seed a point-to-point optimiser that carries
// its own goal term and corrects the endpoint afterwards, so stopping at
// 20 mm cost nothing.
//
// That reasoning does not survive contact with a PATH. Nothing corrects the
// middle of a traced shape, and measured 2026-08-07 the early exit made
// every sample of a 3 cm circle land 8.6–19.4 mm off it — a hard ceiling at
// the 20 mm threshold with nothing above, which is the signature of the
// solver stopping rather than of the arm being unable to reach. Certifying
// or seeding a path therefore needs to ask for something tighter.
struct IKTolerance {
    double converge_position_m = 2e-2;      // stop once inside this
    double converge_orientation_rad = 0.2;
    double accept_position_m = 0.04;        // call the result valid inside this
    double accept_orientation_rad = 0.15;
};

// How the solver draws its random restarts.
//
// The solver explores by re-solving from randomly drawn joint configurations,
// which is why it can find a solution a single deterministic descent would
// miss. Left to `std::random_device` that exploration made the PLANNER
// non-reproducible: measured 2026-08-07, identical circle requests traced to
// 2.406-2.409 mm every time but their joint-space routes differed, with
// closure drift swinging 0.04-14 deg between runs. Repeatable controller
// experiments need the joint route to be repeatable too.
//
// `seed` makes the draw reproducible. `stream` separates draws that must
// NOT coincide: a caller trying eight independent restarts of the same pose
// passes eight different streams, so the attempts still explore different
// branches while the whole set replays identically next run. Deriving the
// stream from the caller's own indices (attempt, path sample) is what keeps
// exploration and determinism from being in tension.
//
// A randomised run is still recorded: the drawn seed is reported, and
// replanning with it reproduces the run exactly. There is no unrecorded
// random seed anywhere in the planning path.
struct IKSeeding {
    std::uint64_t seed = 20260807;
    std::uint64_t stream = 0;

    // Distinct streams from one seed, mixed so that nearby (seed, stream)
    // pairs do not produce correlated sequences — splitmix64's finalizer,
    // which is a standard, well-tested avalanche step.
    std::uint64_t Mixed() const {
        std::uint64_t z = seed + 0x9E3779B97F4A7C15ULL * (stream + 1);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
};

// Structure to hold IK solution.
//
// position_error_m / orientation_error_rad are how far this configuration
// actually lands from the requested pose, kept SEPARATE rather than summed.
// They are the solver's own final measurements; they used to be computed,
// collapsed into quality_score, and then overwritten by the seed distance,
// so the one number that says WHICH half of the pose was unreachable was
// discarded on every attempt. Two things depend on keeping them:
//   - a caller that wants the closest near miss when nothing passed
//     (a rejected solution 6 cm out is a far better trajectory seed than
//     the start configuration, and a much better one than the candidate
//     that merely sat nearest the seed);
//   - diagnosing whether a failure is a position problem or an orientation
//     problem, which a single fused norm cannot distinguish.
// `attempted` separates "the solver ran and got this close" from a
// default-constructed IKSolution, whose joint_angles are zero and mean
// nothing — do not read the errors or the angles when it is false.
struct IKSolution {
    Eigen::Vector<double, 7> joint_angles;
    double quality_score;
    bool is_valid;
    int iterations_used;
    double position_error_m;
    double orientation_error_rad;
    bool attempted;

    IKSolution() : quality_score(std::numeric_limits<double>::max()), is_valid(false),
                   iterations_used(0),
                   position_error_m(std::numeric_limits<double>::infinity()),
                   orientation_error_rad(std::numeric_limits<double>::infinity()),
                   attempted(false) {
        joint_angles.setZero();
    }
};

class AnalyticalIKSolver {
private:
    // Damped Least Squares parameters
    static constexpr double DEFAULT_DAMPING = 0.1;
    static constexpr double MAX_DAMPING = 10.0;
    static constexpr double MIN_DAMPING = 1e-4;
    static constexpr double DAMPING_INCREASE = 3.0;
    static constexpr double DAMPING_DECREASE = 0.3;
    static constexpr int MAX_ITERATIONS = 100;
    static constexpr double MIN_IMPROVEMENT = 1e-3;

    // Null-space joint-limit avoidance. Without this the solver drives
    // straight at the pose and only discovers afterwards that it landed on
    // a joint stop, so it never uses the arm's spare freedom to arrive
    // legally — on 2026-08-06 every one of ten attempts put joint 6 about
    // 38 deg past its limit and was thrown away for it.
    //
    // Deadband, so the interior is untouched: zero effect until a bounded
    // joint comes within ZONE of its limit, then a push inward proportional
    // to the excess. Mirrors basic_control's ReactiveLaw.h (kLimitAvoidZone
    // / kLimitAvoidGain) deliberately, including its 20 deg zone, so the
    // planner and the controller agree about what "near a limit" means.
    //
    // The push is projected into the null space, so it can only choose
    // BETWEEN configurations that reach the same pose — it cannot trade
    // pose accuracy for legality, and it cannot rescue a pose that no legal
    // configuration reaches. GAIN is a per-iteration step in radians, not a
    // velocity, and the existing 0.3 rad step cap below bounds their sum.
    static constexpr double LIMIT_AVOID_ZONE_RAD = 20.0 * M_PI / 180.0;
    static constexpr double LIMIT_AVOID_GAIN = 1.0;
    
    // Utility functions
    static double normalizeAngle(double angle);

    static bool isWithinJointLimits(const Eigen::Vector<double, 7>& joints);
    static double computeQualityScore(const Eigen::Vector<double, 7>& joints, 
                                    const Eigen::Vector<double, 7>& seed);
    
    // Forward kinematics and Jacobian — Pinocchio against the canonical
    // URDF, composed with base_transform exactly as the DH chain used to be
    // (see PinocchioKinematicsAdapter.h). end_effector_frame / left_arm
    // select WHICH arm's chain the solve targets; they must match the arm
    // the caller is planning for, or every solution places a phantom point
    // ~0.12 m from the real end-effector (the tool-vs-flange offset — the
    // 2026-08-06 left-arm "final goal error: 120 mm" bug). Defaults keep
    // every pre-left-arm caller on the right arm's mounted tool.
    static Eigen::Matrix4d forwardKinematics(const Eigen::Vector<double, 7>& joints,
                                           const Eigen::Matrix4d& base_transform,
                                           const std::string& end_effector_frame,
                                           bool left_arm);
    static Eigen::Matrix<double, 6, 7> computeJacobian(const Eigen::Vector<double, 7>& joints,
                                                      const Eigen::Matrix4d& base_transform,
                                                      const std::string& end_effector_frame,
                                                      bool left_arm);

    // Pose difference computation
    static Eigen::Vector<double, 6> computePoseError(const Eigen::Matrix4d& current_pose,
                                                    const Eigen::Matrix4d& target_pose);

    // Damped Least Squares core algorithm
    static IKSolution solveDampedLeastSquares(const Eigen::Matrix4d& target_pose,
                                            const Eigen::Matrix4d& base_transform,
                                            const Eigen::Vector<double, 7>& initial_guess,
                                            const std::string& end_effector_frame,
                                            bool left_arm,
                                            const IKTolerance& tolerance);

    // Random seed generation. The RNG is built HERE from `seeding` rather
    // than kept as a function-local static: a static generator advances
    // across calls, so even a fixed seed would make the result depend on
    // how many solves happened earlier in the process.
    static std::vector<Eigen::Vector<double, 7>> generateRandomSeeds(
        const Eigen::Vector<double, 7>& preferred_seed, int num_seeds,
        const IKSeeding& seeding);

public:
    // Main IK solver interface. end_effector_frame / left_arm: see the
    // private forwardKinematics note above.
    static std::vector<IKSolution> solveIK(const Eigen::Matrix4d& target_pose,
                                         const Eigen::Matrix4d& base_transform,
                                         const Eigen::Vector<double, 7>& seed_config,
                                         int num_attempts = 10,
                                         const std::string& end_effector_frame =
                                             config::kRightEndEffectorFrame,
                                         bool left_arm = false,
                                         const IKTolerance& tolerance = IKTolerance{},
                                         const IKSeeding& seeding = IKSeeding{});

    // Single best solution (fastest)
    static IKSolution solveBestIK(const Eigen::Matrix4d& target_pose,
                                const Eigen::Matrix4d& base_transform,
                                const Eigen::Vector<double, 7>& seed_config,
                                int num_attempts = 10,
                                const std::string& end_effector_frame =
                                    config::kRightEndEffectorFrame,
                                bool left_arm = false,
                                const IKTolerance& tolerance = IKTolerance{},
                                const IKSeeding& seeding = IKSeeding{});

    // Exposed for test_ik_determinism, which must check the DRAWS
    // themselves: whether a plan is reproducible is a property of the
    // restart sequence, and inferring it from solver output alone would be
    // indirect. A thin forwarder rather than making the generator public.
    static std::vector<Eigen::Vector<double, 7>> GenerateSeedsForTest(
        const Eigen::Vector<double, 7>& preferred_seed, int num_seeds,
        const IKSeeding& seeding) {
        return generateRandomSeeds(preferred_seed, num_seeds, seeding);
    }

    // Public forward kinematics for debugging
    static Eigen::Matrix4d computeForwardKinematics(const Eigen::Vector<double, 7>& joints,
                                                   const Eigen::Matrix4d& base_transform,
                                                   const std::string& end_effector_frame =
                                                       config::kRightEndEffectorFrame,
                                                   bool left_arm = false) {
        return forwardKinematics(joints, base_transform, end_effector_frame, left_arm);
    }

};

//=============================================================================
// IMPLEMENTATION
//=============================================================================

inline double AnalyticalIKSolver::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline bool AnalyticalIKSolver::isWithinJointLimits(const Eigen::Vector<double, 7>& joints) {
    for (int i = 0; i < 7; i++) {
        // Skip infinite limits (continuous joints)
        if (KinovaGen3Params::JOINT_LIMITS_LOWER[i] < -1e10 || 
            KinovaGen3Params::JOINT_LIMITS_UPPER[i] > 1e10) {
            continue; // No limits for this joint
        }
        
        if (joints(i) < KinovaGen3Params::JOINT_LIMITS_LOWER[i] || 
            joints(i) > KinovaGen3Params::JOINT_LIMITS_UPPER[i]) {
            return false;
        }
    }
    return true;
}

inline double AnalyticalIKSolver::computeQualityScore(const Eigen::Vector<double, 7>& joints, 
                                                    const Eigen::Vector<double, 7>& seed) {
    double distance = (joints - seed).norm();
    
    // Add penalty for joint limit violations
    double penalty = 0.0;
    for (int i = 0; i < 7; i++) {
        if (joints(i) < KinovaGen3Params::JOINT_LIMITS_LOWER[i]) {
            penalty += std::pow(KinovaGen3Params::JOINT_LIMITS_LOWER[i] - joints(i), 2);
        } else if (joints(i) > KinovaGen3Params::JOINT_LIMITS_UPPER[i]) {
            penalty += std::pow(joints(i) - KinovaGen3Params::JOINT_LIMITS_UPPER[i], 2);
        }
    }
    
    return distance + 10.0 * penalty;
}

namespace detail {

// base_transform * DhRootInBaseLink()^-1 * base_link_M_tool(joints) — the
// same frame-composition identity utils.cpp's forwardKinematics uses, so
// this reduces to exactly what the DH chain used to compute for any
// base_transform, not just the live call site's DhRootInMount().
inline Eigen::Isometry3d ToolPoseInBaseTransform(const Eigen::Vector<double, 7>& joints,
                                                 const Eigen::Matrix4d& base_transform,
                                                 const std::string& end_effector_frame,
                                                 bool left_arm) {
    const auto pose_jacobian = pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
        joints, end_effector_frame, left_arm);
    Eigen::Isometry3d base_link_M_tool = Eigen::Isometry3d::Identity();
    base_link_M_tool.linear() = pose_jacobian.rotation;
    base_link_M_tool.translation() = pose_jacobian.position;
    return Eigen::Isometry3d(base_transform) *
           pinocchio_kinematics_adapter::DhRootInBaseLink().inverse() * base_link_M_tool;
}

} // namespace detail

inline Eigen::Matrix4d AnalyticalIKSolver::forwardKinematics(const Eigen::Vector<double, 7>& joints,
                                                           const Eigen::Matrix4d& base_transform,
                                                           const std::string& end_effector_frame,
                                                           bool left_arm) {
    return detail::ToolPoseInBaseTransform(joints, base_transform, end_effector_frame,
                                           left_arm).matrix();
}

inline Eigen::Matrix<double, 6, 7> AnalyticalIKSolver::computeJacobian(const Eigen::Vector<double, 7>& joints,
                                                                      const Eigen::Matrix4d& base_transform,
                                                                      const std::string& end_effector_frame,
                                                                      bool left_arm) {
    const auto pose_jacobian = pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
        joints, end_effector_frame, left_arm);
    // Rotate base_link-axes columns into base_transform's axes. Both frames
    // are rigidly fixed to each other (compile-time-constant relative
    // orientation), so a point's linear/angular velocity transforms by pure
    // rotation — no translational (adjoint) term applies.
    const Eigen::Matrix3d base_transform_R_base_link =
        base_transform.block<3, 3>(0, 0) *
        pinocchio_kinematics_adapter::DhRootInBaseLink().rotation().transpose();
    Eigen::Matrix<double, 6, 7> J;
    J.topRows<3>() = base_transform_R_base_link * pose_jacobian.jacobian.topRows<3>();
    J.bottomRows<3>() = base_transform_R_base_link * pose_jacobian.jacobian.bottomRows<3>();
    return J;
}

inline Eigen::Vector<double, 6> AnalyticalIKSolver::computePoseError(const Eigen::Matrix4d& current_pose,
                                                                   const Eigen::Matrix4d& target_pose) {
    Eigen::Vector<double, 6> error;
    
    // Position error
    Eigen::Vector3d pos_error = target_pose.block<3,1>(0,3) - current_pose.block<3,1>(0,3);
    
    // Orientation error using simpler approach
    Eigen::Matrix3d R_current = current_pose.block<3,3>(0,0);
    Eigen::Matrix3d R_target = target_pose.block<3,3>(0,0);
    Eigen::Matrix3d R_error = R_target * R_current.transpose();
    
    // Use rotation vector approach (more numerically stable)
    Eigen::Vector3d orient_error;
    double trace = R_error.trace();
    
    if (trace > 2.9) {
        // Near identity - use small angle approximation
        orient_error << (R_error(2,1) - R_error(1,2))/2.0,
                       (R_error(0,2) - R_error(2,0))/2.0,
                       (R_error(1,0) - R_error(0,1))/2.0;
    } else {
        // General case - use axis-angle but with better handling
        double angle = acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
        if (angle < 1e-6) {
            orient_error.setZero();
        } else {
            double sin_angle = sin(angle);
            if (abs(sin_angle) > 1e-6) {
                orient_error << (R_error(2,1) - R_error(1,2)) / (2.0 * sin_angle) * angle,
                               (R_error(0,2) - R_error(2,0)) / (2.0 * sin_angle) * angle,
                               (R_error(1,0) - R_error(0,1)) / (2.0 * sin_angle) * angle;
            } else {
                orient_error.setZero();
            }
        }
    }
    
    error << pos_error, orient_error;
    return error;
}

inline IKSolution AnalyticalIKSolver::solveDampedLeastSquares(const Eigen::Matrix4d& target_pose,
                                                            const Eigen::Matrix4d& base_transform,
                                                            const Eigen::Vector<double, 7>& initial_guess,
                                                            const std::string& end_effector_frame,
                                                            bool left_arm,
                                                            const IKTolerance& tolerance) {
    IKSolution solution;
    solution.joint_angles = initial_guess;
    solution.iterations_used = 0;
    
    double damping = DEFAULT_DAMPING;
    double prev_error_norm = std::numeric_limits<double>::max();
    
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        solution.iterations_used = iter + 1;
        
        // Normalize joint angles
        for (int i = 0; i < 7; i++) {
            solution.joint_angles(i) = normalizeAngle(solution.joint_angles(i));
        }
        
        // Compute current pose
        Eigen::Matrix4d current_pose = forwardKinematics(solution.joint_angles, base_transform,
                                                         end_effector_frame, left_arm);
        
        // Compute pose error
        Eigen::Vector<double, 6> error = computePoseError(current_pose, target_pose);
        double error_norm = error.norm();
        

        
        // Check convergence
        if (error.head<3>().norm() < tolerance.converge_position_m &&
            error.tail<3>().norm() < tolerance.converge_orientation_rad) {
            solution.is_valid = isWithinJointLimits(solution.joint_angles);
            solution.quality_score = error_norm;
            solution.position_error_m = error.head<3>().norm();
            solution.orientation_error_rad = error.tail<3>().norm();
            solution.attempted = true;

            return solution;
        }
        
        // Compute Jacobian
        Eigen::Matrix<double, 6, 7> J = computeJacobian(solution.joint_angles, base_transform,
                                                        end_effector_frame, left_arm);
        
        // Damped least squares update
        Eigen::Matrix<double, 7, 7> JtJ = J.transpose() * J;
        Eigen::Matrix<double, 7, 7> damped_JtJ = JtJ + damping * Eigen::Matrix<double, 7, 7>::Identity();
        
        // Solve for joint angle update
        Eigen::Vector<double, 7> delta_q = damped_JtJ.ldlt().solve(J.transpose() * error);

        // Secondary objective: steer bounded joints off their stops using
        // the redundancy, so a legal configuration is preferred whenever one
        // reaching this pose exists. Joints 1/3/5/7 are continuous on a
        // Gen3 and carry no limit, so they contribute nothing here and are
        // free to absorb the reconfiguration.
        Eigen::Vector<double, 7> avoidance = Eigen::Vector<double, 7>::Zero();
        bool any_near_limit = false;
        for (int i = 0; i < 7; i++) {
            const double lower = KinovaGen3Params::JOINT_LIMITS_LOWER[i];
            const double upper = KinovaGen3Params::JOINT_LIMITS_UPPER[i];
            if (lower < -1e10 || upper > 1e10)
                continue;  // continuous joint: no limit to avoid
            // Signed principal value, as ReactiveLaw.h does — the iterate is
            // normalized at the top of the loop, but the final update is not.
            const double signed_angle = std::remainder(solution.joint_angles(i), 2.0 * M_PI);
            const double excess = std::abs(signed_angle) - (upper - LIMIT_AVOID_ZONE_RAD);
            if (excess > 0.0) {
                any_near_limit = true;
                avoidance(i) = -LIMIT_AVOID_GAIN * excess *
                               (signed_angle < 0.0 ? -1.0 : 1.0);
            }
        }

        // Project into the null space before adding, so the avoidance can
        // never pull the hand off the pose it is converging to: N = I -
        // Jᵀ(JJᵀ + λI)⁻¹J, the damped projector, using the SAME damping the
        // task step above uses so the two stay consistent as it adapts.
        // Skipped entirely when no joint is near a limit, which is the
        // common case and leaves the original behaviour bit-for-bit.
        if (any_near_limit) {
            Eigen::Matrix<double, 6, 6> jjt = J * J.transpose();
            jjt.diagonal().array() += damping;
            const Eigen::Matrix<double, 7, 7> projector =
                Eigen::Matrix<double, 7, 7>::Identity() -
                J.transpose() * jjt.ldlt().solve(J);
            delta_q += projector * avoidance;
        }

        // Apply update with step size limitation
        double step_size = 1.0;
        double max_joint_change = delta_q.cwiseAbs().maxCoeff();
        if (max_joint_change > 0.3) { // Limit to 0.3 rad per step (~17 degrees)
            step_size = 0.3 / max_joint_change;
        }
        
        // Ensure minimum step size to avoid getting stuck
        if (step_size < 0.01) {
            step_size = 0.01;
        }
        
        // Store old joint angles for debugging
        Eigen::Vector<double, 7> old_joints = solution.joint_angles;
        solution.joint_angles += step_size * delta_q;
        
        // Recompute error after joint update for accurate tracking
        Eigen::Matrix4d updated_pose = forwardKinematics(solution.joint_angles, base_transform,
                                                         end_effector_frame, left_arm);
        Eigen::Vector<double, 6> updated_error = computePoseError(updated_pose, target_pose);
        double updated_error_norm = updated_error.norm();
        
        // Adaptive damping
        if (updated_error_norm < prev_error_norm) {
            // Error decreased - reduce damping
            damping = std::max(MIN_DAMPING, damping * DAMPING_DECREASE);
        } else {
            // Error increased - increase damping
            damping = std::min(MAX_DAMPING, damping * DAMPING_INCREASE);
        }
        
        // Check for minimal improvement (relative) using correct error values
        if (iter > 1 && std::abs(prev_error_norm - updated_error_norm) < MIN_IMPROVEMENT * prev_error_norm) {
            break;
        }
        
        // Update prev_error_norm for next iteration
        prev_error_norm = updated_error_norm;
    }
    
    // Final validation with more relaxed criteria
    Eigen::Matrix4d final_pose = forwardKinematics(solution.joint_angles, base_transform,
                                                   end_effector_frame, left_arm);
    Eigen::Vector<double, 6> final_error = computePoseError(final_pose, target_pose);
    
    // Relaxed IK tolerances for better success rate
    bool position_ok = final_error.head<3>().norm() < tolerance.accept_position_m;
    bool orientation_ok = final_error.tail<3>().norm() < tolerance.accept_orientation_rad;
    bool limits_ok = isWithinJointLimits(solution.joint_angles);


    solution.is_valid = (position_ok && orientation_ok && limits_ok);
    solution.quality_score = final_error.norm();
    solution.position_error_m = final_error.head<3>().norm();
    solution.orientation_error_rad = final_error.tail<3>().norm();
    solution.attempted = true;

    return solution;
}

inline std::vector<Eigen::Vector<double, 7>> AnalyticalIKSolver::generateRandomSeeds(
    const Eigen::Vector<double, 7>& preferred_seed, int num_seeds,
    const IKSeeding& seeding) {

    std::vector<Eigen::Vector<double, 7>> seeds;
    seeds.push_back(preferred_seed); // First seed is the preferred one

    if (num_seeds <= 1) return seeds;

    // Local, not static: a static generator carries state between calls, so
    // the same request would draw differently depending on what ran before.
    std::mt19937_64 gen(seeding.Mixed());
    
    for (int i = 1; i < num_seeds; i++) {
        Eigen::Vector<double, 7> seed;
        
        for (int j = 0; j < 7; j++) {
            if (KinovaGen3Params::JOINT_LIMITS_LOWER[j] > -1e10 && 
                KinovaGen3Params::JOINT_LIMITS_UPPER[j] < 1e10) {
                // Joint has limits - sample within them
                std::uniform_real_distribution<double> dist(
                    KinovaGen3Params::JOINT_LIMITS_LOWER[j], 
                    KinovaGen3Params::JOINT_LIMITS_UPPER[j]);
                seed(j) = dist(gen);
            } else {
                // Continuous joint - sample around preferred seed
                std::normal_distribution<double> dist(preferred_seed(j), 1.0);
                seed(j) = normalizeAngle(dist(gen));
            }
        }
        
        seeds.push_back(seed);
    }
    
    return seeds;
}

inline std::vector<IKSolution> AnalyticalIKSolver::solveIK(const Eigen::Matrix4d& target_pose,
                                                         const Eigen::Matrix4d& base_transform,
                                                         const Eigen::Vector<double, 7>& seed_config,
                                                         int num_attempts,
                                                         const std::string& end_effector_frame,
                                                         bool left_arm,
                                                         const IKTolerance& tolerance,
                                                         const IKSeeding& seeding) {
    std::vector<IKSolution> all_solutions;

    // Generate random seeds
    auto seeds = generateRandomSeeds(seed_config, num_attempts, seeding);

    // Try each seed
    for (const auto& seed : seeds) {
        IKSolution solution = solveDampedLeastSquares(target_pose, base_transform, seed,
                                                      end_effector_frame, left_arm,
                                                      tolerance);
        
        if (solution.is_valid || solution.quality_score < 10.0) { // Accept reasonable solutions
            solution.quality_score = computeQualityScore(solution.joint_angles, seed_config);
            all_solutions.push_back(solution);
        }
    }

    // Valid solutions first. Among VALID ones, prefer the one nearest the
    // seed (quality_score) — they all reach the pose, so the tie-break that
    // matters is least joint motion. Among INVALID ones the ordering must be
    // different: none of them reach the pose, so "nearest the seed" selects
    // the candidate that moved least, not the one that got closest. A caller
    // falling back to a near miss wants the closest approach, so rank the
    // rejected ones by their actual pose error instead.
    std::sort(all_solutions.begin(), all_solutions.end(),
              [](const IKSolution& a, const IKSolution& b) {
                  if (a.is_valid != b.is_valid) return a.is_valid;
                  if (a.is_valid) return a.quality_score < b.quality_score;
                  const double a_error = a.position_error_m + a.orientation_error_rad;
                  const double b_error = b.position_error_m + b.orientation_error_rad;
                  return a_error < b_error;
              });

    return all_solutions;
}

inline IKSolution AnalyticalIKSolver::solveBestIK(const Eigen::Matrix4d& target_pose,
                                                const Eigen::Matrix4d& base_transform,
                                                const Eigen::Vector<double, 7>& seed_config,
                                                int num_attempts,
                                                const std::string& end_effector_frame,
                                                bool left_arm,
                                                const IKTolerance& tolerance,
                                                const IKSeeding& seeding) {
    auto solutions = solveIK(target_pose, base_transform, seed_config, num_attempts,
                             end_effector_frame, left_arm, tolerance, seeding);
    
    if (solutions.empty()) {
        return IKSolution(); // Invalid solution
    }
    
    return solutions[0]; // Best solution
}



} // namespace analytical_ik

#endif // ANALYTICAL_IK_H