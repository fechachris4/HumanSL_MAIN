//
// PathIk — inverse kinematics walked along a Cartesian path, seeded by
// continuation.
//
// Solving each pose independently is wrong for a path. A 7-DoF arm has a
// continuous family of configurations reaching most poses, so independent
// solves are free to jump between branches: the elbow flips, and the joint
// trajectory contains a discontinuity the Cartesian path never asked for.
// Seeding each solve from the PREVIOUS solution keeps the walk on one
// branch. Where a sample fails, the last good solution stays the seed, so
// one bad patch does not scatter everything after it.
//
// A failed sample gets a small, deterministic set of null-space perturbations
// around the last valid continuation seed. There is deliberately no large
// random-restart loop here: GPMP2 is the path solver, and this layer only
// needs a plausible joint-space initialization.
//
// Eigen and Pinocchio only: no gtsam (see PinocchioKinematicsAdapter.h for
// why the two cannot share a translation unit). The planner converts at the
// factor-graph boundary.
//

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "CartesianPath.h"
#include "analytical_ik.h"

// One solved sample, with the diagnostics that say whether it is a good
// place to be rather than merely a reachable one.
struct PathIkSample {
    Eigen::Matrix<double, 7, 1> configuration =
        Eigen::Matrix<double, 7, 1>::Zero();
    bool solved = false;
    double position_residual_m = 0.0;
    double orientation_residual_rad = 0.0;
    // Radians from the nearest BOUNDED joint's limit. Continuous joints
    // (1/3/5/7 on a Gen3) have no limit and never set this. Small means the
    // next sample may have nowhere to go.
    double minimum_joint_limit_margin_rad = 0.0;
    // Smallest singular value of the 6x7 task Jacobian. Near zero means a
    // singularity: the arm is losing the ability to move in some direction,
    // and IK there is ill-conditioned. Units mix m/rad and rad/rad, so read
    // its trend along the path rather than its absolute size.
    double minimum_manipulability = 0.0;
    // Number of deterministic null-space alternatives tried after the
    // continuation seed failed. Bounded by kPathIkAlternativeSeedCount.
    int alternative_seed_attempts = 0;
    // True when this sample's joint configuration is an interpolation between
    // valid neighbours rather than a successful IK result.
    bool seed_interpolated = false;
};

struct PathIkResult {
    // True when every unresolved run has a bounded, two-sided interpolation
    // seed for GPMP2. Individual samples may still have solved == false.
    bool success = false;
    std::vector<PathIkSample> samples;
    std::optional<std::size_t> failed_sample;  // first failure, if any
    std::size_t unresolved_samples = 0;
    std::size_t interpolated_samples = 0;
    std::size_t maximum_unresolved_run = 0;
    // Largest per-joint step between consecutive solved samples, radians.
    // A large value is the signature of a branch flip that continuation
    // failed to prevent — the Cartesian step was small but the joints moved
    // a long way.
    double maximum_joint_step_rad = 0.0;
    // For a CLOSED path (a circle), how far the last configuration sits
    // from the first. The tool pose closes exactly; the redundant elbow
    // need not, and a large drift means tracing a second lap would not
    // repeat the first. Zero for an open path.
    double closure_drift_rad = 0.0;
};

// What the arm is, for the walk. Bundled so callers cannot pass a DH table
// belonging to one arm and a frame belonging to the other — the mistake
// that produced the 120 mm tool-vs-flange error on 2026-08-06.
struct PathIkArm {
    Eigen::Matrix4d base_transform = Eigen::Matrix4d::Identity();
    std::string end_effector_frame;
    bool left_arm = false;
};

// A failed continuation solve gets exactly four small alternatives. Keeping
// this visible makes the bounded search policy testable and reviewable.
inline constexpr int kPathIkAlternativeSeedCount = 4;
inline constexpr std::size_t kMaxInterpolatedPathIkGapSamples = 2;
static_assert(kPathIkAlternativeSeedCount >= 3 &&
              kPathIkAlternativeSeedCount <= 5);

// Walks `path` (whose poses must already be in the frame `arm.base_transform`
// is expressed against) solving IK at each sample.
//
// `seed` is the measured configuration: the first sample is chosen partly by
// proximity to it, so the arm does not have to travel across its workspace
// to begin. `tolerance` is how accurately each sample must be reached —
// pass something tight, since the solver's own default stops at 20 mm, which
// is meaningless for a traced path (measured 2026-08-07).
//
// `closed` marks a path whose last sample repeats its first, so closure
// drift is meaningful.
PathIkResult SolvePathIk(const CartesianPath& path, const PathIkArm& arm,
                         const Eigen::Matrix<double, 7, 1>& seed,
                         const analytical_ik::IKTolerance& tolerance,
                         bool closed = false);
