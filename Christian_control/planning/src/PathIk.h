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
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "CartesianPath.h"
#include "analytical_ik.h"

// One path sample. Unsolved samples may still carry an interpolated
// configuration used only as GPMP2's initial guess.
struct PathIkSample {
    Eigen::Matrix<double, 7, 1> configuration =
        Eigen::Matrix<double, 7, 1>::Zero();
    bool solved = false;
    double position_residual_m = 0.0;
    double orientation_residual_rad = 0.0;
};

struct PathIkResult {
    // True when every unresolved run is short and has valid neighbours on
    // both sides. Individual samples may still have solved == false; their
    // configurations are interpolation seeds for GPMP2.
    bool success = false;
    std::vector<PathIkSample> samples;
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

// The planner's operating limits in radians, loaded from joint_limits.yaml by
// the caller. Bounded joints include the configured 2 degree planner margin;
// continuous joints carry the existing +/-1e20 sentinel.
struct PathIkJointLimits {
    Eigen::Matrix<double, 7, 1> lower_rad;
    Eigen::Matrix<double, 7, 1> upper_rad;
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
// `seed` is the measured configuration. Continuous joints are canonicalised
// once at this boundary; a bounded joint outside `limits` is rejected rather
// than silently clamped. `tolerance` is how accurately each sample must be reached —
// pass something tight, since the solver's own default stops at 20 mm, which
// is meaningless for a traced path (measured 2026-08-07).
//
// `closed` marks a path whose last sample repeats its first, so closure
// drift is meaningful.
PathIkResult SolvePathIk(const CartesianPath& path, const PathIkArm& arm,
                         const Eigen::Matrix<double, 7, 1>& seed,
                         const PathIkJointLimits& limits,
                         const analytical_ik::IKTolerance& tolerance,
                         bool closed = false);
