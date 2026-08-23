//
// PathIk — joint-space initialization for a Cartesian path, built from a
// small set of IK anchor poses.
//
// GPMP2 receives the FULL Cartesian path as pose factors and does the
// detailed path fitting itself; this layer only has to hand it a plausible,
// continuous joint-space starting sketch. Requiring IK to succeed at every
// path sample made initialization the most brittle stage of the planner —
// one hard patch of samples killed plans the optimiser could have repaired.
// So IK is solved only at strategically spaced ANCHOR samples, walked in
// order and seeded by continuation (each anchor's solve starts from the
// previous solution, keeping the walk on one branch of the redundant arm's
// solution family). Every sample between anchors is interpolated.
//
// Anchor solves are robust and attempt-bounded: the continuation seed first,
// then a small structured set of null-space perturbations, then a bounded
// random multi-start — stopping at the first solution inside tolerance and
// limits, or at the fixed attempt cap. A failed OPTIONAL anchor is
// dropped and its span interpolated between its solved neighbours; only the
// path ENTRY anchor can make initialization fail, because nothing can
// interpolate toward an unknown entry configuration.
//
// Eigen and Pinocchio only: no gtsam (see PinocchioKinematicsAdapter.h for
// why the two cannot share a translation unit). The planner converts at the
// factor-graph boundary.
//

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "CartesianPath.h"
#include "analytical_ik.h"

// Why one sample failed. Recorded for diagnosis only — the walk already
// branches on exactly this distinction (a converged pose rejected by the
// limit check is a different problem from IK never converging), and losing
// it forces the operator to re-derive it from residuals by eye.
enum class PathIkFailure {
    kNone,           // solved
    kNoConvergence,  // no attempted seed produced a valid IK solution
    kJointLimits,    // IK converged but every solution violated the limits
};

// One path sample. Only anchor samples are solved by IK; every other
// sample carries an interpolated configuration used only as GPMP2's
// initial guess.
struct PathIkSample {
    Eigen::Matrix<double, 7, 1> configuration =
        Eigen::Matrix<double, 7, 1>::Zero();
    bool solved = false;
    double position_residual_m = 0.0;
    double orientation_residual_rad = 0.0;
    // kNone for solved anchors AND for samples that were never attempted
    // (deliberately interpolated); a failure reason is present only on an
    // anchor whose bounded solve found nothing.
    PathIkFailure failure = PathIkFailure::kNone;
    // True when `configuration` is interpolated between solved anchors, not
    // an IK solution — the normal state of every non-anchor sample.
    bool interpolated = false;
};

struct PathIkResult {
    // False only when the path ENTRY anchor could not be solved — the one
    // failure interpolation cannot paper over. Failed intermediate anchors
    // are dropped and their spans interpolated; GPMP2 then gets its chance
    // to repair the rough stretch against the full set of pose factors.
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

// The bounded anchor-solve policy, visible so it stays reviewable.
// Every kPathIkAnchorStride-th sample is an anchor (the entry always is;
// an open path's final sample too). Each anchor gets the continuation
// seed, then kPathIkAlternativeSeedCount structured null-space
// perturbations, then random multi-start — stopping at the FIRST solution
// inside tolerance and limits, or when the fixed attempt cap runs out. The
// attempt cap bounds the work deterministically for a hard anchor.
inline constexpr std::size_t kPathIkAnchorStride = 4;
inline constexpr int kPathIkAlternativeSeedCount = 4;
inline constexpr int kMaxAnchorIkAttempts = 50;
static_assert(kPathIkAlternativeSeedCount >= 3 &&
              kPathIkAlternativeSeedCount <= 5);

// Walks `path` (whose poses must already be in the frame `arm.base_transform`
// is expressed against) solving IK at the anchor samples and interpolating
// the rest.
//
// `seed` is the measured configuration. Continuous joints are canonicalised
// once at this boundary; a bounded joint outside `limits` is rejected rather
// than silently clamped. `tolerance` is how accurately each anchor must be reached —
// pass something tight, since the solver's own default stops at 20 mm, which
// is meaningless for a traced path (measured 2026-08-07).
//
// `closed` marks a path whose last sample repeats its first, so closure
// drift is meaningful and interpolation may wrap around the seam.
// `random_seed` drives the multi-start fallback deterministically — pass
// the run's effective IK seed so a failure reproduces exactly.
PathIkResult SolvePathIk(const CartesianPath& path, const PathIkArm& arm,
                         const Eigen::Matrix<double, 7, 1>& seed,
                         const PathIkJointLimits& limits,
                         const analytical_ik::IKTolerance& tolerance,
                         bool closed = false,
                         std::uint64_t random_seed = 20260807u);

struct TerminalIkCandidate {
    Eigen::Matrix<double, 7, 1> configuration =
        Eigen::Matrix<double, 7, 1>::Zero();
    std::uint64_t stream_id = 0;
    std::size_t attempt_index = 0;
    double position_residual_m = 0.0;
    double orientation_residual_rad = 0.0;
};

std::vector<TerminalIkCandidate> SolveTerminalIkCandidates(
    const PathIkArm& arm, const Eigen::Isometry3d& target,
    const Eigen::Matrix<double, 7, 1>& measured_q,
    const PathIkJointLimits& limits, std::uint64_t effective_seed,
    std::size_t max_candidates = 3);
