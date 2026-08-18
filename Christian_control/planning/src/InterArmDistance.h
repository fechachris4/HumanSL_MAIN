//
// InterArmDistance — do the two arms come too close to each other while
// executing their own trajectories?
//
// The two arms are planned INDEPENDENTLY: each solves against its own model
// and an SDF that contains no second arm. Neither plan knows the other
// exists, so nothing in the single-arm pipeline can answer this. That is
// what this checker is for, and it is deliberately independent of HOW the
// pair was produced — it takes two emitted blocks and compares them.
//
// It compares the arms at CORRESPONDING TIMES rather than treating one arm's
// swept volume as a permanent obstacle. The swept volume of a 12-second
// circle occupies a large part of the shared workspace for the whole
// trajectory, so modelling it as static would forbid almost every useful
// pair. Two arms passing through the same point ten seconds apart never come
// near each other.
//
// TIME CORRESPONDENCE IS NOT EXACT. The two blocks are dispatched
// concurrently but activation is not atomic (there is no pre-commit
// acknowledgement in the controller), so one arm may begin up to some skew
// before the other. Comparing only t against t would therefore check a
// synchronisation the system does not provide. Each left state at t is
// instead checked against right states over [t-skew, t+skew].
//
// Both arms' collision spheres are already expressed in the shared Vicon
// world frame: PlannerModel builds each arm at DhRootInWorld(left_arm), so
// sphereCentersMat returns world-frame centres for either arm and no
// additional transform is involved.
//

#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Dense>

#include "PlannerModel.h"
#include "TimedJointTrajectory.h"

// Which sphere pair was responsible, so a violation is actionable. A bare
// minimum distance says a pair is too close but not which part of which arm,
// and the fix (move the path, re-time, change the posture) depends on that.
struct SpherePairIdentity {
    std::size_t left_sphere = 0;
    std::size_t right_sphere = 0;
    double left_radius_m = 0.0;
    double right_radius_m = 0.0;
    Eigen::Vector3d left_centre_m = Eigen::Vector3d::Zero();   // in `world`
    Eigen::Vector3d right_centre_m = Eigen::Vector3d::Zero();
};

struct InterArmClearanceReport {
    bool evaluated = false;
    std::string error;  // set when evaluated is false

    // Surface-to-surface: centre distance minus both radii. Negative means
    // the collision models overlap.
    double minimum_clearance_m = 0.0;
    double time_of_minimum_s = 0.0;
    // The right-arm time the minimum occurred at. Differs from
    // time_of_minimum_s by at most the skew window, and naming it separately
    // shows WHICH relative timing was the worst case.
    double right_time_of_minimum_s = 0.0;
    SpherePairIdentity closest_pair;

    double required_clearance_m = 0.0;
    double skew_window_s = 0.0;
    std::size_t samples_checked = 0;

    bool clearance_satisfied = false;

    std::string Summary() const;
};

// Both internal joint trajectories sampled on one grid and compared over
// the skew window. Nothing here depends on a controller wire format.
//
// `skew_window_s` is the start-time uncertainty to tolerate: 0 means the two
// arms are assumed to start together, which the system does not guarantee.
// Use the configured maximum_start_skew_ms.
//
// `grid_dt_s` is the comparison grid (500 Hz matches the controller's own
// rate). `required_clearance_m` is what must be maintained.
InterArmClearanceReport CheckInterArmClearance(
    const PlannerModel& left_model, const TimedJointTrajectory& left,
    const PlannerModel& right_model, const TimedJointTrajectory& right,
    double required_clearance_m, double skew_window_s, double grid_dt_s);
