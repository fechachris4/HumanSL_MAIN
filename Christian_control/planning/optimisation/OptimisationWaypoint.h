//
// OptimisationWaypoint — what the optimiser is ASKED to do at one support
// state, as distinct from the geometry the user requested.
//
// The separation is deliberate. `CartesianPath` says where the tool should
// go; it carries no notion of whether a pose is enforced, because that is
// an optimisation decision rather than a property of the shape. Putting a
// `constrained` flag inside the path would let the two drift apart and
// would make "constrained, but no target pose" representable. Here the
// target and its weights travel together inside one `optional`, so that
// state cannot be expressed at all.
//
// Eigen only. Dependencies in this project flow downward —
//
//     geometry / task  ->  Eigen
//     kinematics       ->  Pinocchio
//     optimisation     ->  GTSAM
//
// — and never upward. gtsam and Pinocchio each vendor their own
// boost::serialization overloads for Eigen::Matrix and collide with a hard
// redefinition error if they meet in one translation unit (see
// PinocchioKinematicsAdapter.h), so a gtsam-typed waypoint could not reach
// the Pinocchio-side IK that seeds it. The conversion to gtsam::Pose3
// happens once, where the factor is constructed.
//

#pragma once

#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// A pose the optimiser should pull a support state toward, and how hard.
//
// Both sigmas are gtsam noise-model sigmas: SMALLER means STIFFER, which is
// the opposite of what "tolerance" suggests. They are weights on a cost, not
// a guarantee — the pass/fail requirement is a separate number
// (path_following.maximum_planning_error_m) measured after the solve.
//
// Anisotropic from the start, because the tasks this is for want it: a
// drawing task cares more about the surface normal than the tangent, an
// insertion cares more laterally than axially, and carrying an object
// upright cares about roll and pitch more than yaw. Config may expand one
// scalar into all three axes; the type does not have to lose the ability.
struct PosePrior {
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    Eigen::Vector3d rotation_sigma_rad = Eigen::Vector3d::Constant(0.01);
    Eigen::Vector3d translation_sigma_m = Eigen::Vector3d::Constant(0.01);
};

// One support state of the optimisation problem.
//
// No pose prior means the optimiser is free to place this state however
// smoothness, joint limits and obstacle avoidance prefer. That is what the
// approach phase uses: the arm must arrive at the task path, but the route
// there is not part of what was requested, so constraining it would only
// spend solver effort and risk demanding something unreachable.
struct OptimisationWaypoint {
    double time_s = 0.0;
    std::optional<PosePrior> pose_prior;
};

// Strictly increasing times starting at zero, which the optimiser relies on
// to derive its support interval. Returns false on an empty or unordered
// sequence rather than throwing, so callers can report the offending index.
bool WaypointTimesAreMonotonic(const std::vector<OptimisationWaypoint>& waypoints,
                               std::size_t* first_bad_index = nullptr);
