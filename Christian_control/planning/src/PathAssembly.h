//
// PathAssembly — one optimisation problem covering two phases: getting to
// the task path, and tracing it.
//
// They are shaped differently on purpose.
//
// APPROACH. Task-space UNCONSTRAINED: its waypoints carry no pose prior, so
// only smoothness, joint limits and obstacle avoidance shape them. The arm
// must arrive at the path; the route there was not part of what the user
// asked for, and constraining it would spend solver effort on a segment
// nobody specified and could demand a pose that is not reachable. It is
// NOT a Cartesian straight line and must not be described as one — the
// initial guess interpolates in JOINT space, and the optimiser is then free
// to move it anywhere.
//
// TASK. Every support point carries a pose prior, so the traced shape is
// what was requested.
//
// Waypoint 0 is the measured configuration, pinned by the optimiser's stiff
// JOINT prior. It gets no pose prior: two stiff priors on one state would
// compete, and the controller's splice guard rejects a block whose first
// point is more than 2 deg from where the arm actually is, so the joint
// prior is the one that must win.
//
// Eigen only — see OptimisationWaypoint.h on the dependency direction.
//

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "CartesianPath.h"
#include "OptimisationWaypoint.h"

// Per-joint velocity limits, radians/second, used to pace the approach.
using JointVelocityLimits = Eigen::Matrix<double, 7, 1>;

struct ApproachPacing {
    // Fraction of each joint's velocity limit the approach is paced at.
    // Below 1 because the initial guess is a joint-space interpolation the
    // optimiser will bend; pacing at the limit leaves the solve no room to
    // lengthen a joint's path without exceeding it.
    double velocity_fraction = 0.3;
    double minimum_duration_s = 2.0;
    int waypoints = 5;  // interior approach states, excluding waypoint 0
};

struct AssembledPath {
    std::vector<OptimisationWaypoint> waypoints;
    // Support states pinned to rest: trajectory start, the task-path start
    // (so the arm STOPS before tracing rather than blending an arbitrary
    // approach direction into the path tangent) and the end.
    std::vector<std::size_t> zero_velocity_indices;
    // Index of the first task waypoint, so the validator can measure
    // fidelity over the traced phase alone — the approach has no requested
    // geometry to be faithful to.
    std::size_t task_start_index = 0;
    double total_duration_s = 0.0;
    // Initial guess, one entry per waypoint: joint interpolation across the
    // approach, then the continuation-IK solutions for the task path.
    std::vector<Eigen::Matrix<double, 7, 1>> initial_configurations;
};

// `task_path` must already be in the frame the optimiser works in, and
// `task_configurations` must be the continuation-IK solutions or bounded-gap
// interpolated seeds produced by PathIk, one per sample. `measured` is where
// the arm actually is.
//
// The approach duration is derived from JOINT displacement over joint
// velocity limits, not from Cartesian distance: the approach has no
// commanded Cartesian route, so a Cartesian length would be describing a
// path that does not exist.
//
// Throws std::invalid_argument if the sizes disagree or the task path is
// too short to trace.
AssembledPath AssembleCirclePlan(const CartesianPath& task_path,
                                 const std::vector<Eigen::Matrix<double, 7, 1>>&
                                     task_configurations,
                                 const Eigen::Matrix<double, 7, 1>& measured,
                                 const JointVelocityLimits& joint_velocity_limits,
                                 const ApproachPacing& pacing,
                                 const Eigen::Vector3d& rotation_sigma_rad,
                                 const Eigen::Vector3d& translation_sigma_m);
