//
// PathValidationReport — does GPMP2's final dense internal joint trajectory
// trace the requested mount-frame path without collisions and within arm
// limits?
//
// No joint trajectory crosses the controller boundary. This report sweeps the
// exact final dense result on its uniform grid; only after it passes is every
// state projected to a world-Cartesian CART_TRAJ point. The controller then
// closes Cartesian feedback around that reference, so this is planner-path
// evidence, not a claim that physical execution is identical.
//
// Three errors are reported, and only one of them is the gate:
//
//   e_planner        desired path  vs  GP-dense trajectory
//   e_command        desired path  vs  final dense timed view  <- THE GATE
//   e_reconstruction GP-dense      vs  final dense timed view
//
// The retained field names preserve the report/log API. With the current
// no-decimation dense path, reconstruction should be numerical noise; a larger
// value indicates disagreement inside planner sampling before publication.
//

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "CartesianPath.h"

// Deviation of one trajectory from a reference, over the constrained phase.
struct FidelityError {
    double max_position_m = 0.0;
    double rms_position_m = 0.0;
    double p95_position_m = 0.0;
    double max_orientation_rad = 0.0;
    double worst_time_s = 0.0;          // when the worst point occurred
    double worst_path_parameter = 0.0;  // and where along the path
};

// Circle-specific decomposition. A single distance hides WHICH way the
// trace is wrong: drifting out of the plane is a different failure from
// tracing the wrong radius, and they have different causes.
struct CircleGeometryError {
    bool applicable = false;
    double max_plane_error_m = 0.0;   // |n·(p−c)| — drift out of the plane
    double max_radial_error_m = 0.0;  // ‖(I−nnᵀ)(p−c)‖ − r — wrong radius
};

struct PathValidationReport {
    // --- planning fidelity -----------------------------------------------
    FidelityError planner;         // desired vs GP-dense
    FidelityError command;         // desired vs reconstruction  <- gated
    FidelityError reconstruction;  // GP-dense vs reconstruction
    CircleGeometryError command_circle;

    // --- collision -------------------------------------------------------
    // Named for what it actually checks. The SDF contains the arm's own
    // workspace grid plus an optional operator-supplied box: no wearer, no
    // torso, no second arm. A bare `collision_valid` would read as "safe to
    // move near a person", which this cannot support (CLAUDE.md: never call
    // a plan safe because something would have vetoed it — state what was
    // verified).
    bool modelled_collision_valid = false;
    double minimum_clearance_m = 0.0;
    double minimum_clearance_time_s = 0.0;
    std::string sdf_contents;  // echoed into the report verbatim

    // --- dynamics --------------------------------------------------------
    bool dynamic_limits_valid = false;
    double max_joint_velocity_rad_s = 0.0;
    double max_joint_acceleration_rad_s2 = 0.0;
    double minimum_joint_limit_margin_rad = 0.0;
    bool joint_limits_valid = false;

    // --- start state -----------------------------------------------------
    bool start_state_valid = false;
    double start_configuration_error_rad = 0.0;
    double start_velocity_rad_s = 0.0;
    bool all_finite = false;

    // --- overall ---------------------------------------------------------
    bool optimiser_converged = false;
    bool task_fidelity_valid = false;
    // Every required check passed. NOT a claim that the motion is safe —
    // only that everything MODELLED was verified.
    bool hardware_execution_allowed = false;

    std::string Summary() const;
};

// What the trajectory is measured against and with.
struct ValidationInputs {
    // The requested geometry, in the same frame as the FK below.
    const CartesianPath* desired_task_path = nullptr;
    // Where the task phase begins in trajectory time, so fidelity is
    // measured over the traced phase only — the approach is unconstrained
    // and has no requested geometry to deviate from.
    double task_start_time_s = 0.0;
    double validation_dt_s = 0.002;  // 500 Hz, the controller's own rate
    double maximum_planning_error_m = 0.005;
    double maximum_orientation_error_rad = 0.1;
    Eigen::Matrix<double, 7, 1> measured_start =
        Eigen::Matrix<double, 7, 1>::Zero();
    double start_tolerance_rad = 0.0349;  // 2 deg, the splice guard
    Eigen::Matrix<double, 7, 1> joint_velocity_limits_rad_s =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> joint_acceleration_limits_rad_s2 =
        Eigen::Matrix<double, 7, 1>::Zero();
    // Circle geometry, in the same frame, for the decomposition above.
    bool circle_applicable = false;
    Eigen::Vector3d circle_centre = Eigen::Vector3d::Zero();
    Eigen::Vector3d circle_normal = Eigen::Vector3d::UnitZ();
    double circle_radius_m = 0.0;
};

// The dynamic-limits verdict, per joint: every joint's observed maximum
// against ITS OWN limit. Limits differ per joint (76 deg/s for joints 1-4,
// 66.5 deg/s for 5-7 — Config.h / joint_limits.yaml, 2026-08-13), so a
// max-vs-max comparison would let a joint 5-7 run up to the joint 1-4
// limit unnoticed. Non-positive velocity limits invalidate the verdict;
// non-positive acceleration limits skip the acceleration check (no table
// configured), matching the historical behaviour.
inline bool DynamicLimitsValid(
    const Eigen::Matrix<double, 7, 1>& max_velocity_per_joint_rad_s,
    const Eigen::Matrix<double, 7, 1>& max_acceleration_per_joint_rad_s2,
    const Eigen::Matrix<double, 7, 1>& velocity_limits_rad_s,
    const Eigen::Matrix<double, 7, 1>& acceleration_limits_rad_s2)
{
    if ((velocity_limits_rad_s.array() <= 0.0).any())
        return false;
    if ((max_velocity_per_joint_rad_s.array() >
         velocity_limits_rad_s.array()).any())
        return false;
    if ((acceleration_limits_rad_s2.array() <= 0.0).any())
        return true;
    return (max_acceleration_per_joint_rad_s2.array() <=
            acceleration_limits_rad_s2.array()).all();
}
