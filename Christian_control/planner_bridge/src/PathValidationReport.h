//
// PathValidationReport — does the trajectory the controller will ACTUALLY
// execute trace the shape that was asked for, without collisions and within
// the arm's limits?
//
// The distinction matters. The pipeline is
//
//   GPMP2 dense solve  ->  subsample to <= 1000 points  ->  wire block
//                      ->  controller reconstructs by CUBIC HERMITE
//                      ->  500 Hz commanded joint trajectory  ->  robot
//
// so validating the GP-dense trajectory validates the planner's INTENTION,
// not the arm's motion. Subsampling discards points and Hermite is not the
// GP interpolant, and both distortions land after the optimiser is finished.
// Everything here is therefore measured on the reconstruction of the FINAL
// emitted block, using the controller's own SampleJointTrajectory rather
// than a second implementation of it — a validator that reconstructed what
// it merely believed the controller does could pass a trajectory the
// controller then executes differently.
//
// Three errors are reported, and only one of them is the gate:
//
//   e_planner        desired path  vs  GP-dense trajectory
//   e_command        desired path  vs  500 Hz reconstruction   <- THE GATE
//   e_reconstruction GP-dense      vs  500 Hz reconstruction
//
// e_reconstruction is the transport loss introduced after planning. It is
// reported so that a fidelity failure can be attributed: a large e_planner
// with a small e_reconstruction means the optimiser could not hold the path,
// while the reverse means the wire format threw away the accuracy the
// optimiser achieved. They are not added — they are vector-valued and occur
// at different points along the path.
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
