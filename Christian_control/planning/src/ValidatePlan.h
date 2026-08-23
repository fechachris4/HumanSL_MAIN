#pragma once
#include <optional>
#include <vector>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include "PlanValidationReport.h"
#include "PlannerModel.h"
#include "MountSdf.h"
#include "TrajectoryOptimization.h"
#include "CartesianPath.h"

struct PlanValidationInputs {
    Eigen::Matrix<double, 7, 1> measured_q_rad = Eigen::Matrix<double, 7, 1>::Zero();
    std::optional<Eigen::Matrix<double, 7, 1>> measured_qdot_rad_s;
    Eigen::Matrix<double, 7, 1> position_lower_rad = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> position_upper_rad = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> effective_velocity_rad_s = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> effective_acceleration_rad_s2 = Eigen::Matrix<double, 7, 1>::Zero();
    const std::vector<NamedObstacleField>* obstacle_fields = nullptr;
    double minimum_clearance_m = 0.05;
    gtsam::Pose3 requested_terminal_mount;
    gtsam::Pose3 candidate_terminal_mount;
    PlanStatus intended_status = PlanStatus::kReached;
    const CartesianPath* desired_task_path = nullptr;
    double task_start_time_s = 0.0;
    double validation_dt_s = 0.002;
    // Task acceptance tolerances. Validation criteria, not physical
    // constraints: a caller with a looser task (or a tighter one) sets them
    // here instead of editing validator code.
    double terminal_position_tolerance_m = 0.001;
    double terminal_orientation_tolerance_rad = 0.01;
    // Absolute bound on the dense executed-path deviation (metres). Zero
    // disables the path gate; traced-path solves pass their acceptance
    // tolerance here. A fidelity failure is a SHAPE defect — it asks for
    // geometric repair and is never fixed by stretching time (shape and
    // timing are decoupled).
    double path_position_tolerance_m = 0.0;
    // Numerical consistency tolerance between the trajectory's first state
    // and the measured start the plan was requested from. The planner
    // CONSTRUCTS the trajectory to start at the measured state; validation
    // only cross-checks that invariant, so this is a floating-point
    // consistency bound, not a physical requirement.
    double start_position_tolerance_rad = 1e-8;
    double start_velocity_tolerance_rad_s = 1e-8;
};

PlanValidationReport ValidatePlan(const PlannerModel& model,
                                  const TrajectoryResult& trajectory,
                                  double duration_s,
                                  const PlanValidationInputs& inputs);

// The same scene/self-clearance measurement as dense validation, without a
// trajectory verdict.
PlanValidationReport MeasureConfigurationClearance(
    const PlannerModel& model,
    const Eigen::Matrix<double, 7, 1>& q_rad,
    const std::vector<NamedObstacleField>& obstacle_fields,
    double minimum_clearance_m);
