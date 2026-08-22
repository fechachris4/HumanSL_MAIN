#include "GenerateTrajectory.h"
#include <chrono>
#include "TrajectoryOptimization.h"

// GenerateTrajectory.h declares this free function but never defined it —
// OptimizeTrajectory::optimizeJointTrajectory (TrajectoryOptimization.h) is
// the only implementation in the tree. This is a thin forwarding shim for
// the optional measured start velocity, graph weights and default target_dt.
//
// OptimizeTrajectory::optimizeJointTrajectory never populates
// TrajectoryResult::optimization_duration/initiation_duration on the struct
// it returns (a local duration is computed and discarded — see
// TrajectoryOptimization.cpp:561), leaving both fields default-constructed
// (indeterminate). Timing the call here, at the shim boundary, fills in
// optimization_duration without touching that file; initiation_duration is
// set to zero rather than left indeterminate (this shim does no separate
// initiation step — InitializeTrajectory's timing, if any, belongs to its
// own caller).
TrajectoryResult optimizeJointTrajectory(
    const gpmp2::ArmModel& arm_model,
    const std::vector<NamedObstacleField>& obstacle_fields,
    const gtsam::Values& init_values,
    const gtsam::Pose3& target_pose,
    const gtsam::Vector& start_config,
    const std::optional<gtsam::Vector>& start_vel,
    const JointLimits& pos_limits,
    const JointLimits& vel_limits,
    const size_t total_time_step,
    const double total_time_sec,
    const OptimizerTuning& tuning) {
    OptimizeTrajectory optimizer;
    const auto start_time = std::chrono::steady_clock::now();
    TrajectoryResult result = optimizer.optimizeJointTrajectory(
        arm_model, obstacle_fields, init_values, target_pose, start_config, start_vel,
        pos_limits, vel_limits, total_time_step, total_time_sec, tuning);
    const auto end_time = std::chrono::steady_clock::now();
    result.optimization_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    result.initiation_duration = std::chrono::milliseconds::zero();
    return result;
}
