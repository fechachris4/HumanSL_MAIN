#include "GenerateTrajectory.h"
#include "TrajectoryOptimization.h"

// GenerateTrajectory.h declares this free function but never defined it —
// OptimizeTrajectory::optimizeJointTrajectory (TrajectoryOptimization.h) is
// the only implementation in the tree. This is a thin forwarding shim: zero
// start velocity, class defaults for target_dt/tolerances.
TrajectoryResult optimizeJointTrajectory(
    const gpmp2::ArmModel& arm_model,
    const gpmp2::SignedDistanceField& sdf,
    const gtsam::Values& init_values,
    const gtsam::Pose3& target_pose,
    const gtsam::Vector& start_config,
    const JointLimits& pos_limits,
    const JointLimits& vel_limits,
    const size_t total_time_step,
    const double total_time_sec) {
    OptimizeTrajectory optimizer;
    const gtsam::Vector start_vel = gtsam::Vector::Zero(start_config.size());
    return optimizer.optimizeJointTrajectory(arm_model, sdf, init_values, target_pose,
                                              start_config, start_vel, pos_limits, vel_limits,
                                              total_time_step, total_time_sec);
}
