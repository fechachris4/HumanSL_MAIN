#pragma once
// Core data types for the joint-control MPC seam.
// Eigen-only — deliberately independent of TrajectoryGeneration/utils.h
// (which pulls in GTSAM/GPMP2/yaml-cpp) so this builds on a laptop with no robot.
#include <Eigen/Dense>

namespace jmpc {

// Measured/estimated joint state fed back to a controller each tick.
struct JointState {
    Eigen::VectorXd q;       // joint angles (rad)
    Eigen::VectorXd dq;      // joint velocities (rad/s)
    double t = 0.0;          // time stamp (s)
};

// One control command: the thing an MPC produces and the backend consumes.
struct JointSetpoint {
    Eigen::VectorXd q_des;   // desired joint angles (rad)
    Eigen::VectorXd dq_des;  // desired joint velocities (rad/s), advisory
};

// Per-joint limits. Continuous joints use a very large position bound.
struct JointLimits {
    Eigen::VectorXd q_lower; // rad
    Eigen::VectorXd q_upper; // rad
    Eigen::VectorXd dq_max;  // rad/s (symmetric: |dq| <= dq_max)
};

}  // namespace jmpc
