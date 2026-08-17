// TimedJointTrajectory — a transport-independent Eigen view of GPMP2's
// internal joint-space result. The planner may optimise q/qdot internally;
// this type never crosses the controller boundary, whose only wire contract
// is WorldCartesianTrajectory.

#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

struct TimedJointSample {
    double t_s = 0.0;
    Eigen::Matrix<double, 7, 1> q_rad =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> qdot_rad_s =
        Eigen::Matrix<double, 7, 1>::Zero();
};

struct TimedJointTrajectory {
    std::vector<TimedJointSample> samples;
    double duration_s = 0.0;
    bool valid = false;
    std::string error;
};

using TimedJointSampler = std::function<TimedJointSample(double)>;
