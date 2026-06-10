#pragma once
// Kinova Gen3 7-DoF joint limits, hardcoded to mirror config/joint_limits.yaml
// (radians). Hardcoding keeps the core free of a yaml-cpp dependency.
//   - Joints 1,3,5,7 are continuous rotation -> represented as +/-1e20.
//   - Velocity limit 0.8727 rad/s (50 deg/s), Kinova Table 41.
#include "joint_types.h"

namespace jmpc {

constexpr int kDof = 7;

inline JointLimits gen3_limits() {
    JointLimits L;
    L.q_lower = Eigen::VectorXd(kDof);
    L.q_lower << -1e20, -2.2515, -1e20, -2.5807, -1e20, -2.0996, -1e20;
    L.q_upper = Eigen::VectorXd(kDof);
    L.q_upper <<  1e20,  2.2515,  1e20,  2.5807,  1e20,  2.0996,  1e20;
    L.dq_max = Eigen::VectorXd::Constant(kDof, 0.8727);  // rad/s
    return L;
}

}  // namespace jmpc
