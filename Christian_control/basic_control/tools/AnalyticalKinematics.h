//
// AnalyticalKinematics — closed-form forward kinematics for the right Gen3
// arm, computed as an explicit chain of rigid transforms (no URDF parsing,
// no Pinocchio). Same style as TrajectoryExecution's Fwd_kinematics.cpp, but
// the link offsets below are read directly from this project's own URDF
// (config/GEN3_dual_mounted.urdf), which was diff-checked in August 2026
// against Kinova's official ros_kortex GEN3-7DOF-VISION_ARM_URDF_V12 and
// found identical. Exists as an independent cross-check on Kinematics.cpp's
// Pinocchio-based FK: two differently-implemented paths agreeing rules out
// a coding bug in either one.
//

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Forward kinematics of ConfiguredTool_Link (the mounted tool, not the bare
// flange — see config/GEN3_dual_mounted.urdf) in base_link axes, computed
// from the seven right-arm joint angles (radians, Kortex actuator order,
// continuous joints unwrapped — same convention as DualArmKinematics'
// right_q_rad). No Pinocchio, no file I/O: pure function of q.
Eigen::Isometry3d AnalyticalForwardKinematics(
    const Eigen::Matrix<double, 7, 1>& q_rad);
