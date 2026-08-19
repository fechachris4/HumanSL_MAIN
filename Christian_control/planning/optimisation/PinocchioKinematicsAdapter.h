// PinocchioKinematicsAdapter — the planner side's entry point onto
// control's already-Pinocchio/URDF-based DualArmKinematics. Wraps it
// so utils.cpp / analytical_ik.h can evaluate FK and Jacobians against the
// canonical URDF instead of hand-rolled DH math.
//
// Eigen-only, deliberately: gtsam and Pinocchio each vendor their own
// boost::serialization overloads for Eigen::Matrix (gtsam/base/
// MatrixSerialization.h, pinocchio/serialization/eigen.hpp), and neither
// guards against the other already being defined — the two collide with a
// hard "redefinition" compile error if both ever reach the same
// translation unit. This header must stay includable from files that also
// include gtsam (utils.cpp, analytical_ik.h), so it never includes
// Kinematics.h/Dynamics.h itself; only PinocchioKinematicsAdapter.cpp does.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

namespace pinocchio_kinematics_adapter {

// Mirrors control's PoseJacobian (Kinematics.h) field-for-field, so
// converting between them at the .cpp boundary is a one-line aggregate
// copy — kept as a separate type rather than reusing PoseJacobian directly
// so this header never has to include Kinematics.h (see above).
struct PoseAndJacobian {
    Eigen::Vector3d position;             // meters, base_link axes
    Eigen::Matrix3d rotation;             // orientation, base_link axes
    Eigen::Matrix<double, 6, 7> jacobian; // [linear; angular] x joints 1-7, base_link axes
};

// Tool pose + 6x7 Jacobian in ITS OWN base_link axes (base_link for the
// right arm, left_base_link for the left), for the given end-effector frame
// (a URDF link name naming a frame on THAT arm's chain, e.g.
// "right_tool_link" or "left_end_effector_link"), at the given
// seven-joint configuration for that same arm. left_arm defaults to false
// (right), so every call site written before the left arm existed is
// unchanged.
//
// Not thread-safe: lazily constructs and reuses one DualArmKinematics
// instance per distinct (frame name, arm) pair requested (function-local
// statics with mutable internal state). Fine for planner_bridge's current
// single-threaded batch usage; would need per-thread instances if the
// planner is ever parallelized.
PoseAndJacobian ToolPoseAndJacobianInBaseLink(const Eigen::Matrix<double, 7, 1>& q_rad,
                                              const std::string& end_effector_frame,
                                              bool left_arm = false);

// T_mount_base for one arm, read from the canonical URDF via Pinocchio —
// the ONLY way planner-side code should obtain the mounting transform, so
// that changing config/dual_arm_mounting.yaml (and the URDF generated from
// it) needs no code change anywhere. Never hardcode this.
//
// Constant for a given model: both mounts are fixed joints onto the `mount`
// root, so the result does not depend on the configuration.
//
// `arm` selects which base: false = right (base_link), true = left
// (left_base_link). A bool rather than an enum so this header stays free of
// control's Config.h, for the gtsam/Pinocchio isolation reason above.
Eigen::Isometry3d MountFromBase(bool left_arm);

// The fixed DH-root-to-base_link offset every hand-rolled DH chain in this
// codebase assumes (a pure rotation, zero translation — the DH convention's
// root frame and base_link share an origin, just rotated). Every DH-chain
// caller composes with this to land in base_link, exactly as the DH chain
// used to.
Eigen::Isometry3d DhRootInBaseLink();

} // namespace pinocchio_kinematics_adapter
