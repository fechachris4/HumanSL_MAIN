#pragma once
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "GenerateArmModel.h"
#include "utils.h"

// Where the arm's DH root sits in Vicon `world`: the immutable planning
// snapshot composed with the URDF mounting
// transform composed with the DH convention's pi-about-x offset from
// base_link (measured 2026-08-05, thesis notes §7). Building the gpmp2 arm
// with this base pose is what makes EVERY planner-side quantity — FK, the
// collision spheres, the goal, the SDF it is paired with — come out in one
// shared frame rather than in whichever arm's base_link happens to be
// running. `left_arm` selects which mount.
gtsam::Pose3 DhRootInWorld(bool left_arm,
                           const Eigen::Isometry3d& world_T_mount);

struct PlannerModel {
    DHParameters dh;
    gtsam::Pose3 base_pose;                      // = DhRootInWorld(...)
    Eigen::Isometry3d world_T_mount = Eigen::Isometry3d::Identity();
    std::unique_ptr<gpmp2::ArmModel> arm_model;  // collision-sphere model
    // Which arm this model describes, for every Pinocchio-backed evaluation
    // (utils::forwardKinematics, analytical_ik) that must query the SAME
    // chain the dh table was generated from — mismatching them offsets
    // every pose by the tool-vs-flange difference (~0.12 m).
    bool left_arm = false;
    std::string end_effector_frame;  // "right_tool_link" / "left_end_effector_link"
};

// yaml_path: the build-generated dh_params_tool.yaml (right arm, d7 carries
// the configured-tool offset) or dh_params_flange.yaml (left arm, bare
// flange). has_tool must match which one: true for dh_params_tool.yaml,
// false for dh_params_flange.yaml — it selects whether the collision model
// includes the mounted tool/gripper's footprint (GenerateArmModel.h), and
// which arm/frame the returned model's Pinocchio-backed evaluations query
// (tool <-> right and flange <-> left are physically one-to-one: the tool
// is mounted on the right flange only).
PlannerModel LoadPlannerModel(const std::string& yaml_path, bool has_tool,
                              const Eigen::Isometry3d& world_T_mount);

// Tool position in Vicon world, metres. q_rad: Kortex actuator order.
Eigen::Vector3d ToolPositionInWorld(const PlannerModel& model,
                                   const Eigen::Matrix<double, 7, 1>& q_rad);
// Full pose counterpart (rotation used for the goal-pose prior).
gtsam::Pose3 ToolPoseInWorld(const PlannerModel& model,
                            const Eigen::Matrix<double, 7, 1>& q_rad);
