#pragma once
#include <memory>
#include <string>
#include <Eigen/Dense>
#include "GenerateArmModel.h"
#include "utils.h"

// The DH base frame is rotated pi about x relative to base_link (measured
// 2026-08-05, thesis notes §7). Building the gpmp2 arm with this base pose
// makes every planner-side quantity come out directly in base_link.
gtsam::Pose3 DhBaseInBaseLink();

struct PlannerModel {
    DHParameters dh;
    gtsam::Pose3 base_pose;                      // = DhBaseInBaseLink()
    std::unique_ptr<gpmp2::ArmModel> arm_model;  // collision-sphere model
};

// yaml_path: config/dh_params_tool.yaml (d7 tool-matched, -0.2874).
PlannerModel LoadPlannerModel(const std::string& yaml_path);

// Tool position in base_link, metres. q_rad: Kortex actuator order.
Eigen::Vector3d ToolPositionInBaseLink(const PlannerModel& model,
                                       const Eigen::Matrix<double, 7, 1>& q_rad);
// Full pose counterpart (rotation used for the goal-pose prior).
gtsam::Pose3 ToolPoseInBaseLink(const PlannerModel& model,
                                const Eigen::Matrix<double, 7, 1>& q_rad);
