#include "PlannerModel.h"
#include <cmath>

#include "Config.h"  // basic_control — the two end-effector frame names
#include "PinocchioKinematicsAdapter.h"

namespace {

gtsam::Pose3 ToGtsam(const Eigen::Isometry3d& transform) {
    return gtsam::Pose3(gtsam::Rot3(transform.linear()),
                        gtsam::Point3(transform.translation()));
}

}  // namespace

gtsam::Pose3 DhRootInMount(bool left_arm) {
    // T_mount_dhroot = T_mount_baselink * T_baselink_dhroot. The first factor
    // is read from the URDF through Pinocchio (never hardcoded, so surveying
    // the rig and regenerating the URDF needs no code change here); the
    // second is the DH convention's own pi-about-x offset, which is geometry
    // of the convention rather than of the rig and so is a constant.
    return ToGtsam(pinocchio_kinematics_adapter::MountFromBase(left_arm) *
                   pinocchio_kinematics_adapter::DhRootInBaseLink());
}

PlannerModel LoadPlannerModel(const std::string& yaml_path, bool has_tool) {
    PlannerModel model;
    model.dh = createDHParams(yaml_path);
    model.left_arm = !has_tool;  // tool <-> right, flange <-> left (see the .h)
    model.base_pose = DhRootInMount(model.left_arm);
    model.end_effector_frame = has_tool ? config::kRightEndEffectorFrame
                                        : config::kLeftEndEffectorFrame;
    ArmModel factory;
    model.arm_model = factory.createArmModel(model.base_pose, model.dh, has_tool);
    return model;
}

gtsam::Pose3 ToolPoseInMount(const PlannerModel& model,
                             const Eigen::Matrix<double, 7, 1>& q_rad) {
    return forwardKinematics(model.dh, gtsam::Vector(q_rad), model.base_pose,
                             model.end_effector_frame, model.left_arm);
}

Eigen::Vector3d ToolPositionInMount(const PlannerModel& model,
                                    const Eigen::Matrix<double, 7, 1>& q_rad) {
    return ToolPoseInMount(model, q_rad).translation();
}
