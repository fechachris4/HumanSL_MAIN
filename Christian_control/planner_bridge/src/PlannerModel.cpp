#include "PlannerModel.h"
#include <cmath>

gtsam::Pose3 DhBaseInBaseLink() {
    return gtsam::Pose3(gtsam::Rot3::Rx(M_PI), gtsam::Point3(0, 0, 0));
}

PlannerModel LoadPlannerModel(const std::string& yaml_path) {
    PlannerModel model;
    model.dh = createDHParams(yaml_path);
    model.base_pose = DhBaseInBaseLink();
    ArmModel factory;
    model.arm_model = factory.createArmModel(model.base_pose, model.dh);
    return model;
}

gtsam::Pose3 ToolPoseInBaseLink(const PlannerModel& model,
                                const Eigen::Matrix<double, 7, 1>& q_rad) {
    return forwardKinematics(model.dh, gtsam::Vector(q_rad), model.base_pose);
}

Eigen::Vector3d ToolPositionInBaseLink(const PlannerModel& model,
                                       const Eigen::Matrix<double, 7, 1>& q_rad) {
    return ToolPoseInBaseLink(model, q_rad).translation();
}
