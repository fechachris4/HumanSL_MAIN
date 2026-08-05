#include "PinocchioKinematicsAdapter.h"

#include <cmath>
#include <map>
#include <memory>

#include "Config.h"
#include "Kinematics.h" // pinocchio types — isolated to this translation unit only

namespace pinocchio_kinematics_adapter {
namespace {

Dynamics& SharedDynamics() {
    static Dynamics dynamics(GEN3_DUAL_URDF_PATH);
    return dynamics;
}

// One DualArmKinematics per end-effector frame requested — the frame is
// fixed at construction. All instances share SharedDynamics()'s model/data;
// safe because calls are sequential (never interleaved), each call reads
// its result out of dynamics_.data_ before the next begins.
DualArmKinematics& SharedKinematics(const std::string& end_effector_frame) {
    static std::map<std::string, std::unique_ptr<DualArmKinematics>> instances;
    auto it = instances.find(end_effector_frame);
    if (it == instances.end()) {
        auto kinematics = std::make_unique<DualArmKinematics>(
            SharedDynamics(), config::kLeftNominalRad, config::kRightBaseFrame,
            end_effector_frame);
        it = instances.emplace(end_effector_frame, std::move(kinematics)).first;
    }
    return *it->second;
}

} // namespace

PoseAndJacobian ToolPoseAndJacobianInBaseLink(const Eigen::Matrix<double, 7, 1>& q_rad,
                                              const std::string& end_effector_frame) {
    static KinematicsWorkspace workspace(SharedDynamics());
    const PoseJacobian result =
        SharedKinematics(end_effector_frame).RightPoseAndJacobian(q_rad, workspace);
    return PoseAndJacobian{result.position, result.rotation, result.jacobian};
}

Eigen::Isometry3d WorldFromBase(bool left_arm)
{
    // Any DualArmKinematics instance carries the same mounting transforms —
    // they come from the model, not the end-effector frame — so reuse the
    // one already built for the configured tool frame rather than
    // constructing another.
    const pinocchio::SE3& mount =
        SharedKinematics(config::kRightEndEffectorFrame)
            .WorldFromBase(left_arm ? Arm::kLeft : Arm::kRight);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = mount.rotation();
    transform.translation() = mount.translation();
    return transform;
}

Eigen::Isometry3d DhRootInBaseLink() {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    return transform;
}

} // namespace pinocchio_kinematics_adapter
