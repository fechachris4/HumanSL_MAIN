#include "PinocchioKinematicsAdapter.h"

#include <cmath>
#include <map>
#include <memory>
#include <utility>

#include "Config.h"
#include "Kinematics.h" // pinocchio types — isolated to this translation unit only

namespace pinocchio_kinematics_adapter {
namespace {

Dynamics& SharedDynamics() {
    static Dynamics dynamics(GEN3_DUAL_URDF_PATH);
    return dynamics;
}

// One DualArmKinematics per (end-effector frame, controlled arm) requested
// — both are fixed at construction. All instances share SharedDynamics()'s
// model/data; safe because calls are sequential (never interleaved), each
// call reads its result out of dynamics_.data_ before the next begins.
DualArmKinematics& SharedKinematics(const std::string& end_effector_frame,
                                    bool left_arm) {
    static std::map<std::pair<std::string, bool>, std::unique_ptr<DualArmKinematics>>
        instances;
    const auto key = std::make_pair(end_effector_frame, left_arm);
    auto it = instances.find(key);
    if (it == instances.end()) {
        // end_effector_frame names a frame on the CONTROLLED arm's own
        // chain, so it fills whichever of the constructor's two
        // end-effector-frame slots that arm owns; the other slot gets the
        // compiled default for a frame it never resolves through.
        auto kinematics = std::make_unique<DualArmKinematics>(
            SharedDynamics(), left_arm ? Arm::kLeft : Arm::kRight,
            left_arm ? config::kRightNominalRad : config::kLeftNominalRad,
            config::kRightBaseFrame,
            left_arm ? config::kRightEndEffectorFrame : end_effector_frame,
            config::kLeftBaseFrame,
            left_arm ? end_effector_frame : config::kLeftEndEffectorFrame);
        it = instances.emplace(key, std::move(kinematics)).first;
    }
    return *it->second;
}

} // namespace

PoseAndJacobian ToolPoseAndJacobianInBaseLink(const Eigen::Matrix<double, 7, 1>& q_rad,
                                              const std::string& end_effector_frame,
                                              bool left_arm) {
    static KinematicsWorkspace workspace(SharedDynamics());
    const PoseJacobian result =
        SharedKinematics(end_effector_frame, left_arm)
            .ControlledPoseAndJacobian(q_rad, workspace);
    return PoseAndJacobian{result.position, result.rotation, result.jacobian};
}

Eigen::Isometry3d MountFromBase(bool left_arm)
{
    // Any DualArmKinematics instance carries the same mounting transforms —
    // they come from the model, not the end-effector frame or which arm is
    // controlled — so reuse the one already built for the right arm's
    // configured tool frame rather than constructing another.
    // Copy the cached transform before converting it. Keeping a reference
    // through the conditional-return expression in MountFromBase triggered a
    // compiler dangling-reference warning even though the cache is static;
    // this fixed-size copy makes the lifetime unambiguous.
    const pinocchio::SE3 mount =
        SharedKinematics(config::kRightEndEffectorFrame, /*left_arm=*/false)
            .MountFromBase(left_arm ? Arm::kLeft : Arm::kRight);
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
