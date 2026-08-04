//
// Targets — implementations for Targets.h.
//

#include <utility>

#include "Targets.h"

Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw)
{
    // R = Rz(yaw) · Ry(pitch) · Rx(roll) — the simulation's convention
    // (msc_project controller/transforms.py rotation_from_rpy).
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

// ---------------------------------------------------------------
// PoseTargetSource
// ---------------------------------------------------------------

PoseTargetSource::PoseTargetSource(PoseTarget target)
    : target_(std::move(target))
{}

Reference PoseTargetSource::Get(const RobotState& /*state*/, double /*dt_s*/,
                                ControllerStatus& /*status*/)
{
    Reference reference;
    // The fixed target is STATIONARY: a compiled pose is a place
    // to be, not a motion, so the reference twist stays zero and the Kd
    // term stays pure damping. A source that moves its target (an
    // orientation policy, a Cartesian path) fills the twist instead.
    // An empty rotation means the controller keeps the takeover
    // orientation.
    reference.pose =
        PoseReference{target_.p_desired, target_.rotation, Twist{}, 0};
    return reference;
}
