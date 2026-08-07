#include "PathFrames.h"

#include <stdexcept>

#include "PinocchioKinematicsAdapter.h"

Eigen::Isometry3d PoseToMount(const Eigen::Isometry3d& pose,
                              config::ReferenceFrame frame) {
    switch (frame) {
    case config::ReferenceFrame::kMount:
        return pose;
    case config::ReferenceFrame::kRightBase:
    case config::ReferenceFrame::kLeftBase: {
        // T_mount_pose = T_mount_base * T_base_pose. One multiply carries
        // both the rotation and the translation, which is the whole reason
        // this takes a pose rather than the two halves separately.
        const bool declared_left_arm = frame == config::ReferenceFrame::kLeftBase;
        return pinocchio_kinematics_adapter::MountFromBase(declared_left_arm) * pose;
    }
    }
    throw std::invalid_argument("unhandled reference frame in PoseToMount");
}

CartesianPath PathToMount(const CartesianPath& path) {
    CartesianPath converted;
    converted.frame = config::ReferenceFrame::kMount;
    converted.samples.reserve(path.samples.size());
    for (const PathSample& sample : path.samples) {
        PathSample moved;
        moved.t_s = sample.t_s;  // timing is frame-independent
        moved.pose = PoseToMount(sample.pose, path.frame);
        converted.samples.push_back(moved);
    }
    return converted;
}
