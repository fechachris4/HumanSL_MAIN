#include "PathFrames.h"

#include <stdexcept>

#include "PinocchioKinematicsAdapter.h"

Eigen::Isometry3d ToMount(
    const Eigen::Isometry3d& pose,
    config::ReferenceFrame source_frame,
    const std::optional<Eigen::Isometry3d>& world_T_mount) {
    switch (source_frame) {
    case config::ReferenceFrame::kMount:
        return pose;
    case config::ReferenceFrame::kRightBase:
    case config::ReferenceFrame::kLeftBase: {
        // T_M_pose = T_M_B T_B_pose. One expression carries both rotation
        // and translation through the complete frame chain.
        const bool declared_left_arm =
            source_frame == config::ReferenceFrame::kLeftBase;
        return pinocchio_kinematics_adapter::MountFromBase(declared_left_arm) *
               pose;
    }
    case config::ReferenceFrame::kWorld: {
        if (!world_T_mount || !world_T_mount->matrix().allFinite())
            throw ToMountError(
                "world-declared input needs a valid world_T_mount snapshot "
                "(none was provided)");
        // T_M_pose = T_W_M^-1 T_W_pose.
        return world_T_mount->inverse() * pose;
    }
    }
    throw std::invalid_argument("unhandled reference frame in ToMount");
}

Eigen::Vector3d PointToMount(
    const Eigen::Vector3d& point,
    config::ReferenceFrame source_frame,
    const std::optional<Eigen::Isometry3d>& world_T_mount) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = point;
    return ToMount(pose, source_frame, world_T_mount).translation();
}

Eigen::Matrix3d RotationToMount(
    const Eigen::Matrix3d& rotation,
    config::ReferenceFrame source_frame,
    const std::optional<Eigen::Isometry3d>& world_T_mount) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = rotation;
    return ToMount(pose, source_frame, world_T_mount).linear();
}

CartesianPath PathToMount(
    const CartesianPath& path,
    const std::optional<Eigen::Isometry3d>& world_T_mount) {
    CartesianPath converted;
    converted.frame = config::ReferenceFrame::kMount;
    converted.samples.reserve(path.samples.size());
    for (const PathSample& sample : path.samples) {
        PathSample moved;
        moved.t_s = sample.t_s;  // timing is frame-independent
        moved.pose = ToMount(sample.pose, path.frame, world_T_mount);
        converted.samples.push_back(moved);
    }
    return converted;
}
