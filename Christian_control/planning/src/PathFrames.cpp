#include "PathFrames.h"

#include <stdexcept>

#include "PinocchioKinematicsAdapter.h"

Eigen::Isometry3d PoseToWorld(const Eigen::Isometry3d& pose,
                             config::ReferenceFrame frame,
                             const Eigen::Isometry3d& world_T_mount) {
    switch (frame) {
    case config::ReferenceFrame::kWorld:
        return pose;
    case config::ReferenceFrame::kMount:
        return world_T_mount * pose;
    case config::ReferenceFrame::kRightBase:
    case config::ReferenceFrame::kLeftBase: {
        // T_W_pose = T_W_M T_M_B T_B_pose. One expression carries both
        // rotation and translation through the complete frame chain.
        const bool declared_left_arm = frame == config::ReferenceFrame::kLeftBase;
        return world_T_mount *
               pinocchio_kinematics_adapter::MountFromBase(declared_left_arm) *
               pose;
    }
    }
    throw std::invalid_argument("unhandled reference frame in PoseToWorld");
}

CartesianPath PathToWorld(const CartesianPath& path,
                          const Eigen::Isometry3d& world_T_mount) {
    CartesianPath converted;
    converted.frame = config::ReferenceFrame::kWorld;
    converted.samples.reserve(path.samples.size());
    for (const PathSample& sample : path.samples) {
        PathSample moved;
        moved.t_s = sample.t_s;  // timing is frame-independent
        moved.pose = PoseToWorld(sample.pose, path.frame, world_T_mount);
        converted.samples.push_back(moved);
    }
    return converted;
}
