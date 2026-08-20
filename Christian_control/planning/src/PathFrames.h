//
// PathFrames — the planner's ONE frame-conversion module.
//
// Everything inside the planner — IK, GPMP2, the SDF, validation — is
// expressed in `mount` (the URDF root, midway between the two arm bases).
// Inputs may be declared in mount, left_base, right_base or Vicon `world`;
// they cross into mount exactly once, here, at the planner's edge. The only
// mount->world conversion is the separate output projection that builds the
// controller's WorldCartesianTrajectory (WorldTrajectoryProjection).
//
// Why one pose-level core: converting a full pose between rigidly-related
// frames is one homogeneous multiply, which gets the translation and the
// rotation right together. The point and rotation helpers below call that
// core rather than re-deriving the chain, so the two halves of a pose can
// never be converted inconsistently. That mistake is silent — the result
// still looks like a valid pose.
//
// world_T_mount is one immutable planning snapshot; a world-declared input
// REQUIRES it and is rejected (ToMountError) when it is absent or
// non-finite. Rejection is per-request: the caller reports it and the
// planner keeps serving mount/base requests. Arm-base transforms come from
// the URDF via Pinocchio and are never hardcoded.
//

#pragma once

#include <optional>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "CartesianPath.h"
#include "Config.h"  // control — config::ReferenceFrame

// Thrown when a conversion cannot be performed: a world-declared input with
// no valid world_T_mount snapshot. Distinct from std::invalid_argument so
// the boundary can report "this request needs Vicon" rather than a generic
// parse error.
struct ToMountError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// THE core conversion: one pose, from its declared frame into `mount`.
Eigen::Isometry3d ToMount(
    const Eigen::Isometry3d& pose,
    config::ReferenceFrame source_frame,
    const std::optional<Eigen::Isometry3d>& world_T_mount);

// A position only (a pose with identity rotation). Calls ToMount.
Eigen::Vector3d PointToMount(
    const Eigen::Vector3d& point,
    config::ReferenceFrame source_frame,
    const std::optional<Eigen::Isometry3d>& world_T_mount);

// An orientation only (a pose with zero translation). Calls ToMount.
Eigen::Matrix3d RotationToMount(
    const Eigen::Matrix3d& rotation,
    config::ReferenceFrame source_frame,
    const std::optional<Eigen::Isometry3d>& world_T_mount);

// Every sample of a path, converted into `mount`. The returned path's
// `frame` is kMount, so a converted path cannot be mistaken for an
// unconverted one further downstream.
CartesianPath PathToMount(
    const CartesianPath& path,
    const std::optional<Eigen::Isometry3d>& world_T_mount);
