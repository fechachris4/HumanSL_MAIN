//
// PathFrames — the one place a Cartesian path crosses into Vicon `world`,
// frame everything downstream of the planner's edge works in.
//
// Why this is separate from CartesianPath.h: a path is pure geometry and
// its header must stay dependency-free so the standalone geometry test can
// link without Pinocchio. Resolving a frame NAME to a transform needs the
// URDF, so it lives here instead, behind the Eigen-only adapter.
//
// Why it takes a POSE rather than a point and a rotation: converting a full
// pose between rigidly-related frames is one homogeneous multiply, which
// gets the translation and the rotation right together. BridgeMain's
// existing ToMount/RotationToMount split that into two functions because
// its inputs arrive separately; a path's samples are already poses, so the
// split buys nothing and only creates a way to convert one half and forget
// the other. That mistake is silent — the result still looks like a valid
// pose.
//

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "CartesianPath.h"
#include "Config.h"  // control — config::ReferenceFrame

// One pose, from its declared frame into Vicon world. world_T_mount is one
// immutable planning snapshot; arm-base transforms come from the URDF via
// Pinocchio and are never hardcoded.
Eigen::Isometry3d PoseToWorld(const Eigen::Isometry3d& pose,
                             config::ReferenceFrame frame,
                             const Eigen::Isometry3d& world_T_mount);

// Every sample of a path, converted into `world`. The returned path's
// `frame` is kWorld, so a converted path cannot be mistaken for an
// unconverted one further downstream.
CartesianPath PathToWorld(const CartesianPath& path,
                          const Eigen::Isometry3d& world_T_mount);
