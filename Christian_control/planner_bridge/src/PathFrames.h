//
// PathFrames — the one place a Cartesian path crosses into `mount`, the
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
#include "Config.h"  // basic_control — config::ReferenceFrame

// One pose, from its declared frame into `mount`. Frames are rigidly
// related and static (the rig is bolted down), so this is a constant
// transform read from the URDF via Pinocchio — never a hardcoded constant.
//
// A room/world frame, when Vicon supplies one, is added HERE and nowhere
// else: it composes above mount as T_mount_room, and every caller of this
// function keeps working untouched.
Eigen::Isometry3d PoseToMount(const Eigen::Isometry3d& pose,
                              config::ReferenceFrame frame);

// Every sample of a path, converted into `mount`. The returned path's
// `frame` is kMount, so a converted path cannot be mistaken for an
// unconverted one further downstream.
CartesianPath PathToMount(const CartesianPath& path);
