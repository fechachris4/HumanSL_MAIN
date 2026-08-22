#pragma once

#include <string>
#include <variant>
#include <vector>

#include <Eigen/Dense>

// Persistent planner obstacle geometry, expressed in metres in the mount
// frame. AxisAlignedBox keeps the member spellings used by the existing SDF
// caller until that request path is retired.
struct AxisAlignedBox {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d half_extent = Eigen::Vector3d::Zero();
};

struct MountCylinder {
    Eigen::Vector3d center_mount_m = Eigen::Vector3d::Zero();
    double radius_m = 0.0;
    double height_m = 0.0;
};

using StaticObstacleGeometry = std::variant<AxisAlignedBox, MountCylinder>;

enum class CollisionSphereGroup {
    kMountInterface,
    kProximalArm,
    kUpperArm,
    kForearm,
    kTool
};

struct NamedStaticObstacle {
    std::string id;
    bool enabled = false;
    StaticObstacleGeometry geometry;
    std::vector<CollisionSphereGroup> permitted_sphere_groups;
};

inline const char* StaticObstacleShapeName(const StaticObstacleGeometry& geometry) {
    return std::holds_alternative<AxisAlignedBox>(geometry) ? "box" : "cylinder";
}
