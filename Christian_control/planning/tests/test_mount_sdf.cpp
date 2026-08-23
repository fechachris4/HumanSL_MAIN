#include <cassert>
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "MountSdf.h"

int main(int argc, char** argv) {
    assert(argc == 3);
    MountCylinder cylinder;
    cylinder.radius_m = 0.25;
    cylinder.height_m = 0.5;
    const auto side = QueryStaticObstacle(
        cylinder, Eigen::Vector3d(0.35, 0.0, 0.0), 0.05);
    assert(std::abs(side.clearance_m - 0.05) < 1e-12);
    assert(side.outward_normal_mount == Eigen::Vector3d::UnitX());
    const auto cap = QueryStaticObstacle(
        cylinder, Eigen::Vector3d(0.0, 0.0, 0.3), 0.05);
    assert(std::abs(cap.clearance_m - 0.0) < 1e-12);
    assert(cap.outward_normal_mount == Eigen::Vector3d::UnitZ());
    const auto cylinder_corner = QueryStaticObstacle(
        cylinder, Eigen::Vector3d(0.35, 0.0, 0.35), 0.05);
    assert((cylinder_corner.outward_normal_mount -
            Eigen::Vector3d(1.0, 0.0, 1.0).normalized()).norm() < 1e-12);
    const auto cylinder_inside_side = QueryStaticObstacle(
        cylinder, Eigen::Vector3d(0.1, 0.0, 0.0), 0.05);
    assert(cylinder_inside_side.outward_normal_mount == Eigen::Vector3d::UnitX());
    const auto cylinder_inside_tie = QueryStaticObstacle(
        cylinder, Eigen::Vector3d(0.125, 0.0, 0.125), 0.05);
    assert(cylinder_inside_tie.outward_normal_mount == Eigen::Vector3d::UnitX());

    AxisAlignedBox box;
    box.half_extent = Eigen::Vector3d::Constant(0.2);
    const auto corner = QueryStaticObstacle(
        box, Eigen::Vector3d(0.3, 0.3, 0.3), 0.05);
    assert(std::abs(corner.clearance_m - (std::sqrt(0.03) - 0.05)) < 1e-12);
    assert((corner.outward_normal_mount - Eigen::Vector3d::Constant(1.0 / std::sqrt(3.0))).norm() < 1e-12);
    const auto inside = QueryStaticObstacle(
        box, Eigen::Vector3d::Zero(), 0.05);
    assert(inside.clearance_m < 0.0);
    assert(inside.outward_normal_mount == Eigen::Vector3d::UnitX());
    const auto box_face = QueryStaticObstacle(
        box, Eigen::Vector3d(0.3, 0.0, 0.0), 0.05);
    assert(box_face.outward_normal_mount == Eigen::Vector3d::UnitX());
    const auto box_inside = QueryStaticObstacle(
        box, Eigen::Vector3d(0.19, 0.0, 0.0), 0.05);
    assert(box_inside.outward_normal_mount == Eigen::Vector3d::UnitX());

    const GridGeometry grid = MountGridGeometry();
    const GridBounds bounds = MountGridBounds(grid);
    assert(bounds.min_m.x() < bounds.max_m.x());
    assert(StaticObstacleWithinGridBounds(cylinder, bounds));
    const PlannerModel model = LoadPlannerModel(argv[1], true);
    NamedStaticObstacle filtered;
    filtered.id = "filtered";
    filtered.enabled = true;
    filtered.geometry = cylinder;
    filtered.permitted_sphere_groups = {CollisionSphereGroup::kMountInterface};
    NamedStaticObstacle all_permitted = filtered;
    all_permitted.id = "z_all_permitted";
    all_permitted.permitted_sphere_groups = {
        CollisionSphereGroup::kMountInterface,
        CollisionSphereGroup::kProximalArm,
        CollisionSphereGroup::kUpperArm,
        CollisionSphereGroup::kForearm,
        CollisionSphereGroup::kTool};
    const auto fields = MakeNamedObstacleFields(grid, model,
                                                {filtered, all_permitted});
    assert(fields.size() == 2);
    assert(std::find(fields[0].participating_sphere_indices.begin(),
                     fields[0].participating_sphere_indices.end(), 0) ==
           fields[0].participating_sphere_indices.end());
    assert(std::find(fields[0].participating_sphere_indices.begin(),
                     fields[0].participating_sphere_indices.end(), 1) !=
           fields[0].participating_sphere_indices.end());
    assert(fields[1].participating_sphere_indices.empty());
    std::puts("test_mount_sdf: all assertions passed");
    return 0;
}
