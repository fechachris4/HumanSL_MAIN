#include <cassert>
#include <cmath>
#include <cstdio>

#include "MountSdf.h"

int main() {
    MountCylinder cylinder;
    cylinder.radius_m = 0.2;
    cylinder.height_m = 0.6;
    const auto side = QueryStaticObstacle(
        cylinder, Eigen::Vector3d(0.3, 0.0, 0.0), 0.05);
    assert(std::abs(side.clearance_m - 0.05) < 1e-12);
    assert(side.outward_normal_mount == Eigen::Vector3d::UnitX());
    const auto cap = QueryStaticObstacle(
        cylinder, Eigen::Vector3d(0.0, 0.0, 0.4), 0.05);
    assert(std::abs(cap.clearance_m - 0.05) < 1e-12);
    assert(cap.outward_normal_mount == Eigen::Vector3d::UnitZ());

    AxisAlignedBox box;
    box.half_extent = Eigen::Vector3d::Constant(0.2);
    const auto corner = QueryStaticObstacle(
        box, Eigen::Vector3d(0.3, 0.3, 0.3), 0.05);
    assert(std::abs(corner.clearance_m - (std::sqrt(0.03) - 0.05)) < 1e-12);
    const auto inside = QueryStaticObstacle(
        box, Eigen::Vector3d::Zero(), 0.05);
    assert(inside.clearance_m < 0.0);

    const GridGeometry grid = MountGridGeometry();
    const GridBounds bounds = MountGridBounds(grid);
    assert(bounds.min_m.x() < bounds.max_m.x());
    assert(StaticObstacleWithinGridBounds(cylinder, bounds));
    std::puts("test_mount_sdf: all assertions passed");
    return 0;
}
