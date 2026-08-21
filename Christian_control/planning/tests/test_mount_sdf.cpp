// The SDF grid is STATIC mount-frame geometry: the measured coverage box,
// independent of where the rig sits in Vicon world. These tests pin the
// analytic signed distances at exact stored samples, so interpolation cannot
// conceal an incorrect primitive formula.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "MountSdf.h"

namespace {

void CheckNear(double actual, double expected) {
    assert(std::abs(actual - expected) < 1e-12);
}

bool ThrowsWith(const std::vector<NamedStaticObstacle>& scene,
                const GridGeometry& geometry, const std::string& expected_text) {
    try {
        static_cast<void>(MakeMountSdf(geometry, scene));
    } catch (const std::invalid_argument& error) {
        return std::string(error.what()).find(expected_text) != std::string::npos;
    }
    return false;
}

GridGeometry TestGrid() {
    GridGeometry geometry;
    geometry.origin_mount_m = Eigen::Vector3d(-0.4, -0.4, -0.4);
    geometry.nx = 10;
    geometry.ny = 10;
    geometry.nz = 10;
    geometry.cell_m = 0.1;
    return geometry;
}

}  // namespace

int main() {
    const GridGeometry geometry = MountGridGeometry();

    // The grid IS the measured constants — no snapshot, no recomputation.
    assert(geometry.origin_mount_m ==
           Eigen::Vector3d(kGridOriginXM, kGridOriginYM, kGridOriginZM));
    assert(geometry.nx == kGridNx && geometry.ny == kGridNy &&
           geometry.nz == kGridNz);
    assert(geometry.cell_m == kGridCellM);

    const GridBounds bounds = MountGridBounds(geometry);
    assert(bounds.min_m == geometry.origin_mount_m);
    assert((bounds.max_m -
            (bounds.min_m + Eigen::Vector3d(kGridNx - 1, kGridNy - 1,
                                            kGridNz - 1) * kGridCellM))
               .cwiseAbs()
               .maxCoeff() < 1e-15);

    // A mount-frame box signs correctly: negative inside, positive outside,
    // and distances match the analytic AABB distance at grid samples.
    AxisAlignedBox box;
    box.center = Eigen::Vector3d(0.4, 0.2, 0.0);
    box.half_extent = Eigen::Vector3d(0.1, 0.1, 0.1);
    const NamedStaticObstacle named_box{"test-box", true, box};
    const auto sdf = MakeMountSdf(geometry, std::vector<NamedStaticObstacle>{named_box});
    assert(sdf.getSignedDistance(gtsam::Point3(box.center)) < 0.0);
    assert(sdf.getSignedDistance(gtsam::Point3(0.4, 0.2, 0.5)) > 0.3);

    // Empty or disabled scenes are uniformly free.
    const auto free_sdf = MakeMountSdf(geometry, std::vector<NamedStaticObstacle>{});
    assert(free_sdf.getSignedDistance(gtsam::Point3(box.center)) > 5.0);
    const NamedStaticObstacle disabled_box{"disabled-box", false, box};
    const auto disabled_sdf =
        MakeMountSdf(geometry, std::vector<NamedStaticObstacle>{disabled_box});
    CheckNear(disabled_sdf.getSignedDistance(gtsam::Point3(box.center)), 10.0);

    // This deliberately simple grid puts every query at a stored sample.
    const GridGeometry test_grid = TestGrid();
    MountCylinder cylinder;
    cylinder.center_mount_m = Eigen::Vector3d::Zero();
    cylinder.radius_m = 0.20;
    cylinder.height_m = 0.60;
    const NamedStaticObstacle named_cylinder{"test-cylinder", true, cylinder};
    const auto cylinder_sdf =
        MakeMountSdf(test_grid, std::vector<NamedStaticObstacle>{named_cylinder});
    const auto distance_at = [&cylinder_sdf](double x, double y, double z) {
        return cylinder_sdf.getSignedDistance(gtsam::Point3(x, y, z));
    };
    CheckNear(distance_at(0.0, 0.0, 0.0), -0.20);
    CheckNear(distance_at(0.20, 0.0, 0.0), 0.0);
    CheckNear(distance_at(0.0, 0.0, 0.30), 0.0);
    CheckNear(distance_at(0.20, 0.0, 0.30), 0.0);
    CheckNear(distance_at(0.30, 0.0, 0.40),
              std::sqrt(0.10 * 0.10 + 0.10 * 0.10));

    // Composition means the distance to the nearest enabled primitive.
    AxisAlignedBox second_box;
    second_box.center = Eigen::Vector3d(0.3, 0.0, 0.0);
    second_box.half_extent = Eigen::Vector3d(0.1, 0.1, 0.1);
    const auto composed_sdf = MakeMountSdf(
        test_grid,
        std::vector<NamedStaticObstacle>{named_cylinder,
                                         NamedStaticObstacle{"second-box", true, second_box}});
    CheckNear(composed_sdf.getSignedDistance(gtsam::Point3(0.3, 0.0, 0.0)), -0.10);

    // Full primitive bounds must fit the grid's queryable range. The upper
    // face is exclusive because gpmp2 cannot interpolate exactly at it.
    MountCylinder out_of_grid = cylinder;
    out_of_grid.center_mount_m = Eigen::Vector3d(0.5, 0.0, 0.0);
    assert(ThrowsWith({NamedStaticObstacle{"out-of-grid-cylinder", true, out_of_grid}},
                      test_grid, "out-of-grid-cylinder"));
    AxisAlignedBox on_upper_face;
    on_upper_face.center = Eigen::Vector3d(0.4, 0.0, 0.0);
    on_upper_face.half_extent = Eigen::Vector3d(0.1, 0.1, 0.1);
    assert(ThrowsWith({NamedStaticObstacle{"upper-face-box", true, on_upper_face}},
                      test_grid, "upper-face-box"));
    assert(!StaticObstacleWithinGridBounds(on_upper_face, MountGridBounds(test_grid)));

    // A disabled object may intentionally lie outside the planning grid; it
    // remains part of the persisted scene but cannot affect this SDF.
    const auto disabled_out_of_grid_sdf = MakeMountSdf(
        test_grid,
        std::vector<NamedStaticObstacle>{
            NamedStaticObstacle{"disabled-out-of-grid", false, out_of_grid}});
    CheckNear(disabled_out_of_grid_sdf.getSignedDistance(gtsam::Point3::Zero()), 10.0);

    const std::string scene_description = DescribeStaticScene(
        {named_cylinder, disabled_box}, test_grid);
    assert(scene_description.find("test-cylinder") != std::string::npos);
    assert(scene_description.find("disabled-box") != std::string::npos);

    std::puts("test_mount_sdf: all assertions passed");
    return 0;
}
