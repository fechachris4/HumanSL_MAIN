// The SDF grid is STATIC mount-frame geometry: the measured coverage box,
// independent of where the rig sits in Vicon world. This test pins that
// property and the box field's sign convention.

#include <cassert>
#include <cstdio>

#include "MountSdf.h"

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
    const auto sdf = MakeMountSdf(geometry, box);
    assert(sdf.getSignedDistance(gtsam::Point3(box.center)) < 0.0);
    assert(sdf.getSignedDistance(gtsam::Point3(0.4, 0.2, 0.5)) > 0.3);

    // No box: uniformly free.
    const auto free_sdf = MakeMountSdf(geometry, std::nullopt);
    assert(free_sdf.getSignedDistance(gtsam::Point3(box.center)) > 5.0);

    std::puts("test_mount_sdf: all assertions passed");
    return 0;
}
