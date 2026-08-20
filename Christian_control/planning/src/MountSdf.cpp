#include "MountSdf.h"
#include <cmath>
#include <stdexcept>
#include <vector>
#include <gtsam/base/Matrix.h>

namespace {
double BoxSignedDistance(const Eigen::Vector3d& p, const AxisAlignedBox& box) {
    const Eigen::Vector3d d = (p - box.center).cwiseAbs() - box.half_extent;
    const Eigen::Vector3d outside = d.cwiseMax(0.0);
    const double inside = std::min(d.maxCoeff(), 0.0);
    return outside.norm() + inside;   // standard AABB SDF
}
}  // namespace

GridGeometry MountGridGeometry() {
    GridGeometry geometry;
    geometry.origin_mount_m =
        Eigen::Vector3d(kGridOriginXM, kGridOriginYM, kGridOriginZM);
    geometry.nx = kGridNx;
    geometry.ny = kGridNy;
    geometry.nz = kGridNz;
    geometry.cell_m = kGridCellM;
    return geometry;
}

gpmp2::SignedDistanceField MakeMountSdf(
    const GridGeometry& geometry,
    const std::optional<AxisAlignedBox>& box_mount) {
    if (geometry.nx < 2 || geometry.ny < 2 || geometry.nz < 2 ||
        !geometry.origin_mount_m.allFinite() ||
        !std::isfinite(geometry.cell_m) || geometry.cell_m <= 0.0)
        throw std::invalid_argument("invalid mount SDF grid geometry");
    const gtsam::Point3 origin(geometry.origin_mount_m);
    // gpmp2 layout: one z-slice per matrix; matrix rows = y, cols = x.
    std::vector<gtsam::Matrix> field(
        geometry.nz, gtsam::Matrix(geometry.ny, geometry.nx));
    for (int k = 0; k < geometry.nz; ++k)
        for (int j = 0; j < geometry.ny; ++j)
            for (int i = 0; i < geometry.nx; ++i) {
                const Eigen::Vector3d p =
                    geometry.origin_mount_m +
                    Eigen::Vector3d(i, j, k) * geometry.cell_m;
                field[k](j, i) =
                    box_mount ? BoxSignedDistance(p, *box_mount) : 10.0;
            }
    return gpmp2::SignedDistanceField(origin, geometry.cell_m, field);
}

GridBounds MountGridBounds(const GridGeometry& geometry) {
    GridBounds bounds;
    bounds.min_m = geometry.origin_mount_m;
    // (n - 1), not n: gpmp2 interpolates trilinearly, so a query needs the
    // eight samples surrounding it and the LAST usable coordinate on each
    // axis is sample index n-1, not n. SignedDistanceField::convertPoint3toCell
    // throws SDFQueryOutOfRange beyond that. Returning origin + n*cell here
    // overstated the usable volume by one cell (0.04 m) per upper face, which
    // let a box be accepted whose top cells gpmp2 would silently refuse to
    // query, and made test_grid_coverage's 0.05 m margin really 0.01 m.
    //
    // Upstream quirk, deliberately NOT papered over here: gpmp2's range check
    // admits a query landing EXACTLY on this bound, but signed_distance()
    // then reads both floor(idx) and floor(idx)+1, so the upper face indexes
    // one sample past the end. The accepted range is one sample wider than
    // the interpolatable one. This function returns gpmp2's accepted range,
    // because that is what the SDF will and won't throw on; the exact upper
    // face is excluded by BoxWithinGridBounds in BridgeMain, and the arm is
    // kept 0.05 m clear of it by the measured coverage margin.
    bounds.max_m =
        bounds.min_m +
        Eigen::Vector3d(geometry.nx - 1, geometry.ny - 1, geometry.nz - 1) *
        geometry.cell_m;
    return bounds;
}
