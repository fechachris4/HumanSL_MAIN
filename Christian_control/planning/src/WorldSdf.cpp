#include "WorldSdf.h"
#include <cmath>
#include <limits>
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

GridGeometry WorldGridGeometry(const Eigen::Isometry3d& world_T_mount) {
    if (!world_T_mount.matrix().allFinite())
        throw std::invalid_argument("world_T_mount must be finite");

    const Eigen::Vector3d mount_min(kGridOriginXM, kGridOriginYM, kGridOriginZM);
    const Eigen::Vector3d mount_max =
        mount_min +
        Eigen::Vector3d(kGridNx - 1, kGridNy - 1, kGridNz - 1) * kGridCellM;
    Eigen::Vector3d world_min = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d world_max = -world_min;
    for (int corner = 0; corner < 8; ++corner) {
        const Eigen::Vector3d mount_corner(
            (corner & 1) ? mount_max.x() : mount_min.x(),
            (corner & 2) ? mount_max.y() : mount_min.y(),
            (corner & 4) ? mount_max.z() : mount_min.z());
        const Eigen::Vector3d world_corner = world_T_mount * mount_corner;
        world_min = world_min.cwiseMin(world_corner);
        world_max = world_max.cwiseMax(world_corner);
    }

    // Two cells (0.08 m) exceed the established 0.05 m margin and keep
    // transformed corners strictly away from the upper interpolation face.
    constexpr int kPaddingCells = 2;
    const double padding = kPaddingCells * kGridCellM;
    GridGeometry geometry;
    geometry.cell_m = kGridCellM;
    geometry.origin_world_m =
        ((world_min.array() - padding) / geometry.cell_m).floor().matrix() *
        geometry.cell_m;
    const Eigen::Vector3d upper =
        ((world_max.array() + padding) / geometry.cell_m).ceil().matrix() *
        geometry.cell_m;
    const Eigen::Array3d intervals =
        ((upper - geometry.origin_world_m) / geometry.cell_m).array().round();
    geometry.nx = static_cast<int>(intervals.x()) + 1;
    geometry.ny = static_cast<int>(intervals.y()) + 1;
    geometry.nz = static_cast<int>(intervals.z()) + 1;
    return geometry;
}

gpmp2::SignedDistanceField MakeWorldSdf(
    const GridGeometry& geometry,
    const std::optional<AxisAlignedBox>& box_world) {
    if (geometry.nx < 2 || geometry.ny < 2 || geometry.nz < 2 ||
        !geometry.origin_world_m.allFinite() ||
        !std::isfinite(geometry.cell_m) || geometry.cell_m <= 0.0)
        throw std::invalid_argument("invalid world SDF grid geometry");
    const gtsam::Point3 origin(geometry.origin_world_m);
    // gpmp2 layout: one z-slice per matrix; matrix rows = y, cols = x.
    std::vector<gtsam::Matrix> field(
        geometry.nz, gtsam::Matrix(geometry.ny, geometry.nx));
    for (int k = 0; k < geometry.nz; ++k)
        for (int j = 0; j < geometry.ny; ++j)
            for (int i = 0; i < geometry.nx; ++i) {
                const Eigen::Vector3d p =
                    geometry.origin_world_m +
                    Eigen::Vector3d(i, j, k) * geometry.cell_m;
                field[k](j, i) =
                    box_world ? BoxSignedDistance(p, *box_world) : 10.0;
            }
    return gpmp2::SignedDistanceField(origin, geometry.cell_m, field);
}

GridBounds WorldGridBounds(const GridGeometry& geometry) {
    GridBounds bounds;
    bounds.min_m = geometry.origin_world_m;
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
    // face is excluded by BoxWithinGridBounds below, and the arm is kept
    // 0.05 m clear of it by test_grid_coverage.
    bounds.max_m =
        bounds.min_m +
        Eigen::Vector3d(geometry.nx - 1, geometry.ny - 1, geometry.nz - 1) *
        geometry.cell_m;
    return bounds;
}
