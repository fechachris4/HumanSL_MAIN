#include "WorldSdf.h"
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

gpmp2::SignedDistanceField MakeWorldSdf(const std::optional<AxisAlignedBox>& box) {
    const gtsam::Point3 origin(kGridOriginXM, kGridOriginYM, kGridOriginZM);
    // gpmp2 layout: one z-slice per matrix; matrix rows = y, cols = x.
    std::vector<gtsam::Matrix> field(kGridNz, gtsam::Matrix(kGridNy, kGridNx));
    for (int k = 0; k < kGridNz; ++k)
        for (int j = 0; j < kGridNy; ++j)
            for (int i = 0; i < kGridNx; ++i) {
                const Eigen::Vector3d p = origin + Eigen::Vector3d(i, j, k) * kGridCellM;
                field[k](j, i) = box ? BoxSignedDistance(p, *box) : 10.0;
            }
    return gpmp2::SignedDistanceField(origin, kGridCellM, field);
}

GridBounds WorldGridBounds() {
    GridBounds bounds;
    bounds.min_m = Eigen::Vector3d(kGridOriginXM, kGridOriginYM, kGridOriginZM);
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
        Eigen::Vector3d(kGridNx - 1, kGridNy - 1, kGridNz - 1) * kGridCellM;
    return bounds;
}
