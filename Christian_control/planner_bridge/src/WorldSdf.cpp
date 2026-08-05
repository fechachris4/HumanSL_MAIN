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
    bounds.max_m = bounds.min_m + Eigen::Vector3d(kGridNx, kGridNy, kGridNz) * kGridCellM;
    return bounds;
}
