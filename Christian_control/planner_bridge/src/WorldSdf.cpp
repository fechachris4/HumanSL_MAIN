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
    const gtsam::Point3 origin(-1.2, -1.2, -0.4);
    const double cell = 0.04;
    const int nx = 60, ny = 60, nz = 40;
    // gpmp2 layout: one z-slice per matrix; matrix rows = y, cols = x.
    std::vector<gtsam::Matrix> field(nz, gtsam::Matrix(ny, nx));
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const Eigen::Vector3d p = origin + Eigen::Vector3d(i, j, k) * cell;
                field[k](j, i) = box ? BoxSignedDistance(p, *box) : 10.0;
            }
    return gpmp2::SignedDistanceField(origin, cell, field);
}
