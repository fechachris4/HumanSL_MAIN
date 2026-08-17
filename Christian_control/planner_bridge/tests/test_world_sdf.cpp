// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdio>

#include <gpmp2/obstacle/SDFexception.h>

#include "WorldSdf.h"

namespace {

// True when gpmp2 accepts `point` as an in-range query. The grid's usable
// volume is the thing WorldGridBounds() claims to describe, and gpmp2 is the
// only authority on it — so the bounds are checked against gpmp2 itself
// rather than against a restatement of the same arithmetic.
bool Queryable(const gpmp2::SignedDistanceField& field, const gtsam::Point3& point) {
    try {
        field.getSignedDistance(point);
        return true;
    } catch (const gpmp2::SDFQueryOutOfRange&) {
        return false;
    }
}

}  // namespace

int main() {
    const Eigen::Isometry3d world_T_mount =
        Eigen::Translation3d(1.0, 2.0, 3.0) *
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    const GridGeometry geometry = WorldGridGeometry(world_T_mount);
    const GridBounds bounds = WorldGridBounds(geometry);
    const Eigen::Vector3d centre = (bounds.min_m + bounds.max_m) / 2.0;

    // ---- SDF construction ------------------------------------------------
    // Probes are derived from the grid rather than written as literals, so
    // they stay meaningful if the grid's frame or extents ever move.
    const auto empty = MakeWorldSdf(geometry, std::nullopt);
    assert(empty.getSignedDistance(gtsam::Point3(centre)) > 1.0);

    AxisAlignedBox box{centre, {0.1, 0.1, 0.1}};
    const auto world = MakeWorldSdf(geometry, box);
    assert(world.getSignedDistance(gtsam::Point3(centre)) < 0.0);   // inside

    // 15 cm above the centre is 5 cm clear of the box's +z face.
    const double near =
        world.getSignedDistance(gtsam::Point3(centre + Eigen::Vector3d(0, 0, 0.15)));
    assert(near > 0.0 && near < 0.1);

    // A corner inset by one cell is far from a 0.1 m box at the centre.
    const Eigen::Vector3d inset =
        bounds.min_m + Eigen::Vector3d::Constant(kGridCellM);
    assert(world.getSignedDistance(gtsam::Point3(inset)) > 0.5);

    // ---- WorldGridBounds() really is the queryable volume ----------------
    // This is what pins max = origin + (n-1)*cell. Without it the bounds are
    // just arithmetic nobody checks against gpmp2, and the one-cell overstate
    // this test was written for goes unnoticed again.
    //
    // The upper faces are probed just INSIDE, not exactly on. gpmp2's range
    // check admits the exact upper bound but its interpolator then reads one
    // sample past the end — an upstream inconsistency documented in
    // WorldSdf.cpp and excluded by BoxWithinGridBounds' strict upper compare.
    // Probing exactly on it would assert inside Eigen, testing the bug rather
    // than our contract.
    constexpr double kJustInside = 1e-6;
    for (int corner = 0; corner < 8; ++corner) {
        const Eigen::Vector3d point(
            (corner & 1) ? bounds.max_m.x() - kJustInside : bounds.min_m.x(),
            (corner & 2) ? bounds.max_m.y() - kJustInside : bounds.min_m.y(),
            (corner & 4) ? bounds.max_m.z() - kJustInside : bounds.min_m.z());
        assert(Queryable(world, gtsam::Point3(point)) &&
               "every corner of WorldGridBounds() must be queryable");
    }

    // Every corner of the measured Mount-frame grid, transformed by the
    // same snapshot used for the model, remains queryable with at least the
    // established collision-envelope margin on every WORLD AABB face.
    const Eigen::Vector3d measured_mount_min(
        kGridOriginXM, kGridOriginYM, kGridOriginZM);
    const Eigen::Vector3d measured_mount_max =
        measured_mount_min +
        Eigen::Vector3d(kGridNx - 1, kGridNy - 1, kGridNz - 1) * kGridCellM;
    for (int corner = 0; corner < 8; ++corner) {
        const Eigen::Vector3d mount_corner(
            (corner & 1) ? measured_mount_max.x() : measured_mount_min.x(),
            (corner & 2) ? measured_mount_max.y() : measured_mount_min.y(),
            (corner & 4) ? measured_mount_max.z() : measured_mount_min.z());
        const Eigen::Vector3d world_corner = world_T_mount * mount_corner;
        assert(Queryable(world, gtsam::Point3(world_corner)) &&
               "every transformed Mount-workspace corner must be queryable");
        assert(((world_corner - bounds.min_m).array() >=
                kCollisionEnvelopeMarginM).all());
        assert(((bounds.max_m - world_corner).array() >=
                kCollisionEnvelopeMarginM).all());
    }

    // One micrometre past each of the six faces must be refused. Any larger
    // step would also pass against a bounds value a whole cell too generous,
    // which is exactly the bug this guards.
    constexpr double kJustOutside = 1e-6;
    for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d below = centre;
        below[axis] = bounds.min_m[axis] - kJustOutside;
        assert(!Queryable(world, gtsam::Point3(below)) &&
               "just below a lower face must be out of range");

        Eigen::Vector3d above = centre;
        above[axis] = bounds.max_m[axis] + kJustOutside;
        assert(!Queryable(world, gtsam::Point3(above)) &&
               "just beyond an upper face must be out of range");
    }

    std::printf("grid queryable volume: x [%.3f, %.3f]  y [%.3f, %.3f]  z [%.3f, %.3f]\n",
                bounds.min_m.x(), bounds.max_m.x(), bounds.min_m.y(), bounds.max_m.y(),
                bounds.min_m.z(), bounds.max_m.z());
    return 0;
}
