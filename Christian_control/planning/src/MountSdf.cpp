#include "MountSdf.h"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include <algorithm>
#include <gtsam/base/Matrix.h>

namespace {

void ValidateGridGeometry(const GridGeometry& geometry) {
    if (geometry.nx < 2 || geometry.ny < 2 || geometry.nz < 2 ||
        !geometry.origin_mount_m.allFinite() ||
        !std::isfinite(geometry.cell_m) || geometry.cell_m <= 0.0)
        throw std::invalid_argument("invalid mount SDF grid geometry");
}

void ValidateStaticObstacleGeometry(const StaticObstacleGeometry& geometry) {
    std::visit([](const auto& obstacle) {
        using Obstacle = std::decay_t<decltype(obstacle)>;
        if constexpr (std::is_same_v<Obstacle, AxisAlignedBox>) {
            if (!obstacle.center.allFinite() || !obstacle.half_extent.allFinite() ||
                (obstacle.half_extent.array() <= 0.0).any())
                throw std::invalid_argument("invalid box geometry");
        } else {
            if (!obstacle.center_mount_m.allFinite() ||
                !std::isfinite(obstacle.radius_m) ||
                !std::isfinite(obstacle.height_m) || obstacle.radius_m <= 0.0 ||
                obstacle.height_m <= 0.0)
                throw std::invalid_argument("invalid cylinder geometry");
        }
    }, geometry);
}

double BoxSignedDistance(const Eigen::Vector3d& p, const AxisAlignedBox& box) {
    const Eigen::Vector3d d = (p - box.center).cwiseAbs() - box.half_extent;
    const Eigen::Vector3d outside = d.cwiseMax(0.0);
    const double inside = std::min(d.maxCoeff(), 0.0);
    return outside.norm() + inside;   // standard AABB SDF
}

double CylinderSignedDistance(const Eigen::Vector3d& p,
                              const MountCylinder& cylinder) {
    const Eigen::Vector3d local = p - cylinder.center_mount_m;
    const Eigen::Vector2d q(
        local.head<2>().norm() - cylinder.radius_m,
        std::abs(local.z()) - 0.5 * cylinder.height_m);
    return q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0);
}

double StaticObstacleSignedDistance(const Eigen::Vector3d& p,
                                    const StaticObstacleGeometry& geometry) {
    return std::visit([&p](const auto& obstacle) {
        using Obstacle = std::decay_t<decltype(obstacle)>;
        if constexpr (std::is_same_v<Obstacle, AxisAlignedBox>)
            return BoxSignedDistance(p, obstacle);
        else
            return CylinderSignedDistance(p, obstacle);
    }, geometry);
}
}  // namespace

static gpmp2::SignedDistanceField BuildSingleObstacleSdf(
    const GridGeometry&, const NamedStaticObstacle&);

ObstacleQuery QueryStaticObstacle(const StaticObstacleGeometry& geometry,
                                  const Eigen::Vector3d& point,
                                  double sphere_radius_m) {
    ValidateStaticObstacleGeometry(geometry);
    double distance = 0.0;
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    std::visit([&](const auto& obstacle) {
        using Obstacle = std::decay_t<decltype(obstacle)>;
        if constexpr (std::is_same_v<Obstacle, AxisAlignedBox>) {
            const Eigen::Vector3d delta = point - obstacle.center;
            const Eigen::Vector3d q = delta.cwiseAbs() - obstacle.half_extent;
            distance = q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0);
            const Eigen::Vector3d outside = q.cwiseMax(0.0);
            if (outside.norm() > 0.0) {
                for (int axis = 0; axis < 3; ++axis)
                    normal(axis) = outside(axis) * (delta(axis) >= 0.0 ? 1.0 : -1.0);
                normal.normalize();
            } else {
                Eigen::Vector3d margin = obstacle.half_extent - delta.cwiseAbs();
                int axis = 0;
                if (margin(1) < margin(axis)) axis = 1;
                if (margin(2) < margin(axis)) axis = 2;
                normal(axis) = delta(axis) >= 0.0 ? 1.0 : -1.0;
            }
        } else {
            const Eigen::Vector3d local = point - obstacle.center_mount_m;
            const double radial = local.head<2>().norm();
            const double cap = std::abs(local.z()) - obstacle.height_m * 0.5;
            const Eigen::Vector2d q(radial - obstacle.radius_m, cap);
            distance = q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0);
            if (q.x() > 0.0 && q.y() > 0.0) {
                if (radial > 0.0) normal.head<2>() = local.head<2>() / radial * q.x();
                else normal.x() = q.x();
                normal.z() = (local.z() >= 0.0 ? 1.0 : -1.0) * q.y();
                normal.normalize();
            } else if (q.x() > 0.0 || (q.y() <= 0.0 && q.x() >= q.y())) {
                if (radial > 0.0) normal.head<2>() = local.head<2>() / radial;
                else normal.x() = 1.0;
            } else {
                normal.z() = local.z() >= 0.0 ? 1.0 : -1.0;
            }
        }
    }, geometry);
    return {distance - sphere_radius_m, normal};
}

std::vector<NamedObstacleField> MakeNamedObstacleFields(
    const GridGeometry& grid, const PlannerModel& model,
    const std::vector<NamedStaticObstacle>& scene) {
    std::vector<const NamedStaticObstacle*> enabled;
    for (const auto& obstacle : scene)
        if (obstacle.enabled) enabled.push_back(&obstacle);
    std::sort(enabled.begin(), enabled.end(),
              [](const auto* a, const auto* b) { return a->id < b->id; });
    std::vector<NamedObstacleField> fields;
    for (const auto* obstacle : enabled) {
        std::vector<gpmp2::BodySphere> prohibited;
        std::vector<std::size_t> indices;
        for (std::size_t i = 0; i < model.authored_spheres.size(); ++i) {
            const auto group = model.sphere_groups.at(i);
            if (std::find(obstacle->permitted_sphere_groups.begin(),
                          obstacle->permitted_sphere_groups.end(), group) !=
                obstacle->permitted_sphere_groups.end())
                continue;
            prohibited.push_back(model.authored_spheres[i]);
            indices.push_back(i);
        }
        NamedObstacleField field;
        field.id = obstacle->id;
        field.geometry = obstacle->geometry;
        field.participating_sphere_indices = std::move(indices);
        field.sdf = BuildSingleObstacleSdf(grid, *obstacle);
        field.participating_arm = std::make_unique<gpmp2::ArmModel>(
            model.arm_model->fk_model(), prohibited);
        fields.push_back(std::move(field));
    }
    return fields;
}

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

static gpmp2::SignedDistanceField BuildSingleObstacleSdf(
    const GridGeometry& geometry,
    const NamedStaticObstacle& obstacle) {
    ValidateGridGeometry(geometry);
    const GridBounds grid_bounds = MountGridBounds(geometry);
    try {
        ValidateStaticObstacleGeometry(obstacle.geometry);
    } catch (const std::invalid_argument& error) {
        throw std::invalid_argument("static obstacle '" + obstacle.id + "': " + error.what());
    }
    if (!StaticObstacleWithinGridBounds(obstacle.geometry, grid_bounds)) {
        throw std::invalid_argument("static obstacle '" + obstacle.id +
                                    "' extends outside the mount SDF grid");
    }
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
                field[k](j, i) = StaticObstacleSignedDistance(p, obstacle.geometry);
            }
    return gpmp2::SignedDistanceField(origin, geometry.cell_m, field);
}

GridBounds MountGridBounds(const GridGeometry& geometry) {
    ValidateGridGeometry(geometry);
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
    // face is excluded by StaticObstacleWithinGridBounds below, and the arm
    // is kept 0.05 m clear of it by the measured coverage margin.
    bounds.max_m =
        bounds.min_m +
        Eigen::Vector3d(geometry.nx - 1, geometry.ny - 1, geometry.nz - 1) *
        geometry.cell_m;
    return bounds;
}

GridBounds StaticObstacleBounds(const StaticObstacleGeometry& geometry) {
    ValidateStaticObstacleGeometry(geometry);
    return std::visit([](const auto& obstacle) {
        using Obstacle = std::decay_t<decltype(obstacle)>;
        GridBounds bounds;
        if constexpr (std::is_same_v<Obstacle, AxisAlignedBox>) {
            bounds.min_m = obstacle.center - obstacle.half_extent;
            bounds.max_m = obstacle.center + obstacle.half_extent;
        } else {
            const Eigen::Vector3d half_extent(
                obstacle.radius_m, obstacle.radius_m, 0.5 * obstacle.height_m);
            bounds.min_m = obstacle.center_mount_m - half_extent;
            bounds.max_m = obstacle.center_mount_m + half_extent;
        }
        return bounds;
    }, geometry);
}

bool StaticObstacleWithinGridBounds(const StaticObstacleGeometry& geometry,
                                    const GridBounds& bounds) {
    if (!bounds.min_m.allFinite() || !bounds.max_m.allFinite() ||
        (bounds.min_m.array() > bounds.max_m.array()).any())
        return false;
    try {
        const GridBounds obstacle_bounds = StaticObstacleBounds(geometry);
        return (obstacle_bounds.min_m.array() >= bounds.min_m.array()).all() &&
               (obstacle_bounds.max_m.array() < bounds.max_m.array()).all();
    } catch (const std::invalid_argument&) {
        return false;
    }
}

std::string DescribeStaticScene(const std::vector<NamedStaticObstacle>& scene_mount,
                                const GridGeometry& geometry) {
    const GridBounds bounds = MountGridBounds(geometry);
    std::ostringstream text;
    text << "mount-frame static scene: " << scene_mount.size()
         << " obstacle(s); grid x [" << bounds.min_m.x() << ", " << bounds.max_m.x()
         << ") y [" << bounds.min_m.y() << ", " << bounds.max_m.y()
         << ") z [" << bounds.min_m.z() << ", " << bounds.max_m.z() << ") m";
    for (const NamedStaticObstacle& obstacle : scene_mount)
        text << "; " << obstacle.id << "="
             << (obstacle.enabled ? "enabled " : "disabled ")
             << StaticObstacleShapeName(obstacle.geometry);
    return text.str();
}
