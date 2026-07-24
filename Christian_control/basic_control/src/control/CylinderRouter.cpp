//
// CylinderRouter: deterministic direct / around / over Cartesian routing.
//

#include "control/CylinderRouter.h"

#include <algorithm>
#include <cmath>
namespace
{
    constexpr double kGeometryEpsilon = 1e-9;
    constexpr double kArcStepRad = 15.0 * M_PI / 180.0;
    constexpr double kRoutePaddingM = 0.01;

    double PositiveAngle(double angle)
    {
        const double two_pi = 2.0 * M_PI;
        angle = std::fmod(angle, two_pi);
        return angle < 0.0 ? angle + two_pi : angle;
    }

    void Append(CylinderRoute& route, const Eigen::Vector3d& point)
    {
        if (route.size > 0 &&
            (route.waypoints[route.size - 1] - point).norm() < kGeometryEpsilon)
            return;
        if (route.size < route.waypoints.size())
            route.waypoints[route.size++] = point;
    }

    double RouteLength(const Eigen::Vector3d& start, const CylinderRoute& route)
    {
        double length = 0.0;
        Eigen::Vector3d previous = start;
        for (std::size_t i = 0; i < route.size; ++i) {
            length += (route.waypoints[i] - previous).norm();
            previous = route.waypoints[i];
        }
        return length;
    }

    Eigen::Vector2d RadialDirection(const Eigen::Vector3d& point,
                                    const Eigen::Vector2d& center,
                                    const Eigen::Vector2d& fallback)
    {
        const Eigen::Vector2d radial = point.head<2>() - center;
        if (radial.norm() > kGeometryEpsilon)
            return radial.normalized();
        if (fallback.norm() > kGeometryEpsilon)
            return fallback.normalized();
        return Eigen::Vector2d::UnitX();
    }

    CylinderRoute ArcCandidate(const CylinderKeepout& keepout,
                               const Eigen::Vector3d& start,
                               const Eigen::Vector3d& target,
                               double delta_angle,
                               CylinderRouteKind kind,
                               double route_radius)
    {
        CylinderRoute route;
        route.kind = kind;
        route.effective_target = target;

        const Eigen::Vector2d target_fallback =
            target.head<2>() - keepout.center_xy_m;
        const Eigen::Vector2d start_direction =
            RadialDirection(start, keepout.center_xy_m, target_fallback);
        const Eigen::Vector2d target_direction =
            RadialDirection(target, keepout.center_xy_m, start_direction);
        const double start_angle = std::atan2(start_direction.y(), start_direction.x());
        const double target_angle = std::atan2(target_direction.y(), target_direction.x());

        // The caller supplies direction but not magnitude. Recompute the
        // exact sweep so rounding around ±pi cannot flip the route.
        const double ccw = PositiveAngle(target_angle - start_angle);
        const double sweep =
            delta_angle >= 0.0 ? ccw : (ccw == 0.0 ? 0.0 : ccw - 2.0 * M_PI);
        const int segments =
            std::max(1, static_cast<int>(std::ceil(std::abs(sweep) / kArcStepRad)));

        Eigen::Vector3d ring_start;
        ring_start << keepout.center_xy_m.x() + route_radius * std::cos(start_angle),
                      keepout.center_xy_m.y() + route_radius * std::sin(start_angle),
                      start.z();
        Append(route, ring_start);

        for (int i = 1; i <= segments; ++i) {
            const double fraction = static_cast<double>(i) / segments;
            const double angle = start_angle + fraction * sweep;
            Eigen::Vector3d point;
            point << keepout.center_xy_m.x() + route_radius * std::cos(angle),
                     keepout.center_xy_m.y() + route_radius * std::sin(angle),
                     start.z() + fraction * (target.z() - start.z());
            Append(route, point);
        }
        Append(route, target);
        route.length_m = RouteLength(start, route);
        return route;
    }

    CylinderRoute OverCandidate(const CylinderKeepout& keepout,
                                const Eigen::Vector3d& start,
                                const Eigen::Vector3d& target)
    {
        CylinderRoute route;
        route.kind = CylinderRouteKind::kOver;
        route.effective_target = target;

        const double top_z =
            std::max({keepout.z_max_m + keepout.clearance_m + 0.05,
                      start.z(), target.z()});
        Append(route, Eigen::Vector3d(start.x(), start.y(), top_z));
        Append(route, Eigen::Vector3d(target.x(), target.y(), top_z));
        Append(route, target);
        route.length_m = RouteLength(start, route);
        return route;
    }
} // namespace

const char* CylinderRouteKindName(CylinderRouteKind kind)
{
    switch (kind) {
    case CylinderRouteKind::kDirect:
        return "direct";
    case CylinderRouteKind::kCounterClockwise:
        return "counter-clockwise";
    case CylinderRouteKind::kClockwise:
        return "clockwise";
    case CylinderRouteKind::kOver:
        return "over";
    }
    return "unknown";
}

CylinderRouter::CylinderRouter(const CylinderKeepout& keepout)
    : keepout_(keepout)
{
}

bool CylinderRouter::SegmentIntersects(const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& end) const
{
    if (!keepout_.enabled)
        return false;

    double t_min = 0.0;
    double t_max = 1.0;
    const double dz = end.z() - start.z();
    const double obstacle_z_min =
        keepout_.z_min_m - keepout_.clearance_m;
    const double obstacle_z_max =
        keepout_.z_max_m + keepout_.clearance_m;
    if (std::abs(dz) < kGeometryEpsilon) {
        if (start.z() < obstacle_z_min || start.z() > obstacle_z_max)
            return false;
    } else {
        const double t_a = (obstacle_z_min - start.z()) / dz;
        const double t_b = (obstacle_z_max - start.z()) / dz;
        t_min = std::max(0.0, std::min(t_a, t_b));
        t_max = std::min(1.0, std::max(t_a, t_b));
        if (t_min > t_max)
            return false;
    }

    const Eigen::Vector2d p0 = start.head<2>() - keepout_.center_xy_m;
    const Eigen::Vector2d direction = end.head<2>() - start.head<2>();
    double nearest_t = t_min;
    if (direction.squaredNorm() > kGeometryEpsilon)
        nearest_t = std::clamp(-p0.dot(direction) / direction.squaredNorm(),
                               t_min, t_max);
    const Eigen::Vector2d nearest = p0 + nearest_t * direction;
    const double obstacle_radius = keepout_.radius_m + keepout_.clearance_m;
    return nearest.squaredNorm() <= obstacle_radius * obstacle_radius;
}

CylinderRoute CylinderRouter::Plan(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& requested_target) const
{
    CylinderRoute direct;
    direct.requested_target = requested_target;
    direct.effective_target = requested_target;

    if (!keepout_.enabled) {
        Append(direct, requested_target);
        direct.length_m = (requested_target - start).norm();
        return direct;
    }

    const double obstacle_radius = keepout_.radius_m + keepout_.clearance_m;
    // The extra radial amount keeps straight chords between 15-degree arc
    // waypoints outside the inflated obstacle rather than cutting through it.
    const double route_radius =
        obstacle_radius / std::cos(kArcStepRad / 2.0) + kRoutePaddingM;

    Eigen::Vector3d effective_target = requested_target;
    const bool target_in_height =
        requested_target.z() >= keepout_.z_min_m - keepout_.clearance_m &&
        requested_target.z() <= keepout_.z_max_m + keepout_.clearance_m;
    const Eigen::Vector2d target_radial =
        requested_target.head<2>() - keepout_.center_xy_m;
    if (target_in_height &&
        target_radial.norm() <= obstacle_radius + kGeometryEpsilon) {
        const Eigen::Vector2d start_fallback =
            start.head<2>() - keepout_.center_xy_m;
        const Eigen::Vector2d direction =
            RadialDirection(requested_target, keepout_.center_xy_m, start_fallback);
        effective_target.x() = keepout_.center_xy_m.x() + route_radius * direction.x();
        effective_target.y() = keepout_.center_xy_m.y() + route_radius * direction.y();
        direct.target_adjusted = true;
    }
    direct.effective_target = effective_target;

    if (!SegmentIntersects(start, effective_target)) {
        Append(direct, effective_target);
        direct.length_m = (effective_target - start).norm();
        return direct;
    }

    CylinderRoute counter_clockwise =
        ArcCandidate(keepout_, start, effective_target, 1.0,
                     CylinderRouteKind::kCounterClockwise, route_radius);
    CylinderRoute clockwise =
        ArcCandidate(keepout_, start, effective_target, -1.0,
                     CylinderRouteKind::kClockwise, route_radius);
    CylinderRoute over = OverCandidate(keepout_, start, effective_target);

    CylinderRoute best = counter_clockwise;
    if (clockwise.length_m < best.length_m)
        best = clockwise;
    if (over.length_m < best.length_m)
        best = over;
    best.requested_target = requested_target;
    best.effective_target = effective_target;
    best.target_adjusted = direct.target_adjusted;
    return best;
}

CylinderRouteFollower::CylinderRouteFollower(const CylinderKeepout& keepout)
    : router_(keepout)
{
}

void CylinderRouteFollower::Reset(const Eigen::Vector3d& current)
{
    route_ = router_.Plan(current, current);
    index_ = 0;
}

void CylinderRouteFollower::SetTarget(
    const Eigen::Vector3d& current,
    const Eigen::Vector3d& requested_target)
{
    route_ = router_.Plan(current, requested_target);
    index_ = 0;
}

Eigen::Vector3d CylinderRouteFollower::Update(const Eigen::Vector3d& current)
{
    while (index_ + 1 < route_.size &&
           (current - route_.waypoints[index_]).norm() <=
               router_.keepout().waypoint_tolerance_m)
        ++index_;
    return route_.size == 0 ? current : route_.waypoints[index_];
}

bool CylinderRouteFollower::at_final_waypoint() const
{
    return route_.size > 0 && index_ + 1 == route_.size;
}
