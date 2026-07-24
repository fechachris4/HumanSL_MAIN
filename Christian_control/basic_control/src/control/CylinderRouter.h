//
// CylinderRouter: fixed-size Cartesian waypoint routing around one vertical
// keep-out cylinder. Pure geometry: no robot, I/O, allocation, or GPMP2.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_CYLINDERROUTER_H
#define HUMANSL_MASTERS_PROJECT_2025_CYLINDERROUTER_H

#include <array>
#include <cstddef>

#include <Eigen/Dense>

struct CylinderKeepout {
    bool enabled = false;
    Eigen::Vector2d center_xy_m = Eigen::Vector2d::Zero();
    double radius_m = 0.25;
    double z_min_m = 0.0;
    double z_max_m = 1.8;
    double clearance_m = 0.10;
    double waypoint_tolerance_m = 0.01;
};

enum class CylinderRouteKind {
    kDirect,
    kCounterClockwise,
    kClockwise,
    kOver
};

struct CylinderRoute {
    static constexpr std::size_t kMaxWaypoints = 32;

    std::array<Eigen::Vector3d, kMaxWaypoints> waypoints{};
    std::size_t size = 0;
    CylinderRouteKind kind = CylinderRouteKind::kDirect;
    bool target_adjusted = false;
    Eigen::Vector3d requested_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d effective_target = Eigen::Vector3d::Zero();
    double length_m = 0.0;
};

const char* CylinderRouteKindName(CylinderRouteKind kind);

class CylinderRouter
{
public:
    explicit CylinderRouter(const CylinderKeepout& keepout = {});

    // Direct when clear. Otherwise evaluates counter-clockwise, clockwise
    // and over-the-top candidates and returns the shortest. A target inside
    // the keep-out is moved radially to the route boundary instead of being
    // rejected, so every finite request produces a route.
    CylinderRoute Plan(const Eigen::Vector3d& start,
                       const Eigen::Vector3d& requested_target) const;

    // True when any part of a segment lies inside the finite vertical
    // cylinder after clearance inflation. Public for hardware-free tests.
    bool SegmentIntersects(const Eigen::Vector3d& start,
                           const Eigen::Vector3d& end) const;

    const CylinderKeepout& keepout() const { return keepout_; }

private:
    CylinderKeepout keepout_;
};

// Stateful cursor shared by both Cartesian controllers. Route construction
// happens only on a new target; Update() advances through the fixed waypoint
// array as the measured end effector reaches each point.
class CylinderRouteFollower
{
public:
    explicit CylinderRouteFollower(const CylinderKeepout& keepout = {});

    void Reset(const Eigen::Vector3d& current);
    void SetTarget(const Eigen::Vector3d& current,
                   const Eigen::Vector3d& requested_target);
    Eigen::Vector3d Update(const Eigen::Vector3d& current);

    bool enabled() const { return router_.keepout().enabled; }
    bool at_final_waypoint() const;
    const CylinderRoute& route() const { return route_; }

private:
    CylinderRouter router_;
    CylinderRoute route_;
    std::size_t index_ = 0;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_CYLINDERROUTER_H
