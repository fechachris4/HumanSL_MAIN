#pragma once
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include "StaticScene.h"

// Grid geometry, shared by MakeMountSdf and MountGridBounds so the two can
// never disagree. Expressed in the `mount` frame (the URDF root, at the
// midpoint of the two arm bases), so ONE grid serves both arms and an
// obstacle is described once for the whole rig rather than once per arm.
// Mount is the planner's only internal frame, so the grid is STATIC — it
// never depends on where the rig currently sits in Vicon world.
//
// gpmp2 silently reports "no obstacle" for queries outside the grid, so the
// grid must contain everywhere EITHER arm can put a collision sphere.
//
// MEASURED, not chosen. test_grid_coverage unioned the right (tool) and left
// (flange) collision envelopes over a deterministic extremal sweep plus 200k
// random configurations per arm, and printed a paste-ready block when it
// failed. Measured 2026-08-06, in mount:
//   right  x [-1.052, 1.056]  y [-1.378, 0.654]  z [-0.952, 1.154]
//   left   x [-0.869, 0.870]  y [-0.478, 1.193]  z [-0.768, 0.971]
//   union  x [-1.052, 1.056]  y [-1.378, 1.193]  z [-0.952, 1.154]
// The two differ because the left chain ends at a bare flange and carries no
// gripper spheres — neither envelope can be mirrored from the other.
//
// Re-derive these constants (a coverage sweep like the retired
// test_grid_coverage) if the URDF, the mounting geometry or the sphere
// layout changes. Do NOT hand-tune them.
inline constexpr double kGridOriginXM = -1.12;
inline constexpr double kGridOriginYM = -1.44;
inline constexpr double kGridOriginZM = -1.04;
inline constexpr double kGridCellM = 0.04;
inline constexpr int kGridNx = 57;
inline constexpr int kGridNy = 69;
inline constexpr int kGridNz = 58;
inline constexpr double kCollisionEnvelopeMarginM = 0.05;

struct GridGeometry {
    Eigen::Vector3d origin_mount_m = Eigen::Vector3d::Zero();
    int nx = 0;
    int ny = 0;
    int nz = 0;
    double cell_m = kGridCellM;
};

// The static mount-frame grid built from the measured constants above.
GridGeometry MountGridGeometry();

// The volume the SDF grid can actually be QUERIED over, in `mount`:
// [origin, origin + (n-1)*cell] per axis — see the (n-1) note in the .cpp.
// gpmp2 returns zero obstacle cost for any query outside this volume ("no
// obstacle"), silently — so a `box` that is not fully contained here must be
// rejected before solving, not discovered as a missed obstacle after the
// fact.
struct GridBounds {
    Eigen::Vector3d min_m;
    Eigen::Vector3d max_m;
};
GridBounds MountGridBounds(const GridGeometry& geometry);

// The axis-aligned full bounds of a static primitive in the `mount` frame.
// A vertical cylinder has a circular x-y footprint and flat caps at
// center_mount_m.z() +/- height_m / 2.
GridBounds StaticObstacleBounds(const StaticObstacleGeometry& geometry);

// An object must be contained in the grid's actual interpolatable volume.
// The lower face is included; the upper face is excluded because gpmp2's
// trilinear SDF cannot interpolate there.
bool StaticObstacleWithinGridBounds(const StaticObstacleGeometry& geometry,
                                    const GridBounds& bounds);

// One grid covering both arms' workspaces, in the `mount` frame. Each sample
// contains the minimum signed distance to all enabled primitives, or 10 m for
// an empty scene.
gpmp2::SignedDistanceField MakeMountSdf(
    const GridGeometry& geometry,
    const std::vector<NamedStaticObstacle>& scene_mount);

// Migration-only overload. Task 3 switches callers to the named scene API
// and removes this seam.
gpmp2::SignedDistanceField MakeMountSdf(
    const GridGeometry& geometry,
    const std::optional<AxisAlignedBox>& box_mount);

// Human-readable planner diagnostics for the persisted mount-frame scene.
std::string DescribeStaticScene(const std::vector<NamedStaticObstacle>& scene_mount,
                                const GridGeometry& geometry);
