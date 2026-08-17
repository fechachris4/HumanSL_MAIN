#pragma once
#include <optional>
#include <Eigen/Dense>
#include <gpmp2/obstacle/SignedDistanceField.h>

struct AxisAlignedBox {           // metres, Vicon world frame
    Eigen::Vector3d center;
    Eigen::Vector3d half_extent;
};

// Grid geometry, shared by MakeWorldSdf and WorldGridBounds so the two can
// never disagree. Expressed in the `mount` frame (the URDF root, at the
// midpoint of the two arm bases), so ONE grid serves both arms and an
// obstacle is described once for the whole rig rather than once per arm.
//
// gpmp2 silently reports "no obstacle" for queries outside the grid, so the
// grid must contain everywhere EITHER arm can put a collision sphere.
//
// MEASURED, not chosen. test_grid_coverage unions the right (tool) and left
// (flange) collision envelopes over a deterministic extremal sweep plus 200k
// random configurations per arm, and prints a paste-ready block when it
// fails. Measured 2026-08-06, in mount:
//   right  x [-1.052, 1.056]  y [-1.378, 0.654]  z [-0.952, 1.154]
//   left   x [-0.869, 0.870]  y [-0.478, 1.193]  z [-0.768, 0.971]
//   union  x [-1.052, 1.056]  y [-1.378, 1.193]  z [-0.952, 1.154]
// The two differ because the left chain ends at a bare flange and carries no
// gripper spheres — neither envelope can be mirrored from the other.
//
// Re-derive by running test_grid_coverage if the URDF, the mounting geometry
// or the sphere layout changes. Do NOT hand-tune these to make it pass.
inline constexpr double kGridOriginXM = -1.12;
inline constexpr double kGridOriginYM = -1.44;
inline constexpr double kGridOriginZM = -1.04;
inline constexpr double kGridCellM = 0.04;
inline constexpr int kGridNx = 57;
inline constexpr int kGridNy = 69;
inline constexpr int kGridNz = 58;
inline constexpr double kCollisionEnvelopeMarginM = 0.05;

struct GridGeometry {
    Eigen::Vector3d origin_world_m = Eigen::Vector3d::Zero();
    int nx = 0;
    int ny = 0;
    int nz = 0;
    double cell_m = kGridCellM;
};

// Axis-aligned WORLD grid enclosing all eight transformed corners of the
// measured Mount-frame workspace. Extra padding keeps those corners away
// from gpmp2's non-interpolatable upper face and preserves the established
// 0.05 m collision-envelope margin after arbitrary Mount rotations.
GridGeometry WorldGridGeometry(const Eigen::Isometry3d& world_T_mount);

// One grid covering both arms' workspaces, in the `mount` frame. ("World"
// here means the scene the arms move through, not a coordinate frame — the
// frame is named in the constants block above.) Cells hold distance to the
// box surface (negative inside); with no box, a uniform large free distance.
gpmp2::SignedDistanceField MakeWorldSdf(
    const GridGeometry& geometry,
    const std::optional<AxisAlignedBox>& box_world);

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
GridBounds WorldGridBounds(const GridGeometry& geometry);
