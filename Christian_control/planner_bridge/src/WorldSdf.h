#pragma once
#include <optional>
#include <Eigen/Dense>
#include <gpmp2/obstacle/SignedDistanceField.h>

struct AxisAlignedBox {           // metres, base_link
    Eigen::Vector3d center;
    Eigen::Vector3d half_extent;
};

// Grid geometry, shared by MakeWorldSdf and WorldGridBounds so the two can
// never disagree. z spans -0.4..1.6 m, covering the zero-config tool
// height (1.3073 m measured 2026-08-05) with headroom.
inline constexpr double kGridOriginXM = -1.2;
inline constexpr double kGridOriginYM = -1.2;
inline constexpr double kGridOriginZM = -0.4;
inline constexpr double kGridCellM = 0.04;
inline constexpr int kGridNx = 60;
inline constexpr int kGridNy = 60;
inline constexpr int kGridNz = 50;

// Grid covering the right-arm workspace: origin (-1.2,-1.2,-0.4),
// cell 0.04 m, 60x60x50 cells (x/y/z extents: see WorldGridBounds()).
// Cells hold distance to the box surface (negative inside); with no box,
// a uniform large free distance.
gpmp2::SignedDistanceField MakeWorldSdf(const std::optional<AxisAlignedBox>& box);

// The world-frame (base_link) volume the SDF grid actually covers:
// [origin, origin + (nx,ny,nz)*cell] per axis. gpmp2 returns zero obstacle
// cost for any query outside this volume ("no obstacle"), silently — so an
// `--box` that is not fully contained here must be rejected before solving,
// not discovered as a missed obstacle after the fact.
struct GridBounds {
    Eigen::Vector3d min_m;
    Eigen::Vector3d max_m;
};
GridBounds WorldGridBounds();
