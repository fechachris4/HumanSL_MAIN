#pragma once
#include <optional>
#include <Eigen/Dense>
#include <gpmp2/obstacle/SignedDistanceField.h>

struct AxisAlignedBox {           // metres, base_link
    Eigen::Vector3d center;
    Eigen::Vector3d half_extent;
};

// Grid covering the right-arm workspace: origin (-1.2,-1.2,-0.4),
// cell 0.04 m, 60x60x40 cells. Cells hold distance to the box surface
// (negative inside); with no box, a uniform large free distance.
gpmp2::SignedDistanceField MakeWorldSdf(const std::optional<AxisAlignedBox>& box);
