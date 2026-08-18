#include "PathValidation.h"
#include <cmath>
#include <cstdio>

namespace {
// Mirrors config::kJointSoftwareLimitDeg (Config.h:168): Table 39's upper
// magnitude less the 2 deg software margin, capped by the firmware warn
// threshold. j2 126.9 = 128.9 - 2 (tighter than its 130 warn limit); j4/j6
// are already capped by their warn limits (145.0, 118.0). Pinned against
// the live Config.h constant by tests/test_waypoints.cpp.
constexpr std::array<double, 7> kValidationLimitDeg = {
    0, 126.9, 0, 145.0, 0, 118.0, 0};

// Continuous joints (1/3/5/7) have no software stop, so GPMP2's flat
// (unwrapped) configuration space is unconstrained on them. If a seed
// ever lands near the wrap seam again — StartState wraps it now, but this
// is the last check before the controller's own ±360 deg ingest gate —
// the optimizer is free to settle a solution a full revolution away from
// what it planned geometrically. Mirrors Targets.cpp's
// kContinuousJointBoundDeg so the bridge rejects the same trajectories
// the controller would, before they leave this process.
constexpr double kContinuousJointBoundDeg = 360.0;
}  // namespace

const std::array<double, 7>& ValidationLimitsDeg() { return kValidationLimitDeg; }

std::optional<std::string> ValidateJointPath(
    const std::vector<gtsam::Vector>& trajectory_pos) {
    const std::array<double, 7>& limit_deg = ValidationLimitsDeg();
    for (std::size_t s = 0; s < trajectory_pos.size(); ++s)
        for (int j = 0; j < 7; ++j) {
            const double q_deg = trajectory_pos[s](j) * 180.0 / M_PI;
            const bool continuous = (j == 0 || j == 2 || j == 4 || j == 6);
            const double bound = continuous ? kContinuousJointBoundDeg : limit_deg[j];
            if (std::abs(q_deg) > bound) {
                char buffer[128];
                std::snprintf(buffer, sizeof buffer,
                              "support state %zu: joint %d at %.1f deg exceeds ±%.1f",
                              s, j + 1, q_deg, bound);
                return std::string(buffer);
            }
        }
    return std::nullopt;
}
