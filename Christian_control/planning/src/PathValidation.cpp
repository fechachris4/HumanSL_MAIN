#include "PathValidation.h"
#include <cmath>
#include <cstddef>
#include <cstdio>

#include "Config.h"

std::optional<std::string> ValidateJointPath(
    const std::vector<gtsam::Vector>& trajectory_pos) {
    constexpr double kRadToDeg = 180.0 / M_PI;
    for (std::size_t s = 0; s < trajectory_pos.size(); ++s)
        for (std::size_t j = 0; j < config::limits::kBoundedMask.size(); ++j) {
            if (!config::limits::kBoundedMask[j]) continue;
            const double lower_deg = config::limits::kControllerLowerDeg[j];
            const double upper_deg = config::limits::kControllerUpperDeg[j];
            const double q_deg =
                trajectory_pos[s](static_cast<int>(j)) * kRadToDeg;
            if (q_deg < lower_deg || q_deg > upper_deg) {
                char buffer[128];
                std::snprintf(buffer, sizeof buffer,
                              "support state %zu: joint %zu at %.1f deg is "
                              "outside [%.1f, %.1f]",
                              s, j + 1, q_deg, lower_deg, upper_deg);
                return std::string(buffer);
            }
        }
    return std::nullopt;
}
