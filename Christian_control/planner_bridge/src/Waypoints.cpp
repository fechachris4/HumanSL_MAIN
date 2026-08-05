#include "Waypoints.h"
#include <cmath>
#include <cstdio>

std::optional<std::string> ValidateJointPath(
    const std::vector<gtsam::Vector>& trajectory_pos) {
    constexpr double kLimitDeg[7] = {0, 130.0, 0, 145.0, 0, 118.0, 0};
    for (std::size_t s = 0; s < trajectory_pos.size(); ++s)
        for (int j : {1, 3, 5}) {
            const double q_deg = trajectory_pos[s](j) * 180.0 / M_PI;
            if (std::abs(q_deg) > kLimitDeg[j]) {
                char buffer[128];
                std::snprintf(buffer, sizeof buffer,
                              "support state %zu: joint %d at %.1f deg exceeds ±%.0f",
                              s, j + 1, q_deg, kLimitDeg[j]);
                return std::string(buffer);
            }
        }
    return std::nullopt;
}

std::vector<Eigen::Vector3d> SampleCartesianWaypoints(
    const PlannerModel& model, const std::vector<gtsam::Vector>& trajectory_pos,
    std::size_t max_count, double min_spacing_m) {
    std::vector<Eigen::Vector3d> waypoints;
    if (trajectory_pos.size() < 2) return waypoints;
    Eigen::Matrix<double, 7, 1> q(trajectory_pos.front());
    Eigen::Vector3d last_kept = ToolPositionInBaseLink(model, q);
    for (std::size_t s = 1; s + 1 < trajectory_pos.size(); ++s) {
        q = trajectory_pos[s];
        const Eigen::Vector3d p = ToolPositionInBaseLink(model, q);
        if ((p - last_kept).norm() >= min_spacing_m &&
            waypoints.size() + 1 < max_count) {   // reserve one slot for the goal
            waypoints.push_back(p);
            last_kept = p;
        }
    }
    q = trajectory_pos.back();
    waypoints.push_back(ToolPositionInBaseLink(model, q));
    return waypoints;
}

std::string FormatTargetLine(const Eigen::Vector3d& position_m) {
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%.6f %.6f %.6f",
                  position_m.x(), position_m.y(), position_m.z());
    return std::string(buffer);
}
