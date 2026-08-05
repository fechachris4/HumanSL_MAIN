#include "Waypoints.h"
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

std::vector<Eigen::Vector3d> SampleCartesianWaypoints(
    const PlannerModel& model, const std::vector<gtsam::Vector>& trajectory_pos,
    std::size_t max_count, double min_spacing_m,
    std::vector<Eigen::Quaterniond>* rotations_xyzw) {
    std::vector<Eigen::Vector3d> waypoints;
    if (rotations_xyzw) rotations_xyzw->clear();
    if (trajectory_pos.size() < 2) return waypoints;
    Eigen::Matrix<double, 7, 1> q(trajectory_pos.front());
    Eigen::Vector3d last_kept = ToolPositionInBaseLink(model, q);
    for (std::size_t s = 1; s + 1 < trajectory_pos.size(); ++s) {
        q = trajectory_pos[s];
        const Eigen::Vector3d p = ToolPositionInBaseLink(model, q);
        if ((p - last_kept).norm() >= min_spacing_m &&
            waypoints.size() + 1 < max_count) {   // reserve one slot for the goal
            waypoints.push_back(p);
            if (rotations_xyzw)
                rotations_xyzw->push_back(ToolPoseInBaseLink(model, q).rotation().toQuaternion());
            last_kept = p;
        }
    }
    q = trajectory_pos.back();
    const Eigen::Vector3d goal = ToolPositionInBaseLink(model, q);
    // The goal always wins: drop any trailing kept intermediates that fall
    // within min_spacing_m of it (GPMP2 trajectories cluster support states
    // near the goal), so the final consecutive pair still meets the spacing
    // guarantee. Evicted rotations are popped in lockstep so the two arrays
    // stay 1:1.
    while (!waypoints.empty() &&
           (goal - waypoints.back()).norm() < min_spacing_m) {
        waypoints.pop_back();
        if (rotations_xyzw) rotations_xyzw->pop_back();
    }
    waypoints.push_back(goal);
    if (rotations_xyzw)
        rotations_xyzw->push_back(ToolPoseInBaseLink(model, q).rotation().toQuaternion());
    return waypoints;
}

std::string FormatTargetLine(const Eigen::Vector3d& position_m) {
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%.6f %.6f %.6f",
                  position_m.x(), position_m.y(), position_m.z());
    return std::string(buffer);
}

std::string FormatTargetLine(const Eigen::Vector3d& position_m,
                             const Eigen::Quaterniond& rotation_xyzw) {
    char buffer[128];
    std::snprintf(buffer, sizeof buffer, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f",
                  position_m.x(), position_m.y(), position_m.z(),
                  rotation_xyzw.x(), rotation_xyzw.y(), rotation_xyzw.z(),
                  rotation_xyzw.w());
    return std::string(buffer);
}
