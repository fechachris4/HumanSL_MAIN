// GoalCommand — one typed operator request for a mount-frame Cartesian task.

#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Dense>

enum class GoalKind { kPoint, kCircle };
enum class GoalOrientation { kInherit, kFixedRpy, kRadialInward };

struct GoalCommand {
    std::uint64_t command_id = 0;
    GoalKind kind = GoalKind::kPoint;

    Eigen::Vector3d point_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d circle_centre_m = Eigen::Vector3d::Zero();
    double circle_radius_m = 0.0;
    Eigen::Vector3d circle_normal = Eigen::Vector3d::UnitX();
    double circle_duration_s = 0.0;

    GoalOrientation orientation = GoalOrientation::kInherit;
    Eigen::Vector3d fixed_rpy_rad = Eigen::Vector3d::Zero();
};

std::optional<std::string> ValidateGoalCommand(const GoalCommand& command);
