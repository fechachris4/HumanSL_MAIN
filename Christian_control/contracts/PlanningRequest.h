// PlanningRequest — fixed-size typed controller-to-planner snapshot.

#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Geometry>

#include "GoalCommand.h"

enum class PlanningArm { kRight, kLeft };

struct PlanningRequest {
    std::uint64_t request_id = 0;
    PlanningArm arm = PlanningArm::kRight;
    std::uint64_t vicon_sequence = 0;
    std::uint32_t vicon_frame_number = 0;
    double receive_steady_s = 0.0;
    double age_s = 0.0;
    Eigen::Isometry3d world_T_mount = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 7, 1> q_rad =
        Eigen::Matrix<double, 7, 1>::Zero();
    GoalCommand goal;
};

inline constexpr double kPlanningRequestMaximumAgeS = 0.05;

std::optional<std::string> ValidatePlanningRequest(
    const PlanningRequest& request);
