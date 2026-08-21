#include "PlanningRequest.h"

#include <cmath>

namespace {
bool Finite(double value) { return std::isfinite(value); }
}  // namespace

std::optional<std::string> ValidatePlanningRequest(
    const PlanningRequest& request)
{
    if (request.request_id == 0)
        return "request_id must be nonzero";
    if (request.vicon_sequence == 0)
        return "vicon_sequence must be nonzero";
    if (!Finite(request.receive_steady_s) || request.receive_steady_s < 0.0)
        return "receive_steady_s must be finite and nonnegative";
    if (!Finite(request.age_s) || request.age_s < 0.0 ||
        request.age_s > kPlanningRequestMaximumAgeS)
        return "age_s must be in [0, 0.05]";
    if (!request.world_T_mount.matrix().allFinite())
        return "world_T_mount must be finite";
    const Eigen::Matrix3d& rotation = request.world_T_mount.linear();
    if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() >
            1e-6 ||
        std::abs(rotation.determinant() - 1.0) > 1e-6)
        return "world_T_mount rotation must be proper orthonormal";
    if (!request.q_rad.allFinite())
        return "q_rad must be finite";
    if (const std::optional<std::string> error =
            ValidateGoalCommand(request.goal))
        return "goal: " + *error;
    return std::nullopt;
}
