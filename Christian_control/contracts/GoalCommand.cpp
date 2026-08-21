#include "GoalCommand.h"

#include <cmath>

std::optional<std::string> ValidateGoalCommand(const GoalCommand& command)
{
    if (command.command_id == 0)
        return "command_id must be nonzero";
    if (!command.fixed_rpy_rad.allFinite())
        return "goal orientation must be finite";
    if (command.kind == GoalKind::kPoint) {
        if (!command.point_m.allFinite())
            return "point goal must be finite";
        if (command.orientation == GoalOrientation::kRadialInward)
            return "point goal cannot use radial orientation";
        return std::nullopt;
    }
    if (!command.circle_centre_m.allFinite() ||
        !command.circle_normal.allFinite())
        return "circle goal must be finite";
    if (!std::isfinite(command.circle_radius_m) ||
        command.circle_radius_m <= 0.0)
        return "circle radius must be finite and positive";
    if (!std::isfinite(command.circle_duration_s) ||
        command.circle_duration_s <= 0.0)
        return "circle duration must be finite and positive";
    if (command.circle_normal.norm() < 1e-9)
        return "circle normal must be nonzero";
    if (command.orientation == GoalOrientation::kInherit)
        return "circle goal requires fixed or radial orientation";
    return std::nullopt;
}
