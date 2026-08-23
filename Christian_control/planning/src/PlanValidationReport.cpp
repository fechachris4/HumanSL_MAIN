#include "PlanValidationReport.h"

const char* PlanStatusName(PlanStatus status) {
    switch (status) {
    case PlanStatus::kReached: return "REACHED";
    case PlanStatus::kGoalBlocked: return "GOAL_BLOCKED";
    case PlanStatus::kFailed: return "FAILED";
    }
    return "FAILED";
}

bool IsExecutable(PlanStatus status) {
    return status == PlanStatus::kReached || status == PlanStatus::kGoalBlocked;
}
