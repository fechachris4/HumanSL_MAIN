#include "SceneBlockerSummary.h"

#include <algorithm>

const char* CollisionSphereGroupName(CollisionSphereGroup group) {
    switch (group) {
        case CollisionSphereGroup::kMountInterface: return "mount_interface";
        case CollisionSphereGroup::kProximalArm: return "proximal_arm";
        case CollisionSphereGroup::kUpperArm: return "upper_arm";
        case CollisionSphereGroup::kForearm: return "forearm";
        case CollisionSphereGroup::kTool: return "tool";
    }
    return "tool";
}

namespace {

SceneBlockerLine MakeLine(const SceneViolationEvidence& violation,
                          const std::vector<CollisionSphereGroup>& sphere_groups) {
    SceneBlockerLine line;
    line.object_id = violation.object_id;
    line.sphere_index = violation.sphere_index;
    line.group = sphere_groups[violation.sphere_index];
    line.worst_clearance_m = violation.clearance_m;
    line.worst_time_s = violation.time_s;
    line.attempts_blocked = 1;
    return line;
}

bool IsScreeningRow(const CandidateEvidence& attempt) {
    return attempt.stage == "terminal_ik";
}

}  // namespace

std::vector<SceneBlockerLine> SummarizeSceneBlockers(
    const std::vector<CandidateEvidence>& attempts,
    const std::vector<CollisionSphereGroup>& sphere_groups) {
    std::vector<SceneBlockerLine> lines;
    for (const CandidateEvidence& attempt : attempts) {
        if (IsScreeningRow(attempt)) continue;
        for (const SceneViolationEvidence& violation :
             attempt.validation.first_scene_violations) {
            auto existing = std::find_if(
                lines.begin(), lines.end(), [&](const SceneBlockerLine& line) {
                    return line.object_id == violation.object_id &&
                           line.sphere_index == violation.sphere_index;
                });
            if (existing == lines.end()) {
                lines.push_back(MakeLine(violation, sphere_groups));
            } else {
                ++existing->attempts_blocked;
                if (violation.clearance_m < existing->worst_clearance_m) {
                    existing->worst_clearance_m = violation.clearance_m;
                    existing->worst_time_s = violation.time_s;
                }
            }
        }
    }
    std::sort(lines.begin(), lines.end(),
              [](const SceneBlockerLine& a, const SceneBlockerLine& b) {
                  return a.worst_clearance_m < b.worst_clearance_m;
              });
    return lines;
}

ScreenedTerminalSummary SummarizeScreenedTerminals(
    const std::vector<CandidateEvidence>& attempts,
    const std::vector<CollisionSphereGroup>& sphere_groups) {
    ScreenedTerminalSummary summary;
    for (const CandidateEvidence& attempt : attempts) {
        if (!IsScreeningRow(attempt)) continue;
        if (attempt.validation.first_scene_violations.empty()) continue;
        const SceneViolationEvidence& violation =
            attempt.validation.first_scene_violations.front();
        ++summary.rejected_count;
        if (summary.rejected_count == 1 ||
            violation.clearance_m < summary.worst.worst_clearance_m)
            summary.worst = MakeLine(violation, sphere_groups);
    }
    return summary;
}

std::optional<DynamicAttemptEvidence> ClosestDynamicAttempt(
    const std::vector<CandidateEvidence>& attempts) {
    std::optional<DynamicAttemptEvidence> closest;
    double closest_worst_ratio = 0.0;
    for (const CandidateEvidence& attempt : attempts) {
        if (attempt.disposition != "dynamic_attempts_exhausted") continue;
        const double worst_ratio =
            std::max(attempt.validation.max_velocity_ratio,
                     attempt.validation.max_acceleration_ratio);
        if (!closest || worst_ratio < closest_worst_ratio) {
            closest_worst_ratio = worst_ratio;
            closest = DynamicAttemptEvidence{
                attempt.validation.max_velocity_ratio,
                attempt.validation.max_acceleration_ratio,
                attempt.duration_s};
        }
    }
    return closest;
}
