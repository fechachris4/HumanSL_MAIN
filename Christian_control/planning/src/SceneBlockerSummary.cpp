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

std::vector<SceneBlockerLine> SummarizeSceneBlockers(
    const std::vector<CandidateEvidence>& attempts,
    const std::vector<CollisionSphereGroup>& sphere_groups) {
    std::vector<SceneBlockerLine> lines;
    for (const CandidateEvidence& attempt : attempts) {
        for (const SceneViolationEvidence& violation :
             attempt.validation.first_scene_violations) {
            auto existing = std::find_if(
                lines.begin(), lines.end(), [&](const SceneBlockerLine& line) {
                    return line.object_id == violation.object_id &&
                           line.sphere_index == violation.sphere_index;
                });
            if (existing == lines.end()) {
                SceneBlockerLine line;
                line.object_id = violation.object_id;
                line.sphere_index = violation.sphere_index;
                line.group = violation.sphere_index < sphere_groups.size()
                                 ? sphere_groups[violation.sphere_index]
                                 : CollisionSphereGroup::kTool;
                line.worst_clearance_m = violation.clearance_m;
                line.worst_time_s = violation.time_s;
                line.attempts_blocked = 1;
                lines.push_back(std::move(line));
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
