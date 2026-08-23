//
// SummarizeSceneBlockers — the aggregation behind the "scene blockers"
// lines a FAILED plan summary prints. Pure logic over recorded candidate
// evidence: no planner, no model, no robot.
//
// Why it exists: on 2026-08-23 a traced-circle plan failed scene_clearance
// 18 times and the summary said only "error: scene_clearance" — the
// violating sphere, its time on the path, and the clearance deficit were
// all collected per attempt and then never printed.
//

#include <cstdio>
#include <string>

#include "SceneBlockerSummary.h"

namespace {

int failures = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++failures;
    }
}

CandidateEvidence AttemptWithViolation(const std::string& object,
                                       std::size_t sphere, double time_s,
                                       double clearance_m) {
    CandidateEvidence evidence;
    evidence.disposition = "scene_clearance";
    SceneViolationEvidence violation;
    violation.object_id = object;
    violation.sphere_index = sphere;
    violation.time_s = time_s;
    violation.clearance_m = clearance_m;
    evidence.validation.first_scene_violations.push_back(violation);
    return evidence;
}

}  // namespace

int main() {
    const std::vector<CollisionSphereGroup> groups = {
        CollisionSphereGroup::kMountInterface, CollisionSphereGroup::kProximalArm,
        CollisionSphereGroup::kProximalArm,    CollisionSphereGroup::kProximalArm,
        CollisionSphereGroup::kUpperArm,       CollisionSphereGroup::kUpperArm,
        CollisionSphereGroup::kUpperArm,       CollisionSphereGroup::kUpperArm};

    // No attempts, or attempts without violations: nothing to report.
    {
        const auto lines = SummarizeSceneBlockers({}, groups);
        Check(lines.empty(), "no attempts produces no blocker lines");
    }
    {
        CandidateEvidence clean;
        clean.disposition = "route_seed_ik_failure";
        const auto lines = SummarizeSceneBlockers({clean}, groups);
        Check(lines.empty(), "attempt without violations produces no lines");
    }

    // One blocker seen by several attempts: a single line carrying the
    // worst (most negative) clearance and how many attempts it blocked.
    {
        const std::vector<CandidateEvidence> attempts = {
            AttemptWithViolation("torso", 7, 4.2, 0.031),
            AttemptWithViolation("torso", 7, 5.0, 0.012),
            AttemptWithViolation("torso", 7, 4.6, 0.027)};
        const auto lines = SummarizeSceneBlockers(attempts, groups);
        Check(lines.size() == 1, "same object+sphere collapses to one line");
        if (lines.size() == 1) {
            Check(lines[0].object_id == "torso", "object id kept");
            Check(lines[0].sphere_index == 7, "sphere index kept");
            Check(lines[0].group == CollisionSphereGroup::kUpperArm,
                  "sphere 7 resolves to its group");
            Check(lines[0].worst_clearance_m == 0.012,
                  "worst clearance is the minimum across attempts");
            Check(lines[0].worst_time_s == 5.0,
                  "time reported is the worst attempt's time");
            Check(lines[0].attempts_blocked == 3, "all blocking attempts counted");
        }
    }

    // Distinct blockers stay distinct, ordered worst-first.
    {
        const std::vector<CandidateEvidence> attempts = {
            AttemptWithViolation("torso", 7, 4.2, 0.030),
            AttemptWithViolation("torso", 5, 1.1, -0.004)};
        const auto lines = SummarizeSceneBlockers(attempts, groups);
        Check(lines.size() == 2, "different spheres get separate lines");
        if (lines.size() == 2) {
            Check(lines[0].sphere_index == 5 && lines[0].worst_clearance_m == -0.004,
                  "worst blocker listed first");
        }
    }

    // A sphere index outside the group table must not crash the summary.
    {
        const std::vector<CandidateEvidence> attempts = {
            AttemptWithViolation("torso", 99, 0.0, 0.0)};
        const auto lines = SummarizeSceneBlockers(attempts, groups);
        Check(lines.size() == 1 && lines[0].group == CollisionSphereGroup::kTool,
              "out-of-table sphere index falls back without crashing");
    }

    // Group names for the printed line.
    Check(std::string(CollisionSphereGroupName(CollisionSphereGroup::kUpperArm)) ==
              "upper_arm",
          "group name matches the yaml spelling");
    Check(std::string(CollisionSphereGroupName(CollisionSphereGroup::kMountInterface)) ==
              "mount_interface",
          "mount interface name matches the yaml spelling");

    if (failures == 0) std::printf("test_scene_blocker_summary: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
