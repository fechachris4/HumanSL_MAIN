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
    evidence.duration_attempt = 1;
    evidence.duration_s = 14.1;
    SceneViolationEvidence violation;
    violation.object_id = object;
    violation.sphere_index = sphere;
    violation.time_s = time_s;
    violation.clearance_m = clearance_m;
    evidence.validation.first_scene_violations.push_back(violation);
    return evidence;
}

// A terminal-IK screening row: a static posture check on one candidate
// terminal configuration. Its violation times are meaningless (hardcoded 0)
// and it must never be reported as a trajectory blocker.
CandidateEvidence ScreenedTerminal(std::size_t sphere, double clearance_m) {
    CandidateEvidence evidence = AttemptWithViolation("torso", sphere, 0.0, clearance_m);
    evidence.stage = "terminal_ik";
    evidence.disposition = "terminal_scene_clearance";
    evidence.duration_attempt = 0;
    evidence.duration_s = 0.0;
    return evidence;
}

CandidateEvidence DynamicExhausted(double velocity_ratio,
                                   double acceleration_ratio,
                                   double duration_s) {
    CandidateEvidence evidence;
    evidence.disposition = "dynamic_attempts_exhausted";
    evidence.duration_attempt = 3;
    evidence.duration_s = duration_s;
    evidence.validation.max_velocity_ratio = velocity_ratio;
    evidence.validation.max_acceleration_ratio = acceleration_ratio;
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

    // Terminal-IK screening rows are static posture checks, not trajectory
    // states (2026-08-23: 16 screened candidates printed as "blockers at
    // t=0.00 s", implying an impossible 142 mm clearance jump within 4 ms
    // of a start pinned to 1e-12 rad). They must be counted separately.
    {
        const std::vector<CandidateEvidence> attempts = {
            ScreenedTerminal(15, -0.014), ScreenedTerminal(15, -0.010),
            ScreenedTerminal(14, 0.0444),
            AttemptWithViolation("torso", 7, 4.2, 0.030)};
        const auto lines = SummarizeSceneBlockers(attempts, groups);
        Check(lines.size() == 1 && lines[0].sphere_index == 7,
              "screening rows are excluded from trajectory blockers");
        const auto screened = SummarizeScreenedTerminals(attempts, groups);
        Check(screened.rejected_count == 3, "all screened candidates counted");
        Check(screened.worst.sphere_index == 15 &&
                  screened.worst.worst_clearance_m == -0.014,
              "worst screened candidate kept");
        Check(screened.worst.group == CollisionSphereGroup::kTool,
              "screened sphere resolves through the group table");
    }
    {
        const auto screened = SummarizeScreenedTerminals(
            {AttemptWithViolation("torso", 7, 4.2, 0.030)}, groups);
        Check(screened.rejected_count == 0, "no screening rows, no summary");
    }

    // The closest dynamic-repair attempt: smallest worst-ratio among
    // exhausted attempts, so the summary can say how near the repair came.
    {
        const std::vector<CandidateEvidence> attempts = {
            DynamicExhausted(1.011, 0.778, 15.5),
            DynamicExhausted(0.863, 1.031, 21.5),
            DynamicExhausted(0.873, 1.036, 21.3)};
        const auto closest = ClosestDynamicAttempt(attempts);
        Check(closest.has_value(), "exhausted attempts produce a closest");
        if (closest) {
            Check(closest->duration_s == 15.5,
                  "closest is the smallest worst-ratio attempt");
            Check(closest->velocity_ratio == 1.011 &&
                      closest->acceleration_ratio == 0.778,
                  "closest carries both ratios");
        }
    }
    {
        const auto closest = ClosestDynamicAttempt(
            {AttemptWithViolation("torso", 7, 4.2, 0.030)});
        Check(!closest.has_value(), "no exhausted attempts, no closest");
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
