#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "PlanSolver.h"
#include "StaticScene.h"

// One static-scene blocker, aggregated across every candidate attempt of a
// failed solve. Presentation-side only: this reads the evidence the search
// already recorded (CandidateEvidence::validation.first_scene_violations)
// and never touches planning behaviour.
struct SceneBlockerLine {
    std::string object_id;
    std::size_t sphere_index = 0;
    CollisionSphereGroup group = CollisionSphereGroup::kTool;
    double worst_clearance_m = 0.0;  // minimum across attempts (deficit if < floor)
    double worst_time_s = 0.0;       // trajectory time of that worst attempt
    std::size_t attempts_blocked = 0;
};

// yaml spelling of a sphere group ("upper_arm"), matching
// obstacles.scene.<object>.permitted_sphere_groups in planner.yaml so a
// printed blocker names the exact exemption knob that governs it.
const char* CollisionSphereGroupName(CollisionSphereGroup group);

// Collapses the scene violations of all TRAJECTORY attempts to one line per
// distinct (object, sphere), keeping each blocker's worst clearance, ordered
// worst-first. Terminal-IK screening rows (stage "terminal_ik") are static
// posture checks whose violation times are meaningless — they are excluded
// here and counted by SummarizeScreenedTerminals instead. `sphere_groups`
// maps authored sphere index to its group (PlannerModel::sphere_groups); an
// index outside the table reports kTool rather than failing — the summary
// must never be the thing that throws.
std::vector<SceneBlockerLine> SummarizeSceneBlockers(
    const std::vector<CandidateEvidence>& attempts,
    const std::vector<CollisionSphereGroup>& sphere_groups);

// Terminal candidates rejected at the static screening stage for scene
// contact: how many, and the worst offender. A large count with one sphere
// says most of the goal's IK branches put that link inside the obstacle —
// exactly the evidence that decides between "diversify the search" and
// "fix the scene model".
struct ScreenedTerminalSummary {
    std::size_t rejected_count = 0;
    SceneBlockerLine worst;
};
ScreenedTerminalSummary SummarizeScreenedTerminals(
    const std::vector<CandidateEvidence>& attempts,
    const std::vector<CollisionSphereGroup>& sphere_groups);

// The nearest miss among duration-repair attempts that exhausted the
// attempt budget: the one whose worst dynamic ratio is smallest. Lets a
// dynamic_attempts_exhausted failure say HOW close the repair came
// (2026-08-23: a collision-free circle died at acceleration ratio 1.031
// with the cap at 3 attempts, and the summary gave no hint).
struct DynamicAttemptEvidence {
    double velocity_ratio = 0.0;
    double acceleration_ratio = 0.0;
    double duration_s = 0.0;
};
std::optional<DynamicAttemptEvidence> ClosestDynamicAttempt(
    const std::vector<CandidateEvidence>& attempts);
