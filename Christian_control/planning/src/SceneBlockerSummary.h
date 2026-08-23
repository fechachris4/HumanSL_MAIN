#pragma once

#include <cstddef>
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

// Collapses the scene violations of all attempts to one line per distinct
// (object, sphere), keeping each blocker's worst clearance, ordered
// worst-first. `sphere_groups` maps authored sphere index to its group
// (PlannerModel::sphere_groups); an index outside the table reports kTool
// rather than failing — the summary must never be the thing that throws.
std::vector<SceneBlockerLine> SummarizeSceneBlockers(
    const std::vector<CandidateEvidence>& attempts,
    const std::vector<CollisionSphereGroup>& sphere_groups);
