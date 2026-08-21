#pragma once
//
// PlannerConfig — config/planner.yaml, loaded strictly.
//
// "Strictly" is what the file buys over compiled constants: every key is
// required, unknown keys are refused, and every value is range-checked, so
// a typo or a fat-fingered magnitude fails the run with a message naming
// the key instead of quietly planning something else. Nothing here falls
// back to a silent default.
//
// What is deliberately NOT loadable — joint limits, the SDF grid extent,
// the collision-sphere radius, and everything in control's Config.h —
// is listed with its reason at the top of config/planner.yaml. Safety
// policy stays compiled (docs/decisions/compiled-config.md).
//

#include <cstdint>
#include <string>
#include <vector>

#include "StaticScene.h"
#include "TrajectoryOptimization.h"  // OptimizerTuning

// How the plan is shaped in time. The defaults are the literals PlanSolver
// used before this file existed; test_planner_config holds the checked-in
// YAML to them, so the file and the code cannot drift apart unnoticed.
struct MotionConfig {
    // 0.25 m/s / 1.0 s floor since 2026-08-13: bring-up pacing (0.05 /
    // 4.0) retired with the speed-limit raise (docs/motion-limits-map.md).
    double nominal_speed_mps = 0.25;
    double min_duration_s = 1.0;
    int waypoints = 10;
};

// Path following. Two DIFFERENT kinds of number live here and must never
// be conflated: the sigmas are optimiser WEIGHTS (gtsam sigmas — smaller is
// stiffer), while maximum_planning_error_m is the pass/fail REQUIREMENT the
// finished trajectory is measured against. Tightening a weight does not
// tighten the requirement, and meeting the requirement is not guaranteed by
// any weight.
struct PathFollowingConfig {
    // Weights on each traced waypoint's pose prior.
    double position_prior_sigma_m = 0.005;
    double rotation_prior_sigma_rad = 0.01;
    // The GATE, applied to the reconstructed trajectory the controller will
    // actually execute (not to the optimiser's own cost).
    double maximum_planning_error_m = 0.005;
    double maximum_orientation_error_rad = 0.1;
    // Dense validation rate. 0.002 s = 500 Hz, the controller's own rate.
    double validation_dt_s = 0.002;
    // Approach phase: paced from joint displacement over joint velocity
    // limits, never from Cartesian distance (it has no commanded Cartesian
    // route to measure). 0.9 / 0.1 s since 2026-08-13, with the
    // speed-limit raise (docs/motion-limits-map.md).
    double approach_velocity_fraction = 0.9;
    double approach_min_duration_s = 0.1;
    int approach_waypoints = 5;
    // Circle sampling by geometry, not by an arbitrary count:
    // e = r(1 - cos(pi/N)) sets N from the chord error you will accept.
    double max_chord_error_m = 0.001;
};

// IK run provenance retained by planner.yaml and the run report. Path IK uses
// deterministic continuation plus bounded null-space alternatives; these
// fields do not control its search or point-goal IK.
struct SeedingConfig {
    std::uint64_t ik_seed = 20260807;
    bool randomised = false;
};

struct PlannerConfig {
    MotionConfig motion;
    SeedingConfig seeding;
    // The seed ACTUALLY used this run: `seeding.ik_seed`, or the drawn value
    // when randomised. Reported and stored in session metadata.
    std::uint64_t effective_ik_seed = 20260807;
    PathFollowingConfig path_following;
    OptimizerTuning optimizer;
    // Named obstacle geometry, persisted once in config/planner.yaml. Every
    // value is in the mount frame, in metres; enabled objects remain present
    // when disabled so the panel does not lose an intentionally hidden shape.
    std::vector<NamedStaticObstacle> scene;

    // Provenance: which file this came from, and a digest of its exact
    // bytes. Both are echoed on every run so a session log identifies the
    // tuning that produced its trajectory. FNV-1a 64 is a change detector,
    // not a cryptographic hash — named so nobody mistakes it for one.
    std::string source_path;
    std::uint64_t source_fnv1a64 = 0;
};

// Throws std::runtime_error naming the offending key on a missing key, an
// unknown key, a wrong type, or an out-of-range value.
PlannerConfig LoadPlannerConfig(const std::string& path);

// Every effective value, one `key = value` per line, plus the source path
// and digest. Written to the bridge's diagnostics stream on every run.
std::string EffectiveConfigText(const PlannerConfig& config);
