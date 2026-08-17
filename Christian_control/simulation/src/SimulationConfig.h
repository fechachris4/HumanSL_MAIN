//
// SimulationConfig — the one plain-data record describing an ideal-mode
// dual-arm simulation run (Plan 02 Task 5). Data only; the coordinator
// (DualSimulationRunner) consumes it, SimMain fills it from argv.
//
// Ideal sensing mode, by declaration (not truth leakage): the Mount state
// below is simultaneously the plant's prescription (MujocoBackend writes
// the pose into the Mount mocap body every tick) and the ideal sensor
// reading (the coordinator builds an exact WorldSample from the same
// sample every 2 ms tick, age 0, advancing sequence). Realistic 100 Hz
// sensing is Plan 03 Task 2.
//
// There is deliberately no Mount TWIST field. The sensed twist is the
// time derivative of the Mount motion being prescribed, so it belongs to
// whatever describes that motion — here the scripted MountMotionConfig,
// from which one SampleMountMotion(t) call returns pose and twist
// together. A separate configurable twist could not be checked against
// the pose, so a run could report a moving Mount to the controller while
// the plant held it still, and the controller would feed forward a base
// velocity that does not exist. (Plan 02's world_V_mount was deleted on
// 2026-08-17 for exactly that reason.)
//
// Units and frames: metres, radians, seconds; world_T_mount_at_zero is a
// Mount pose in Vicon-world axes W (State.h conventions).
//
// Validation ownership (validate once, at the boundary that consumes the
// value): control_dt_s and physics_substeps are validated by
// MujocoBackend's constructor; the two ExecutionConfig snapshots by
// ArmExecutionCore's constructor (ValidateExecutionConfig); mount_motion
// by SampleMountMotion itself, which the runner first calls in Start(),
// so a meaningless motion throws there rather than being re-checked here;
// only the fields nobody else owns are checked below.
//

#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include "ExecutionConfig.h"
#include "MountMotion.h" // MountMotionConfig
#include "State.h"       // CartesianPose

struct SimulationConfig {
    // Generated MJCF (model/humansl_dual_gen3.xml) and the kinematic
    // authority URDF the production Pinocchio kinematics load.
    std::string model_xml_path;
    std::string urdf_path;

    double control_dt_s = 0.002; // exactly the 500 Hz control tick, s
    int physics_substeps = 1;    // mj_step calls per control tick
    bool headless = true;        // false only under --viewer

    // Run LENGTH is deliberately not here. The coordinator never read it
    // — it steps one tick per Step() call — and the length now carries a
    // rule this record could not express: with the viewer, an absent or
    // zero --duration-s means "run until the window closes", which this
    // file's validator would have had to accept as a legal zero while
    // still rejecting it for headless runs. SimMain owns the run length,
    // because SimMain owns the loop (removed from here 2026-08-17).

    // The prescribed Mount motion, and the pose it swings about (the
    // pose at zero displacement; with the default phase 0 that is also
    // the pose at t = 0 — MountMotion.h states the convention). Together
    // they are the ONE description of the Mount: SampleMountMotion(t)
    // returns from them the pose that drives the plant and the twist the
    // ideal WorldSample carries, so plant and sensor cannot disagree.
    // The default motion is kStatic, i.e. the Mount stands still at the
    // anchor with an exactly zero twist — Plan 02's behaviour, now stated
    // rather than assumed.
    CartesianPose world_T_mount_at_zero;
    MountMotionConfig mount_motion;

    // Per-arm execution-core snapshots. Production values by default;
    // tests inject deviations (e.g. a tiny following-error limit) to force
    // a deterministic single-arm stop — the same construction-time
    // injection pattern Plan 01 introduced.
    ExecutionConfig right_execution = ProductionExecutionConfig();
    ExecutionConfig left_execution = ProductionExecutionConfig();
};

// Throws std::invalid_argument naming the offending field. Checks only
// what no other constructor owns (see the ownership note above).
inline void ValidateSimulationConfig(const SimulationConfig& config)
{
    if (config.model_xml_path.empty())
        throw std::invalid_argument(
            "SimulationConfig: model_xml_path must not be empty");
    if (config.urdf_path.empty())
        throw std::invalid_argument(
            "SimulationConfig: urdf_path must not be empty");
    if (!config.world_T_mount_at_zero.position_m.allFinite() ||
        !config.world_T_mount_at_zero.rotation.allFinite())
        throw std::invalid_argument(
            "SimulationConfig: non-finite world_T_mount_at_zero");
}
