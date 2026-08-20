//
// BaselineBridge — the known-good planner (branch baseline/planner-0807 =
// commit 5abc1b2c, 2026-08-07) run as a SUBPROCESS, adapted into the typed
// contract the current controller consumes.
//
// Why this exists: the baseline planner is the last state empirically
// proven to plan the 0.1 m circle at 4.6 mm and emit it (2026-08-20 record,
// muve ~/Desktop/HumanSL_baseline_evidence/). Its wire output is a timed
// JOINT trajectory block (TRAJ_BEGIN … deg, deg/s … TRAJ_END on stdout);
// the current controller consumes a WorldCartesianTrajectory. This adapter
// bridges exactly that boundary and nothing else:
//
//   baseline binary (frozen planner maths, frozen tuning, its own
//   planner.yaml / joint_limits.yaml / DH resolved beside itself)
//     -> parse the joint block (deg -> rad, times as emitted)
//     -> ProjectWorldTrajectory through the CURRENT model's FK
//     -> the same mailbox handoff SolveWorldTrajectoryForRequest uses.
//
// What is deliberately NOT reinterpreted, and what is:
//   - arm identity, timing, and joint values pass through 1:1 (unit
//     conversion deg->rad only); every count/duration is logged.
//   - the start state is wrapped to principal radians BEFORE the
//     subprocess call, reproducing the baseline's own CSV-transport
//     behaviour (its --start-deg path predates that wrap — feeding raw
//     [0,360) Kortex angles would recreate the 2026-08-19 joint-band
//     failure). This is the one accommodation, and it is applied here at
//     the boundary, not by editing the baseline.
//   - the goal file is the CURRENT panel-edited goal.yaml (the baseline
//     parses it; verified 2026-08-20). Its tuning/limit files are its own.
//   - Cartesian projection and execution use the CURRENT model
//     (URDF/mount geometry), while the baseline resolved its mount-frame
//     goal against ITS OWN 2026-08-07 URDF (base spacing 0.113 m vs the
//     current measured 0.075 m). The joint trajectory itself is
//     geometry-free, so it is executed faithfully; but a goal stated in
//     mount coordinates is interpreted by the BASELINE's geometry. The
//     diagnostics say so on every solve rather than hiding it.
//   - the executed control law is the CURRENT controller's Cartesian
//     following, not the 2026-08-07 joint-space law.
//
#pragma once

#include <ostream>

#include "PlannerRuntime.h"

// Same signature as SolveWorldTrajectoryForRequest, so Main.cpp selects
// between them with one function pointer (InProcessPlanner's solve seam).
// Uses config.baseline_bridge_binary; exit_code mirrors the baseline
// binary's own codes (0 emitted, 4 validation rejected, ...), with the
// adapter's own failures reported as nonzero + diagnostics.
PlannerSolveResult SolveBaselineBridgeForRequest(
    const PlanningRequest& request,
    const PlannerRuntimeConfig& config,
    std::ostream& diagnostics);
