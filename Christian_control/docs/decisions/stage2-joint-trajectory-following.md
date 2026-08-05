# Decision: the joint-trajectory source is the only motion path

Date: 2026-08-05. Supersedes the Cartesian waypoint parts of
`stage15-bridge-workflow.md` and `stage1-planner-bridge.md`; those records
stay as history and are not edited.

## What replaced what

The bridge used to solve a path with GPMP2, thin it to at most eight
Cartesian tool positions, and write them as `x y z` lines. The controller
then re-planned motion *between* those points with its own seventh-order
rest-to-rest profile (`TrajectoryProfile.h`, driven by `PoseTargetSource`).

Planning therefore happened twice, and the second time with less
information: the solve's joint velocities were discarded and reinvented
from a scalar profile that knew only the straight-line distance between
consecutive waypoints. The arm came to rest at every waypoint because each
segment was rest-to-rest by construction.

Now the bridge emits the solve's own support states as one framed block of
timed joint rows (`TRAJ_BEGIN <count>` / rows of `t q1..q7 v1..v7` /
`TRAJ_END`), and `JointTrajectorySource` samples that block at 500 Hz with
cubic Hermite interpolation, tracked by
`q̇_cmd = q̇_ref + Kp_j·(q_ref − q_meas)`. The planner owns timing; the
controller follows.

This deliberately reinstates planner-owned timing, reversing the
2026-08-04 playback removal. The safety argument is different this time,
and it is what made the reversal acceptable:

- the whole block is validated on receipt, off the RT thread, against the
  compiled joint limits and velocity clip — nothing is published unless it
  passes as a whole;
- activation is gated by a splice guard: the block's first point must be
  within `kTrajStartToleranceDeg` (2 deg) of the measured position on every
  joint, or the block is dropped whole and the rejection reported;
- the joint-space following-error stop (`kTrajFollowingErrorStopDeg`) runs
  throughout.

The actuation path is untouched: per-joint velocity clip, position
integration, lead clamp, servoing guard and fault policy are exactly what
they were. What changed is the reference source, not any safety layer.

## Deleted

Gated on, and released by, a supervised hardware run of the new path
(`runs/2026-08-05/loop_log_20260805_183117.csv`: trajectory activated and
completed, no rejections, `faults_observed = 0`, clean `user_stop` after
41.6 s and 20,765 cycles).

Controller:

- `src/TrajectoryProfile.h` and `tests/test_trajectory_profile.cpp`
- `PoseTarget`, `PoseTargetMailbox`, `PoseTargetSource`,
  `NotifyPoseTargetSourceOnArrivalEdge`, and pose-line parsing
  (`ParsePoseTarget`) from `src/Targets.h/.cpp`
- `RunPoseTargetInputFromPipe` / `RunPoseTargetInputFromFd`; the surviving
  `RunTargetInput`/`RunTargetInputFromPipe` no longer take a pose mailbox
- `config::kUseJointTrajectorySource` (the switch), and the now-unused
  `kProfileMaxSpeedMps` / `kProfileMaxAccelerationMps2` /
  `kProfileMaxJerkMps3`
- `RotationFromRpy`, which existed for the retired pose grammar

Bridge:

- `SampleCartesianWaypoints` and both `FormatTargetLine` overloads
- `--emit-waypoints` and `--emit-orientation`
- `src/Waypoints.{h,cpp}` renamed to `src/PathValidation.{h,cpp}`, keeping
  only `ValidateJointPath` and `ValidationLimitsDeg` — the name now matches
  the job
- `tests/test_waypoints.cpp` renamed to `tests/test_path_validation.cpp`
  (ctest `waypoints` → `path_validation`)

## Also removed: world-frame target lines

The `WORLD x y z` / `BASE x y z` target grammar added earlier the same day
lived in `ParsePoseTarget` and went with it. The world-frame *kinematics*
are unaffected and remain in `DualArmKinematics`: `WorldFromBase`,
`ToolPoseInWorld`, and `PointBaseToWorld` / `PointWorldToBase`, plus
`tools/print_dual_arm_fk`. See `custom-urdf.md`.

If world frame is wanted for the pipeline again, it belongs on the planner
side — `goal.yaml`, `PlanSolver`, the bridge CLI, and the SDF grid extents
(`WorldSdf.h`'s `kGridOrigin*`, currently base_link-framed) — because that
is the path the arm actually follows now. Re-adding it to the controller's
target grammar would re-create what this record deletes.

## What the pipe accepts now

Only `TRAJ_BEGIN … TRAJ_END` blocks. Any other line, while the accumulator
is idle, is rejected to stderr with a diagnostic naming `TRAJ_BEGIN` rather
than silently ignored — an operator pasting an old `x y z` line gets told
why nothing moved, instead of watching a stationary arm.

## Tests

Both suites green: `basic_control` 10, `planner_bridge` 8.

Coverage was ported rather than dropped where the behaviour survived. The
partial-line, EOF and writer-reconnect pipe tests (the Stage 1 EOF failure
mode) still exercise `RunTargetInput`/`RunTargetInputFromPipe`, now driving
them with trajectory blocks. The two "thread survives a bad block" tests
used a following pose line as their probe; they now send a following valid
block and assert it is published, which tests the same accumulator reset
through the surviving grammar.
