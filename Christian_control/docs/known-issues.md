# Known issues and lessons learned

Technical explanations behind rules in `../AGENTS.md`. Add new entries here,
not to AGENTS.md/CLAUDE.md.

## 50 deg/s joint speed soft limit → WRONG_SERVOING_MODE

The arm's base enforces a 50 deg/s joint speed soft limit (this arm's
configuration; read it with `tools`' `./query_limits`). Streaming position
steps at or above that rate are not followed: the joint stands still,
tracking error grows, and at ~5 deg the arm faults out of low-level
servoing — the error surfaces as `WRONG_SERVOING_MODE` mid-move.

Consequence: motion.txt speed validation (`kDefaultSpeedLimits` in
`src/Motion.h`) rejects speeds above 45 deg/s (10% margin). Do not raise it.
Note these validation limits are deliberately separate from the URDF joint
limits (see `decisions/custom-urdf.md`).

## Silent mid-move stall reported as success (2026-07-14)

A −80° joint-1 move at 20 deg/s stopped after −33.3° (~1.7 s in) yet
printed "Move finished.": the robot stopped following the stream — while
still answering every cyclic `Refresh` without an error — and the old
completion check only tested whether OUR commanded setpoints had reached
the target, never the measured position. The per-run log had also been
overwritten by later runs (single fixed `move_log.csv`), so the robot-side
trigger could not be recovered.

Consequences (all in `src/Motion.cpp` / `src/Record.{h,cpp}`):

- success now requires every moving joint to MEASURE within
  `kReachedToleranceDeg` (0.5°) of its target after the settle hold;
- a tracking watchdog aborts when |commanded − measured| exceeds
  `kTrackingErrorLimitDeg` (3°) for `kTrackingErrorCycleLimit` (50)
  consecutive cycles — below the ~5° robot-side kick-out, above the ~2°
  start-up transient;
- base/actuator fault banks abort the move; Kortex call failures are
  caught and reported instead of crashing out;
- move logs are per-run (`move_log_<timestamp>.csv`) and carry velocity,
  torque, current, fault/warning banks and arm state, so the next stall
  can be attributed from the file.

Pinocchio's runtime needed `libcoal.so.3.0.1` plus boost, octomap,
urdfdom-model, and tinyxml shared libraries that were not shipped with the
bundled build. Fix: copy the .so files out of the corresponding cmeel wheels
(PyPI) into `HumanSL_MAIN/third_party/lib`. If the controller fails to start
with "cannot open shared object file", add the missing library the same way.

## FK cross-check offset

Our Pinocchio forward kinematics agrees with the pose the robot reports to
~2 cm once the gripper TCP offset is added (see the startup FK cross-check
in `src/Kinematics.cpp`).
