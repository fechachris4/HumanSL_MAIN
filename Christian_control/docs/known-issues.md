# Known issues and lessons learned

Technical explanations behind rules in `../AGENTS.md`. Add new entries here,
not to AGENTS.md/CLAUDE.md.

## 50 deg/s joint speed soft limit → WRONG_SERVOING_MODE

The arm's base enforces a 50 deg/s joint speed soft limit (this arm's
configuration; read it with `tools`' `./query_limits`). Streaming position
steps at or above that rate are not followed: the joint stands still,
tracking error grows, and at ~5 deg the arm faults out of low-level
servoing — the error surfaces as `WRONG_SERVOING_MODE` mid-move.

Consequence: joints-mode speed validation (`kDefaultSpeedLimits` in
`src/Motion.h`, checked against `config::kJointSpeedsDegS` via
`static_assert` in `src/Config.h`) rejects speeds above 45 deg/s (10%
margin) at compile time. Do not raise it. Note these validation limits are
deliberately separate from the URDF joint limits (see
`decisions/custom-urdf.md`).

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

## Reactive/Cartesian velocity servo — tried, then removed (2026-07-17)

A task-space controller (`src/Controller.h/cpp`, `run_reactive_control`) was
built and run on the arm: PD on pose+twist error → damped-least-squares IK →
joint velocities, integrated client-side into position setpoints and
streamed at 1 kHz. It repeatedly faulted:

- a large initial pose error (>1 m/149°, because the configured target was
  far from the arm's actual position) saturated 6-7 joints at the speed
  clamp on the very first cycle, in a direction that was no longer the
  least-squares solution (per-joint clamping does not preserve the DLS
  direction);
- one run drove joint 4 across what is very likely a *configured* position
  soft limit near −19.6° (raw 340.43°) — far inside the Gen3's ±147.8°
  factory range, so arm-specific, not a code defect;
- once parked past that point, every subsequent run faulted on the very
  first cyclic exchange, before any motion, with only the base's aggregate
  `JOINT_FAULT` bit visible (per-actuator fault detail was not being logged
  in every revision);
- more generally, this control path used low-level velocity-mode streaming,
  which Kinova's own issue trackers document real problems with (joints
  drifting under gravity at commanded-zero velocity: kortex issue #42,
  closed as "not planned"; joints sometimes not responding to velocity
  commands at all: Kinova-kortex2_Gen3_G3L issue #156).

Removed 2026-07-17 at Christian's request — see
`decisions/reactive-control-removal.md`. The only motion path now is
`move_joints_relative` (`src/Motion.cpp`): one-shot relative joint moves,
still low-level position streaming, but bounded to small, deliberate,
one-joint-at-a-time motions rather than a continuous auto-servo toward a
potentially-unreachable or limit-violating target.

The joint-4 limit question itself is still open: check the arm's web
dashboard (`http://192.168.1.10`) for that actuator's active safety state
and configured position limits before relying on this arm again — it will
reject that joint's motion under `kJoints` mode too, since the arm's own
firmware enforces the limit regardless of which controller drives it.

## FK cross-check offset

Our Pinocchio forward kinematics agrees with the pose the robot reports to
~2 cm once the gripper TCP offset is added (see the startup FK cross-check
in `src/Kinematics.cpp`).
