# Reduce controller to a one-shot connect–read–hold–restore sequence

Date: 2026-07-20
Status: accepted

## Decision

Replace the controller's move executor with a fixed 11-step program: load
configuration and URDF model, connect (TCP + UDP), enter LOW_LEVEL_SERVOING,
read one feedback frame, extract all 7 joint positions and velocities from
that frame, send one holding command equal to the measured positions, print,
restore SINGLE_LEVEL_SERVOING on every exit path, and exit via RAII teardown.

## Why

The control loop is being rebuilt from a minimal, verified base (single
reader, no hidden stages). The smallest useful hardware program is the
takeover-and-release handshake itself: it exercises the mode switch, the
single-frame read, the command path, and the restore — with zero requested
displacement — before any loop is layered on top.

## What was removed, and where it lives

Checkpoint commit c47525b3 contains everything removed:

- `move_joints_relative` and its machinery (Ramp, probe stage, hold-accept
  stage, tracking watchdog, fault decoding/abort policy, settle window);
- the `Record` module (100 Hz recorder, per-cycle move log CSV, timestamped
  filenames, plot runner) — `scripts/plot_move.py` stays for historical logs;
- the `Timing` module (latency benchmarks);
- `Config.h`'s `StartupMode`, move parameters (`kJointDeltasDeg`,
  `kJointSpeedsDegS` + static speed-limit assert), and recording settings;
- Measure's thin wrappers `measure_joint_angles` / `measure_joint_velocities`.

Kept compiled though currently uncalled: `Kinematics` (FK + FK-vs-robot
check) and `Measure::measure_configuration` — the Pinocchio bridge the next
loop will need.

## Safety notes carried forward

- The read must follow the mode switch (its round trip lets the base finish
  entering LOW_LEVEL_SERVOING; commanding earlier fails with
  WRONG_SERVOING_MODE).
- The restore of SINGLE_LEVEL_SERVOING is guarded (warn, don't throw) and
  runs on success and failure paths alike, before the error report, so the
  first failure reason is preserved.
- The 45 deg/s command-speed ceiling (10% under the base's enforced 50 deg/s
  soft limit) remains binding for any future motion path. (Raised 2026-07-22
  with the arm-reconfiguration precondition, and consolidated into the
  single derived clip in `src/app/Config.h` — `qdot-limit-raise.md`.)
