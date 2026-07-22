# Single cyclic position loop with an operator target thread

Date: 2026-07-20
Status: accepted (supersedes the "no loop" state of `one-shot-reduction.md`)

## Decision

Rebuild the controller as one cyclic position loop (`src/control/Loop.cpp`)
on top of the one-shot base:

- **Single reader**: one `RefreshFeedback` only, to seed after the servoing
  mode switch. Every subsequent cycle reuses the feedback returned by
  `Refresh(command)` — one network exchange per cycle, matching Kinova's
  cyclic example (the repository's one-exchange-per-tick contract, then in
  `docs/robotics-contracts.md`, since removed).
- **Bumpless takeover**: commanded = target = measured at the seed; the
  first frame is an unchanged holding command.
- **Targets from stdin**: a separate input thread (`src/control/Target.cpp`)
  parses lines of 7 absolute joint degrees, validates them (finite; limited
  joints inside the URDF position limits after normalizing to [-180, 180)),
  and stores the latest valid target in a mutex-protected `TargetStore`.
  The loop snapshots it each cycle. The thread polls stdin with a 100 ms
  timeout so shutdown can interrupt it (a blocking `getline` cannot be
  interrupted portably).
- **Bounded stepping**: each cycle every commanded angle moves at most
  `kMaxCommandSpeedDegS × period` toward the target (`MoveTowards`, exact
  landing, no overshoot). Continuous joints (1/3/5/7) step along the
  shortest wrap direction; limited joints (2/4/6) use the direct signed
  difference so the path cannot cross the forbidden arc.
- **No printing/allocation/file I/O in the loop**: samples go to a
  preallocated ring buffer (`LoopLog`, `src/hardware/Record.cpp`, capacity
  `kLogCapacitySeconds`), written to one timestamped CSV after the loop.
  Fault reports are recorded as a stop reason and printed after the loop.
- **Stop policy** (unchanged from the previous executor): live actuator
  fault, base fault other than the latched JOINT_FAULT summary bit, or the
  arm leaving LOW_LEVEL_SERVOING stops the loop; JOINT_FAULT alone is a
  post-loop note, never cleared by the program. Exchange exceptions are the
  communication stop reason. The first failure is preserved; the servoing
  restore (guarded, warn-don't-throw) runs on every exit path.
- **Readiness gate**: before the takeover, main reads one standalone frame
  and refuses to start on any live fault (`RobotReadyForTakeover`).
- Exit code 0 only on a clean operator stop (Ctrl+C).

## Why

The previous executor mixed ramp staging, watchdogs, and logging into one
move function. This design separates: pure stepping math (`MoveTowards`),
operator input (`Target`), telemetry (`Record`), primitives (`Motion`), and
the loop (`Loop`) — with the single-reader and one-exchange-per-tick
contracts enforced structurally rather than by discipline.

## Safety numbers

- `kMaxCommandSpeedDegS` (Config.h, 10 deg/s) is statically checked against
  `kCommandSpeedCeilingDegS` (then in Motion.h, 45 deg/s = 10% under the
  base's enforced 50 deg/s soft limit; the ceiling was later merged into
  the single derived clip in Config.h — `qdot-limit-raise.md`).
- Target position limits (Config.h) come from `config/GEN3_custom.urdf`,
  rounded down: joint 2 ±138.0°, joint 4 ±152.4°, joint 6 ±127.7°.

## Not carried over (yet)

The old tracking-error watchdog (3° for 50 cycles → freeze at measured) and
the arrival/settle check were part of the one-shot *move* semantics; this
loop has no notion of "move complete". If tracking supervision is wanted
again, add it as an explicit loop policy — the fault-bank and servoing-state
checks alone do not detect a silently-not-following joint below the arm's
~5° internal limit.

Hardware-free tests: `tests/test_control_logic.cpp` (CTest target
`control_logic`) covers `MoveTowards`, normalization, target validation, and
`TargetStore` semantics.
