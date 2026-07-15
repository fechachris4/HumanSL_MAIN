# Agent rules for Christian_control

Durable, repository-wide rules only. Details live in `docs/` — link there
instead of expanding this file (see "Keeping these files short").

## Project purpose

Controls and monitors a Kinova Gen3 7-DoF robot in C++: Kortex API to talk
to the arm, Pinocchio (URDF model) for kinematics/dynamics. Active code is
`basic_control/`.

## Architecture

One topic per file pair; `main.cpp` coordinates, modules implement.
Full module-ownership table and layout: `docs/architecture.md`.

- `main.cpp` must read like high-level pseudocode: wiring, main-loop call,
  shutdown, exit code — no Kortex/FK/CSV implementation detail, no duplicate
  clients, no reconnecting inside the loop. Full may/must-not contract:
  `docs/architecture.md`.
- Code migrates out of main the second time it is needed; mechanics live in
  modules, policy (rates, filenames, IPs) lives in `src/Config.h`.
- `Dynamics` is reused from `../TrajectoryExecution` — do not edit it.

## Safety

- Read-only by default: never add code that moves the arm unless explicitly
  requested. When movement is requested, low-level (BaseCyclic) control is
  the project's chosen route, and safety notes go in README.md.
- NEVER run `./controller` as a "test" while a valid motion.txt is
  discoverable — it is a robot-moving command. Test config changes with
  deliberately invalid files, or add/use a dry-run mechanism.
- Read-only changes may be tested on the arm freely; anything that moves the
  arm needs the user present with the e-stop.
- motion.txt speed validation is capped at 45 deg/s — do not raise it (the
  arm faults above its 50 deg/s soft limit; see `docs/known-issues.md`).

## Coding rules

- No command-line flags or positional arguments for runtime configuration:
  fixed settings in `src/Config.h`, motion parameters in `motion.txt`.
- Loops that run until stopped check the `g_stop` atomic flag (set by
  SIGINT) and exit cleanly (flush files, restore servoing mode, let RAII
  close sessions).
- Fixed-rate loops pace with `sleep_until` on a fixed grid, never `sleep_for`.
- Terminal output stays quiet: heartbeats at ~1 Hz; bulk data goes to files.
- The user is learning C++: keep code simple, explain idioms in comments
  where they carry intent (why, not what).

## Scope discipline

- Don't add features, refactor, or introduce abstractions beyond what the
  task requires. A bug fix doesn't need surrounding cleanup, and a one-shot
  operation usually doesn't need a helper.
- Don't design for hypothetical future requirements: do the simplest thing
  that works well. Avoid premature abstraction and half-finished
  implementations.
- Don't add error handling, fallbacks, or validation for scenarios that
  cannot happen. Trust internal code and framework guarantees; only
  validate at system boundaries (user input, external APIs).
- Don't use feature flags or backwards-compatibility shims when you can
  just change the code.

## Build

`cd basic_control && mkdir -p build && cd build && cmake .. && make` →
executable `controller`. Links the bundled Kortex API and Pinocchio from
`../../third_party` (runtime .so notes: `docs/known-issues.md`).

## Keeping these files short

Do not append every change, lesson, or debugging note to AGENTS.md or
CLAUDE.md. Only add rules here that are durable and affect most future
tasks. Everything else goes elsewhere:

- technical explanations and module details → `docs/`
- important architectural decisions → `docs/decisions/`
- temporary or historical notes → `docs/archive/`, or delete them when no
  longer useful

## Maintenance duty

After every change: keep `docs/architecture.md`'s table, CLAUDE.md, and
README.md consistent with the code, and verify the build (`cmake .. && make`).
