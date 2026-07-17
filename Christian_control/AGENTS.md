# Christian_control subsystem instructions

These rules supplement the repository-root `AGENTS.md`. The shared C++, control
safety, verification, architecture, and robotics contracts remain mandatory.

## Scope and architecture

`basic_control/` is the actively documented single-arm Kinova Gen3 controller.
It uses the Kortex API for the arm and the shared `TrajectoryExecution/Dynamics`
implementation for Pinocchio kinematics/dynamics.

- Keep `basic_control/src/main.cpp` at the orchestration level: model and client
  construction, high-level calls, shutdown, and exit status.
- Kortex connection mechanics belong in `Connect`, state acquisition in
  `Measure`, kinematics in `Kinematics`, motion and its watchdogs in `Motion`,
  recording in `Record`, and timing diagnostics in `Timing`.
- Treat `../TrajectoryExecution/Dynamics` as a shared dependency. Do not change it
  as incidental local cleanup.
- The current executable's settings live in `basic_control/src/Config.h`. Until a
  deliberate configuration migration is requested, keep it as the single source
  rather than adding a hidden second configuration path.
- Full module ownership and the `main.cpp` contract are in
  `docs/architecture.md`; program flow and Kortex calls are in
  `docs/CONTROLLER_CODE_MAP.md`.

## Hardware facts and restrictions

- `basic_control/controller` is hardware-moving by default because
  `config::kStartupMode` is `kJoints`. Never run it as a build, sanitizer, or
  smoke test. Every hardware session still requires the explicit authorization
  and operator conditions in the root instructions.
- The only current motion path is the relative joint move in `src/Motion.cpp`.
  The previous Cartesian/reactive controller was removed after hardware faults;
  do not reintroduce it without an explicit design and safety discussion. See
  `docs/decisions/reactive-control-removal.md` and `docs/known-issues.md`.
- `kJointSpeedsDegS` is statically limited against
  `Motion.h::kDefaultSpeedLimits`, currently 45 deg/s. Do not raise this ceiling;
  the arm's configured 50 deg/s soft limit has caused mid-move faults.
- `Measure::read_feedback` is the single standalone feedback read. A cyclic
  command loop uses the feedback returned by the same `Refresh(command)` exchange
  and must not add a second state read.
- Motion exit paths must preserve the first failure, stop cleanly, and restore
  single-level servoing whenever communication remains valid.

## Loop and shutdown behavior

- Long-running loops observe the `g_stop` atomic set by SIGINT and exit through
  orderly flushing, servo-mode restoration, and RAII session teardown.
- Fixed-rate loops use `sleep_until` on a fixed grid. Terminal output remains
  low-rate; bulk telemetry belongs in files through the repository logging
  contract.

## Build and safe validation

Build from `Christian_control/basic_control` with:

```bash
cmake -S . -B build
cmake --build build
```

Building does not authorize executing `controller` or another Kortex-linked
binary. Prefer compilation, offline scripts, and hardware-free tests.

## Documentation

Keep stable module ownership in `docs/architecture.md`, empirical faults in
`docs/known-issues.md`, and durable choices in `docs/decisions/`. Update them only
when their underlying behavior or decision changes; do not append task history to
this file or `CLAUDE.md`.
