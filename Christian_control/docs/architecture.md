# Architecture — basic_control

Module ownership for `basic_control`. Update this table whenever a
file is added, removed, or repurposed.

## Layout

One topic per file pair; `main.cpp` coordinates, modules implement.
Inside `basic_control/`:

- `src/` — application code, one subfolder per technical layer (the layers
  defined in the root `docs/architecture.md`); headers stay next to their
  .cpp — nothing external consumes them. `src/` is the include root, so
  local includes are layer-qualified (`#include "hardware/Connect.h"`):
  - `src/app/` — application orchestration (`main.cpp`, `Config.h`)
  - `src/control/` — control laws (`Controller.h` — the interface +
    `RobotState`, arm-feedback-only by hard rule; `ResolvedRate` — the
    position-only Cartesian controller; `ReactiveLaw`/`ReactivePose` — the
    6-DoF pose controller ported from the simulation;
    `Target` — operator input)
  - `src/loop/` — the Runner: cycle order, timing, clamping, logging —
    MOVES THE ARM; no control math
  - `src/actuation/` — actuation strategy (`Actuation` interface —
    Prepare/Restore may do hardware I/O, Apply is pure;
    `PositionIntegration` — the q_command integrator)
  - `src/safety/` — safety policy and reporting (`Supervisor` — stop
    classification + readiness gate; `FaultReport` — bank decoding, stop /
    fault-change reports)
  - `src/math/` — mathematics and models (`Kinematics`,
    `DualArmKinematics`)
  - `src/hardware/` — external boundaries (`Connect`, `Measure`, `Record`)
- `tests/` — hardware-free tests (CTest)
- `scripts/` — offline Python analysis (not part of the build)
- `config/` — the mounted 14-joint runtime model
  (`GEN3_dual_mounted.urdf`) and runtime configuration; see
  `decisions/custom-urdf.md`

## Module ownership

| File | Owns |
|---|---|
| `src/app/main.cpp` | the story: load and validate the mounted dual model plus right-arm adapter, connect only to the right arm, readiness check, state printout (right joints + right-base-frame FK), input thread, loop call, log flush, exit code |
| `src/app/Options.{h,cpp}` | runtime configuration front-end: CLI > TOML (explicit `--config` path, or the compiled default file `config/control.toml` when it exists — fixed absolute path, never cwd-dependent) > compiled defaults; law selection, gains/thresholds/target_file, never arm ownership or fault-stop policy (`kStopOnFault` compile-time only); unknown keys/flags are hard errors; effective-config echo + CSV `#` preamble (`decisions/runtime-config.md`) |
| `src/app/Config.h` | compiled right-only hardware ownership, right base/end-effector frame names, fixed left nominal state, Kortex session settings, unchanged timing/gains/command limits/safety thresholds, and base-frame reach telemetry |
| `src/hardware/Connect.{h,cpp}` | Kortex sessions (TCP 10000 + real-time UDP 10001), exception-safe RAII per channel: fails fast if the arm is unreachable, closes whatever opened on any exit path, never throws from a destructor |
| `src/hardware/Measure.{h,cpp}` | reading sensors: `read_feedback` — the single standalone robot state reader, the only `RefreshFeedback` call in the program (the loop instead reuses `Refresh(command)`'s reply) |
| `src/hardware/Record.{h,cpp}` | telemetry: `LoopLogSample`, `LoopLog` (preallocated ring buffer; `push` is alloc/I-O-free and loop-safe), CSV output after the loop, timestamped filenames |
| `src/math/Kinematics.{h,cpp}` | generic Pinocchio FK and model-root-frame Jacobian primitives; owns the dynamic 6×nv preallocated workspace |
| `src/math/DualArmKinematics.{h,cpp}` | validates the 14-joint mounted model, assembles full q from measured right + nominal left joints by name, computes full FK/Jacobian, transforms the right tool pose and Jacobian into `base_link`, and selects the seven right columns in Kortex order |
| `src/safety/Supervisor.{h,cpp}` | safety policy: `LoopStop`/`LoopResult`, `ClassifyStop` (following error FIRST, then live faults, then arm state), `RobotReadyForTakeover` (pre-takeover gate) |
| `src/safety/FaultReport.{h,cpp}` | fault-bank decoding (base + actuator, named bits) and the human-readable stop / fault-change reports — printing only, no policy |
| `src/math/Dls.h` | damped least squares, header-only Eigen: `qdot = Jpᵀ(JpJpᵀ+λ²I)⁻¹v` via LDLT, no explicit inverse, no allocation; plus `ClampedCycleDt` (integration dt ≤ 2×nominal) — hardware-free-tested |
| `src/JointVector.h` | `JointVector` = `std::array<double,7>`, Kortex actuator order — the joint-space value type used across layers |
| `src/hardware/Cyclic.{h,cpp}` | `CyclicSession`: owns the BaseCyclic command frame — `Seed()` (the standalone read + actuator-slot init, AFTER the mode switch) and `Send()` (wrap to [0,360), stamp frame/command ids, one `Refresh` per cycle) |
| `src/safety/ServoingGuard.{h,cpp}` | RAII servoing-mode ownership — the ONLY caller of `SetServoingMode`: ctor enters LOW_LEVEL, dtor restores SINGLE_LEVEL by unwinding on every exit path (guarded, warn-don't-throw) |
| `src/control/Controller.h` | `RobotState` (q, q̇, t — Eigen only) with the hard rule: a field belongs here only if fillable every cycle from arm feedback alone; external sensors use store injection |
| `src/actuation/Actuation.h`, `PositionIntegration.{h,cpp}` | actuation strategy: `Prepare` (seed, may do I/O) / `Apply` (pure: clamped q̇ → setpoints) / `TrackingErrorDeg` (the guard signal) / `Restore` (teardown, may do I/O — F5 asymmetry); PositionIntegration owns the q_command integrator |
| `src/control/Target.{h,cpp}` | operator desired right end-effector targets in the right-arm `base_link` frame: position parse (3 finite numbers; deliberately no reachability check) + pose parse (3 or 6 numbers; RPY radians, Rz·Ry·Rx), stores, stdin input, and watched target file |
| `src/control/Controller.h` | the controller interface: `RobotState` (arm-feedback-only hard rule; external sensors via store injection), `ControllerStatus` (telemetry/UX data — controllers do no I/O), `Controller` (`Reset` at T5, `DesiredVelocity` per cycle → q̇ rad/s BEFORE clamping; pure computation) |
| `src/control/ResolvedRate.{h,cpp}` | the Cartesian resolved-rate control law: frame check at construction, `Reset` seeds p_desired = p(q), `DesiredVelocity` = FK+Jacobian from the SAME q → Kp error → DLS; arrival notice as edge-triggered status data |
| `src/control/ReactiveLaw.h` | the reactive 6-DoF pose law's equations, header-only pure Eigen (ported from the simulation, `decisions/reactive-pose-port.md`): rotation log, PD task twist with per-term enable flags, 6-DoF DLS, damped null-space centering with [0,360)-wrap fix and continuous-joint mask — hardware-free-tested incl. cross-validation fixtures from the Python sim law |
| `src/control/ReactivePose.{h,cpp}` | the reactive pose controller: binds ReactiveLaw to the Controller interface — frame check at construction, `Reset` seeds the pose target (position AND orientation) from FK, `DesiredVelocity` = pose+6×7 Jacobian from the SAME q → pose/twist error → law; fills σ_min (full J) and rot_error_rad status |
| `src/loop/Runner.{h,cpp}` | THE loop — MOVES THE ARM, controller-agnostic: numbered takeover T1-T6 / teardown D1-D3 sequence (spec in Runner.h), per-cycle order (dt → RobotState → controller → clamp → `Actuation::Apply` → `CyclicSession::Send` → sample → fault-edge prints → `ClassifyStop` → log → sleep_until grid), no printing/allocation beyond the documented edge-triggered exceptions |
| `tests/test_control_logic.cpp` | hardware-free CTest coverage (runs everywhere): `DampedLeastSquares` (incl. singular case), `ClampedCycleDt`, target parsing, `TargetStore`, `PositionIntegration` (seed/integrate/tracking-error wrap) |
| `tests/test_reactive_law.cpp`, `tests/reactive_fixtures.h` | hardware-free CTest coverage (runs everywhere) for the reactive pose law: rotation log vs pin.log3, 6-DoF DLS closed form, null-space wrap/mask/annihilation, pose parsing/store — plus cross-validation fixtures generated from the Python simulation law by `scripts/gen_reactive_fixtures.py` (checked in; regenerate only when the sim law changes) |
| `tests/test_supervisor.cpp`, `tests/test_resolved_rate.cpp`, `tests/test_dual_arm_model.cpp` | Linux hardware-free CTest battery (bundled libs are Linux ELF): stop ordering; controller law; 14-joint load, mount geometry, left nominal state, and exact full-to-right Jacobian selection |
| `tests/replay_controller.cpp` | the decision-14 replay harness (Linux): feeds a baseline CSV through ResolvedRate → clamp → PositionIntegration and reports per-joint agreement vs the recorded commands |
| `scripts/plot_move.py`, `scripts/plot_joint.py` | offline analysis of loop/move logs |
| `Dynamics` (external) | reused from `../TrajectoryExecution` — do not edit |

## main.cpp contract

`main.cpp` must read like high-level pseudocode and coordinate existing
components rather than contain their detailed implementations.

main.cpp MAY:

- connect to the robot
- construct required clients
- construct the log buffer and target store, start/join the input thread
- call high-level helper functions (readiness check, the loop)
- write the finished log to disk
- perform orderly shutdown
- handle top-level exceptions
- return an appropriate exit code

main.cpp must NOT:

- contain detailed Kortex connection implementation
- contain the control loop, stepping math, or fault policy
- contain forward-kinematics mathematics
- contain CSV formatting logic
- reconnect
- create duplicate clients
- duplicate existing helper functions
- print robot state every iteration
