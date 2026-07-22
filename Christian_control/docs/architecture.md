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
    Cartesian controller; `Target` — operator input)
  - `src/loop/` — the Runner: cycle order, timing, clamping, logging —
    MOVES THE ARM; no control math
  - `src/actuation/` — actuation strategy (`Actuation` interface —
    Prepare/Restore may do hardware I/O, Apply is pure;
    `PositionIntegration` — the q_command integrator)
  - `src/safety/` — safety policy and reporting (`Supervisor` — stop
    classification + readiness gate; `FaultReport` — bank decoding, stop /
    fault-change reports)
  - `src/math/` — mathematics and models (`Kinematics`)
  - `src/hardware/` — external boundaries (`Connect`, `Measure`, `Record`)
- `tools/` — standalone diagnostic executables
- `tests/` — hardware-free tests (CTest)
- `scripts/` — offline Python analysis (not part of the build)
- `config/` — our copy of the arm's URDF (`GEN3_custom.urdf`, see
  `decisions/custom-urdf.md`)

## Module ownership

| File | Owns |
|---|---|
| `src/app/main.cpp` | the story: model+config (asserts nv == 7), connect, readiness check, state printout (joints + FK end-effector position), input thread, loop call, log flush, exit code |
| `src/app/Config.h` | central runtime configuration: robot IP (default 192.168.1.10), Kortex session login/timeouts, `kControlDtS` (0.01 s — the single timing source; period/frequency derived), controller gains (`kKpCartesian`, `kDlsLambda`), `kQdotLimitDegS` (derived at compile time: `kQdotLimitSafetyFactor` 0.9 × `kModelVelocityLimitsDegS` ≈ 71.6/62.9 deg/s — the one place speed limits are set, `qdot-limit-raise.md`), `kFollowingErrorLimitDeg` (3 deg guard), controlled frame, log capacity, `kModelVelocityLimitsDegS` (authoritative deg/s values behind the URDF limits) — named constants, edit and rebuild |
| `src/hardware/Connect.{h,cpp}` | Kortex sessions (TCP 10000 + real-time UDP 10001), exception-safe RAII per channel: fails fast if the arm is unreachable, closes whatever opened on any exit path, never throws from a destructor |
| `src/hardware/Measure.{h,cpp}` | reading sensors: `read_feedback` — the single standalone robot state reader, the only `RefreshFeedback` call in the program (the loop instead reuses `Refresh(command)`'s reply) |
| `src/hardware/Record.{h,cpp}` | telemetry: `LoopLogSample`, `LoopLog` (preallocated ring buffer; `push` is alloc/I-O-free and loop-safe), CSV output after the loop, timestamped filenames |
| `src/math/Kinematics.{h,cpp}` | forward kinematics (Pinocchio) and `position_and_jacobian` (frame position + 3×7 translational Jacobian from the SAME q, `LOCAL_WORLD_ALIGNED`; preallocated `KinematicsWorkspace`) |
| `src/safety/Supervisor.{h,cpp}` | safety policy: `LoopStop`/`LoopResult`, `ClassifyStop` (following error FIRST, then live faults, then arm state), `RobotReadyForTakeover` (pre-takeover gate) |
| `src/safety/FaultReport.{h,cpp}` | fault-bank decoding (base + actuator, named bits) and the human-readable stop / fault-change reports — printing only, no policy |
| `src/math/Dls.h` | damped least squares, header-only Eigen: `qdot = Jpᵀ(JpJpᵀ+λ²I)⁻¹v` via LDLT, no explicit inverse, no allocation; plus `ClampedCycleDt` (integration dt ≤ 2×nominal) — hardware-free-tested |
| `src/JointVector.h` | `JointVector` = `std::array<double,7>`, Kortex actuator order — the joint-space value type used across layers |
| `src/hardware/Cyclic.{h,cpp}` | `CyclicSession`: owns the BaseCyclic command frame — `Seed()` (the standalone read + actuator-slot init, AFTER the mode switch) and `Send()` (wrap to [0,360), stamp frame/command ids, one `Refresh` per cycle) |
| `src/safety/ServoingGuard.{h,cpp}` | RAII servoing-mode ownership — the ONLY caller of `SetServoingMode`: ctor enters LOW_LEVEL, dtor restores SINGLE_LEVEL by unwinding on every exit path (guarded, warn-don't-throw) |
| `src/control/Controller.h` | `RobotState` (q, q̇, t — Eigen only) with the hard rule: a field belongs here only if fillable every cycle from arm feedback alone; external sensors use store injection |
| `src/actuation/Actuation.h`, `PositionIntegration.{h,cpp}` | actuation strategy: `Prepare` (seed, may do I/O) / `Apply` (pure: clamped q̇ → setpoints) / `TrackingErrorDeg` (the guard signal) / `Restore` (teardown, may do I/O — F5 asymmetry); PositionIntegration owns the q_command integrator |
| `src/control/Target.{h,cpp}` | operator desired end-effector position: parse (3 finite numbers, meters, base frame — deliberately no reachability check), `TargetStore` (mutex-protected latest + sequence), stdin input thread (polls, so shutdown can interrupt it) |
| `src/control/Controller.h` | the controller interface: `RobotState` (arm-feedback-only hard rule; external sensors via store injection), `ControllerStatus` (telemetry/UX data — controllers do no I/O), `Controller` (`Reset` at T5, `DesiredVelocity` per cycle → q̇ rad/s BEFORE clamping; pure computation) |
| `src/control/ResolvedRate.{h,cpp}` | the Cartesian resolved-rate control law: frame check at construction, `Reset` seeds p_desired = p(q), `DesiredVelocity` = FK+Jacobian from the SAME q → Kp error → DLS; arrival notice as edge-triggered status data |
| `src/loop/Runner.{h,cpp}` | THE loop — MOVES THE ARM, controller-agnostic: numbered takeover T1-T6 / teardown D1-D3 sequence (spec in Runner.h), per-cycle order (dt → RobotState → controller → clamp → `Actuation::Apply` → `CyclicSession::Send` → sample → fault-edge prints → `ClassifyStop` → log → sleep_until grid), no printing/allocation beyond the documented edge-triggered exceptions |
| `tests/test_control_logic.cpp` | hardware-free CTest coverage (runs everywhere): `DampedLeastSquares` (incl. singular case), `ClampedCycleDt`, target parsing, `TargetStore`, `PositionIntegration` (seed/integrate/tracking-error wrap) |
| `tests/test_supervisor.cpp`, `tests/test_resolved_rate.cpp` | hardware-machine CTest battery (bundled libs are Linux ELF): `ClassifyStop` ordering/priorities; `ResolvedRate` vs closed-form DLS on the real URDF, arrival-edge semantics |
| `tests/replay_controller.cpp` | the decision-14 replay harness (Linux): feeds a baseline CSV through ResolvedRate → clamp → PositionIntegration and reports per-joint agreement vs the recorded commands |
| `scripts/plot_move.py`, `scripts/plot_joint.py` | offline analysis of loop/move logs |
| `tools/query_limits.cpp` | separate READ-ONLY executable `query_limits`: prints robot kinematic hard/soft limits via ControlConfig |
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
