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
  - `src/control/` — control (`Motion` primitives, `Loop` — the cyclic
    controller, moves the arm; `Target` — operator input)
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
| `src/control/Motion.{h,cpp}` | command primitives, all 7 joints (`JointVector` = `std::array<double,7>`): `send_positions` (position frame — wrap to [0,360), stamp frame/command ids, `Refresh`), `enter_low_level_servoing` / `restore_single_level_servoing` (guarded restore) |
| `src/control/Target.{h,cpp}` | operator desired end-effector position: parse (3 finite numbers, meters, base frame — deliberately no reachability check), `TargetStore` (mutex-protected latest + sequence), stdin input thread (polls, so shutdown can interrupt it) |
| `src/control/Loop.{h,cpp}` | THE controller — MOVES THE ARM: resolved-rate loop (`RunResolvedRateLoop`, 100 Hz, actuators in default POSITION mode): takeover seeds q_command = q_measured (ONLY at startup) + holding frame; per cycle FK+Jacobian from the same q_measured → Kp error → DLS → per-joint clamp (`kQdotLimitDegS`) → q_command += q̇·dt (measured dt, clamped) → position frame, one `Refresh(command)` exchange reusing its feedback, stop policy via `safety/Supervisor`, no printing/allocation in the loop, guarded SINGLE_LEVEL restore on every exit path |
| `tests/test_control_logic.cpp` | hardware-free CTest coverage: `DampedLeastSquares` (incl. singular case), `ClampedCycleDt`, target parsing, `TargetStore` |
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
