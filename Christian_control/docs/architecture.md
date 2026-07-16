# Architecture — basic_control

Detail behind the summary in `../AGENTS.md`. Update this table whenever a
file is added, removed, or repurposed.

## Layout

One topic per file pair; `main.cpp` coordinates, modules implement.
Inside `basic_control/`:

- `src/` — application code (headers next to their .cpp — nothing external
  consumes them)
- `tools/` — standalone read-only diagnostic executables
- `scripts/` — offline Python analysis (not part of the build)
- `config/` — our copy of the arm's URDF (`GEN3_custom.urdf`, see
  `decisions/custom-urdf.md`)

## Module ownership

| File | Owns |
|---|---|
| `src/main.cpp` | the story: wiring, main loop call, shutdown, exit code |
| `src/Config.h` | central runtime configuration: robot IP (default 192.168.1.10), **startup mode** (`kStartupMode`: `kReactive` default/`kJoints`/`kRecord`) and joints-mode move parameters (`kJointDeltasDeg`/`kJointSpeedsDegS`, speed-checked against `Motion.h`'s `kDefaultSpeedLimits` via `static_assert`), recording rate, output filenames, reactive-controller target pose (position + fixed-axis rpy in degrees, base frame) and gains/clamps — named constants, edit and rebuild |
| `src/Connect.{h,cpp}` | Kortex sessions (TCP 10000 + real-time UDP 10001), RAII |
| `src/Measure.{h,cpp}` | reading sensors; deg→Pinocchio-config conversion |
| `src/Kinematics.{h,cpp}` | forward kinematics (Pinocchio), FK-vs-robot check |
| `src/Controller.{h,cpp}` | task-space control: read-only pose-error report (startup), and `run_reactive_control` — 1 kHz reactive servo to the Config.h target pose (PD on pose+twist error → damped-least-squares IK → optional null-space centering on joints 2/4/6 (`kUseNullspaceCentering`, default off) → speed clip at `kReactiveSpeedLimitDegS` → integrated setpoints with `kCtrlLeadRad` anti-windup lead clamp), runs until Ctrl+C, freezes at measured on every stop path; per-cycle `ControlTraceSample` CSV; MOVES THE ARM in reactive mode |
| `src/Record.{h,cpp}` | fixed-rate logging loop, all CSV formatting (recording + move log `MoveLogSample` + reactive trace `ControlTraceSample`), timestamped log filenames, post-move plot runner (`plot_move_log` → `scripts/plot_move.py`), heartbeat |
| `src/Motion.{h,cpp}` | low-level cyclic motion, all 7 joints (`JointVector` = `std::array<double,7>`); exports `ServoingModeGuard` (RAII servoing mode), `send_positions` (one cyclic exchange), `kDefaultSpeedLimits`, and `speeds_within_limits` (compile-time check used by `Config.h`'s `static_assert`) for reuse by Controller/Config; success requires every moving joint to MEASURE within `kReachedToleranceDeg` of target after a 1 s settle hold; aborts (per-joint report) on tracking failure, robot fault, or Kortex error; per-cycle move logging (commanded/measured/error, velocity, torque, current, fault/warning banks → `move_log_<timestamp>.csv`); MOVES THE ARM |
| `src/Timing.{h,cpp}` | latency benchmarks (feedback, control cycle, compute) |
| `scripts/plot_move.py` | offline analysis of a move log (default: newest `move_log_*.csv`): tracking/overshoot/dt stats, matplotlib plots (matplotlib in user site-packages) |
| `scripts/plot_joint.py` | offline single-joint plot from a move log (default: newest `move_log_*.csv`; pandas + matplotlib): commanded vs measured over time, joint chosen 1-7, PNG + optional `--show` window |
| `tools/query_limits.cpp` | separate READ-ONLY executable `query_limits`: prints robot kinematic hard/soft limits via ControlConfig; ignores `config::kStartupMode` |
| `Dynamics` (external) | reused from `../TrajectoryExecution` — do not edit |

## main.cpp contract

`main.cpp` must read like high-level pseudocode and coordinate existing
components rather than contain their detailed implementations.

main.cpp MAY:

- connect to the robot
- construct required clients
- configure the sampling rate
- initialise timing and storage
- call high-level helper functions
- run the main loop
- perform orderly shutdown
- handle top-level exceptions
- return an appropriate exit code

main.cpp must NOT:

- contain detailed Kortex connection implementation
- contain forward-kinematics mathematics
- contain CSV formatting logic
- contain large logging implementations
- reconnect inside the loop
- create duplicate clients
- duplicate existing helper functions
- print robot state every iteration
- send movement commands unless explicitly requested
