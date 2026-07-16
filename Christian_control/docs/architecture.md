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
- `motion.txt` — optional runtime motion config at the basic_control root
  (found via the executable-parent search: cwd → build/ → basic_control/)

## Module ownership

| File | Owns |
|---|---|
| `src/main.cpp` | the story: wiring, main loop call, shutdown, exit code |
| `src/Config.h` | central runtime configuration: robot IP, recording/log filenames, simple-hold gain and 0.1° command-lead clamp, plus legacy-controller target/gains/clamps |
| `src/Connect.{h,cpp}` | Kortex sessions (TCP 10000 + real-time UDP 10001), RAII |
| `src/Measure.{h,cpp}` | reading sensors; deg→Pinocchio-config conversion |
| `src/Kinematics.{h,cpp}` | forward kinematics (Pinocchio), FK-vs-robot check |
| `src/controllers/simple_joint_position_hold/` | active baseline: pure `target - measured` proportional step plus 1 kHz measured-start position-hold loop; position setpoints only; per-cycle simple trace; MOVES THE ARM |
| `src/controllers/legacy_advanced/` | preserved position-only Cartesian controller, selected only by `mode: legacy_advanced`; FK/Jacobian → PD/DLS → integrated position setpoints; MOVES THE ARM |
| `src/Record.{h,cpp}`, `src/SimpleHoldRecord.cpp` | all CSV formatting: recording, joint moves, simple hold, and legacy advanced traces; timestamped filenames and post-move plotting |
| `src/Motion.{h,cpp}` | low-level cyclic position exchange, servoing-mode RAII, joint-delta moves, and `motion.txt` modes (`joints`, `simple_joint_position_hold`, `legacy_advanced`) |
| `src/Timing.{h,cpp}` | latency benchmarks (feedback, control cycle, compute) |
| `scripts/plot_move.py` | offline analysis of a move log (default: newest `move_log_*.csv`): tracking/overshoot/dt stats, matplotlib plots (matplotlib in user site-packages) |
| `scripts/plot_joint.py` | offline single-joint plot from a move log (default: newest `move_log_*.csv`; pandas + matplotlib): commanded vs measured over time, joint chosen 1-7, PNG + optional `--show` window |
| `tools/query_limits.cpp` | separate READ-ONLY executable `query_limits`: prints robot kinematic hard/soft limits via ControlConfig; ignores motion.txt |
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
