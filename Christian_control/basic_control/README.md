# basic_control

My controller for the **right Kinova Gen3 7-DoF arm**, using the complete
mounted dual-arm model for kinematics. It
combines:

- the bundled Kinova **Kortex API** (`../../third_party/kortex_api`) to talk
  to the arm, and
- **Pinocchio** (via the `Dynamics` class reused from `TrajectoryExecution`)
  for kinematics from the arm's URDF.

It does **not** use the HumanSL planning stack (GTSAM / GPMP2 / Vicon).

The program has ONE controller and one **reference source** (architecture
diagram at the top of `src/Controller.h`). The source says WHERE the
end-effector should be each cycle; the `TrackingController` says HOW to
move toward it. It takes over the arm in low-level servoing (actuators in
their default POSITION mode) and starts at the compiled fixed position
target (`kFixedTargetM` in `Config.h`). It then accepts queued stdin
positions. New research inputs (Vicon, a Python bridge) are new sources,
never new controllers.

Pose references run the reactive law — full 6-DoF pose, ported
from the simulation (msc_project) and cross-validated against it
(`../docs/decisions/reactive-pose-port.md`):

    e_pos = p_desired − p(q);   e_rot = log3(R_desired · R(q)ᵀ)
    ẋ = Kp·e_pose + Kd·e_twist
    q̇_raw = Jᵀ (J Jᵀ + λ² I₆)⁻¹ ẋ  [+ null-space centering, currently off]
    q̇_i   = clamp(q̇_raw_i, ±kQdotLimit_i)      (45 deg/s)
    q_command += q̇_clipped · dt              (persistent integrator)

All laws use the same clamp and integrator, with q_command streamed as
position setpoints at 500 Hz (`kControlDtS`, the single timing source).
Low-level VELOCITY
mode was tried and abandoned — the actuator's inner velocity loop has no
gravity compensation (hardware evidence + Kinova kortex issues
#42/#93/#156). Design: `../docs/decisions/resolved-rate-position-integration.md`;
history: `../docs/decisions/cartesian-velocity-controller.md` and earlier.

## Layout

- `src/` — one file per job, named for that job. The layout deliberately
  mirrors `msc_project/controller/` so the two projects are navigable the
  same way (restructured 2026-08-03):

  | file | job | msc_project counterpart |
  | --- | --- | --- |
  | `Config.h` | every compiled setting; `--log` is the only runtime argument | `config/control.toml` |
  | `State.h` | the records that cross boundaries: `RobotState`, `ControllerStatus`, `Reference`, `ReferenceSource` | `state.py` + `backend.py` |
  | `ReactiveLaw.h` | the pose equations 1-6, header-only, no robot | `reactive_controller.py` |
  | `Controller.h/.cpp` | **THE controller** — tracks whichever reference channel is set | `servo.py` |
  | `Targets.h/.cpp` | position targets, strict stdin parsing, eight-entry SPSC mailbox, `PoseTargetSource` | `desired_pos.py` |
  | `Actuation.h/.cpp` | `PositionIntegration`: q_command integrator + lead limiter | `position_actuation.py` |
  | `Kinematics.h/.cpp` | Pinocchio FK/Jacobians + the `DualArmKinematics` adapter | `pin_fk.py` |
  | `Safety.h/.cpp` | stop classification, readiness gate, fault decoding, `ServoingGuard` | (hardware-only) |
  | `Hardware.h/.cpp` | `Connect`, `CyclicSession`, the run log + CSV writer | (hardware-only) |
  | `Runner.h/.cpp` | **the loop — moves the arm**: takeover T1-T6, cycle order, teardown D1-D2 | `runner.py` |
  | `Main.cpp` | the program: wiring, config echo, reports | `main.py` |

  Everything above `Kinematics` is pure Eigen — no Kortex, no Pinocchio —
  which is what lets the portable tests build and run anywhere.
- `tests/` — tests (CTest); the portable suite runs anywhere, the rest
  links the bundled Linux libraries (hardware machine only)
- `tools/` — standalone robot utilities: `clear_faults`,
  `read_safety_limits`, `set_joint_limits`, `probe_direction`,
  `make_synthetic_log`
- `scripts/` — offline Python analysis (not part of the build)
- `config/` — the tracked mounted dual-arm runtime URDF

## Build and test

```bash
cd Christian_control/basic_control
mkdir -p build && cd build
cmake .. && make
ctest            # hardware-free control-logic tests
```

## Use

> Run from `basic_control/build/`. **MOVES THE ARM**: workspace clear,
> e-stop in hand, authorization required for every session.

```bash
./controller                       # current-pose takeover, then terminal target
./controller --log my_run.csv      # name the CSV; the only runtime argument
```

The process acquires exclusive host-side ownership of `192.168.1.10` before
opening any robot connection. A concurrent second `basic_control` process exits
without contacting the arm; this guard does not cover other Kortex programs.

**There is no runtime configuration.** `--log` is the only flag; every
gain, term switch, and limit is a compiled constant in `src/Config.h`.
During a run, stdin accepts exactly `x y z` as three finite values in metres
in `base_link`; this is target input, not configuration. Every run still
echoes its effective configuration and embeds it as `#` lines in the CSV,
so each data file stays self-describing.

**Read `Config.h` before every session** — with no runtime override, the
compiled values are the only thing standing between you and the arm. In
particular check `kFixedTargetM`, `kQdotLimitDegS`, and `kStopOnFault`.

What a run does, in order:

1. Loads the 14-joint mounted dual URDF and validates the exact joint-name
   mapping. The left seven joints are held at the compiled nominal model
   state. The only TCP/UDP connection and command frame are for the right arm.
2. Clears faults unconditionally right after connecting (`Base::ClearFaults`
   — the same operation as the dashboard's "Clear faults", commands no
   motion), waits 500 ms, then runs the readiness check on a fresh
   post-clear frame. A fault that re-latches is live and still refuses the
   takeover. Prints the joint state and the **current end-effector
   position** — that printed `x y z` is what a "hold here" target looks
   like.
3. Enters LOW_LEVEL_SERVOING, seeds the position integrator from the
   measured state (q_command = q_measured) and captures the current pose as
   the hold pose, then sends one unchanged holding frame — the arm holds.
   Actuators stay in their default POSITION mode.
4. The arm holds its measured takeover pose first, then follows a bounded
   Cartesian profile from that pose to the compiled terminal target
   `kFixedTargetM`, preserving the captured takeover orientation. The stdin
   thread accepts one `x y z` target per line in metres, `base_link` only.
   It rejects malformed, trailing, or non-finite input. The parser does not
   perform reachability, collision, or path validation. At most eight live targets
   are queued in FIFO order. They cannot bypass the terminal target: each
   target activates only after the previous target emits its arrival edge,
   and the final target holds.

   Arrival requires both position error ≤ 1 mm and orientation error
   ≤ 1 mrad. After arrival the source holds for two seconds, then advances
   exactly one queued target. Each terminal-to-terminal Cartesian reference
   follows the compiled seventh-order rest-to-rest profile (currently
   0.025 m/s, 0.05 m/s², and 0.25 m/s³ limits). The reactive law tracks that
   shaped reference; the independent 45 deg/s joint-rate clip remains active.
5. Ctrl+C stops cleanly: the integrator stops updating (the position servo
   holds the last setpoint), single-level servoing is restored, and the
   last unwritten telemetry is flushed. Exit 0 only on a clean operator
   stop with no faults seen — a fault the loop was told to ignore still
   taints the exit code and prints a decoded report, so a nonzero exit
   after an apparently normal run means "check the CSV".

Telemetry goes to `runs/YYYY-MM-DD/loop_log_YYYYMMDD_HHMMSS.csv`, written
by a writer thread **as the run happens** (100 ms drains) rather than in
one write at the end. A run that dies without unwinding — SIGKILL, the
IDE's stop button, a debugger detach, a crash — therefore keeps every row
up to its last drain, and the file always ends on a complete row. The
whole run is kept, at roughly 175 KB/s (~300 MB for 30 minutes at the
500 Hz loop rate): prune `runs/` rather than shortening the record.

If the writer ever fails to keep up with the loop, the dropped samples are
counted and reported at exit instead of being silently overwritten — and
the resulting hole is visible to `analyze_run.py`'s integrity report as a
`time_s` gap.

First hardware runs: start from the printed current position and change
**one coordinate by a few centimeters**.

The common-frame model and hardware-free tests do not prove physical
mount calibration, collision safety, or safe robot behavior.

## Planner bridge (Stage 1) — supervised runs only

`Christian_control/planner_bridge/` (`planner_bridge` binary, own build
directory) is a separate, hardware-free process: one GPMP2 solve per
invocation, emitted as `x y z` lines on this controller's existing stdin
target grammar (design: `../docs/decisions/stage1-planner-bridge.md`).
It is a new **source**, not a new controller — the controller code above
is unchanged.

Offline check (no robot, safe anytime; zero-config tool sits at
`(0.0, -0.0246, 1.3073)` in `base_link`, so this goal is reachable from a
single solve):

```bash
./planner_bridge --start-deg 0 0 0 0 0 0 0 --goal 0.15 0.075 1.207
```

Hardware run (requires explicit authorization, operator present, e-stop
in reach; the controller consumes bridge waypoints only after reaching
its compiled terminal target):

**Hold a persistent write end open on the FIFO for the whole session,
before starting the controller — do not let each `planner_bridge` run be
the FIFO's only writer.** A named FIFO delivers EOF to its reader once
every writer has closed. The controller's stdin reader,
`RunPoseTargetInputFromFd` (`src/Targets.cpp`), treats EOF as final: on
read, if `read_count == 0` it processes any trailing partial line and then
`break`s out of its poll loop for good (`src/Targets.cpp` — the loop that
calls `poll`/`read` on `input_fd`). There is no re-open or retry — once
that loop exits, the controller silently never reads another target for
the rest of the run. If each bridge invocation opens and closes
`/tmp/bridge_targets` for its own `>` redirect, the *first* run's exit
closes the only writer, stdin reader hits EOF and exits, and **every
subsequent bridge run writes into a pipe nobody is reading, exits 0, and
the operator has no indication the new goal was never delivered.**

Open the persistent writer in the *operator's own shell* — a shell
builtin FD, not the controller process — so it survives every
`planner_bridge` run:

```bash
mkfifo /tmp/bridge_targets
./controller --log < /tmp/bridge_targets        # terminal 1
exec 3>/tmp/bridge_targets                      # terminal 2, ONCE, before any bridge run —
                                                 # keeps a writer open so the
                                                 # controller's stdin reader never sees EOF
./planner_bridge --state-csv <today's run csv> \
    --goal <x y z> >&3                          # terminal 2, repeat per replan — write to FD 3, not >
exec 3>&-                                       # terminal 2, ONCE, at the end of the session
```

Stop-and-replan: wait for hold (arrival messages in terminal 1), then
re-run the bridge with a new goal (`>&3`, not `>`); each run reads the
fresh CSV state. Do not close FD 3 between runs — only at the very end of
the session.

Exit codes (`RunBridge`): 0 targets emitted (also returned by `--help`),
1 bad arguments, 2 start state unavailable, 3 solve failed, 4 validation
rejected the plan. A non-zero exit writes nothing to the FIFO. At most 8
waypoints are ever emitted per solve (`PoseTargetMailbox::kCapacity`,
`Targets.h`) — the same queue this section's stdin target input already
fills.

An optional `--box CX CY CZ HX HY HZ` obstacle must lie fully inside the
SDF grid's checked volume — `x [-1.2, 1.2]`, `y [-1.2, 1.2]`,
`z [-0.4, 1.6]` m in `base_link` (`WorldSdf.h` `WorldGridBounds()`) — or
the run is rejected (exit 1) before solving. Outside that volume gpmp2
reports zero obstacle cost with no warning, so an unchecked box would be
silently ignored rather than avoided.

**Stage 1 limitation**: the controller tracks the reactive law's own
path between bridge waypoints, not GPMP2's planned joint path — only the
sampled waypoints are guaranteed to lie on the planned path. See the
decision record for the full argument.

## Offline analysis

After a run (never during — all scripts are offline-only and never touch
the robot):

```bash
python3 scripts/analyze_run.py            # newest run: integrity + tracking report
python3 scripts/analyze_run.py <run.csv>  # a specific run
python3 scripts/measure_delay.py <run.csv> # pipeline delay from a step run
```

`analyze_run.py` prints a log-integrity report first (timestamp gaps,
overruns, dropped-cycle estimate — it refuses to compute statistics from a
log with >1% holes unless `--force`), then matches commanded and actual
position **by timestamp interpolation, never by row index**, estimates the
command-to-feedback lag by cross-correlation, and writes
`<run>_tracking.pdf` / `<run>_error.pdf` next to the CSV with an RMS/max
error summary (raw and lag-compensated).

### Measuring command-to-motion delay (one-time calibration)

The pipeline's built-in reaction delay (command sent → arm starts moving)
should be measured once per setup and reported separately, so it doesn't
pollute tracking-error numbers. Procedure (hardware session — all safety
rules above apply):

1. Move the arm to a pose well inside the workspace, start `./controller`,
   and let it hold for at least 2 seconds without typing anything.
2. Type one target: the printed hold position with **one axis changed by
   +0.02 m** (2 cm keeps speeds trivial). It queues after the compiled
   target reaches its arrival tolerance.
3. Wait for "target reached" plus ~2 seconds, then Ctrl+C.
4. `python3 scripts/measure_delay.py` — prints the time from the command
   step to first motion above the noise floor, and the time to settle
   within 1 mm. The script refuses runs that don't contain exactly one
   clean step.

## Safety — read before every session

- **The controller's explicit motion limit is a per-joint velocity clamp** —
  currently **45 deg/s on every joint** (`kQdotLimitDegS`, equal to
  `kModelVelocityLimitsDegS` in `Config.h`). This is below the rated Table 40
  limits of 79.64 deg/s (joints 1–4) and
  69.91 deg/s (joints 5–7), which this clamp held until 2026-08-03. This is a
  client-side limit; the actuator firmware safeties are separate, and a
  stream that outruns them can fault mid-move.
  The target source shapes Cartesian reference speed, acceleration, and jerk;
  these are trajectory-generation limits, not independent measured-motion
  safety guards. The reactive error correction and per-joint clip can still
  change the executed speed and path, so the CSV remains the evidence.
- **Input validation is deliberately narrow**: stdin validates only the target
  syntax and finiteness. It does not prove inverse-kinematics feasibility,
  collision clearance, joint-limit clearance, or a safe path.
- **Low-level servoing bypasses the robot's motion supervisor** — no
  onboard planning, obstacle avoidance or self-collision avoidance. The
  end effector travels the straight line to the target and the DLS solution
  moves all 7 joints, so check the links and surroundings too.
- Firmware `JOINT_LIMIT` thresholds are the actual joint-position
  enforcement. At startup the controller owns these thresholds: on every
  connection it re-applies as needed and verifies the bounded joints 2/4/6;
  continuous joints 1/3/5/7 remain unset. There is no separate client-side
  joint-position *clamp*: before sending, the software stops and holds the
  last safe seven-joint frame when a bounded joint would move farther
  outward past its conservative software boundary (while allowing inward
  recovery). The boundary is the smaller of the configured firmware warning
  and the [Gen3 User Guide Table 39](https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf)
  position magnitude less 2°: J2/J4/J6 are 126.9°/145°/118°; continuous
  joints remain unbounded. It never widens a firmware threshold.
  The controller's velocity clip, spherical input reach screen, and
  following-error stop are separate protections, not joint-position
  enforcement.
- Before takeover, a read-only `GetKinematicHardLimits` gate prints all
  seven live hard joint-speed limits and refuses a configured qdot clip that
  exceeds one. The bundled Kortex 2.7.0 `KinematicLimits` schema has no
  `joint_position_limits` field, so this RPC does **not** verify positions;
  Table 39 and the model tests are their source. `./read_safety_limits`
  runs the same no-motion gate for a pre-session check, without making a
  second ControlConfig client.
- **A live base or actuator fault stops the run.** `kStopOnFault` is `true`
  in `Config.h`; faults are decoded, logged, and terminate command streaming.
- What still ends the run unconditionally: a following error above
  `kFollowingErrorLimitDeg` (3°), a live fault when `kStopOnFault` is true,
  loss of low-level servoing, exchange failure, then the software
  joint-limit warning guard, then a per-actuator cyclic acknowledgement that
  stays unchanged for 25 completed replies (50 ms), plus enabled consecutive
  non-finite-command and overrun guards. An acknowledgement stall is a
  downstream-feedback-path signal, not a physical-motion detector. That
  ordering means a simultaneous live state cannot be hidden by the held-frame
  warning; an ignored live fault still taints the exit result. On any of these
  the loop stops streaming (the position servo holds the last setpoint) and
  restores SINGLE_LEVEL servoing — guarded, with a warning if it fails. If you
  see such a warning, check the arm (web dashboard) before running anything
  else.
