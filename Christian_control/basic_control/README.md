# basic_control

My controller for the **right Kinova Gen3 7-DoF arm**, using the complete
mounted dual-arm model for kinematics. It
combines:

- the bundled Kinova **Kortex API** (`../../third_party/kortex_api`) to talk
  to the arm, and
- **Pinocchio** (via the `Dynamics` class reused from `TrajectoryExecution`)
  for kinematics from the arm's URDF.

It does **not** use the HumanSL planning stack (GTSAM / GPMP2 / Vicon).

The program offers two control laws over the same loop: it takes over the
arm in low-level servoing (actuators in their default POSITION mode) and
drives the end-effector toward targets typed on stdin.

`resolved-rate` (default) — position-only:

    e = p_desired − p(q_measured);   v_d = Kp · e
    q̇_raw = Jpᵀ (Jp Jpᵀ + λ² I₃)⁻¹ v_d      (damped least squares)
    q̇_i   = clamp(q̇_raw_i, ±kQdotLimit_i)      (79.6 deg/s joints 1–4, 69.9 joints 5–7)
    q_command += q̇_clipped · dt              (persistent integrator)

`reactive-pose` (`--controller reactive-pose`) — full 6-DoF pose, ported
from the simulation (msc_project) and cross-validated against it
(`../docs/decisions/reactive-pose-port.md`):

    e_pos = p_desired − p(q);   e_rot = log3(R_desired · R(q)ᵀ)
    ẋ = Kp·e_pose [+ Kd·e_twist, default off]
    q̇_raw = Jᵀ (J Jᵀ + λ² I₆)⁻¹ ẋ  [+ null-space centering, default off]

Both laws use the same clamp and integrator, with q_command streamed as
position setpoints at 1 kHz (`kControlDtS`, the single timing source).
Low-level VELOCITY
mode was tried and abandoned — the actuator's inner velocity loop has no
gravity compensation (hardware evidence + Kinova kortex issues
#42/#93/#156). Design: `../docs/decisions/resolved-rate-position-integration.md`;
history: `../docs/decisions/cartesian-velocity-controller.md` and earlier.

## Layout

- `src/` — one subfolder per technical layer: `app/` (orchestration),
  `control/` (control laws + operator input), `actuation/` (command-state
  strategy), `loop/` (the Runner — moves the arm), `safety/` (policy +
  reporting), `math/` (kinematics, DLS), `hardware/` (Kortex I/O and
  telemetry)
- `tools/` — standalone diagnostic executables
- `tests/` — tests (CTest) + the CSV replay harness; the portable suite
  runs anywhere, the rest links the bundled Linux libraries (hardware
  machine only)
- `scripts/` — offline Python analysis (not part of the build)
- `config/` — the tracked mounted dual-arm runtime URDF plus runtime config

## Files

- `src/app/main.cpp` — thin coordinator: model+config, connect, readiness
  check, state printout, input thread, loop call, log flush, exit code
- `src/app/Config.h` — compiled right-only hardware ownership, left nominal
  model state, end-effector frame, log prefix, plus defaults: `kControlDtS`
  (0.001 s — the single timing source), `kKpCartesian` (1.0 /s),
  `kDlsLambda` (0.1), `kQdotLimitDegS` (equal to the 79.6/69.9 deg/s
  model limits), `kStopOnFault` (compile-time only), selected end-effector
  frame, supervisor counter limits, log capacity
- `src/app/Options.*` — runtime overrides, precedence CLI > TOML > compiled
  (controller selection, gains, thresholds and input selection;
  never fault-stop policy): `../docs/decisions/runtime-config.md`
- `src/hardware/Connect.*` — RAII: both Kortex sessions (TCP 10000 +
  real-time UDP 10001)
- `src/hardware/Measure.*` — `read_feedback`, the program's single
  standalone `RefreshFeedback`
- `src/hardware/Cyclic.*` — `CyclicSession`: owns the command frame; seed
  read + the one stamped `Refresh` exchange per cycle
- `src/hardware/Record.*` — telemetry: preallocated ring buffer (`LoopLog`),
  written to a timestamped CSV after the loop; `push` is loop-safe
- `src/math/Kinematics.*` — FK and `position_and_jacobian` (position,
  rotation + 3×7 translational Jacobian from the same measured q)
- `src/math/DualArmKinematics.*` — explicit adapter: measured right 7 plus
  nominal left 7 into full q, full 6×14 Jacobian, selected right 7 columns
- `src/math/Dls.h` — damped least squares (LDLT, no explicit inverse),
  header-only and hardware-free-tested
- `src/safety/Supervisor.*` — stop classification (following error first,
  then live faults, then arm state) + the pre-takeover readiness gate
- `src/safety/FaultReport.*` — fault-bank decoding and the stop /
  fault-change reports
- `src/safety/ServoingGuard.*` — RAII servoing-mode ownership: LOW_LEVEL on
  construction, guaranteed SINGLE_LEVEL restore on destruction
- `src/control/Controller.h` — the controller interface (`RobotState`,
  arm-feedback-only by hard rule; pure computation, no I/O)
- `src/control/ResolvedRate.*` — the Cartesian control law (see above)
- `src/control/Target.*` — desired end-effector position: stdin thread,
  parsing (3 finite numbers; deliberately no reachability check), latest-
  value store
- `src/actuation/*` — `Actuation` strategy + `PositionIntegration` (the
  q_command integrator)
- `src/loop/Runner.*` — **the loop — moves the arm**: takeover sequence
  T1-T6, per-cycle order, teardown D1-D3 (spec in `Runner.h`)
- `tools/query_limits.cpp` — separate read-only executable: prints the
  robot's reported kinematic hard limits

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
./controller                       # loads ../config/control.toml if present,
                                   # else compiled defaults
./controller --kp 0.8              # one-off gain override
./controller --config gains.toml   # explicit file instead of the default one
./controller --help                # full option + TOML-key list
```

The everyday workflow is **edit `config/control.toml`, run the bare
binary**: that checked-in file is loaded automatically (compiled absolute
path — never a working-directory lookup) and selects the control law
(`controller = "reactive-pose"`), gains, term switches, and optionally a
`target_file`. Precedence stays CLI > TOML > compiled defaults.

With `target_file` set (reactive-pose only), the controller also watches
that file: during a run, edit and save it with one line — `x y z` or
`x y z roll pitch yaw` — and the arm retargets, same as typing on stdin
(latest source wins). The file's content at startup is deliberately
ignored: a stale target file never starts a motion.

Every run echoes its full effective configuration (each value tagged
compiled/default/toml/cli, plus which config file was loaded) and embeds
it as `#` lines in the CSV, so every data file is self-describing. Safety
policy is not runtime-configurable.

1. Loads the 14-joint mounted dual URDF and validates the exact joint-name
   mapping. The left seven joints are held at the compiled nominal model
   state. The only TCP/UDP connection and command frame are for the right arm.
2. Readiness check on one feedback frame: a live fault refuses startup
   (faults are never cleared here — use the Kinova web dashboard). Prints
   the joint state and the **current end-effector position** — that printed
   `x y z` is what a "hold here" target looks like.
3. Enters LOW_LEVEL_SERVOING, seeds the position integrator and the
   desired position from the measured state (q_command = q_measured,
   p_desired = p_current), and sends one unchanged holding frame — the arm
   holds. Actuators stay in their default POSITION mode.
4. Type a desired position — **x y z in meters, dual-model world/common
   mount frame**:

   ```
   0.45 0.10 0.30
   ```

   With `--controller reactive-pose` a 3-number line moves the position
   target and keeps the current orientation target; a 6-number line
   `x y z roll pitch yaw` (radians, R = Rz(yaw)·Ry(pitch)·Rx(roll)) sets
   both. The arrival notice is position-based in both laws; judge
   orientation convergence from the CSV's `rot_error_rad` column.

   The end-effector moves toward it at `Kp × distance` (1.0 /s × error —
   e.g. a 10 cm error starts at 0.1 m/s and slows exponentially as it
   converges). A new line replaces the target immediately; mid-motion
   retargeting is normal. Invalid lines are rejected with a reason.
5. Ctrl+C stops cleanly: the integrator stops updating (the position servo
   holds the last setpoint), single-level servoing is restored, and the
   last unwritten telemetry is flushed. Exit 0 only on this clean stop;
   faults print a decoded report and exit 1.

Telemetry goes to `runs/YYYY-MM-DD/loop_log_YYYYMMDD_HHMMSS.csv`, written
by a writer thread **as the run happens** (100 ms drains) rather than in
one write at the end. A run that dies without unwinding — SIGKILL, the
IDE's stop button, a debugger detach, a crash — therefore keeps every row
up to its last drain, and the file always ends on a complete row. The
whole run is kept, at roughly 350 KB/s (~600 MB for 30 minutes at 1 kHz):
prune `runs/` rather than shortening the record.

If the writer ever fails to keep up with the loop, the dropped samples are
counted and reported at exit instead of being silently overwritten — and
the resulting hole is visible to `analyze_run.py`'s integrity report as a
`time_s` gap.

First hardware runs: start from the printed current position and change
**one coordinate by a few centimeters**.

The common-frame model and hardware-free tests do not prove physical
mount calibration, collision safety, or safe robot behavior.

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
2. Type exactly one target: the printed hold position with **one axis
   changed by +0.02 m** (2 cm keeps speeds trivial).
3. Wait for "target reached" plus ~2 seconds, then Ctrl+C.
4. `python3 scripts/measure_delay.py` — prints the time from the command
   step to first motion above the noise floor, and the time to settle
   within 1 mm. The script refuses runs that don't contain exactly one
   clean step.

## Safety — read before every session

- **The controller's explicit motion limit is a per-joint velocity clamp** — 79.6 deg/s
  (joints 1–4) / 69.9 deg/s (joints 5–7) (`kQdotLimitDegS`, equal to
  `kModelVelocityLimitsDegS` in `Config.h`). This is a
  client-side limit; the actuator firmware safeties are separate, and a
  stream that outruns them can fault mid-move.
  There is NO Cartesian velocity,
  acceleration, or workspace limiting (explicit design choice — see the
  decision record). Speed is `Kp × error` up to the clamp. A far target
  pins the clamp for the whole transit (allowed — the saturation stop was
  removed 2026-07-23), and a saturated joint distorts the motion
  direction: the arm drifts toward the target but NOT in a straight line.
- **No reachability check**: an unreachable target makes the controller
  push toward it until you retarget, stop, or the arm faults.
- **Low-level servoing bypasses the robot's motion supervisor** — no
  onboard planning, obstacle avoidance or self-collision avoidance. The
  end effector travels the straight line to the target and the DLS solution
  moves all 7 joints, so check the links and surroundings too.
- This arm has **configured position limits far inside the factory range**
  (joint 4 near −19.6°, joint 6 near +36° — both found by faulting into
  them; check/adjust via the Kinova web dashboard). The controller does not
  know them; the arm faults if a solution path crosses one.
- On any live fault, loss of low-level servoing, or exchange failure the
  loop stops streaming (the position servo holds the last setpoint) and
  restores SINGLE_LEVEL servoing — guarded, with a warning if it fails.
  If you see such a warning, check the arm (web dashboard) before running
  anything else.
