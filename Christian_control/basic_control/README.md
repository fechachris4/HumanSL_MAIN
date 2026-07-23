# basic_control

My controller for a **single Kinova Gen3 7-DoF** arm. It combines:

- the bundled Kinova **Kortex API** (`../../third_party/kortex_api`) to talk
  to the arm, and
- **Pinocchio** (via the `Dynamics` class reused from `TrajectoryExecution`)
  for kinematics from the arm's URDF.

It does **not** use the HumanSL planning stack (GTSAM / GPMP2 / Vicon).

The program is a resolved-rate Cartesian controller: it takes over the arm
in low-level servoing (actuators in their default POSITION mode) and drives
the end-effector toward positions typed on stdin:

    e = p_desired − p(q_measured);   v_d = Kp · e
    q̇_raw = Jpᵀ (Jp Jpᵀ + λ² I₃)⁻¹ v_d      (damped least squares)
    q̇_i   = clamp(q̇_raw_i, ±kQdotLimit_i)      (71.6 deg/s joints 1–4, 62.9 joints 5–7)
    q_command += q̇_clipped · dt              (persistent integrator)

with q_command streamed as position setpoints at 100 Hz. Low-level VELOCITY
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
- `config/` — our copy of the arm's URDF (`GEN3_custom.urdf`)

## Files

- `src/app/main.cpp` — thin coordinator: model+config, connect, readiness
  check, state printout, input thread, loop call, log flush, exit code
- `src/app/Config.h` — the compiled defaults: robot IP, `kControlDtS`
  (0.01 s — the single timing source), `kKpCartesian` (1.0 /s),
  `kDlsLambda` (0.1), `kQdotLimitDegS` (0.9 × model limits ≈ 71.6/62.9
  deg/s clip), `kStopOnFault` (compile-time only), `kEndEffectorFrame`,
  supervisor counter limits, log capacity
- `src/app/Options.*` — runtime overrides, precedence CLI > TOML >
  compiled (gains/thresholds only — never safety policy; no config-file
  auto-discovery): `../docs/decisions/runtime-config.md`
- `src/hardware/Connect.*` — RAII: both Kortex sessions (TCP 10000 +
  real-time UDP 10001)
- `src/hardware/Measure.*` — `read_feedback`, the program's single
  standalone `RefreshFeedback`
- `src/hardware/Cyclic.*` — `CyclicSession`: owns the command frame; seed
  read + the one stamped `Refresh` exchange per cycle
- `src/hardware/Record.*` — telemetry: preallocated ring buffer (`LoopLog`),
  written to a timestamped CSV after the loop; `push` is loop-safe
- `src/math/Kinematics.*` — FK and `position_and_jacobian` (position + 3×7
  translational Jacobian from the same measured q)
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
  robot's kinematic hard/soft limits

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
./controller                       # compiled defaults
./controller --kp 0.8              # one-off gain override
./controller --config gains.toml   # gains/thresholds from an explicit file
./controller --help                # full option + TOML-key list
```

Every run echoes its full effective configuration (each value tagged
compiled/toml/cli) and embeds it as `#` lines in the CSV, so every data
file is self-describing. Safety policy is not runtime-configurable.

1. Loads the URDF (checks the model has exactly 7 velocity variables) and
   connects (TCP + UDP).
2. Readiness check on one feedback frame: a live fault refuses startup
   (faults are never cleared here — use the Kinova web dashboard). Prints
   the joint state and the **current end-effector position** — that printed
   `x y z` is what a "hold here" target looks like.
3. Enters LOW_LEVEL_SERVOING, seeds the position integrator and the
   desired position from the measured state (q_command = q_measured,
   p_desired = p_current), and sends one unchanged holding frame — the arm
   holds. Actuators stay in their default POSITION mode.
4. Type a desired position — **x y z in meters, robot base frame**:

   ```
   0.45 0.10 0.30
   ```

   The end-effector moves toward it at `Kp × distance` (1.0 /s × error —
   e.g. a 10 cm error starts at 0.1 m/s and slows exponentially as it
   converges). A new line replaces the target immediately; mid-motion
   retargeting is normal. Invalid lines are rejected with a reason.
5. Ctrl+C stops cleanly: the integrator stops updating (the position servo
   holds the last setpoint), single-level servoing is restored, telemetry
   (most recent 600 s) is written to `run_YYYYMMDD_HHMMSS.csv`. Exit 0 only on this
   clean stop; faults print a decoded report and exit 1.

First hardware runs: start from the printed current position and change
**one coordinate by a few centimeters**.

## Safety — read before every session

- **The only limiting is a per-joint velocity clamp** — ≈71.6 deg/s
  (joints 1–4) / ≈62.9 deg/s (joints 5–7) (`kQdotLimitDegS`, derived at
  compile time in `Config.h` as 10% under the model limits, which the
  base's configured speed soft limits must match: verify with
  `./query_limits` first, see `../docs/decisions/qdot-limit-raise.md`;
  streams that outrun what the base actually enforces fault mid-move).
  There is NO Cartesian velocity,
  acceleration, or workspace limiting (explicit design choice — see the
  decision record). Speed is `Kp × error` up to the clamp, and a
  saturated joint distorts the motion direction. Type nearby targets.
- **No reachability check**: an unreachable target makes the controller
  push toward it until you retarget, stop, or the arm faults.
- **Low-level servoing bypasses the robot's motion supervisor** — no
  onboard planning or self-collision avoidance. The DLS solution moves all
  7 joints; check the surroundings, not just the end-effector path.
- This arm has **configured position limits far inside the factory range**
  (joint 4 near −19.6°, joint 6 near +36° — both found by faulting into
  them; check/adjust via the Kinova web dashboard, and `tools/query_limits`
  prints the base-enforced limits). The controller does not know them;
  the arm faults if a solution path crosses one.
- On any live fault, loss of low-level servoing, or exchange failure the
  loop stops streaming (the position servo holds the last setpoint) and
  restores SINGLE_LEVEL servoing — guarded, with a warning if it fails.
  If you see such a warning, check the arm (web dashboard) before running
  anything else.
