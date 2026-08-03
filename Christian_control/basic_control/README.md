# basic_control

My controller for the **right Kinova Gen3 7-DoF arm**, using the complete
mounted dual-arm model for kinematics. It
combines:

- the bundled Kinova **Kortex API** (`../../third_party/kortex_api`) to talk
  to the arm, and
- **Pinocchio** (via the `Dynamics` class reused from `TrajectoryExecution`)
  for kinematics from the arm's URDF.

It does **not** use the HumanSL planning stack (GTSAM / GPMP2 / Vicon).

The program has ONE controller and pluggable **reference sources**
(selected by `kReferenceSource` in `Config.h`; architecture diagram at the
top of `src/Controller.h`). A source says WHERE the end-effector or joints
should be each cycle; the `TrackingController` says HOW to move toward it.
It takes over the arm in low-level servoing (actuators in their default
POSITION mode) and tracks whichever reference channel the source provides
— a pose (typed targets) or a joint configuration (trajectory files). New
research inputs (Vicon, a Python bridge) are new sources, never new
controllers. (Historical modes: position-only `resolved-rate` removed
2026-08-03, superseded; the welded per-mode controller classes
(`ReactivePose`, `TrajectoryPlayback`) were decomposed into
source + controller the same day; git history has both.)

Pose references run the reactive law — full 6-DoF pose, ported
from the simulation (msc_project) and cross-validated against it
(`../docs/decisions/reactive-pose-port.md`):

    e_pos = p_desired − p(q);   e_rot = log3(R_desired · R(q)ᵀ)
    ẋ = Kp·e_pose + Kd·e_twist
    q̇_raw = Jᵀ (J Jᵀ + λ² I₆)⁻¹ ẋ  [+ null-space centering, currently off]
    q̇_i   = clamp(q̇_raw_i, ±kQdotLimit_i)      (45 deg/s, temporary bring-up clip)
    q_command += q̇_clipped · dt              (persistent integrator)

`kReferenceSource = "trajectory"` executes one
GPMP2-planned joint trajectory (produced offline by
`TrajectoryGeneration/tools/plan_move`; contract and gates in
`src/Trajectory.h`, design in
`../docs/decisions/trajectory-playback.md`):

    q̇_d = Δq_ref(t, t+dt)/dt  +  kp · wrap(q_ref(t) − q_measured)

The file is validated before any hardware session (velocity ≤ 90% of the
clip, Table 43 acceleration, position ranges, consistency, rest at both
ends) and the measured position must match the trajectory start within
0.2°/joint — otherwise the run refuses before, or holds at, the takeover.
Completion holds the final point until Ctrl+C.

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
  | `Targets.h/.cpp` | operator targets: parse, store, stdin/file threads, `PoseTargetSource` | `desired_pos.py` |
  | `Trajectory.h/.cpp` | the file contract, the joint-tracking law, `TrajectorySource` | `trajectory.py` |
  | `Actuation.h/.cpp` | `PositionIntegration`: q_command integrator + lead limiter | `position_actuation.py` |
  | `Kinematics.h/.cpp` | Pinocchio FK/Jacobians + the `DualArmKinematics` adapter | `pin_fk.py` |
  | `Safety.h/.cpp` | stop classification, readiness gate, fault decoding, `ServoingGuard` | (hardware-only) |
  | `Hardware.h/.cpp` | `Connect`, `CyclicSession`, the run log + CSV writer | (hardware-only) |
  | `Runner.h/.cpp` | **the loop — moves the arm**: takeover T1-T5, cycle order, teardown D1-D3 | `runner.py` |
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
./controller                       # everything comes from Config.h
./controller --log my_run.csv      # name the CSV; the only runtime argument
```

**There is no runtime configuration.** `--log` is the only flag; every
gain, term switch, limit, and the reference-source selection is a compiled
constant in `src/Config.h`, so changing behaviour means editing that file
and rebuilding. (The TOML/CLI configuration front-end — `Options.{h,cpp}`,
`config/control.toml`, `--kp`, `--config` — was removed on 2026-08-03; git
history has it, and `../docs/decisions/runtime-config.md` records why it
existed.) Every run still echoes its effective configuration and embeds it
as `#` lines in the CSV, so each data file stays self-describing.

**Read `Config.h` before every session** — with no runtime override, the
compiled values are the only thing standing between you and the arm. In
particular check `kReferenceSource`, `kUseFixedTarget`, `kQdotLimitDegS`,
and `kStopOnFault`.

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
4. Where it goes next depends on `kReferenceSource`:

   - **`"operator"` with `kUseFixedTarget = true`** (the current default):
     there is **no stdin thread**. The arm drives to the compiled
     `kFixedTargetM` **immediately** after the takeover, keeping the
     takeover orientation. Check that target against the printed current
     position before you start.
   - **`"operator"` with `kUseFixedTarget = false`**: type a target —
     **x y z in meters, right-arm `base_link` frame** — on stdin:

     ```
     0.45 0.10 0.30
     ```

     A 3-number line moves the position target and keeps the current
     orientation target; a 6-number line `x y z roll pitch yaw` (radians,
     R = Rz(yaw)·Ry(pitch)·Rx(roll)) sets both. A new line replaces the
     target immediately; mid-motion retargeting is normal. Invalid lines
     are rejected with a reason.
   - **`"trajectory"`**: replays `kTrajectoryFile` as soon as the takeover
     completes, then holds the final point. No operator input.

   The arrival notice is position-based; judge orientation convergence from
   the CSV's `rot_error_rad` column.

   The end-effector moves toward a pose target at `Kp × distance`, with
   `kKpCartesian = 10.0 /s` — so a 10 cm error commands **1.0 m/s** at the
   start, decaying exponentially as it converges, subject to the per-joint
   velocity clamp. This gain is aggressive; a target a long way from the
   current pose pins the clamp for the whole transit.
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

- **The controller's explicit motion limit is a per-joint velocity clamp** —
  currently **45 deg/s on every joint** (`kQdotLimitDegS`, equal to
  `kModelVelocityLimitsDegS` in `Config.h`). That is a temporary bring-up
  value well below the rated Table 40 limits of 79.64 deg/s (joints 1–4) and
  69.91 deg/s (joints 5–7), which this clamp held until 2026-08-03. This is a
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
- **A live fault does NOT currently stop the run.** `kStopOnFault` is
  `false` in `Config.h` — the 2026-07-20 fault-ignoring experiment, still
  switched on. Faults are decoded, printed on their edges and logged, but
  the loop keeps commanding. The run prints a `FAULT-STOP DISABLED` warning
  at takeover for exactly this reason. **ATTENDED USE ONLY: you are the
  stop.** Set `kStopOnFault = true` to restore the fault stop.
- What still ends the run unconditionally: a following error above
  `kFollowingErrorLimitDeg` (3°, checked before the fault bits so the
  experiment policy cannot mask it), loss of low-level servoing, and
  exchange failure. On any of these the loop stops streaming (the position
  servo holds the last setpoint) and restores SINGLE_LEVEL servoing —
  guarded, with a warning if it fails. If you see such a warning, check the
  arm (web dashboard) before running anything else.
