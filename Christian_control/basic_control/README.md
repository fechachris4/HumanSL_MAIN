# basic_control

My controller for a **single Kinova Gen3 7-DoF** arm. It combines:

- the bundled Kinova **Kortex high-level Base API** (`../../third_party/kortex_api`)
  to talk to the arm, and
- **Pinocchio** (via the `Dynamics` class reused from `TrajectoryExecution`)
  to compute model-based quantities like gravity torques from the arm's URDF.

It does **not** use the HumanSL planning stack (GTSAM / GPMP2 / Vicon).

## Layout

- `src/` — the controller application
- `tools/` — standalone diagnostic executables (read-only)
- `scripts/` — offline analysis (Python, not part of the build)
- `config/` — our copy of the arm's URDF (`GEN3_custom.urdf`)
- `motion.txt` — optional runtime motion config, kept at this root so the
  executable's search (cwd → build/ → basic_control/) finds it

## Files

- `src/main.cpp` — thin coordinator: connects, calls the helpers, runs the
  recording loop, shuts down cleanly (see `../AGENTS.md` contract)
- `src/Config.h` — all runtime settings as named constants: robot IP,
  recording rate, output filenames; edit and rebuild (no CLI flags)
- `src/Connect.h` / `Connect.cpp` — RAII: opens both Kortex sessions (TCP 10000 +
  real-time UDP 10001) on construction, closes them on destruction
- `src/Measure.h` / `Measure.cpp` — sensor reading via the real-time feedback
  channel, plus conversion to the Pinocchio configuration; never moves the arm
- `src/Kinematics.h` / `Kinematics.cpp` — forward kinematics (Pinocchio) and the
  startup FK-vs-robot cross-check
- `src/Controller.h` / `Controller.cpp` — task-space control: a read-only
  pose-error report at startup, and the **reactive servo** (**moves the arm**,
  see "Reactive mode" below): PD on pose + velocity error, damped-least-squares
  inverse kinematics, optional null-space joint centering (off by default),
  integrated position setpoints streamed at 1 kHz until Ctrl+C
- `src/Record.h` / `Record.cpp` — fixed-rate recording loop: timestamps, dt, CSV
  rows, 1 Hz heartbeat, clean stop via the Ctrl+C flag
- `src/Motion.h` / `Motion.cpp` — **moves the arm**: low-level cyclic motion of
  all 7 joints simultaneously (relative deltas, per-joint speed caps); a move
  only succeeds when every moving joint *measures* within 0.5° of its target,
  and it aborts early — with a per-joint report — on tracking failure, robot
  fault, or communication error; Ctrl+C freezes the motion; servoing mode
  restored by RAII on every exit path
- `src/Timing.h` / `Timing.cpp` — latency benchmarks (feedback round-trip, full
  control cycle, dynamics compute), writing stats to a log stream
- `tools/query_limits.cpp` — separate read-only executable (`./query_limits`):
  prints the robot's kinematic hard limits and per-mode soft limits; never
  moves the arm, ignores motion.txt
- `scripts/plot_move.py` — offline analysis of a move log: per-joint final error,
  overshoot, max tracking error, dt statistics, and commanded-vs-measured
  plots (PNG) for every joint that moved
- `scripts/plot_joint.py` — offline plot of a single joint (pick 1–7) from a move
  log: commanded vs measured angle over time, saved as PNG, `--show` for an
  interactive window

## Build

```bash
cd Christian_control/basic_control
mkdir -p build && cd build
cmake .. && make
```

## Use

> Run from `basic_control/build/`.

```bash
# No command-line flags. Robot IP (192.168.1.10), recording rate (100 Hz)
# and output filenames live in src/Config.h — edit there and rebuild.
./controller

# MOVES THE ARM — workspace clear, e-stop in hand.
# A move runs when a motion.txt file is found (no CLI flags). Searched in
# order: current dir, the executable's dir (build/), its parent
# (basic_control/). The program prints which file it uses. Minimal file:
#   deltas_deg: 0 10 0 -15 0 5 0      # 7 relative deg; 0 = hold that joint
#   speeds_deg_s: 0 5 0 5 0 3 0       # optional per-joint deg/s
#   default_speed_deg_s: 5            # optional; fallback 5 deg/s
./controller
```

What it does right now:

1. Loads the arm model (`config/GEN3_custom.urdf`) into Pinocchio.
2. Connects to the arm (both channels) and clears faults.
3. Prints an FK cross-check: our Pinocchio forward kinematics vs the pose the
   robot itself reports (they agree to ~2 cm once the gripper TCP offset is added).
4. Prints the controller's position error: target (from `Config.h`) minus the
   FK end-effector position, plus its norm and the current EE orientation as
   fixed-axis roll-pitch-yaw in degrees — computed once, no motion.
5. Records all 7 joint angles at the chosen rate into the CSV (with timestamp
   and dt columns), printing a 1 Hz heartbeat, until Ctrl+C.

## Reactive mode

**Moves the arm, continuously, until you press Ctrl+C.** Armed by a
`motion.txt` containing just:

```
mode: reactive
```

The arm then servos its end-effector toward the target pose in `src/Config.h`
(`kTargetPosition` + `kTargetOrientationRPYDeg`, base frame; roll-pitch-yaw
about the fixed base axes, degrees) and *keeps* servoing
— push the target error and it fights back. Each 1 ms cycle: FK + Jacobian
(Pinocchio) → PD on position/orientation/velocity error → damped-least-squares
joint rates → optional null-space centering on joints 2/4/6
(`kUseNullspaceCentering`, off by default: flip to `true` and rebuild to
compare — rerun the same target and diff the two `control_trace_*.csv` files;
each run prints ON/off at startup) → clip to
`kReactiveSpeedLimitDegS` → integrate the position setpoints → clamp the lead
over the measurement to `kCtrlLeadRad` (anti-windup; also bounds tracking
error well under the ~5 deg low-level-servoing kick-out) → stream. One trace
row per cycle goes to `control_trace_<timestamp>.csv` (errors, commanded
twist, joint rates before/after clipping, clamp flags, fault banks).

Safety behavior: Ctrl+C (and any fault or communication error) freezes the
command at the *measured* position, and RAII restores single-level servoing on
every exit path. All gains and clamps are constants in `Config.h` (edit and
rebuild). Speeds are clamped per joint; never raise the limit past the
project's 45 deg/s rule.

First-run procedure (workspace clear, e-stop in hand):

1. Read-only run (no motion.txt): copy the printed EE position and rpy angles
   into `kTargetPosition` / `kTargetOrientationRPYDeg`, rebuild.
2. Zero-gain run: set all gains in Config.h to 0, rebuild, run reactive — the
   arm must hold perfectly still. This validates the loop, timing (`dt_s` in
   the trace), logging, and Ctrl+C with zero motion risk.
3. Hold-in-place run: restore the gains (target = current pose from step 1).
   Expect a millimetre-level hold; check the trace for velocity noise.
4. Small step: offset `kTargetPosition` by 2–3 cm in z, watch it converge,
   then tune `kKpPos`/`kKpRot` up gradually.

If `motion.txt` is present, it instead moves all 7 joints simultaneously by
the configured relative deltas via low-level 1 kHz streaming (each joint
ramping at its own speed cap), then exits. Without the file the program is
entirely read-only. A bad config exits with an error before connecting.

A move succeeds (exit code 0, "Move finished.") only when every moving
joint's **measured** angle ends within 0.5° of its target after a 1 s settle
hold — sending all the commands is not enough, because the arm can stop
following them without any API error (that happened; see
`../docs/known-issues.md`). The move stops early and fails, printing each
moving joint's requested/measured displacement and final error, when:

- tracking error (commanded − measured) exceeds 3° for 50 consecutive
  cycles (the command is frozen at the measured position first),
- the robot reports a base or actuator fault,
- a Kortex send/receive call fails,
- or the final measured position misses the target.

Every move is logged to its own file, `move_log_YYYY-MM-DD_HH-MM-SS.csv`
(prefix in `src/Config.h`), so no run overwrites another's evidence. One row
per 1 kHz cycle: time, dt, then per joint the commanded and measured angles,
tracking error, velocity, torque, motor current, and fault/warning banks,
plus the arm state, base fault/warning banks, and whether the cyclic
exchange succeeded. After the last joint reaches its commanded target the
loop keeps holding and logging for 1 s so overshoot/settling is captured.

After each move the controller automatically runs `scripts/plot_move.py` on
that run's log: tracking stats in the terminal, PNG plots (all moving
joints + dt) next to the CSV. Needs `python3` with numpy/matplotlib; if
they're missing it says so and the move result stands. Re-run by hand, or
plot a single joint, with:

```bash
python3 ../scripts/plot_move.py            # newest move_log_*.csv: stats + PNG
python3 ../scripts/plot_joint.py 2 --show  # one joint: PNG + interactive window
```

## Safety

- Without `motion.txt` (recording mode) the program never moves the arm.
- A present `motion.txt` **moves the arm on startup**: keep the workspace
  clear and the e-stop in hand — and delete/rename the file afterwards so a
  later plain run doesn't repeat the move. Start with small deltas and low
  speeds (default 5 deg/s). Joint 7 (wrist rotation) is the safest first test.
- Ctrl+C during a move freezes the arm where it is, then hands control back
  to the robot's supervisor (restored by RAII even on errors).
- **Low-level servoing bypasses the robot's motion supervisor** — no onboard
  trajectory planning or self-collision avoidance. Each joint ramps linearly
  and independently; check the combined path is collision-free before sending.
- **The base enforces a 50 deg/s joint speed soft limit** (this arm's
  configuration; read it with `./query_limits`). Streaming position steps at
  or above it is not followed — the joint stands still, tracking error grows,
  and at ~5 deg the arm faults out of low-level servoing (the error surfaces
  as `WRONG_SERVOING_MODE` mid-move). Config validation therefore rejects
  speeds above 45 deg/s (10% margin).
