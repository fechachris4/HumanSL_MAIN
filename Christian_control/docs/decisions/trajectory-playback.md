# Trajectory playback: planner → file → the existing loop

Date: 2026-07-31
Status: accepted (software-verified; first hardware run pending)

## Decision

The GPMP2/GTSAM planner (`TrajectoryGeneration`) connects to the hardware
path as a **file handoff into a third controller**, not as a linked
dependency and not through Kortex's native trajectory APIs:

1. `TrajectoryGeneration/tools/plan_move` (standalone build, no Kortex)
   plans ONE free-space right-arm move offline and writes a
   `trajectory_format = 1` CSV — 7 joints in Kinova order, deg / deg/s,
   uniform implicit time in [0.5, 2] ms steps, rest at both ends
   (`basic_control/src/Trajectory.h` is the contract; it was
   `src/control/TrajectoryFile.h` when this was written).
2. `controller --controller playback --trajectory <file>` executes it
   through the UNCHANGED loop: velocity interface → per-joint clamp →
   PositionIntegration → 1 kHz cyclic stream, with every existing guard
   (following-error, faults, servoing, counters) live.

The playback law is feed-forward plus a small correction:
`q̇_d = Δq_ref(t, t+dt)/dt + kp·wrap(q_ref − q_meas)`. The feed-forward
uses exact reference-position differences over the measured dt window, so
the integrator telescopes onto the reference with zero discretization
drift; playback time advances by the same measured dt the integrator
uses, so timing jitter slows playback slightly rather than causing a
command jump. All reference-vs-measured comparisons are wrapped
(`std::remainder`) — the file's continuous angles never fight the arm's
[0,360) feedback.

## Why not the alternatives

- **Kortex-native (`PlayPreComputedJointTrajectory`)**: execution would
  move into the base's internal controller, bypassing every basic_control
  guard and the per-cycle CSV; zero in-repo precedent; no incremental
  bring-up path. Rejected for the milestone, open for later comparison.
- **Streaming planner waypoints into the reactive laws**: P-only tracking
  (kp = 1/s) lags a moving target by ~1 s with no timing guarantee, and
  abandons the joint-space path the optimizer actually validated.
- **Linking GTSAM/GPMP2 into the controller binary**: heavy dependencies
  in the hardware binary for no milestone benefit; planning must never
  run in the control callback anyway.

## Safety gates (all must pass before motion)

1. **Producer gate**: plan_move re-loads its own output and validates it
   with basic_control's own `TrajectoryFile.cpp` compiled in — one
   validation authority. FAIL → nonzero exit, file must not be executed.
2. **Consumer gate**: the controller re-validates before ANY hardware
   session: peak velocity ≤ 90% of the command clip (the clamp can never
   engage on a validated file — it stays as a backstop), acceleration ≤
   Kinova Table 43, wrapped position range for bounded joints 2/4/6,
   velocity/position consistency, per-sample steps, rest at both ends.
3. **Start-state gate**: measured position must match the trajectory's
   first row within `start_mismatch_limit_deg` (default 0.2°/joint) on
   the pre-takeover read; `TrajectoryPlayback::Reset` re-checks at the
   takeover and REFUSES into a permanent hold if the arm moved between.
4. **In-run**: the untouched 3° following-error stop, fault stops, and
   counters; completion = hold at the final reference until Ctrl+C.

Parameter evidence (run `loop_log_20260731_121752`): still-state tracking
error 0.006°, moving (~5 deg/s) 0.089°, dt jitter mean 1.115 ms / max
4.83 ms → `kStartMismatchLimitDeg = 0.2` (≈30× noise, 15× under the
guard), `kPlaybackKp = 0.5 /s` (correction < 0.05 deg/s in normal play).

`kPlaybackKp` is deliberately far below the `null_gain = 10` scale that
produced the 2026-07-27 windup incident. Feed-forward dominates in normal
play: the P term only absorbs the (gated, small) start offset and losses
to the velocity clamp, so there is nothing for it to wind up against.

`kStartMismatchLimitDeg` is checked twice — before the takeover in `Main`,
and again at `Reset`, because the arm can move in between. Failing the
second check is a permanent hold, not a retry.

## Telemetry

`log_format = 3` appends `ref_j1..7`, `playback_t_s`, `playback_state`
(0 none / 1 playing / 2 done / 3 refused) so planned, commanded and
measured motion are comparable per cycle from one CSV.

## Known scope limits

Free-space SDF only (no obstacles, no Vicon); single right arm; the
planner's DH model differs from the URDF (grasp-point tool frame), so
compare displacement NORMS between plan_move's printout and the
controller's FK cross-check, not absolute positions; the arm's CONFIGURED
soft limits are inside the model ranges — confirm them in the Kinova web
dashboard before every session.
