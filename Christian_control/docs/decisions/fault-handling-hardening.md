# Fault-handling hardening (2026-07-20)

Four safety-behavior changes from a critical review of `basic_control`,
chosen to make the program smaller and fail louder rather than more
defensive.

## Startup no longer clears faults

`main.cpp` called `base->ClearFaults()` unconditionally before motion — a
pattern inherited from `TrajectoryExecution/src/KinovaTrajectory.cpp`. It
erased protective state (collision, over-torque, e-stop) without anyone
looking at it, immediately before the default startup mode moves the arm.

The call is deleted, with no replacement logic. Clearing a fault is now a
deliberate human act (Kinova web dashboard).

**Correction (2026-07-20, same day):** the original rationale claimed a
faulted arm refuses `LOW_LEVEL_SERVOING`. Hardware evidence disproved this
for the base's `JOINT_FAULT` (16) summary bit: with that bit latched, every
actuator bank clear and the arm `SERVOING_READY`, the Kinova Web App moves
the arm normally — the bit is a latched historical aggregate, not a live
interlock. What actually blocked our moves that day was our own fault check
treating any nonzero base bank as fatal (both aborted moves logged exactly
one CSV row). See "Fault policy refinement" below.

## The servoing-mode restore is guarded

The single SINGLE_LEVEL restore at the end of `move_joints_relative` is a
network call, and the path most likely to reach it in a bad state (link died
mid-move) was the one where it threw — past main's per-joint report and plot
step. It is now wrapped in try/catch: on failure the operator is warned that
the arm may still be in low-level servoing, and the move returns false. The
function's contract is again "reports by return value, never throws past
validation".

## `report_faults` also checks `arm_state`

The arm's own "you are no longer the controller" signal
(`arm_state != ARMSTATE_SERVOING_LOW_LEVEL`) was logged every cycle but
never acted on; detection waited up to 50 ms for the tracking-error filter.
The check is folded into the existing `report_faults` guard, so the loop
gains no new branch. Evidence for the exact comparison: a healthy 7 s move
log (`move_log_2026-07-14_19-42-38.csv`, 7001 cycles) reports state 6
(`ARMSTATE_SERVOING_LOW_LEVEL`) on every cycle including the first, so no
startup-transient allowance is needed. The tracking watchdog remains — it
catches the distinct failure of servoing-but-not-following.

## `time_control_cycle` is removed

It was dead code (never called from `main`), the only Timing function that
commanded the arm, held its own unguarded LOW_LEVEL entry/restore, and
carried a fourth hand-rolled copy of the frame-id/command-id stamping that
`Motion::send_positions` exists to centralize. Deleting it makes the
diagnostics layer read-only. If the benchmark is ever needed again, time
`send_positions` holding the current pose — that measures the real control
path, stamping included.

Alongside these, `main.cpp` now refuses to start a move or a recording if
the log file cannot be opened (a silently failed `ofstream` previously meant
a hardware move with zero evidence recorded).

## Fault policy refinement (2026-07-20, later the same day)

`report_faults` originally aborted on any nonzero base fault bank. That
treated the latched `JOINT_FAULT` summary bit as a live fault and blocked
moves on a healthy arm. The stop condition is now live-signal based:

- abort immediately on: any actuator fault bit, any base fault bit other
  than `JOINT_FAULT` (e-stop, emergency line, safety events), the arm
  leaving `ARMSTATE_SERVOING_LOW_LEVEL`, or a rejected/failed cyclic
  exchange (existing exception path);
- base `JOINT_FAULT` with all actuator banks clear: printed once as a
  diagnostic note, move continues, bit is never cleared by the program.

`move_joints_relative` also gained a staged start: (1) an unchanged holding
command must be accepted cleanly before anything moves; (2) every moving
joint is probed with a tiny slow move (<= 0.2 deg at 2 deg/s,
`kProbeDeltaDeg`/`kProbeSpeedDegS`) under the same watchdogs; (3) only then
does the requested ramp run, continuing from the probe's end. A
not-actually-ready arm is thus caught within 0.2 deg.

Comparison with Kinova's official low-level examples: their sequence is
identical to ours (set LOW_LEVEL, init command from feedback, cyclic
Refresh) except they call `ClearFaults()` unconditionally first — which is
why the samples never encounter the latched summary bit. We keep not
clearing; the Web App moves in SINGLE_LEVEL, which the latched bit does not
gate either.
