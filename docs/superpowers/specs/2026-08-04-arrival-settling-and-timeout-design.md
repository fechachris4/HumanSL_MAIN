# Arrival settling debounce and non-arrival timeout

Date: 2026-08-04
Branch: codex/restore-controller
Status: design — awaiting review before implementation

## Problem

`TrackingController` handles target arrival with a single positive signal and
no negative one:

1. **Untrustworthy positive signal.** It declares a target reached on the
   **first single control cycle** whose measured position error falls to or
   below `kArrivalToleranceM` (1 mm), orientation within tolerance
   (`Controller.cpp`, the `arrived_edge` block). A measurement is one 2 ms
   snapshot. A single noisy sample, or the tool tip passing through the 1 mm
   ball on its way to a small overshoot, is enough to latch arrival, start the
   2 s hold, and advance the target queue from a pose the arm has not actually
   settled at.

2. **No negative signal.** If the arm parks short of a target and never
   reaches tolerance — an unreachable typed target, a pose near a singularity
   or joint limit where it oscillates just outside 1 mm, or a quiet stall —
   the source stays in its await-arrival phase **forever, silently**. Nothing
   tells the operator the controller is not reaching the target.

Both are **latent**. In the accepted validation run
(`runs/2026-08-04/validation_circuit_final.csv`, 185,966 cycles, two full
circuits) neither triggered: at all nine settling points the error entered
1 mm and stayed there with zero excursions, holding within ~0.31 mm worst
case, and every target was reached essentially instantly after its motion
profile completed. This work hardens signals that a clean run did not happen
to break; they matter when conditions are worse — higher sensor noise, a
target near a singularity or joint limit, or a marginally reachable typed
target.

## What this does and does not solve

Solves:

- The positive "arrived" signal becomes trustworthy — it now means "held
  within tolerance for a sustained window," not "touched tolerance once."
- The operator gains a negative signal: when the arm is parked at a target
  and has not reached it within a timeout, the run **announces** it.

Does not solve, by design:

- It does not help the arm reach targets it otherwise could not. The positive
  gate only avoids *falsely* declaring arrival; the negative gate only
  *reports* a failure to arrive.
- It does not attempt recovery. The chosen response to non-arrival is
  report-and-hold (below), not retry, re-plan, or skip.

## Decisions (settled during brainstorming)

- **Two sibling monitors, not one.** The positive check
  (`ArrivalSettlingMonitor`) has its interface already fixed by an
  uncommitted test; the negative check (`ArrivalTimeoutMonitor`) is new. They
  are kept separate so the existing test's contract is not disturbed. Both are
  pure, header-only, and live in `src/Arrival.h`. The negative signal depends
  on the positive one: "did not arrive" is only meaningful against a
  trustworthy "arrived."

- **Debounce on accumulated time, not a cycle count.** Robust to scheduler
  jitter; consistent with the existing 2 s target hold, which already
  accumulates measured `dt` in `Targets.cpp`.

- **Settling window = 0.15 s**, as `config::kArrivalDwellS`. Grounded in the
  validation record's measured closed-loop response lag of 144 ms: the arrival
  check reads *measured* pose, so the window must exceed the physical lag to
  confirm the real arm settled rather than only the command. 0.15 s is ~7.5%
  of the existing 2 s hold. Telemetry shows the settled band stays within
  ~0.31 mm across any such window, so a genuine hold never false-resets the
  accumulator.

- **Non-arrival timeout reuses the existing `config::kTargetHoldS` (2.0 s)**,
  adding no new constant. Measured from the moment the arm is parked and
  waiting at a target (its motion profile complete), **not** from target
  activation — so a far target's normal travel time never trips it. Telemetry
  shows real arrivals complete essentially instantly after profile completion,
  making 2.0 s a >10× margin with effectively no false-alarm risk, while still
  surfacing a genuine stall within that time. The reuse encodes a symmetric
  rule — the arm waits up to one hold-time to arrive, then holds for one
  hold-time — and keeps Config.h from growing a second number. Caveat,
  documented at the constant: because the two durations are now coupled,
  shortening `kTargetHoldS` also shortens the non-arrival timeout; if it were
  ever cut near the settling window it could raise false "not reached"
  reports. That failure is benign (it only reports early; it never moves the
  arm) and is called out in a comment at `kTargetHoldS`.

- **Response to non-arrival: report and keep holding.** On timeout, print and
  log a one-time "target NOT reached" message with the residual distance, and
  otherwise change nothing — the arm keeps holding the target reference and
  the queue does not advance. This requires **no change to motion or the phase
  machine**: a target that is never reached already holds indefinitely today;
  the timeout only adds the announcement. The following-error stop, joint
  guard, freshness stop, and `kStopOnFault` remain active, and hardware runs
  are supervised with the e-stop present, so an indefinite benign hold is
  acceptable. The operator decides whether to Ctrl+C, wait, or intervene.

- **Adopt the interface already fixed by the uncommitted test.**
  `tests/test_control_logic.cpp` (working-tree diff, committed nowhere yet)
  already contains `TestArrivalSettling()`, which pins the
  `ArrivalSettlingMonitor` contract and `#include "Arrival.h"`. The
  implementation, the header, and the `main()` call do not yet exist, so the
  `test_control_logic` target currently fails to compile. This work finishes
  that and adds the timeout monitor and its test.

## Components

### `src/Arrival.h` (new, header-only)

Pure, no allocation, no I/O; mirrors `Freshness.h`.

**`ArrivalSettlingMonitor`** — interface is exactly what the existing test
exercises:

- `explicit ArrivalSettlingMonitor(double hold_s)`
- `bool Update(bool in_tolerance, double dt_s)` — returns whether arrival is
  currently reported:
  - `in_tolerance == false` → reset accumulator to 0, return false (holds even
    when the gate is disabled).
  - `in_tolerance == true`, `hold_s <= 0` → gate disabled: return true
    (today's instantaneous behaviour stays reachable).
  - `in_tolerance == true`, `hold_s > 0`, `dt_s` finite and > 0 → accumulate
    `dt_s`; return `accumulated >= hold_s`.
  - `in_tolerance == true` but `dt_s` non-finite or ≤ 0 → neither accumulate
    nor reset; return current reported state.
- `double settled_s() const` — accumulated in-tolerance time.
- `void Rearm()` — reset accumulator to 0 for a new target.

**`ArrivalTimeoutMonitor`** — new; mirror-image structure:

- `explicit ArrivalTimeoutMonitor(double timeout_s)`
- `bool Update(bool waiting, double dt_s)` — `waiting` means "parked at a
  target and not yet arrived" (arrival-eligible and arrival not reported).
  Returns a **one-shot** edge: true only on the cycle the accumulated waiting
  time first reaches `timeout_s`.
  - `waiting == false` → reset elapsed to 0, return false (arrival happened or
    no target is parked).
  - `waiting == true`, `timeout_s <= 0` → gate disabled: never fire, return
    false.
  - `waiting == true`, `timeout_s > 0`, `dt_s` finite and > 0 → accumulate
    `dt_s`; if it crosses `timeout_s` and has not already fired for this
    target, latch fired and return true; otherwise return false.
  - `waiting == true` but `dt_s` non-finite or ≤ 0 → neither accumulate nor
    reset; return false.
- `double waited_s() const` — accumulated waiting time.
- `void Rearm()` — reset elapsed to 0 and clear the fired latch for a new
  target.

### `src/Config.h`

Add **one** new constant, in the existing arrival block beside
`kArrivalToleranceM`:

- `inline constexpr double kArrivalDwellS = 0.15;` — commented with the
  144 ms-lag rationale.

No second constant: the non-arrival timeout reuses `kTargetHoldS`. Add a
comment at `kTargetHoldS` noting it also serves as the non-arrival timeout, so
the coupling is visible to anyone changing it. A non-positive value would
disable the timeout, matching the freshness-guard convention, but the arrival
block does not expose an independent knob for it by design.

### `src/State.h` — `ControllerStatus`

Add one field, `bool not_reached_edge = false`, set true on the cycle the
timeout monitor fires (parallel to the existing `arrived_edge`). Default false
so no other reader is affected.

### `src/Controller.{h,cpp}` — integration

- Add members `ArrivalSettlingMonitor arrival_monitor_` (constructed with
  `config::kArrivalDwellS`) and `ArrivalTimeoutMonitor timeout_monitor_`
  (constructed with `config::kTargetHoldS`).
- The existing in-tolerance boolean
  (`arrival_eligible && position_arrived && orientation_arrived`) becomes the
  argument to `arrival_monitor_.Update(in_tolerance, dt_s)` each cycle.
- `arrived_edge` fires on the rising edge of the settling monitor's reported
  state, preserving the once-per-target semantics `Runner`/`Targets` depend
  on (the existing `arrival_reported_` latch and sequence-change re-arm are
  retained).
- `waiting` = `arrival_eligible && !arrival_reported_`; feed
  `timeout_monitor_.Update(waiting, dt_s)` each cycle and set
  `status.not_reached_edge` on its returned edge. Once `arrived_edge` has
  fired for a target, `waiting` is false, so the timeout cannot fire for a
  target that was reached.
- On sequence change (new target) call both `arrival_monitor_.Rearm()` and
  `timeout_monitor_.Rearm()` alongside the existing re-arm. `Reset()` re-arms
  both and leaves the arrival latch set, so the takeover hold pose fires
  neither arrival nor timeout.

### `src/Runner.cpp` — reporting

Beside the existing `if (status.arrived_edge)` block that prints
"target reached", add a branch: when `status.not_reached_edge`, print a
one-time "target NOT reached: <residual> mm short after <timeout> s (holding;
Ctrl+C to abort)" using `status.arrival_error_m`/`p_desired`/`p_current`
already on `status`. No queue or motion action is taken.

## Data flow

Controller computes measured pose → position/orientation error →
in-tolerance and waiting booleans → **monitors** → `arrived_edge` /
`not_reached_edge` → `Runner` prints "target reached" (and notifies
`PoseTargetSource::OnArrived()` → 2 s dwell → next queued target) or prints
"target NOT reached" (and does nothing else; the arm keeps holding). Net
effect on a normal arrival: it fires ~0.15 s later; per-target time becomes
~0.15 s settle + 2.0 s hold ≈ 2.15 s. Net effect on a stalled target: within
~2.0 s the operator is told, and the arm holds as it does today.

## Telemetry

**No new CSV column; `log_format` stays 7.** Arrival fires 0.15 s later and a
non-arrival is announced on stdout; the per-cycle target and actual positions
(`pd_x..z`, `p_x..z`) are already logged every row, so both the settled
residual and a stalled approach are fully reconstructable offline, and
`sigma_min` / `lead_limited_j*` already expose *why* an approach lingers.
Adding a column would bump `log_format` and force the Python parsers
(`scripts/runlog.py`, `analyze_run.py`, `test_runlog_compat.py`) to change in
lockstep — more blast radius than this warrants.

## Testing (hardware-free, test-first)

- **Positive:** the existing `TestArrivalSettling()` covers
  `ArrivalSettlingMonitor` — single-cycle rejection, threshold at its own
  0.1 s fixture (independent of the 0.15 s production constant), excursion
  reset, full-window re-earn, `Rearm`, bad-`dt` no-op, and the disabled-gate
  escape hatch. Wire it into `main()` and implement the monitor to pass it.
- **Negative:** write a new `TestArrivalTimeout()` first, then implement
  `ArrivalTimeoutMonitor` to pass it. Cases: no fire before the threshold;
  a single one-shot edge at the threshold with no re-fire while still waiting;
  `waiting` going false before the threshold (arrival first) never fires;
  `Rearm` resets elapsed and the fired latch; non-finite/≤0 `dt` neither
  accumulates nor resets; non-positive `timeout_s` disables the gate. Wire it
  into `main()`.
- Build `test_control_logic` with warnings visible; run the CTest
  hardware-free suite.
- **No hardware run.** The positive change can only *delay* an arrival; the
  negative change only *reports* and changes no motion. Neither can loosen a
  guard or make motion more aggressive. Supervised hardware confirmation of
  the end-to-end behaviour remains a separate, later step and is not claimed
  here.

## Safety classification

Level 2: arrival drives when the arm stops holding and advances to the next
target. No existing guard, fault policy, motion limit, or timing source is
weakened. `kArrivalToleranceM`, the following-error stop, joint-limit guard,
freshness stop, and `kStopOnFault` are untouched. The timeout path issues no
command and no stop; it only sets a status flag the Runner prints.

## Rollout

1. Implement to green on `codex/restore-controller` (settling monitor +
   timeout monitor + wiring + both tests).
2. Full hardware-free suite + diff review against this spec.
3. Only then, with explicit confirmation from Christian, merge
   `codex/restore-controller` into `master`. Not before: the branch presently
   does not compile (`Arrival.h` missing), so it must not reach `master` until
   green.
