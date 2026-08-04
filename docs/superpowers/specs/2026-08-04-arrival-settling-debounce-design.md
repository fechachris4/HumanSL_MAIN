# Arrival settling debounce

Date: 2026-08-04
Branch: codex/restore-controller
Status: design — awaiting review before implementation

## Problem

`TrackingController` declares a target reached on the **first single control
cycle** whose measured position error falls to or below
`kArrivalToleranceM` (1 mm), with orientation within tolerance
(`Controller.cpp`, the `arrived_edge` block). A measurement is one 2 ms
snapshot. A single noisy sample, or the tool tip passing through the 1 mm
ball on its way to a small overshoot, is therefore enough to latch arrival,
start the 2 s hold, and advance the target queue from a pose the arm has not
actually settled at.

This is a **latent** defect. In the accepted validation run
(`runs/2026-08-04/validation_circuit_final.csv`, 185,966 cycles, two full
circuits) it never triggered: at all nine settling points the error entered
1 mm and stayed there with zero excursions, holding within ~0.31 mm worst
case. The fix hardens a signal that a clean run did not happen to break; it
matters when conditions are worse than that run — higher sensor noise, a
target near a singularity or joint limit where the arm oscillates around the
tolerance, or a marginally reachable typed target.

## What this does and does not solve

Solves: the positive "arrived" signal becomes trustworthy — it now means
"held within tolerance for a sustained window," not "touched tolerance once."

Does not solve, by design (separate future work):

- It does not help the arm reach targets it otherwise could not; it only
  avoids *falsely* declaring arrival.
- It adds no negative "this target will never be reached" signal. Detecting
  a stalled approach (arm parked short of the target) is a distinct
  timeout / stall guard, out of scope here.

## Decisions (settled during brainstorming)

- **Debounce on accumulated time, not a cycle count.** Robust to scheduler
  jitter; consistent with the existing 2 s target hold, which already
  accumulates measured `dt` in `Targets.cpp`.
- **Window = 0.15 s**, as a new `config::kArrivalDwellS`. Grounded in the
  validation record's measured closed-loop response lag of 144 ms: the
  arrival check reads *measured* pose, so the window must exceed the
  physical lag to confirm the real arm settled rather than only the command.
  0.15 s is ~7.5% of the existing 2 s hold. The telemetry shows the settled
  band stays within ~0.31 mm across any such window, so a genuine hold never
  false-resets the accumulator.
- **Adopt the interface already fixed by the uncommitted test.**
  `tests/test_control_logic.cpp` (working-tree diff, committed nowhere yet)
  already contains `TestArrivalSettling()`, which pins the
  `ArrivalSettlingMonitor` contract and `#include "Arrival.h"`. The
  implementation, the header, and the `main()` call do not yet exist, so the
  `test_control_logic` target currently fails to compile. This work finishes
  that.

## Components

### `src/Arrival.h` — `ArrivalSettlingMonitor` (new, header-only)

Pure, no allocation, no I/O; mirrors `Freshness.h`. Interface is exactly
what the existing test exercises:

- `explicit ArrivalSettlingMonitor(double hold_s)`
- `bool Update(bool in_tolerance, double dt_s)` — returns whether arrival is
  currently reported:
  - `in_tolerance == false` → reset accumulator to 0, return false (holds
    even when the gate is disabled).
  - `in_tolerance == true`, `hold_s <= 0` → gate disabled: return true
    (today's instantaneous behaviour stays reachable).
  - `in_tolerance == true`, `hold_s > 0`, `dt_s` finite and > 0 → accumulate
    `dt_s`; return `accumulated >= hold_s`.
  - `in_tolerance == true` but `dt_s` non-finite or ≤ 0 → neither accumulate
    nor reset; return current reported state.
- `double settled_s() const` — accumulated in-tolerance time.
- `void Rearm()` — reset accumulator to 0 for a new target.

### `src/Config.h`

Add `inline constexpr double kArrivalDwellS = 0.15;` beside
`kArrivalToleranceM`, commented with the 144 ms-lag rationale.

### `src/Controller.{h,cpp}` — integration

- Add an `ArrivalSettlingMonitor arrival_monitor_` member, constructed with
  `config::kArrivalDwellS`.
- The existing in-tolerance boolean
  (`arrival_eligible && position_arrived && orientation_arrived`) becomes the
  argument to `arrival_monitor_.Update(in_tolerance, dt_s)` each cycle.
- The existing "fire once per target" latch (`arrival_reported_`) and the
  sequence-change re-arm are retained; add `arrival_monitor_.Rearm()`
  alongside the existing re-arm so a new target starts from zero settled
  time. `Reset()` also re-arms and leaves the latch set, so the takeover
  hold pose still never fires arrival.
- `arrived_edge` fires on the rising edge of the monitor's reported state,
  preserving the once-per-target semantics `Runner`/`Targets` depend on.

## Data flow (unchanged except the gate)

Controller computes measured pose → position/orientation error →
in-tolerance boolean → **monitor** → `arrived_edge` → `Runner` prints
"target reached" and notifies `PoseTargetSource::OnArrived()` → 2 s dwell →
next queued target. Net effect: arrival fires ~0.15 s later; per-target time
becomes ~0.15 s settle + 2.0 s hold ≈ 2.15 s.

## Telemetry — open call for review

Recommended: **no new CSV column.** Arrival simply fires 0.15 s later; the
existing `arrival_error_m` still records the residual at the (now debounced)
arrival, and `sigma_min` / `lead_limited_j*` already expose why a slow
approach lingers. Adding a settled-time column would bump the CSV
`log_format` (currently 7) and force the Python parsers
(`scripts/runlog.py`, `analyze_run.py`, `test_runlog_compat.py`) to change
in lockstep — more blast radius than this fix warrants.

Reviewer decision: accept no-new-column, or add a `settled_s` column (and
own the log_format bump + parser updates) for stronger arrival evidence.

## Testing

- The existing `TestArrivalSettling()` covers the monitor: single-cycle
  rejection, threshold at 100 ms (its own 0.1 s fixture — independent of the
  0.15 s production constant), excursion reset, full-window re-earn, `Rearm`,
  bad-`dt` no-op, and the disabled-gate escape hatch. Wire it into `main()`.
- Build `test_control_logic` with warnings visible; run the CTest
  hardware-free suite.
- No hardware run. This change can only *delay* an arrival, never make
  motion more aggressive, so it cannot loosen a guard; supervised hardware
  confirmation of the end-to-end behaviour remains a separate, later step and
  is not claimed here.

## Safety classification

Level 2: arrival drives when the arm stops holding and advances to the next
target. No existing guard, fault policy, motion limit, or timing source is
weakened. `kArrivalToleranceM`, the following-error stop, joint-limit guard,
freshness stop, and `kStopOnFault` are untouched.

## Rollout

1. Implement to green on `codex/restore-controller`.
2. Full hardware-free suite + diff review.
3. Only then, with explicit confirmation, merge `codex/restore-controller`
   into `master`. Not before: the branch presently does not compile
   (`Arrival.h` missing), so it must not reach `master` until green.
