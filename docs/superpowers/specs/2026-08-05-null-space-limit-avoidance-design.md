# Null-space joint-limit avoidance (replacing midpoint centering)

**Date:** 2026-08-05
**Status:** approved design, pending implementation plan
**Scope:** `Christian_control/basic_control` reactive-pose law, its Config, its
tests and fixtures, and the paired Python simulation law in
`~/msc_project/controller/reactive_controller.py`.

## Problem

The reactive controller needs a secondary (null-space) objective that keeps
the bounded joints 2/4/6 away from their software position limits without
fighting the Cartesian task. Evidence from the 2026-08-05 runs:

- **Centering off** (run 01:07): the task motion walked joint 6 to exactly
  −118.0° — its software boundary — and the joint-boundary guard stopped the
  run (`exit_reason = joint_limit_warning` at t = 2.3 s). Nothing steers the
  internal posture, so the DLS solution spends bounded joints past their
  limits.
- **Midpoint centering on, gain 23** (run 01:36): the arm froze in a stable
  equilibrium 218 mm short of the target. The centering error is ~2 rad *by
  design* in the working posture (j4 works around 90–116°, its midpoint is
  0°), and the damped projector (λ = 0.1) leaks a fraction
  ≈ λ²/(σ²+λ²) of the enormous centering velocity into task space; the loop
  parks where Kp·e_pos balances that leak. Memory record:
  `null-gain-23-stall-equilibrium`.

Conclusion: absolute midpoint centering is the wrong objective for this arm
and task — the useful configurations live far from the joint midpoints. The
objective must be **zero across the whole working range** and act only when a
joint approaches its limit.

## Decision (Approach A — linear-gradient deadband avoidance)

Gradient-projection limit avoidance with an activation zone
(Liégeois 1977-style secondary objective; activation-zone formulation as in
Chiaverini's redundancy chapter, Springer Handbook of Robotics). Weighted-DLS
(Chan & Dubey 1995) was considered and rejected because it replaces the
null-space channel rather than filling it, and touches the fixture-locked
task solve. A smoothstep-shaped push was rejected as an unneeded second shape
parameter at 500 Hz with the existing 1 s ramp.

Per bounded joint, with signed position `s = remainder(q, 2π)` (rad,
(−π, π]), software limit magnitude `L` (rad; 0 = unbounded joint), zone
width `zone` (rad), gain `k` (1/s):

```
z = L − zone                       # zone entry
objective = 0                      if L == 0 or |s| ≤ z
objective = −k · (|s| − z) · sign(s)   otherwise
```

The objective vector is projected through the **same damped projector as
today**: `q̇_null = N · objective`, `N = I₇ − Jᵀ(JJᵀ + λ²I₆)⁻¹J`.

Properties:

- Exactly zero everywhere inside the zone: no task fighting, no projector
  leak in normal operation (structurally removes the 218 mm stall mode).
- Push grows linearly with penetration; at the limit it equals `k · zone`
  (2.0/s × 20° ≈ 40 deg/s before projection — firm, bounded).
- Continuous at zone entry (value 0 there), so no velocity step.
- Beyond the limit the push keeps growing linearly; the per-joint 45 deg/s
  command clip and the 1 s null ramp bound it. No separate cap.

## Changes

### 1. ReactiveLaw.h

- **Remove** `NullSpaceVelocity` (wrap-to-midpoint objective).
- **Add** `LimitAvoidanceVelocity(jacobian, q_rad, limit_rad, zone_rad,
  gains)` implementing the deadband objective + damped projection above.
  `limit_rad` is a 7-vector of signed limit magnitudes, 0 = unbounded.
- **Rename** `ReactivePoseGains.null_gain_s_inv` →
  `limit_avoid_gain_s_inv`.
- `ReactiveSolution` / `SolveReactiveVelocityDetailed` /
  `SolveReactiveVelocity` keep their shape; the detailed solve calls the new
  objective. Task/null/leak decomposition semantics unchanged.
- Header comment: document that the null channel now carries limit
  avoidance only, and why centering was removed (reference this spec).

### 2. Config.h

- **Remove:** `kNullGain`, `kNullMidpointDeg`, `kNullCenteringMask`.
- **Add:** `kLimitAvoidZoneDeg = 20.0`, `kLimitAvoidGain = 2.0` (1/s).
- **Keep:** `kNullSpaceEnabled` — set **true** (enabling the channel is the
  point of the change); `kNullRampDurationS = 1.0` (protects a takeover that
  starts inside the zone, e.g. run 01:11 with j4 at 145°);
  `kJointSoftwareLimitDeg` as the anchor. The zone spans
  `[limit − 20°, limit]`, entirely inside the guard boundary.
- Zone/gain constants live next to the software-limit block with a comment
  linking objective, guard, and this spec.

### 3. Controller.cpp

- Constructor converts `kJointSoftwareLimitDeg` and `kLimitAvoidZoneDeg` to
  radians once (replacing the midpoint/mask conversion).
- The `UnitRamp(state.t_s, kNullRampDurationS)` multiplier applies to
  `limit_avoid_gain_s_inv` exactly as it did to the centering gain.

### 4. Main.cpp (CSV preamble)

- Drop the `null_gain` line; add `limit_avoid_zone_deg` and
  `limit_avoid_gain`. `null_space_enabled` line stays. No `log_format` bump:
  the column schema (format 8) is unchanged; only `#` preamble keys change.

### 5. Python simulation law (msc_project, outside this repo)

- Replace midpoint centering with the identical deadband objective in
  `controller/reactive_controller.py`; add zone/gain to its config object.
- Fix the stale `MSC_PROJECT` path in
  `basic_control/scripts/gen_reactive_fixtures.py`
  (`~/Projects/Code/msc_project` → `~/msc_project`).
- Regenerate `tests/reactive_fixtures.h`. New cases: (a) one bounded joint
  inside its zone → active push, 1e-6 tolerance with tiny fixture damping
  (projector-damping deviation unchanged); (b) all joints outside their
  zones → output must equal the null-disabled case to 1e-12 (objective is
  exactly zero, not merely small).

### 6. Telemetry — no schema change

`taskvel_j*`, `nullvel_j*`, `null_leak_mps` (log_format 8) and the 1 Hz
status line already report whatever the null channel carries. Expected
post-change reading: `null=0.0 leak=0.000` in the working range; a burst only
when a bounded joint enters its zone. This is also the on-hardware acceptance
check.

### 7. Tests (TDD, hardware-free)

New law tests in `tests/test_reactive_law.cpp` (written first, must fail for
the expected reason before implementation):

- objective exactly zero for every bounded joint inside its zone (norm == 0);
- push is inward on the positive side (`s > z` → negative objective) and on
  the negative side (`s < −z` → positive objective);
- magnitude at the limit equals `k · zone`;
- unbounded joints (limit 0) contribute nothing at any position;
- with an undamped projector the projected result stays in the Jacobian null
  space (existing property, retargeted);
- `SolveReactiveVelocityDetailed` parts still sum to the total; leak twist is
  the Jacobian image of the projected push; disabled → zero null part.

Removed with the feature: the centering-specific tests (wrap-to-midpoint,
masked-joint cases). Fixture cross-validation (`TestAgainstSimulationFixtures`)
must pass after regeneration. Supervisor/status-line tests untouched.

## Safety review notes (Level 2 — motion law change)

- The software joint-boundary guard, firmware JOINT_LIMIT thresholds, and
  all stop policies are **untouched backstops**. Avoidance makes boundary
  holds unlikely, never impossible: a strong task demand can still saturate
  through the clip; the guard then holds the frame as before.
- Frames/units: objective computed on signed joint positions in rad,
  `remainder(q, 2π)`; limits converted from the deg constants once; row
  order linear-then-angular unchanged; gain in 1/s.
- No new real-time cost concerns: the objective is 7 scalar comparisons plus
  the same projector solve as before, and it short-circuits to zero outside
  the zones.
- Offline evidence: law tests + regenerated fixtures + full build + CTest.
- Hardware validation (separate, requires explicit authorization): Christian
  present, workspace clear, e-stop in hand; watch `null=`/`leak=` stay 0.0 in
  the working range and the push engage inside a zone without a
  `joint_limit_warning` stop on the 01:07 scenario.
- Interacting uncommitted edits noted at design time: `kTakeoverHoldS = 0.05`
  (Christian's, kept) and `kNullGain = 23` with `kNullSpaceEnabled = false`
  (superseded/deleted by this change).

## Out of scope

- Manipulability-gradient ascent (singularity-configuration avoidance) — may
  become a second null-space objective later; σ_min telemetry already exists
  to justify it with data.
- Weighted-DLS task solve, reach/workspace validation of typed targets, and
  the planned Python-backend reference source (`controller-simplification-plan`).
