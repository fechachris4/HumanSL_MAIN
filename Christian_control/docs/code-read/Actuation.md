# Actuation.cpp / Actuation.h — line-by-line read

Actuation is the last software stage before the robot: it turns the
already-clamped joint velocity q̇ into this cycle's absolute position
setpoints by integration, and it owns the persistent command state that
requires (`q_command_rad_`). One class, `PositionIntegration`, plus one
free helper, `ClampedCycleDt`.

The header's lifecycle contract (Actuation.h:5-8) is the thing to
internalize before touching anything: `Prepare` and `Restore` MAY do
hardware I/O; `Apply` must be **pure** — no I/O, no blocking, no
allocation — because it runs inside every 2 ms cycle.

Execution order (who calls what):

1. `PositionIntegration actuation(config::kCommandLeadLimitDeg)` — Main.cpp:543
2. `ClampedCycleDt(...)` — Runner.cpp:191, every cycle after the first
3. `actuation.Prepare(state)` — Runner.cpp:166, at takeover (T4)
4. `actuation.Apply(...)` — Runner.cpp:270, every cycle
5. `actuation.Restore()` — Runner.cpp:401, on every exit path (D1)

(`TrackingErrorDeg` is declared here too but nothing in the run calls it —
see below.)

---

## `ClampedCycleDt` (Actuation.h:26-29)

```cpp
inline double ClampedCycleDt(double measured_dt_s, double nominal_dt_s)
{
    return std::min(measured_dt_s, 2.0 * nominal_dt_s);
}
```

- **What**: the dt the integrator may use — the *measured* elapsed cycle
  time, capped at twice the nominal 2 ms period. `inline` in a header
  means every .cpp may include this definition without a
  "multiple definition" link error.
- **Why**: if the scheduler stalls the loop for, say, 80 ms, integrating
  `q̇ · 0.080` in one step would command a large position jump — and the
  base ejects low-level servoing on jumps (~5°). Capping dt turns a stall
  into "the arm moved less than intended", which the next cycles correct.
- **If changed**: raising the cap re-opens the position-jump ejection;
  removing the cap entirely makes any OS hiccup a potential fault. Note the
  *overrun stop counter* in the Runner uses the un-clamped dt — clamping
  here hides the stall from the integrator, not from the record.

## Constructor (Actuation.cpp:18-22)

```cpp
PositionIntegration::PositionIntegration(double command_lead_limit_deg)
    : command_lead_limit_rad_(command_lead_limit_deg / kRadToDeg)
{}
```

- **What**: store the command-lead limit, converted to radians once, so all
  internal math is single-unit. The default argument (Actuation.h:49-50)
  is `std::numeric_limits<double>::infinity()` — "no limit" — which is why
  `Apply` tests `std::isfinite` before limiting. Main passes
  `config::kCommandLeadLimitDeg` = 1.0°.
- **Why the lead limit exists**: it bounds how far the integrated setpoint
  may run ahead of the wrapped measurement. With a frozen joint the
  setpoint stops 1° ahead instead of winding up. Config.h notes the
  side-effect: 1° is *below* the 3° following-error guard, so the guard
  can never fire while the limiter is active — nothing stops a frozen
  plant by itself. **FLAG `Actuation.cpp:18-22` | edit-hazard** — the
  constructor looks like unit bookkeeping, but the value it stores is the
  reason the following-error guard is effectively disabled; changing
  either constant without the other changes which protection can fire.

## `Prepare` (Actuation.cpp:24-29) — seed the command at takeover

```cpp
q_command_rad_ = state.q_rad;
```

- **What**: set the persistent command exactly where the robot measurably
  is. The ONLY time command state is seeded from measurement.
- **Why**: the first streamed position command must equal the current
  position or the servo would jump at takeover. The header
  (Actuation.h:53-57) is emphatic that the zero-initialized member is not a
  safe substitute — commanding "all joints to 0°" from an arbitrary pose
  would be a violent full-speed move.
- **If removed/reordered**: the Runner calls Prepare at T4 after the seed
  read; calling Apply first would integrate from zero — see above. This is
  the sharpest cliff in the file.

## `Apply` (Actuation.cpp:31-75) — the per-cycle integration step

Signature (Actuation.h:64-67): takes the clamped q̇ (rad/s), the measured
state, dt, and *writes into* two caller-owned `JointVector`s —
`setpoints_deg` (what gets sent) and `setpoint_velocity_deg_s` (what the
log records). Returns an `ApplyStatus` (Actuation.h:44-47): the
unconstrained "requested" setpoint per joint plus a `lead_limited` flag —
the requested-vs-sent half of the run record.

Per joint (the loop at 38-73):

- **Line 41** — `previous`: the last commanded position for this joint,
  from the persistent state.
- **Line 44** — `proposed = previous + q̇ · dt`: the Euler integration
  step. Note it integrates from the previous *command*, not from the
  measurement — that is what makes the command trajectory smooth and
  independent of feedback noise.
- **Line 47** — record `proposed` (in degrees) as `requested_deg` *before*
  the limiter can change it.
- **Lines 51-53** — the wrap trick. Feedback comes wrapped to one turn;
  the command is continuous. `std::remainder(a, 2π)` returns `a` reduced to
  (−π, π], so `measured_near` is the measurement shifted by whole turns to
  the turn *nearest the proposal*. Without this, a command of 359° against
  feedback of 1° would look like a 358° lead instead of 2°. First-time
  note: `std::remainder` differs from `std::fmod` — remainder rounds to
  the *nearest* multiple (result in ±half-modulus), fmod truncates
  (result keeps the sign of the argument). This file needs remainder
  semantics. **FLAG `Actuation.cpp:51-54` | edit-hazard** — swapping in
  `fmod`, or comparing raw wrapped feedback, reintroduces the 358°-lead
  bug; everything downstream (lead limiting, the logged velocities) sits
  on this shift.
- **Lines 54-63** — `lead = proposed - measured_near`; if a finite limit is
  configured and `|lead|` exceeds it, the command is re-anchored to
  `measured_near ± limit` (`std::copysign(limit, lead)` gives the limit
  with lead's sign) and the joint is flagged `lead_limited`. The command
  is thereby also pulled onto the same angular turn as the feedback.
- **Line 66** — otherwise accept the proposal.
- **Lines 70-71** — the logged velocity is derived from what was *actually
  applied*: `(new − previous)/dt`. It is smaller than the requested q̇
  exactly when the limiter intervened. The `dt_s > 0.0 ? ... : 0.0`
  guards a division by zero on a degenerate dt.
- **Line 72** — the setpoint in degrees, still continuous; wrapping to
  [0,360) happens later in `CyclicSession::Send`.

**FLAG `Actuation.cpp:31-75` | hides-work** — "Apply a velocity" actually
performs four jobs per joint: integration, turn-alignment of the feedback,
lead limiting (a safety behaviour), and derivation of the logged velocity.
The lead limiter in particular is a control-behaviour change (the arm slows
to feedback pace when leading) hiding inside what reads as bookkeeping.

## `TrackingErrorDeg` (Actuation.cpp:77-91)

- **What**: per joint, `|command − measured|` with the same
  whole-turn correction (`remainder(..., 360)` on degrees), returned as
  `std::optional<JointVector>`. First-time note: `std::optional<T>` is a
  box that either holds a `T` or holds nothing (`std::nullopt`) — a typed
  replacement for "return a special value to mean N/A". The header
  (Actuation.h:69-73) says nullopt would mean "no guard available".
- **Reality check**: this implementation always returns a value, and *no
  production code calls it* — the Runner's following-error stop works from
  the logged sample (`cmd_j*` vs `meas_j*`) inside `ClassifyStop`
  (Safety), and the only callers are tests
  (tests/test_control_logic.cpp:106,120). **FLAG `Actuation.cpp:77-91` |
  unnecessary** — dead in the run path; kept alive by tests. If the
  fixed-target simplification wants a smaller surface, this method and its
  optional-based contract can go once the tests assert on the Safety-side
  computation instead.

## `Restore` (Actuation.cpp:93-96)

- **What**: nothing. In POSITION mode the arm simply holds the last
  commanded setpoint, so there is nothing to undo before the
  ServoingGuard restores SINGLE_LEVEL.
- **Why it exists anyway**: the teardown contract (D1 before D2/D3 in the
  Runner) gives actuation a slot to undo hardware state, because *other*
  actuation strategies (the removed resolved-rate one commanded
  velocities) genuinely needed one. **FLAG `Actuation.cpp:93-96` |
  unnecessary** — a documented no-op kept for lifecycle symmetry; harmless,
  but with exactly one actuation left it is interface generality nothing
  uses.

## The private state (Actuation.h:80-84)

- `command_lead_limit_rad_` — set once in the constructor.
- `q_command_rad_` — a fixed-size Eigen vector
  (`Eigen::Matrix<double,7,1>`), zero-initialized at construction. This is
  THE persistent integrator state: every cycle's setpoint is one Euler
  step from it, and it is valid only between `Prepare` and teardown.
