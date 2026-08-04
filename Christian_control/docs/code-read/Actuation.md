# Actuation.cpp / Actuation.h — line-by-line read

*(Current source map: line references below are checked against HEAD.)*

Actuation is the last software stage before the robot: it turns the
already-clamped joint velocity q̇ into this cycle's absolute position
setpoints by integration, and it owns the persistent command state that
requires (`q_command_rad_`). One class, `PositionIntegration`, plus one
free helper, `ClampedCycleDt`.

`PositionIntegration` owns no hardware lifecycle at HEAD: `Prepare` seeds
its persistent command state outside the loop and `Apply` is **pure** — no
I/O, no blocking, no allocation — because it runs inside every 2 ms cycle.

Execution order (who calls what):

1. `PositionIntegration actuation(config::kCommandLeadLimitDeg)` — Main.cpp:397
2. `ClampedCycleDt(...)` — Runner.cpp:184–186, every cycle after the first
3. `actuation.Prepare(state)` — Runner.cpp:160, at takeover (T4)
4. `actuation.Apply(...)` — Runner.cpp:255–257, every cycle
5. `servoing_guard.Restore(std::cout)` — Runner.cpp:390, after the loop

There is no `PositionIntegration::TrackingErrorDeg()` or
`PositionIntegration::Restore()` at HEAD. Following-error classification is
performed by `Safety::ClassifyStop` on the command and feedback recorded by
the Runner; servoing teardown belongs to `ServoingGuard`.

---

## `ClampedCycleDt` (Actuation.h:24-35)

```cpp
inline double ClampedCycleDt(double measured_dt_s, double nominal_dt_s)
{
    if (!std::isfinite(measured_dt_s) || !std::isfinite(nominal_dt_s) ||
        measured_dt_s <= 0.0 || nominal_dt_s <= 0.0)
        return 0.0;
    return std::min(measured_dt_s, 2.0 * nominal_dt_s);
}
```

- **What**: the dt the integrator may use — the *measured* elapsed cycle
  time, capped at twice the nominal 2 ms period; non-positive or non-finite
  measured or nominal values fail safe to zero. `inline` in a header means
  every .cpp may include this definition without a "multiple definition"
  link error.
- **Why**: if the scheduler stalls the loop for, say, 80 ms, integrating
  `q̇ · 0.080` in one step would command a large position jump — and the
  base ejects low-level servoing on jumps (~5°). Capping dt turns a stall
  into "the arm moved less than intended", which the next cycles correct.
- **If changed**: raising the cap re-opens the position-jump ejection;
  removing the cap entirely makes any OS hiccup a potential fault. Note the
  *overrun stop counter* in the Runner uses the un-clamped dt — clamping
  here hides the stall from the integrator, not from the record. A direct
  invalid input cannot reverse or poison an integration step.

## Constructor (Actuation.cpp:18-22)

```cpp
PositionIntegration::PositionIntegration(double command_lead_limit_deg)
    : command_lead_limit_rad_(command_lead_limit_deg / kRadToDeg)
{}
```

- **What**: store the command-lead limit, converted to radians once, so all
  internal math is single-unit. The default argument (Actuation.h:57-58)
  is `std::numeric_limits<double>::infinity()` — "no limit" — which is why
  `Apply` tests `std::isfinite` before limiting. Main passes
  `config::kCommandLeadLimitDeg` = 1.0°.
- **Why the lead limit exists**: when excessive lead is detected, the lead
  projection targets a command exactly 1° from the wrapped measurement. It
  is followed by a final envelope
  that limits the sent delta to `abs(qdot_clamped * dt)`. On discontinuous
  feedback the envelope wins, so recovery can temporarily leave actual lead
  above 1°; the unchanged 3° following-error stop is then reachable as the
  backstop. **FLAG `Actuation.cpp:18-22` | edit-hazard** — the constructor
  looks like unit bookkeeping, but changing this value changes the recovery
  target and its interaction with the following-error guard.

## `Prepare` (Actuation.cpp:24-29) — seed the command at takeover

```cpp
q_command_rad_ = state.q_rad;
```

- **What**: set the persistent command exactly where the robot measurably
  is. The ONLY time command state is seeded from measurement.
- **Why**: the first streamed position command must equal the current
  position or the servo would jump at takeover. The header
  (Actuation.h:60-65) is emphatic that the zero-initialized member is not a
  safe substitute — commanding "all joints to 0°" from an arbitrary pose
  would be a violent full-speed move.
- **If removed/reordered**: the Runner calls Prepare at T4 after the seed
  read; calling Apply first would integrate from zero — see above. This is
  the sharpest cliff in the file.

## `Apply` (Actuation.cpp:31-124) — the per-cycle integration step

Signature (Actuation.h:76-79): takes the clamped q̇ (rad/s), the measured
state, dt, and *writes into* two caller-owned `JointVector`s —
`setpoints_deg` (what gets sent) and `setpoint_velocity_deg_s` (what the
log records). Returns an `ApplyStatus` (Actuation.h:45-54): the
unconstrained "requested" setpoint per joint plus a `lead_limited` flag —
the requested-vs-sent half of the run record. It also names a bounded joint
whose final command would move farther outward past its warning threshold;
in that case it holds the complete prior command frame for the Runner to
send once and stop on.

Before the loop, direct non-positive or non-finite dt is mapped to zero; the
normal Runner steady-clock path supplies a positive dt. Per joint (the loop
at 42-88):

- **Line 45** — `previous`: the last commanded position for this joint,
  from the persistent state.
- **Lines 48-50** — `proposed = previous + q̇_clamped · effective_dt`: the Euler
  integration step from the already-clamped velocity. It integrates from the
  previous *command*, not from the measurement — that is what makes the
  command trajectory smooth and independent of feedback noise.
- **Lines 51-53** — record `proposed` (in degrees) as `requested_deg`
  *before* either command constraint can change it.
- **Lines 54-59** — the wrap trick. Feedback comes wrapped to one turn;
  the command is continuous. `std::remainder(a, 2π)` returns `a` reduced to
  (−π, π], so `measured_near` is the measurement shifted by whole turns to
  the turn *nearest the proposal*. Without this, a command of 359° against
  feedback of 1° would look like a 358° lead instead of 2°. First-time
  note: `std::remainder` differs from `std::fmod` — remainder rounds to
  the *nearest* multiple (result in ±half-modulus), fmod truncates
  (result keeps the sign of the argument). This file needs remainder
  semantics. **FLAG `Actuation.cpp:54-59` | edit-hazard** — swapping in
  `fmod`, or comparing raw wrapped feedback, reintroduces the 358°-lead
  bug; everything downstream (lead limiting, the logged velocities) sits
  on this shift.
- **Lines 60-69** — `lead = proposed - measured_near`; if a finite limit is
  configured and `|lead|` exceeds it, the lead candidate is re-anchored to
  `measured_near ± limit` (`std::copysign(limit, lead)` gives the limit
  with lead's sign) and the joint is flagged `lead_limited`. The flag means
  this projection was active, not necessarily that it won the final command.
- **Lines 72-79** — the final envelope clamps that candidate to
  `previous ± abs(qdot_clamped * dt)`. It wins when a discontinuous feedback
  sample would otherwise demand a larger jump, so the sent command may stay
  more than 1° from measurement until later cycles recover the lead.
- **Lines 82-108** — before committing candidates, every bounded joint with
  a non-zero warning is checked in signed degrees. `std::remainder` maps
  Kortex raw 355° to −5° and maps the candidate delta to its nearest turn,
  so a representation wrap cannot resemble a position-limit crossing. A
  candidate only trips when it is beyond ±warning *and* still moves farther
  outward; a joint already beyond warning may move inward. On a trip,
  **all** setpoints retain their previous command and all applied velocities
  are zero; no persistent command state is advanced.
- **Lines 111-118** — the logged velocity is derived from what was *actually
  applied* after both constraints: `(new − previous)/effective_dt`. Invalid
  direct dt is zero, so it reports an exact hold rather than dividing or
  hiding movement.
- **Line 118** — the setpoint in degrees, still continuous; wrapping to
  [0,360) happens later in `CyclicSession::Send`.

**FLAG `Actuation.cpp:31-124` | hides-work** — "Apply a velocity" actually
performs six jobs per joint: integration, turn-alignment of the feedback,
lead projection, final rate limiting, joint-warning frame hold, and
derivation of the logged velocity. `req_j - cmd_j` therefore shows the
combined effect of the command constraints, not only lead recovery.

## The private state (Actuation.h:80-84)

- `command_lead_limit_rad_` — set once in the constructor.
- `q_command_rad_` — a fixed-size Eigen vector
  (`Eigen::Matrix<double,7,1>`), zero-initialized at construction. This is
  THE persistent integrator state: every cycle's setpoint is one Euler
  step from it, and it is valid only between `Prepare` and teardown.
