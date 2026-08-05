# ReactiveLaw.h — line-by-line read

**Entry point:** header-only — there is no .cpp. In the running program the
only caller is `TrackingController::DesiredVelocity`
(Controller.cpp:144–152), which calls `UnitRamp` and then
`SolveReactiveVelocityDetailed` once per pose-channel cycle;
`SolveReactiveVelocityDetailed` in turn calls `TaskTwist`,
`DampedLeastSquares6` and (when enabled) `LimitAvoidanceVelocity`.
`RotationLog` and `TwistError` are called directly by
the controller (Controller.cpp:87–88, 97). Execution order per cycle is
therefore: equations 1–2 in the controller, then 3–6 in here — the same
order the file's banner lists them.

This file is the *math*, deliberately isolated: Eigen only, fixed-size
matrices, no allocation, no robot, no Pinocchio — which is exactly what lets
`tests/test_reactive_law.cpp` cross-validate it against the Python law
(`reactive_controller.py`) on identical inputs. The rule at line 16: change
equations here, change gains in Config.h — never mix the two.

### Lines 1–24 — the banner
The six-equation summary, the porting provenance, and the two DELIBERATE
deviations from the simulation law. **[edit-hazard, lines 18–24]** Both
deviations are hardware-truth encodings, not style: (a) the null-space
projector is damped because the sim's undamped pseudoinverse is
ill-conditioned exactly where DLS is protecting the task solution; (b) the
deadband limit-avoidance objective wraps to (−π, π] because Kortex reports
positions in [0, 360) while MuJoCo's q is continuous. "Simplifying" either
back to match Python re-introduces the hardware failure each one prevents.

### Lines 26–35 — includes
Eigen Dense + Geometry (AngleAxis lives in Geometry), and State.h solely for
the `Twist` struct that equation 2 subtracts.

### Lines 37–49 — `struct ReactivePoseGains`
Plain data: five gains with their units in the comments (the `_s_inv` suffix
means 1/s — a proportional gain that converts an error in meters or radians
into a velocity), the DLS damping λ, and four enable switches. The design
point (line 37): a disabled term contributes *exactly zero*, so the staged
bring-up (P-only → +Kd → +limit avoidance) is pure configuration, no code
edits.
All members have default values, so `ReactivePoseGains gains;` is a valid
P-position-only baseline with zero gains. **[edit-hazard, line 45]**
`position_enabled` defaults true and is the one switch
`ConfiguredGains()` in Controller.cpp never assigns — it has no Config.h
constant. It works today by defaulting; anyone wiring a config toggle for it
must touch both files.

### Lines 51–59 — `UnitRamp`
A clamped linear ramp 0→1 over `duration_s`; `duration_s <= 0` means "no
ramp, full strength immediately". `std::clamp(x, lo, hi)` returns x limited
to [lo, hi]. Used only to fade the null-space gain in over the first second
after takeover (Controller.cpp:122–123), so takeover cannot begin with a
full projected joint transient — the *task* law is never ramped. All the
functions here are `inline`: required for functions *defined* in a header
(otherwise every .cpp including it would define a duplicate symbol and the
link would fail), and a hint that they are small enough to embed at the call
site.

### Lines 61–68 — `RotationLog` (equation 1, rotation part)
The SO(3) logarithm: turn a rotation matrix into the 3-vector
angle × axis (direction = rotation axis, length = rotation angle in
radians). Implemented with Eigen's `AngleAxisd` constructor instead of
Pinocchio's `log3` — same math, no dependency. The comment carries the
load-bearing fact: AngleAxis returns angle ∈ [0, π], so the error can never
describe "the long way around" — a 350° error comes out as 10° about the
opposite axis. The controller feeds it `R_desired · R_measuredᵀ`, the
rotation still needed.

### Lines 70–85 — `TwistError` (equation 2)
`reference − measured`, split into linear (rows 0–2) and angular (rows 3–5)
halves with Eigen's `.head<3>()` / `.tail<3>()` (compile-time-sized views of
the first/last three entries). Row order matches the Jacobian's — stated
twice in this file because mixing the halves is the classic silent bug. The
second paragraph of the comment is the behavioural insight: with the
default-zero reference `Twist{}` this reduces to −measured, i.e. the Kd term
becomes pure damping toward standstill (what every current source produces);
a *moving* target must supply its own twist here or the damping term will
fight the very motion the target asked for (State.h:68–73 repeats this
warning at the `PoseReference` definition).

### Lines 87–104 — `TaskTwist` (equation 3)
Assembles the desired 6-vector [v; ω]: start from zero, add
`Kp_pos · e_pos` into the linear half if position is enabled,
`Kp_rot · e_rot` into the angular half if orientation is enabled, then `+=`
the Kd contributions on both halves if the velocity term is enabled. Note
position/orientation *assign* while velocity *adds* — the Kd term stacks on
top of the P term; a disabled switch leaves its rows exactly zero, honoring
the "disabled = contributes zero" contract.

### Lines 106–117 — `DampedLeastSquares6` (equation 4)
The damped pseudoinverse solve: q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ẋ, but computed without
ever forming an inverse — build the 6×6 `JJᵀ`, add λ² down the diagonal
(`jjt.diagonal().array() += lambda*lambda` — `.array()` switches Eigen to
element-wise mode so `+=` scalar works), then `jjt.ldlt().solve(twist)`.
LDLT is a Cholesky-family factorization for symmetric matrices: solving the
linear system directly is faster and numerically better behaved than
computing an inverse and multiplying. All sizes are compile-time (6×7, 6×6),
so nothing allocates — safe inside the 2 ms cycle. What λ buys, per the
comment: near a singular pose the undamped solution's joint velocities blow
up toward infinity; λ trades a little tracking accuracy for a solution that
stays finite everywhere. This is the safety-relevant line of the whole file.

### Lines 119–128 — `DampedLeastSquares` (3×7 translational variant)
Same construction for a position-only 3×7 Jacobian. **[unnecessary, lines
119–128]** Unnecessary *for this binary*: the comment itself says the
control path uses the 6-DoF version above; this one exists for the
direction-probing tool. It is dead weight in the controller's read, kept
here so the tool shares one verified implementation — reasonable, but know
that deleting it breaks the probe tool, not the controller.

### Lines 131–169 — `LimitAvoidanceVelocity` (equations 5–6)
The secondary objective, replacing wrap-to-midpoint centering on 2026-08-05
(design rationale:
`docs/superpowers/specs/2026-08-05-null-space-limit-avoidance-design.md` —
centering fought the task from everywhere and the damped projector leaked it
into task space, a 218 mm stall equilibrium). It is a *deadband*: exactly
zero everywhere except a small zone approaching each bounded joint's
software limit, using only motion that does not disturb the end-effector
task.

- **147–160 — equation 5, the deadband objective.** Per joint, skip if
  `limit_rad[i] <= 0` (the sentinel for an unbounded joint — continuous
  joints 1/3/5/7 always have `kJointSoftwareLimitDeg == 0`). Otherwise wrap
  the position with `std::remainder(q_rad[i], 2π)` to (−π, π], compute
  `excess = |signed_rad| − (limit_rad[i] − zone_rad)`, and only if
  `excess > 0` (inside the zone) set `objective[i] = −gain · excess`,
  signed back toward zero. Outside the zone `objective[i]` stays exactly
  `0.0` — this is what makes the term inert across the whole working range,
  unlike centering's everywhere-nonzero pull toward a midpoint.
  **[edit-hazard, lines 154–159]** Drop the wrap and a joint reported at
  350° gets measured as if it were 10° short of 0° rather than 10° past
  360° — the hardware deviation (b) from the banner, easy to lose in a
  refactor because the un-wrapped version looks cleaner and matches the
  Python.
- **163–167 — equation 6, the damped projector.**
  `N = I₇ − Jᵀ(JJᵀ + λ²I)⁻¹J`: multiplying any joint velocity by N removes
  the component that would move the end-effector, leaving pure internal
  ("null-space") motion. Damped with the same λ as the task solve —
  deviation (a) from the banner. **[unnecessary, lines 163–167]** Note it
  rebuilds `JJᵀ + λ²I` and refactors it, even though `DampedLeastSquares6`
  just solved the identical system in the same cycle — duplicated fixed-size
  work, kept because each function is self-contained and testable in
  isolation. A shared factorization would be faster but couple the
  functions; at 6×6 the cost is negligible, so this is a "know it's
  duplicated" flag, not a "fix it" one.
- **168** — return `N · objective`: the limit-avoidance velocity, projected.

### Lines 185–226 — `SolveReactiveVelocityDetailed` / `SolveReactiveVelocity` (equations 3–6 composed)
`SolveReactiveVelocityDetailed` (the function the controller actually calls,
Controller.cpp:150) builds the task twist (eq 3), solves DLS for the task
joint velocity (eq 4), and, only if `null_space_enabled`, adds the projected
limit-avoidance velocity (eqs 5–6), keeping the two terms separate in a
`ReactiveSolution` for telemetry. `SolveReactiveVelocity` is the thin
sum-only wrapper. Returns the raw q̇ — the
signature comment repeats the layering contract one last time: **the Runner
clamps, the actuation integrates — never this law.** This function has no
idea speed limits exist; if that division of labour ever blurs (a clamp
added here, or the Runner's clamp removed on the assumption the law is
bounded), the single-speed-limit property of the program is gone.
