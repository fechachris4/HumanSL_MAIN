# Obstacle-Aware Graded Planner

**Date:** 2026-08-22  
**Status:** Approved
**Classification:** C — planner motion-generation and emission behaviour  
**Evidence level:** source audit, recorded-run diagnosis and hardware-free replay;
physical execution remains unproven

## 1. Objective

The planner must answer a well-formed request with motion whenever its bounded
search finds a model-valid executable trajectory.

- A point goal may take any model-valid route to the requested pose.
- A Cartesian trace is a soft geometric objective. It may deviate locally to
  avoid obstacles and should return to the requested trace, with the requested
  terminal pose exact when a validated exact-terminal route is found.
- If bounded search finds no validated exact-terminal route but does find a
  validated shortened route, the planner emits that shortened route and reports
  the shortfall as `GOAL_BLOCKED`.
- If it finds no validated executable route, it returns `FAILED` and no
  trajectory.

Obstacle factors shape candidate trajectories inside GPMP2. Dense validation
is the final model-validity gate, not the primary obstacle-handling mechanism.
This is a bounded local search, so neither `GOAL_BLOCKED` nor `FAILED` proves
that the requested pose is globally unreachable.

The design also preserves the approved start-state contract: every candidate
begins at the measured state that caused the request, and no fallback may move
that state.

This design narrowly supersedes the start-state specification's “no target
change” non-goal only for a validated, explicitly reported `GOAL_BLOCKED`
terminal. Exact start position, supplied start velocity, controller continuity
and every other start-state decision remain unchanged.

## 2. Evidence and current contradictions

The current source does not implement this contract:

- `SolveToPosition()` uses one soft terminal workspace prior. It can miss the
  requested pose and point plans receive only `ValidateJointPath()`; their
  modelled obstacle clearance is not computed.
- `SolveAlongPath()` uses obstacle factors during optimization, but current
  verdict code treats negative modelled clearance as a warning. A colliding
  traced path can therefore be emitted.
- Intermediate trace error is measured, but stale names and comments still
  call the old millimetre threshold a gate. A valid obstacle detour cannot be
  both allowed and rejected for leaving the original curve.
- `PlanOutcome` and `PathPlanOutcome` expose `bool ok`; solver completion,
  model validity, task completion and transport success are therefore easy to
  conflate.
- A successful exact terminal IK seed is selected before route feasibility is
  known, although different redundant-arm IK branches can have different
  collision-free routes.

The supervised left-arm session
`runs/2026-08-22/session_213028` is the pre-change physical characterization.
It proved exact start-state preservation but produced no motion: the traced
plan was rejected after the optimizer moved far from the requested path under
a conflicting torso-cylinder model. Sparse path IK residuals were small, but
the final GPMP2 trajectory was not. The run therefore distinguishes endpoint
or waypoint IK feasibility from full route feasibility.

## 3. Physical and mathematical contract

### 3.1 Frames, state and time

All planning variables remain in the planner's Mount frame `M` until the
existing world projection boundary. The requested and emitted TCP is `E`.

```text
M_T_E(q) = M_T_base * base_T_E(q)
```

Joint vectors use seven-element Kinova actuator order. Position is radians,
velocity is radians per second, translation is metres and planner time is
seconds from the immutable request snapshot.

For every candidate:

```text
q(0)    = C(q_meas)
qdot(0) = qdot_meas       when supplied by the same live feedback snapshot
```

`C` is the one existing principal-angle canonicalization. An offline request
without measured velocity has no start-velocity equality; absence never means
zero. The asynchronous result remains a reference, not measured state.

### 3.2 Modelled collision feasibility

Let `P` be the configured set of prohibited pairs `(robot sphere, named scene
object)`. For candidate trajectory `q(t)`, modelled feasibility requires

```text
clearance_p(q(t)) >= obstacles.minimum_clearance_m  for every p in P and dense t
```

`minimum_clearance_m` is the hard modelled boundary. The existing obstacle
factor activation distance becomes `preferred_clearance_m`: it shapes routes
and ranks validated candidates, but clearance benefit saturates at that value.
This keeps clearance important without rewarding a much larger detour for
negligible clearance beyond the preferred margin. Configuration requires
`0 <= minimum_clearance_m <= preferred_clearance_m`; both are explicit metres,
not hidden constants.

This is explicitly modelled-scene evidence. It is not a claim of human
clearance, unmodelled cable clearance or physical-world safety.

The current anonymous union SDF cannot represent this rule: after taking the
minimum over objects, it has lost object identity and cannot permit one
base/attachment overlap while retaining every forearm/torso check. It is
therefore replaced by one plain obstacle-field record per enabled named
object. Each record contains that object's existing primitive geometry, its
SDF, and a filtered GPMP2 arm model containing only the participating spheres.
The filtered model reuses the same arm kinematics and copied sphere metadata;
it is not a second robot model.

The scene entry carries `permitted_sphere_groups`, required and empty by
default. Stable groups separate `mount_interface` from `proximal_arm`,
`upper_arm`, `forearm` and `tool`; their mapping to the existing authored
spheres is owned beside `GenerateArmModel`. This avoids permitting an entire
proximal link merely because one mounting-interface sphere may intentionally
overlap the torso model. For every obstacle, all groups not explicitly
permitted are prohibited. A permission requires a Christian-confirmed physical
reason. No overlap is auto-permitted because the measured start contains it.

```yaml
minimum_clearance_m: 0.05
preferred_clearance_m: 0.10
scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.24
    height_m: 1.0
    permitted_sphere_groups: []
```

The initial 0.05 m hard value preserves the already-approved boundary; 0.10 m
is the first capped preference to benchmark, not a new claim about physical
human clearance. Change either only from recorded route and tracking evidence.

Optimization adds one scene-obstacle factor per object using that object's
filtered arm model. Dense validation independently evaluates every prohibited
sphere against the same named primitive. A pure primitive query returns signed
distance, outward surface normal and obstacle identity for the existing box
and finite vertical cylinder geometry; deterministic scene order breaks equal
distance ties. That same query identifies the first blocking object used by
route seeding and by evidence. The current combined `MakeMountSdf()` path is
removed rather than retained beside the per-object path.

The existing explicit self-collision sphere pairs remain separate from scene
pairs. Their configured separation is evaluated densely and is mandatory for
an executable result. Scene-collision escalation does not change the
self-collision weight.

### 3.3 Terminal semantics

An exact terminal candidate must satisfy all three conditions:

```text
final FK translation error <= 0.001 m
final FK orientation error <= 0.01 rad
joint position limits
terminal prohibited-pair clearance
```

IK generates candidate configurations, but exactness is decided from
`M_T_E(q(T))` after optimization, never from the IK seed's internal residual.
Each exact terminal configuration is imposed as a joint-space equality at the
last support state, so GPMP2 cannot silently move away from the accepted IK
branch. The same terminal equality applies to the chosen shortened candidate.
The generator retains at most three deterministic, joint-space-distinct IK
branches. Route feasibility is evaluated before choosing among them.

If no exact candidate produces a validated route, shortened candidates are
generated. Their default ranking is lexicographic, without a hidden
translation/rotation scalar:

1. modelled scene/self-collision, joint-position, velocity and acceleration
   feasibility are mandatory for a candidate after its bounded repair attempts;
2. candidates preserving requested orientation within tolerance form the
   first tier; within that tier minimize terminal position error;
3. only when that tier is empty, minimize orientation error first and position
   error second;
4. report both shortfalls.

The shortened generator is bounded and explicit. It evaluates requested
orientation at 16 fixed position fractions from the requested terminal back
toward the measured start TCP, nearest first. For each named object that blocks
an exact terminal or exact route, it also evaluates the primitive query's
nearest outward-clearance projection and the two opposing tangent points. The
same fixed IK seed streams are used for every target. If no exact-orientation
candidate survives, safe near-miss configurations returned by those same IK
attempts form the relaxed-orientation tier. Every tested target, IK stream and
residual is recorded; at most three joint-space-distinct terminal
configurations are retained after the ranking above.

This searches a declared finite set, not the continuous pose space. The report
says `best_validated_bounded_candidate` and
`closest_found_position_distance`, never “globally closest reachable”. A
future task may explicitly choose position-priority fallback; this design adds
no such option or hidden default.

### 3.4 Cartesian trace semantics

For ordinary current path tasks, intermediate requested poses contribute soft
position and orientation objectives:

```text
minimize  J_path_position + J_path_orientation + J_smoothness + J_obstacle
subject to the approved exact start state
```

Modelled clearance, finite values, effective joint-position, velocity and
acceleration limits, and terminal correctness are final executable conditions.
They disqualify one candidate; they do not end the request while another route,
terminal branch or shortened candidate remains. Intermediate path deviation is
quality evidence, not a rejection condition. A detoured circle that reaches its
requested terminal pose is `REACHED`, not blocked.

The planner reports mean, RMS, p95 and maximum position deviation, maximum
orientation deviation, worst path parameter and terminal rejoin error. Current
point requests have a free route plus the terminal rules in section 3.3;
current circle requests have soft intermediate geometry plus those terminal
rules. A future hard-constrained path is a separate task type and behaviour
proposal; this migration adds no flag for it.

## 4. One bounded candidate search

There remains one GPMP2 implementation. No planner manager, retry service,
fallback class hierarchy or second motion planner is introduced.

### 4.1 Candidate tuple

One candidate is the plain tuple:

```text
(terminal kind, terminal IK branch, route hypothesis, duration)
```

The route hypothesis owns its normal or bypass scene-collision strength; there
is no independent collision-scale search dimension. The candidate pool is
deterministic. IK may use reproducible seed streams to discover different
terminal branches, but random perturbations do not count as different routes.
Wall-clock expiry does not determine which IK seeds exist: fixed attempt caps
and recorded seed/stream indices make the same request reproduce the same
candidate pool.

### 4.2 Exact phase

Exact-terminal candidates are tried first. The generator retains at most three
terminal IK branches, ordered by joint displacement from the measured start
and then by deterministic seed stream. For each branch it tries at most three
route hypotheses:

1. the normal interpolation or requested-trace seed;
2. a deterministic bypass on one side of the first blocking obstacle;
3. the geometrically opposite bypass.

To construct bypass seeds, the planner samples the normal seed, uses the
per-object primitive query from section 3.2 to identify the first prohibited
sphere/object penetration and its outward normal, and constructs opposing
tangential detours. For point goals, the detour supplies a Cartesian midpoint
that is solved to joint space. For a trace, it locally offsets only the
colliding seed region before continuation IK. The original task poses remain
the GPMP2 objectives. These are seed hypotheses; dense validation, not seed
geometry, decides feasibility.

The two bypass hypotheses use `0.1 * obstacles.collision_sigma` for named-scene
factors while self-collision retains the configured base sigma. This is one
declared collision-specific escalation, not a generic retry or extra tuple
dimension. Bypass hypotheses are entered only when the normal candidate's
dense failure includes prohibited scene collision.

For one terminal branch, all entered route hypotheses are evaluated and the
best validated route is selected. A later terminal branch is entered only if
the preceding branch produced no validated route. Thus each phase attempts at
most three terminal branches times three route hypotheses: nine route
candidates. Each route candidate may have the duration attempts in section
4.4, making the worst case 27 GPMP2 solves per phase and 54 across exact plus
shortened phases. The attempt table reports both actual and maximum counts.

These are initial runtime budgets, not reliability claims. Candidate success,
clearance and solve time are reviewed after offline benchmarking. A budget
change is a later explicit tuning decision, not an automatic consequence of a
failed benchmark.

### 4.3 Shortened phase

The shortened phase begins whenever the entire bounded exact phase produces no
validated exact-terminal trajectory. This includes both:

- no valid exact terminal IK candidate; and
- valid exact terminal candidates for which every bounded route attempt fails.

Shortened terminal candidates use the ranking in section 3.3, retain at most
three distinct terminal configurations, and use the same maximum of three
deterministic route hypotheses per terminal. A validated shortened route
produces `GOAL_BLOCKED`; exhausted shortened search produces `FAILED`. This
applies to point and traced requests: a shortened trace reports terminal
shortfall and cannot claim rejoin.

### 4.4 Duration attempts

Duration repair is orthogonal to route search. A geometrically valid candidate
is re-solved from scratch at a longer duration when dense measurement finds a
time-dependent effective velocity or acceleration violation.

- Maximum three total duration attempts for that candidate.
- Exact `q(0)` and supplied `qdot(0)` are re-applied on every attempt.
- No post-hoc velocity scaling.
- Each longer-duration proposal targets a dynamic ratio of `0.999`:

  ```text
  duration_scale = max(max_velocity_ratio / 0.999,
                       sqrt(max_acceleration_ratio / 0.999))
  ```

  This margin applies only to the next fresh solve. Dense validation retains
  the hard executable boundary `max_velocity_ratio <= 1` and
  `max_acceleration_ratio <= 1`.
- Collision, joint-position, terminal, IK, non-finite or optimizer failure does
  not trigger a duration attempt; the bounded candidate search continues where
  another candidate remains.
- If measured `qdot(0)` itself exceeds the effective velocity limit, return
  `FAILED` immediately with `start_velocity_over_effective_limit`. Every
  candidate must preserve that same exact measurement, so no branch, route,
  shortened target or duration can repair it. Never replace the measurement or
  manufacture zero velocity.
- A candidate still above either effective dynamic limit after three attempts
  is abandoned; this does not become whole-request `FAILED` until all exact and
  shortened candidates are exhausted.

The official [Kinova Gen3 7-DoF User Guide](https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf)
documents hardware speed limits of 1.39 rad/s
for joints 1–4 and 1.22 rad/s for joints 5–7, and hardware acceleration limits
of 5.2 rad/s² for joints 1–4 and 10.0 rad/s² for joints 5–7. The repository
stores these physical tables separately from derived operating limits:

```text
effective_velocity_j     = velocity_planner_fraction     * hardware_velocity_j
effective_acceleration_j = acceleration_planner_fraction * hardware_acceleration_j
```

Fractions are explicit configuration, dimensionless, in `(0, 1]`. They are not
buried literals. The checked-in acceleration fraction begins at `1.0`; any
future headroom reduction requires measured tracking evidence and a separate
approved tuning change. The existing 7-DoF grouping must not be replaced with
the 6-DoF grouping. The dense acceleration estimate is now a documented
physical planning boundary and participates in duration repair and final
candidate feasibility.

## 5. Validation and outcomes

Point and traced plans use the existing dense validation path before world
projection. It samples the final timed GPMP2 trajectory on the configured
uniform `path_following.validation_dt_s` grid, including both endpoints; no
second validator or wire-format reconstruction is introduced. It measures:

- exact measured start position and supplied start velocity;
- finite joint states and timestamps;
- prohibited-pair clearance under the same scene policy used by GPMP2;
- configured self-collision-pair separation;
- bounded-joint position limits;
- per-joint effective velocity limits;
- per-joint effective acceleration limits, used for duration repair and final
  candidate feasibility;
- terminal pose correctness for the candidate type;
- trace quality metrics when a requested path exists.

The old `PlanVerdict::{Accept, Warning, Reject}` gate and both `bool ok` fields
are replaced rather than wrapped. Both existing outcome structs carry one
shared plain status:

```text
REACHED       validated trajectory, exact requested terminal pose
GOAL_BLOCKED  validated trajectory, deliberately shortened terminal pose
FAILED        no executable trajectory
```

Only `REACHED` and `GOAL_BLOCKED` may carry a trajectory. `FAILED` carries the
stage and reason but no default or stale trajectory. Optimizer convergence,
factor costs, IK failures, route-seed failures and path deviation are search or
quality evidence; none is an independent whole-request rejection gate. Exact
start is imposed by equality. A measured mismatch after solving is therefore
classified as an invalid numerical trajectory, not as a second configurable
protective threshold.

Weighted GPMP2 objectives are recorded but never compared across route
hypotheses with different collision strengths. Define route clearance quality
as `min(minimum_clearance, preferred_clearance_m)`, so extra distance beyond the
preferred margin cannot justify a larger detour. Among validated routes for one
exact point terminal, select greatest capped clearance quality, then shortest
duration, then smallest integrated joint travel. For a trace, compute position
RMS on a common, uniformly sampled requested path parameter so duration and
sample count cannot bias the number; select lowest RMS, then lowest maximum
deviation, then greatest capped clearance quality. Shortened routes first use
the terminal ranking in section 3.3 and then the corresponding route rule. All
selected routes have already passed the same hard model-validity conditions.

## 6. Data flow and ownership

```text
PlanningRequest(q_meas, qdot_meas, task, world_T_mount, provenance)
  -> validate measured start over prohibited pairs
  -> generate bounded exact terminal/route candidates
  -> GPMP2 solve + dense validation per candidate
  -> if no exact route: bounded shortened terminal/route candidates
  -> choose validated trajectory
  -> REACHED | GOAL_BLOCKED | FAILED
  -> world Cartesian projection only for executable outcomes
  -> existing atomic planner/controller handoff
```

`PlanSolver` owns candidate ordering, solve attempts and final selection.
Existing IK code owns IK calculations. One replacement scene conversion owns
sphere-group expansion, per-object SDF construction and the primitive
distance/normal query. Dense validation owns executable model checks and
quality measurements. `BridgeMain` owns reporting, artifact writing and
projection/handoff. Nothing enters the 500 Hz core until the existing
validated world-trajectory boundary.

## 7. Candidate and run evidence

Every candidate records, in one machine-readable attempt table:

- exact or shortened terminal kind;
- terminal IK branch identity and pose residual;
- normal/positive-bypass/negative-bypass route identity;
- collision scale and duration attempt;
- optimizer convergence, final factor costs and solve time;
- minimum prohibited-pair clearance, pair identity and time;
- joint/dynamic extrema;
- terminal position and orientation error;
- trace mean, RMS, p95 and maximum deviation where applicable;
- final candidate disposition and precise reason.

The final plan metadata derives from the selected outcome after validation. It
must not say `ok` for a rejected or failed attempt.

Using the existing archived-artifact plotting path, planner work produces the
following evidence-backed visuals from the same request, scene, candidate
table, plan and controller log:

1. requested, planned and measured TCP paths with named obstacle geometry;
2. prohibited-pair clearance versus time, with the required line;
3. planned and measured `q` and `qdot`, including start mismatch;
4. candidate outcome, solve time, clearance and rejection reason.

Plots expose failures and remaining limits as well as the selected result. No
new panel planning, FK, SDF or collision code is part of this migration.

## 8. Lean implementation and removal contract

Expected owners, subject to the implementation plan's source audit:

- `PlanSolver.*`: candidate loop, duration-only repair and selection;
- existing IK/initialization files: bounded terminal branches and bypass seed
  construction, without a second solver abstraction;
- `StaticScene.*`, `PlannerConfig.*`, `MountSdf.*` and arm-sphere definition:
  stable prohibited-pair configuration, capped clearance policy, per-object
  fields and one conversion path;
- existing joint-limit loader/generator: hardware acceleration values and the
  explicit planner fraction, producing effective per-joint acceleration limits
  beside the existing velocity derivation;
- the existing `ValidatePath.*` and `PathValidationReport.*`, renamed rather
  than wrapped: common dense point/path validation and quality metrics;
- `BridgeMain.cpp` and existing debug artifacts: statuses, candidate table and
  projection only for executable outcomes;
- existing panel scene-config parser/editor: preserve and edit the required
  `permitted_sphere_groups` field without adding collision calculations;
- existing archived-artifact plotting path: visualization only, without a new
  panel planner implementation.

Replacement ledger:

```text
PlanOutcome::ok / PathPlanOutcome::ok        -> shared outcome status
PlanVerdict Accept/Warning/Reject            -> dense validity + outcome status
point-only ValidateJointPath emission gate   -> common dense validation
path-error rejection/gate semantics          -> quality metrics only
collision-warning emission semantics         -> prohibited-pair hard validation
single prematurely selected terminal seed    -> bounded terminal/route tuples
combined anonymous scene SDF                  -> named per-object obstacle fields
shared scene/self collision factor argument  -> explicit scene and self sigmas
```

Old production branches, configuration keys, panel fields, tests, comments,
includes and documentation are removed in the same migration. No compatibility
aliases or duplicate legacy outcomes remain. Historical run artifacts and
dated decision records remain labelled history.

Concept budget:

- add one shared outcome enum, one plain candidate-evidence struct, one plain
  named-obstacle-field record and one stable collision-sphere-group vocabulary;
- add no manager, service, registry, factory, plugin system, event bus or
  generic retry framework;
- prefer extending the existing start-state test and validation tests over
  creating many executables;
- the minimality reviewer may request deletion or consolidation, never more
  protective machinery.

## 9. Verification before hardware

Existing `session_213028` and the archived point failure remain independent
pre-change evidence. The implementation must not rely only on tests it creates.

One focused planner behaviour fixture set covers distinct contracts:

1. obstructed direct point route reaches the exact pose through a validated
   alternative route, while an explicitly permitted mount-interface pair is
   ignored and a prohibited proximal-arm pair is still enforced;
2. obstructed trace detours, rejoins the terminal pose and is not rejected for
   intermediate path error;
3. exact terminal IK exists but bounded exact routes fail; shortened search
   returns a validated `GOAL_BLOCKED` trajectory;
4. no validated shortened route returns `FAILED` with no trajectory;
5. nonzero measured start velocity remains exact through alternative routes
   and duration repair; a synthetic velocity/acceleration excess causes a
   longer from-scratch solve and the final candidate meets both effective
   dynamic limits.

Expected clearance and endpoint residuals are recomputed from primitive scene
geometry and FK outputs, not by calling the verdict helper under test. Mutation
checks must fail if collision is changed back to warning-only, an exact-route
failure skips shortened search, path quality becomes a generic rejection, or
the measured start is replaced.

Run the focused tests, the affected hardware-free planning/runtime suites, a
fresh build of the planner and controller targets, and offline replay of the
archived requests. Benchmark all candidate attempts and plot route diversity,
clearance, task deviation and solve time. Report whether the initial budgets
produce genuinely distinct search coverage at acceptable latency; changing a
budget remains a separate approved tuning decision.

## 10. Supervised physical validation

Offline success does not complete this motion change. The first physical run
is a small clear-space point move on one identified arm. It must prove that an
accepted plan activates, the arm physically moves, and planned/measured start
state and tracking remain continuous. A later modelled-obstacle run occurs only
after the enabled scene geometry and any permitted collision pairs match the
observed supported rig.

Each run follows the engineering contract's hardware authorization gate and
the current runtime runbook: the exact command, arm/IP, target and expected
motion, active limits, stop conditions and recovery path are stated; Christian
confirms supervision, clear workspace and reachable e-stop; then the agent
runs that exact command. Any planner `FAILED`, controller continuity rejection,
fault, unexpected direction, following error, stale world state or operator
stop ends the attempt without improvising another motion.

Completion evidence is the archived supported-rig session plus plots showing:

- nonzero measured motion after trajectory activation;
- exact planner start state and accepted controller splice;
- requested, planned and measured TCP trajectories;
- planned-versus-measured joint position and velocity;
- modelled prohibited-pair clearance and all remaining warnings.

Compilation, optimizer success, trajectory emission or command acknowledgement
alone is not physical proof.

## 11. Limiting cases and falsifiable predictions

1. **Clear exact point:** the normal exact candidate validates and returns
   `REACHED`; shortened search is never entered.
2. **Blocked direct route:** at least one obstacle-side seed differs
   geometrically from the normal seed and can validate without changing the
   terminal pose.
3. **Detoured trace:** intermediate path error rises in the obstacle region,
   then falls on rejoin; the result is `REACHED` if the terminal pose is exact.
4. **Exact IK, no bounded exact route:** all exact attempts are recorded,
   shortened search runs, and a valid shortened route returns `GOAL_BLOCKED`.
5. **Terminal inside prohibited geometry:** no exact terminal candidate
   validates; a shortened result stops with nonnegative required clearance and
   reports both pose shortfalls.
6. **Permitted mounting overlap:** a configured permitted pair contributes no
   obstacle factor and no validation failure, while every prohibited pair is
   still measured.
7. **Genuine prohibited start collision:** no candidate solve runs and the
   request returns `FAILED` with the pair identity.
8. **Moving replan:** every exact and shortened attempt preserves supplied
   `qdot(0)`; substituting zero is detectable.
9. **Dynamic-only excess:** a longer-duration re-solve changes the trajectory
   while preserving the exact start and ends within documented effective
   velocity and acceleration limits; collision or optimizer failure produces
   no duration retry.
10. **Local-search limitation:** a `GOAL_BLOCKED` or `FAILED` report lists the
    exhausted bounded candidates and never says the exact goal was proven
    unreachable.

## 12. Explicit non-goals

- No global completeness or globally closest-pose claim.
- No generic “GPMP2 failed, retry” mechanism.
- No second planner or replacement of GPMP2.
- No controller-law, world-projection, atomic handoff or 500 Hz change.
- No planned joint posture crossing the pose/twist-only controller boundary.
- No automatic scene geometry, permitted-pair or physical TCP inference.
- No human-clearance claim from the static planner model.
- No hard-constrained-path mode in this migration.
- No guarantee that every syntactically valid request is physically possible.

The intended reliability statement is narrower and testable: any request for
which the bounded planner finds a validated exact or shortened trajectory is
represented honestly and sent through the existing controller and safety path;
only the absence of a validated result within the declared bounded search
yields no motion.
