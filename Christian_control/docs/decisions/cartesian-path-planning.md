# Cartesian path planning, end to end

Learning record, 2026-08-07. Follows the protocol in the repository root
`CLAUDE.md`. Supersedes nothing; extends `cartesian-path-tasks.md`, which
covered the geometry and the reachability probe.

## What this adds

A circle described in `goal.yaml` is now planned, emitted, reconstructed with
the controller's own interpolator, densely validated, and released only if
every modelled check passes.

## The pipeline, and why it is measured where it is

    circle spec  ->  CartesianPath  ->  continuation IK (PathIk)
                 ->  assembled waypoints (free approach + constrained task)
                 ->  GPMP2 solve  ->  emit (<=1000 points)
                 ->  CONTROLLER's cubic-Hermite reconstruction at 500 Hz
                 ->  dense validation  ->  gate

Validation runs on the LAST step, not the GPMP2 output. Subsampling to the
wire cap discards points and Hermite is not the GP interpolant, so both
distortions land after the optimiser has finished. Measuring the optimiser's
own result would certify the planner's intention rather than the arm's
motion. `ReconstructBlock.cpp` calls `JointTrajectoryAccumulator::Feed` and
`SampleJointTrajectory` directly — the controller's real code, not a
reimplementation of it.

Three errors are reported and one is the gate:

- `e_planner` — desired vs GP-dense
- `e_command` — desired vs the 500 Hz reconstruction **(gated)**
- `e_reconstruction` — GP-dense vs reconstruction (wire transport loss)

They are reported separately, not summed: a fidelity failure is attributable
to the optimiser or to the wire format, and the two have different fixes.

## Confirmed bug: GPMP2 assumes a uniform support interval

**Symptom.** The first working end-to-end circle reported 110 mm of path
error and was refused, while the probe had certified the same circle at
0.11 mm.

**Evidence.** The report's own decomposition located it: 104 mm out-of-plane
against 8 mm radial, with the worst point at path parameter 0.000 and
t = 2.000 s — exactly the approach's minimum duration. The error was
perpendicular to the circle and at the very start of the traced window,
which is the signature of measuring the wrong time window rather than of a
badly traced circle.

**Cause.** `densifyTrajectory` interpolates with a single
`delta_t = total_duration / total_steps`. The assembly had given the
approach 2 s over 6 states and the circle 12 s over 14 — non-uniform. GPMP2
stretched every segment equally, so the emitted timestamps were not the
requested ones and the validator measured the approach as though it were the
circle.

**Fix.** Both phases are placed on ONE uniform grid: the task path's own
sample spacing sets the interval, and the approach is given the whole number
of intervals covering its required duration (which can only lengthen it,
never shorten it below its floor). `AssembleCirclePlan` now verifies the grid
and throws rather than emitting timestamps GPMP2 would silently change.

**Result.** Same circle: 110 mm → **2.4 mm**, `hardware_execution_allowed`.

## Two further bugs found the same way

- The point-goal diagnostic dereferenced `parsed.goal` on the path route,
  printing uninitialised memory (`goal: [6.9e-310, ...]`). Guarded.
- The path route had no `CoutRedirectGuard`, so the legacy optimiser's
  `Creating arm trajectory...` was the first line of the emitted block — and
  in the real binary stdout IS the controller's pipe. Guarded, and verified
  by checking the block's first line is `TRAJ_BEGIN`.

## Deliberately reported, not gated: closure drift

Closure drift measures how far the redundant elbow ends from where it began
while the tool pose closes exactly. Measured across identical runs it varies
between 0.04° and 14°, while the traced circle stays at 2.406–2.409 mm.

Gating on it would fail plans that trace perfectly well, so it is reported.
For one lap the drift is harmless; it matters only if a second lap must
repeat the first.

The variance has a cause worth recording: `analytical_ik` seeds from
`std::random_device`, so **the planner is not reproducible run to run**. The
traced geometry is stable; the joint-space route is not. That is a real
limitation for repeatable controller experiments and a candidate fix (a
seeded generator) for the sensitivity work.

## Verification

- `ctest`: planner_bridge **14/14**, basic_control **11/11**.
- Point-to-point regression: unchanged at **1.34181 mm**, byte-identical
  diagnostics.
- Gate proven in both directions: a reachable circle is cleared and emitted
  (2.4 mm); with `maximum_planning_error_m` tightened to 0.1 mm the same plan
  is refused with **zero bytes emitted** while every other check still
  reports honestly.
- Unreachable circle (350 mm radius): refused at IK with the failing sample
  named, zero bytes emitted.
- Strict parsing: both-present, neither-present, negative radius, degenerate
  normal, `fixed` without an orientation, and an unknown shape are each
  refused with a named error.
- The emitted block was independently parsed with the controller's own
  accumulator: 8308 samples over 16.6 s, first point equal to the measured
  start to 0.00°, starting from rest.

## Remaining limitations

- `modelled_collision_valid` certifies against the arm-workspace grid plus an
  optional operator box. **No wearer, no torso, no second arm.** Every report
  states this; a green check is not a statement that the motion is safe near
  a person.
- Not reproducible run to run (above).
- The approach is task-space unconstrained, so its route is collision-checked
  and smooth but not predictable before the solve.
- Acceleration limits are derived from velocity limits (reach the limit in
  ~0.5 s) because no acceleration table exists in `joint_limits.yaml`.
- `5 mm` is a starting value to be measured, not a justified requirement. The
  1/2/5/10 mm sensitivity sweep is the next piece of work.
- **Nothing has run on hardware.**

## Report-ready explanation

Point-to-point planning constrains only the endpoint, so the route between is
whatever the optimiser finds cheap. Tracing a shape requires a pose
constraint at every support state, which raises two further questions: how
the arm reaches the start of the shape, and whether the trajectory that
finally reaches the robot still traces what was asked. The approach is left
task-space unconstrained — the arm must arrive, but the route was never part
of the request, and constraining it spends solver effort on a segment nobody
specified. Validation is performed on the controller's own reconstruction of
the emitted trajectory rather than the optimiser's output, because
subsampling to the wire format and cubic-Hermite reconstruction both distort
the plan after optimisation has finished; separating planner error from
transport error makes a fidelity failure attributable to one or the other.
Doing this exposed that the underlying optimiser assumes a constant support
interval, so a plan whose two phases had different natural sample spacings
was emitted with timestamps other than those requested — a fault invisible in
the optimiser's own cost and detectable only by measuring the artefact that
would actually be executed.

## Experiments this enables

- Tolerance sensitivity at 1/2/5/10 mm: convergence rate, solve time,
  achieved deviation, minimum clearance, smoothness. The threshold then comes
  from measurement rather than preference.
- Traceable-radius envelope over a grid of circle centres.
- `e_reconstruction` against waypoint count, quantifying what the 1000-point
  wire cap costs in path accuracy.
