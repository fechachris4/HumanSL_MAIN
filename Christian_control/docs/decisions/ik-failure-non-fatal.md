# IK failure is no longer fatal to a plan

Learning record, 2026-08-06. Follows the protocol in the repository root
`CLAUDE.md`.

## Reported symptom

Running `planner_bridge --arm left` sometimes exited 3 with
`error: solve failed: Failed to solve IK for end pose`, sending nothing to
the controller. The arm did not move at all — not even partway toward the
requested point.

## Expected behaviour

A position the arm can physically reach should produce some motion toward
it, or at minimum an explanation of how close the planner could get.

## Where the failure appeared

`PlanSolver::SolveToPosition` builds the goal as the requested position
combined with the tool orientation at the start configuration, then calls
`InitializeTrajectory::initJointTrajectoryFromTarget`. That ran the damped
least-squares solver in `analytical_ik.h` ten times, each drawing its own
spread of random seeds. If no attempt produced an accepted solution it threw
`std::runtime_error`, which `SolveToPosition` caught into `outcome.error`,
and `BridgeMain` reported as exit 3.

## Confirmed root cause of the *reported* behaviour

Inverse kinematics here only builds the **starting sketch** the GPMP2
optimiser refines. The goal itself is enforced separately inside the factor
graph by `GaussianPriorWorkspacePoseArm`, a soft cost term. So a failed IK
means a worse initial guess, not an impossible plan — but the code treated
it as fatal and abandoned an optimisation it could still have run.

## Fix

IK failure no longer throws. `initJointTrajectoryFromTarget` now returns a
`TrajectoryInit` carrying both the values and their provenance:

- `kSolvedIk` — a pose passed every acceptance test (unchanged behaviour);
- `kNearMiss` — nothing passed, so the sketch runs toward the closest
  configuration the solver actually reached;
- `kHeldStart` — no candidate at all, so every waypoint holds the start
  configuration and the optimiser's goal term is the only thing pulling.

Supporting changes: `IKSolution` retains its final position and orientation
errors, which were previously computed, collapsed into `quality_score`, and
overwritten by the seed distance. Rejected candidates are now ranked by how
close they got rather than by how little they moved, because "nearest the
seed" selects the laziest near miss, not the best one. `BridgeMain` reports
the provenance *before* validation and emission, so it is visible on runs
that go on to fail.

No new veto was added. Per `CLAUDE.md`, the plan is described rather than
gated: `final_goal_error_m` states what the optimiser actually achieved.

## Evidence

Christian's failing case (run 3, 2026-08-06), start
`108.84 -83.50 82.49 37.35 -152.73 46.94 -151.25` deg, goal
`left_base [0.0239, -0.563535, 0.0235506]`.

Before: exit 3, no trajectory.
After: a plan is produced and reported as
`NEAR MISS ... 38.29 mm and 0.135 deg from the requested pose`.

Five repeats gave 38.27–38.78 mm and 0.106–0.137 deg — stable, not noise.

Regression check: the case that already worked still reports
`solved IK pose` at 1.34 mm. planner_bridge suite 10/10.

## Confirmed cause of the IK failure: the inherited goal orientation

Christian's original hypothesis — that demanding the start pose's
orientation at the goal is what makes IK fail — is **confirmed**.

An intermediate conclusion in this record was wrong and is recorded here
deliberately. From the acceptance test alone (`position_ok &&
orientation_ok && limits_ok`, at 40 mm / 0.15 rad / the joint limits) it
looked as though orientation was innocent: the near miss achieved 0.135°
of orientation error and 38.29 mm of position error, both inside their
thresholds, so `limits_ok` had to be the failing term. That inference was
sound but the conclusion drawn from it — "joint limits, not orientation" —
was not, because it treated the two as alternatives when they are cause and
mechanism.

**Direct measurement.** A temporary per-attempt diagnostic printed each
candidate's joint angles and any limit violation. All ten attempts agreed:

- **joint 6 outside its ±120.3° limit by 36–40°**, landing near ±158°, in
  every single attempt;
- joint 4 within ±1.6° of zero every time — a straight elbow, i.e. the arm
  fully extended;
- position error pinned at ~38.3 mm and orientation error at ~0.14°.

The damped least-squares solver is distributing an infeasible request: it
holds orientation almost exactly, and pays for it with 38 mm of position
error *and* a joint 6 violation. That the orientation term is the one it
refuses to give up is itself evidence of where the binding requirement is.

**Controlled comparison.** The same goal position
`left_base [0.0239, -0.563535, 0.0235506]`, planned from two different
start configurations:

| start | result |
|---|---|
| `108.84 -83.50 82.49 37.35 -152.73 46.94 -151.25` | NEAR MISS, j6 out by ~38° |
| `-26.83 -113.52 92.11 109.07 81.18 -24.70 -138.06` | solved IK pose, 1.34 mm |

The goal position is identical and the goal orientation is the only thing
that differs, because `PlanSolver` copies it from the start pose. So the
**position is reachable; the pose is not.** The orientation inherited from
the first start can only be achieved with joint 6 roughly 38° past its
stop. Both starts have a perfectly legal joint 6 themselves (46.9° and
−24.7°); it is the demanded goal orientation that drives it out.

## Limit-aware IK: built, and it does not rescue this pose

`solveDampedLeastSquares` now carries a null-space joint-limit avoidance
term, mirroring `basic_control`'s `ReactiveLaw.h` (same 20° deadband zone,
same damped projector `N = I − Jᵀ(JJᵀ + λI)⁻¹J`). It is a deadband, so it is
inert until a bounded joint comes within the zone, leaving the common case
untouched.

On this case it changes nothing: joint 6 stays ~36–39° outside its limit and
the near miss stays at ~38.3 mm. That is the correct behaviour, not a bug,
and it was verified rather than assumed. Temporarily removing the projection
gives joint 6 back — it comes inside its limit, out by 0.05–0.9° — but the
pose error jumps to **82 mm and 14.6°**.

That comparison is the proof: the mechanism is live, and the projector is
correctly refusing to trade pose accuracy for legality. **No configuration
reaches this pose with a legal joint 6.** The projection was restored.

The term is still worth having — it prefers a legal configuration whenever
one exists that reaches the requested pose, which is a real improvement for
both arms — but it cannot rescue a pose that no legal configuration reaches.

## Explicit goal orientation, and the decisive experiment

`PlanRequest` gained an optional `goal_rotation`, and a goal block may now
carry `orientation_rpy_deg: [roll, pitch, yaw]` in degrees, read in the same
declared frame as the position, using R = Rz*Ry*Rx. Rotations convert
between frames differently from points — only the rotational parts compose —
so `RotationToControlledBase` is separate from `ToControlledBase`. When no
orientation is given the old inheritance still applies, but the bridge now
says so on every such run instead of leaving it silent.

The controlled experiment, from the identical start configuration that could
not plan at all:

| goal orientation | result |
|---|---|
| inherited from start pose | exit 4, NEAR MISS, 38.6 mm, joint 6 illegal |
| explicit, `[12.3, -63.0, 107.3]` deg | **solved IK pose, 0.063 mm** |

Same start, same goal position; only the orientation source differs. This is
the cleanest statement of the whole problem: the position was always
reachable, and an orientation nobody chose was what made the request
impossible.

## Why this matters for the next step

Option 2 — relaxing the orientation requirement — is the right direction,
and the measurement says specifically what it has to achieve: relieve
joint 6. Two forms are worth considering, and the evidence does not yet
choose between them:

- demote orientation to a secondary objective solved in the null space, so
  the hand arrives at the best available orientation rather than a demanded
  one; or
- keep the orientation but make the solver **limit-aware**, steering away
  from joint 6's stop while solving rather than discovering the violation
  after convergence. The arm has four continuous joints (1, 3, 5, 7) and so
  considerable freedom to reconfigure; `basic_control`'s reactive law
  already does exactly this via `kLimitAvoidZoneDeg` / `kLimitAvoidGain`.

The first changes what the arm is asked for; the second changes only how
the solver searches. The second is preferable if it suffices, because it
leaves the requested pose intact — but a pose needing j6 38° past its stop
may be infeasible for any solver, in which case only the first helps.

## Remaining limitations

- On this case the emitted plan ends **428 mm** from the requested goal.
  The planner now tries and reports honestly, but the result is poor; that
  is a separate quality problem from the refusal that was fixed.
- Most repeats now stop at a *different* veto: `ValidateJointPath` rejects
  the path because joint 5 winds to −360.1°, marginally past a ±360° bound.
  Joint 5 is continuous, so this is a winding/unwrapping bound rather than a
  physical limit. It is the next crude guard in the chain and a candidate
  for the guard work.
- Which of joints 2, 4 or 6 violates its limit has not been isolated.
- Nothing here has been run on hardware.

## Report-ready explanation

Inverse kinematics answers "what joint angles put the hand here". In this
planner it is used only to produce an initial guess for a trajectory
optimiser that carries its own goal term, so an IK failure degrades the
guess rather than making the problem unsolvable. The implementation
nevertheless treated failure as fatal and abandoned the plan, so a target
the arm could approach produced no motion at all. Making failure survivable
— falling back to the closest configuration the solver reached, and
reporting which fallback was used — turns a refusal into a best effort with
a stated quality. Measuring the solver's residual error separately in
position and orientation then showed that the failures were caused by joint
limits rather than by the orientation constraint that had been suspected,
which redirects the next fix from orientation relaxation to limit-aware
redundancy resolution.

## Experiments that could support the thesis

- Success rate and achieved goal error, over a sampled set of reachable
  targets, before and after the fallback.
- The same sample classified by which acceptance test rejected it
  (position, orientation, joint limits) — this record establishes the
  method on one case; a distribution would make the argument.
- Achieved goal error against initialisation source, showing the cost of a
  degraded guess.
