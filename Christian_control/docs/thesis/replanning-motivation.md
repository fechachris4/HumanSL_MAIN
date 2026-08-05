# Why the system needs replanning — thesis motivation notes

Working notes for the thesis write-up. Records why online replanning is
needed, when and how that need was realised, and why the staged designs
(replan-while-moving, receding horizon) are useful. Dates cite commits in
this repository; numbers marked *(session notes)* come from experiment
logs and should be re-verified against `runs/` before appearing in the
thesis.

## 1. The problem in one paragraph

The system has a global motion planner (`TrajectoryGeneration/`, a
GPMP2/GTSAM dual-arm optimiser producing densified 1 kHz joint
trajectories) and a 1 kHz reactive controller
(`Christian_control/basic_control`, a task-space reactive law with
null-space joint-limit avoidance and a safety-kernel design). Each is
sound alone, but each fails in a way the other fixes. A planned
trajectory executed open loop cannot respond to anything that changes
after planning time — a moving obstacle, a moved target, a safety clamp,
or tracking drift. A reactive controller responds instantly but is
purely local: it has no collision model and no lookahead, so it can be
attracted into configurations a global planner would have routed around.
Replanning — re-solving the global problem online from the arm's actual
state — is the mechanism that closes the loop between the two, and is
the natural end point of this project's architecture.

## 2. Timeline of the realisation (evidence from the repository)

The need for replanning was not assumed at the start; it emerged from
building and then deliberately dismantling the open-loop alternative.

- **2026-07-15 → 2026-07-24** — first controllers: a reactive task-space
  servo (`0161f39b`), a resolved-rate velocity controller (`a28da5ed`),
  and the ported 6-DoF reactive pose law (`63d9d312`). Diagnosis of
  velocity saturation on unreachable targets was an early sign that a
  purely reactive law needs an upstream source of *feasible* references.
- **Week of 2026-07-27** — trajectory integration investigated. The
  GPMP2 solve was measured at ~54.5 ms *(session notes)* — far too slow
  to run inside a 1 ms control tick, which fixed the two-rate
  architecture (fast tracking loop, slow planning loop) as the only
  viable shape. Lead compensation in the collision-aware Cartesian
  planner reduced hand-off error from 57.5 mm to 4.2 mm *(session
  notes)* — the same forward-prediction technique a mid-motion replan
  splice requires.
- **2026-07-31** — open-loop trajectory playback implemented in full,
  with contract, whole-path validation, and a decision record
  (`34102b9f`, `288d827a`; see
  `docs/decisions/whole-path-validation.md`). This was the "execute a
  static plan" design carried to completion.
- **2026-08-04** — playback removed (`f64325c0`), returning the
  controller to fixed-target-only. The removal was a simplification
  decision, but it also reflected the core limitation: a static
  1000-point trajectory replayed open loop gives no principled answer
  to "what happens when reality diverges from the plan mid-execution."
  The subsequent safety hardening the same day (stale-acknowledgement
  stop `532f789f`, non-arrival timeout `547448c6`) is all machinery for
  *detecting* divergence — with no component able to *resolve* it.
- **2026-08-05** — the reactive law reached its intended form: deadband
  joint-limit avoidance in the null space (`c94aaf0d`, `f2aef175`).
  Hardware experiments the same week showed its local nature concretely:
  with null-space centering gain 23, the arm settled into a stall
  equilibrium 218 mm short of the target *(session notes)* — a local
  law balancing internal objectives with no global view of the task.
- **2026-08-05 (design discussion)** — the pieces were put together:
  the controller detects divergence but can only stop or hold; the
  planner can resolve divergence but only runs offline. The missing
  capability is replanning, and the architecture question "who owns
  what happens when plan and reality diverge — track, hold, or
  replan?" was answered: **replan**.

## 3. Why open-loop execution is not enough (thesis argument)

1. **The world is not static at execution time.** The HumanSL setting is
   a shared human-robot workspace with Vicon tracking; obstacles and
   targets move on timescales comparable to trajectory duration. A plan
   is a prediction, and its collision-freedom is only as durable as the
   world model it was solved against.
2. **The arm does not follow the plan exactly.** Tracking error, safety
   clamps (velocity caps, joint-limit deadbands, stop conditions), and
   faults all push the executed state off the planned state. The
   remainder of a static plan, executed from the wrong state, is no
   longer the trajectory that was validated.
3. **Detection without resolution is a dead end.** The controller
   already detects divergence (arrival settling, non-arrival timeout,
   staleness stops) but its only responses are stop and hold. Safe, but
   task progress dies with every disturbance. Replanning converts
   divergence from a terminal condition into a recoverable one.
4. **The reactive law cannot absorb the planner's job.** It is local by
   construction: no obstacle model, no lookahead, provably capable of
   stalling in spurious equilibria (the gain-23 experiment). Global
   collision-aware routing must come from the optimiser.

## 4. Why the staged designs are useful

### Stage 1 — stop-and-replan (baseline)

On a trigger (deviation threshold, obstacle motion, target change) the
arm stops, GPMP2 re-solves from rest, and execution restarts. No
splicing or prediction needed. Its value is architectural: it forces
into existence the planner-as-a-process boundary, the plan message
format, the per-plan validation gate (validated against the
*controller's* model — real joint limits, reach, current hardware
constraints — never the planner's YAML alone), and the trigger logic.
Every one of those components carries forward unchanged. It is also the
experimental baseline the later stages are measured against
(time-to-goal under disturbance, number of stops).

### Stage 2 — replan-while-moving

Periodic or event-triggered re-solves while the arm executes, spliced
into the current motion. Adds exactly two new problems on top of
stage 1: (a) forward-prediction of the splice state — plan not from
where the arm is but from where it will be one solve-time ahead, the
lead-compensation technique already demonstrated in July; and (b)
continuity at the splice — the new plan is constrained to match
position and velocity at hand-off, expressible in GPMP2 as a fixed
start state and start velocity. Usefulness: the arm keeps moving
through disturbances, so task time degrades gracefully instead of
resetting on every world change; this is the behaviour a shared-
workspace system actually needs, and the thesis's main contribution.

### Stage 3 — receding-horizon replanning

Continuous replanning toward a rolling horizon, MPC-style: at each
planning cycle only the leading segment of the latest plan is ever
executed. Usefulness: it dissolves the splice as a discrete event —
divergence is corrected every cycle by construction — and it is the
principled framework (feedback via repeated optimisation) that
connects the thesis to the MPC literature. Cost: solve rate becomes the
binding constraint; at ~54 ms per GPMP2 solve the ceiling is ~10–18 Hz
before margin, so stage 3 likely requires warm-starting and possibly
incremental factor-graph updates rather than full re-solves. Scoped as
future work / stretch goal.

## 5. Failure ownership (safety argument for the write-up)

Replanning is added *above* the safety kernel, never inside it. The
1 kHz controller never blocks on the planner: it tracks the last valid
plan; if that plan goes stale or runs out before a fresh one arrives it
degrades to hold-position; every incoming plan passes whole-path
validation before acceptance, and a rejected plan is treated identically
to a late one. A planner crash therefore degrades to exactly the
already-validated fixed-target behaviour. This preserves the safety
argument built through the 2026-08-04 hardening commits.

## 6. Literature anchors (verify before citing)

- GPMP2: Mukadam, Dong, Yan, Dellaert, Boots — "Continuous-time
  Gaussian process motion planning via probabilistic inference," IJRR
  37(11), 2018. Basis of `TrajectoryGeneration/`.
- Incremental replanning in the same framework: the GPMP2 line of work
  includes iSAM2-based incremental re-solves (and STEAP, Mukadam et
  al., Autonomous Robots 2019) — directly relevant justification for
  stages 2–3.
- Receding-horizon control: Mayne, Rawlings, Rao, Scokaert — "Constrained
  model predictive control: stability and optimality," Automatica 2000,
  for the feedback-via-repeated-optimisation principle behind stage 3.
- Local reactive deformation as the contrasting approach: Quinlan &
  Khatib, "Elastic bands: connecting path planning and control," ICRA
  1993 — useful to argue why the local/global split is the right
  decomposition.

## 7. Model-agreement check (measured 2026-08-05, hardware-free)

Before Stage 1 was planned, the planner's kinematic model
(`TrajectoryGeneration/config/dh_params.yaml`, standard DH per
`utils.cpp` `createDHTransform`) was compared against the controller's
model (URDF chain, verbatim from `tools/AnalyticalKinematics.cpp`, itself
verified against the controller's Pinocchio FK) over 501 joint
configurations (zero + 500 uniform random within limits, seed 42):

- **Flange to flange: 0.36 mm mean / 0.49 mm max** position disagreement
  after removing one fixed base-frame rotation. The residual is URDF
  decimal truncation, not modelling error. The chains agree.
- The fixed offset is a **180° rotation of the DH base frame relative to
  `base_link`** (Kinova Table 94 convention; visible as sign-flipped y/z
  at zero config). The Stage 1 bridge must apply this one constant
  transform — either by constructing the planner's `Arm` with the
  flipped `base_pose` or by converting its outputs.
- **Tool points differ by design: ~20 mm** (planner grasp site =
  flange + 0.14 m; controller `ConfiguredTool_Link` = flange + 0.12 m).
  One tool point must be chosen at the interface; this is a decision,
  not an error.
- Scope: this validates the geometric chains. gpmp2's internal `Arm` FK
  (same standard-DH family) and the collision-sphere model are not
  covered; one planned-vs-controller-FK spot check at Stage 1 closes
  that.

## 8. Numbers to re-verify before the thesis

- 54.5 ms GPMP2 solve latency — re-measure on current hardware/branch.
- 57.5 mm → 4.2 mm lead-compensation improvement — locate the run logs.
- Gain-23 stall, 218 mm short — locate the run log and telemetry.
- Solve-rate ceiling for stage 3 — derive from re-measured solve time.
