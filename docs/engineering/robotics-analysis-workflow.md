# Robotics Analysis Workflow

**Status:** Binding for agent-executed implementation work.
**Companion to:** `humansl-engineering-contract.md` (the rules) — this file is
the process that enforces them. Approved by Christian 2026-08-17.

## The task anchor (prepended to every agent brief)

> Your job is to resolve the stated robotics question, not merely complete
> file edits. Before implementation, define the physical system, frames,
> units, timestamps, governing equations, assumptions, controllable and
> observed variables, limiting cases and falsifiable predictions. Decompose
> end-effector behaviour into reference, Mount motion, calibration,
> kinematics, sensing, latency, control and actuation contributions. Do not
> implement until the analysis reviewer accepts this model. Every
> implementation change must trace to an accepted equation, invariant or
> measurement. If the mathematics is ambiguous, stop and report the
> ambiguity rather than selecting a convenient convention.

## Robotics Analysis Packet

Required before implementation for frame, controller, Vicon, kinematics and
model tasks. Contents:

1. **Physical objective** — the phenomenon, not the software feature.
2. **System decomposition** — `world_T_tcp = world_T_mount · mount_T_base ·
   base_T_tcp(q)`; which component the task changes.
3. **Motion decomposition** — one declared twist convention; base-motion vs
   joint-motion terms; how the measured-twist term compensates base motion.
4. **Error decomposition** — Vicon, calibration, kinematics, timing,
   controller, actuation, flex contributions.
5. **Time decomposition** — physics substeps, 500 Hz control, 100 Hz Vicon,
   asynchronous planner, ZOH/age behaviour.
6. **Limiting cases** — predicted behaviour for stationary Mount, constant
   translation, rotation about the TCP, repeated Vicon samples, wrong
   calibration, constant reference, infeasible plan, rejected replacement.
7. **Falsifiable predictions** — expected plots and signs written before
   results exist.

Tiering (to keep this practical, not procrastination):

- Frame/controller/Vicon/kinematics/model tasks: full packet.
- IPC/serialization tasks: contracts, timing, invariants only.
- Panel tasks: state ownership, safety boundaries, evidence provenance only.
- Build/mechanical changes: a short dependency argument.

## Roles per significant task

1. **Robotics analyst** (read-only) — produces the packet.
2. **Adversarial analysis reviewer** (read-only) — tries to break the
   derivation: transform order, spatial-vs-body twist mixing, Jacobian
   reference frame/point, whether the proposed evidence distinguishes
   competing explanations. Implementation waits for acceptance.
3. **Implementation agent** — the only writer; one at a time; implements only
   the accepted mathematical contract; every change traces to an equation,
   invariant or measurable requirement.
4. **Evidence reviewer** (read-only) — compares results against the
   predictions written before implementation, not only against tests the
   implementer created; asks whether fundamentally wrong physics could still
   pass.

Specialist review lenses at milestone gates: controls, state estimation,
experimental design, wearable robotics, safety (diagnostics must not silently
become stops — contract §10), simplification (remove generic AI-style
structure). One synthesis pass reconciles disagreements. Christian remains the
decision-maker.

## Mechanical enforcement

Violations should fail builds/tests, not depend on memory:

- `humansl_execution_core` links only Eigen/Pinocchio/pthread; architecture
  tests fail on MuJoCo/Kortex/Vicon-SDK symbols or includes in core sources.
- `humansl_sim` link test fails on any Kortex symbol or robot address.
- Hardware and simulation targets must link the same core library.
- The simulation adapter exposes only hardware-shaped structures (measured
  joint state; world Mount sample with age/sequence). Simulator ground truth
  travels in a separate `sim_`-namespaced validation contract that production
  control code cannot see.
- Pre-refactor characterization traces replay after refactors (Plan 01 gate).

## Code-shape and simplicity gate (added 2026-08-17)

Mathematical correctness is not enough; the second gate protects readability.

**Agent safety is not robot safety.** Agent rules (never run hardware,
preserve files, audit first) live in the agent workflow. Robot safety (limits,
finite-value checks, stale feedback, stop priority, shutdown) lives in one
small explicit runtime safety boundary. Research diagnostics go to telemetry.
Speculative defensive checks — guards with no defined hazard — are not added.
An agent must never translate its own cautious operating instructions into
production-code gates. Every proposed runtime guard must answer: the specific
hazard; where it is first observable; the required response (reject / limit /
hold / stop / log only); the single owning component; how it is tested; and
whether an equivalent check already exists. No answers, no guard.

**The readable pipeline.** One orchestration point shows:
CycleInput → ControllerLaw → raw qdot → SafetyPolicy → limited qdot →
PositionIntegrator → ActuatorRequest → CycleResult → TelemetrySnapshot →
asynchronous writer. Kortex lifecycle and the MuJoCo adapter sit outside it.
A reader of the execution core must never meet CSV formatting, plotting,
Kortex startup policy, terminal output or planner IPC.

**Telemetry observes; it is not sprinkled.** Components return result structs
(ControllerResult, SafetyResult, CycleResult); one boundary call assembles the
TelemetrySnapshot; one asynchronous writer serializes it. Controller
mathematics does not know CSV exists. Sim-only truth uses the `sim_`
extension. The goal is predictable locality, not literally one file.

**Code-shape rules per task:** one owner per concern; pure controller/safety
functions with no logging/printing/global mutation; validate once at the
boundary; no speculative abstractions (manager/service/registry/event
bus/factory need replaced complexity or two real implementations); file-touch
budget declared before editing — unexpected spread means stop and report the
hidden coupling; concept budget in the task report (concepts added/removed,
new classes, new branches, new safety gates, why each is unavoidable);
readable main path; prefer deletion/consolidation over layering a new
abstraction on a still-alive old path.

**Simplicity review** (separate reviewer, after the evidence review): can the
control cycle be understood from one file; one obvious owner; telemetry out
of the mathematics; no repeated validation across layers; no agent caution
turned into runtime policy; could one result struct replace several helpers;
more concepts removed than added; will the next person know where to change
this; fewer branches/files possible? A task fails this review even when every
test passes.

**Full per-task chain:** robotics analysis → mathematical review → code-shape
proposal ("owned by X; main flow visible in Y; telemetry assembled at Z;
touches these files; adds one concept, removes two") → simplicity review →
implementation → behaviour/evidence review → final readability review.

## Milestone gate checklist

- Physical purpose explainable in a paragraph; equation-to-code map visible.
- Frames/units in names or explicit contracts; one owner per responsibility.
- No duplicate logging/configuration/safety path introduced.
- At least one independent or negative (mutation) check per motion change.
- Architecture and replay tests pass; diff contains no unrelated refactoring.
- Unproven hardware behaviour stated explicitly (`offline-validated only`).
- A fresh reviewer can explain one complete 500 Hz cycle from source; if not,
  the milestone is not finished even if tests pass.
