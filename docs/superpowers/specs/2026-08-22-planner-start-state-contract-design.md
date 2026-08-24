# Planner Start-State Contract

**Date:** 2026-08-22  
**Status:** Architecture approved in chat; written-form review pending  
**Classification:** C — planner motion-generation behaviour change  
**Evidence level:** recorded-data diagnosis and hardware-free analysis only

## Objective

A plan must begin from the measured joint state that caused the planning
request. During live replanning, it must also begin with the measured joint
velocity from that same feedback snapshot. The planner must not move the
initial position to improve another objective, and it must not silently assume
the arm has stopped.

This corrects the failed right-arm run in
`runs/2026-08-22/session_194737`: GPMP2 returned successfully, but moved the
first state far enough that the projected Cartesian trajectory began 28.1 mm
and 0.03065 rad from the measured pose. The controller correctly rejected that
trajectory and continued holding.

## Physical and mathematical contract

Let one immutable live planning snapshot contain

```text
(q_meas, qdot_meas, world_T_mount, goal, request provenance)
```

where `q_meas` and `qdot_meas` come from the same successful Kortex cyclic
feedback reply. Joint order is Kinova actuator order, position is radians, and
velocity is radians per second. The bundled Kortex API exposes actuator
feedback position and angular velocity in the same feedback message; the
official API describes the velocity field as actuator angular velocity in
degrees per second. `ExecutionCore` performs the existing degrees-to-radians
conversion once.

The planner's flat joint coordinates use the existing canonicalisation

```text
q_plan = C(q_meas) = WrapToPrincipalRad(q_meas)
```

for all seven joints. Velocity is not wrapped. The required support-state
boundary is

```text
q(0)    = q_plan
qdot(0) = qdot_meas          for every live PlanningRequest
```

For the standalone offline planner, measured velocity may be unavailable. Its
contract is therefore

```text
q(0) = q_plan
qdot(0) unconstrained        when no measured velocity was supplied
```

Absence never means zero. A stationary live arm supplies a measured zero; an
offline request without velocity supplies no initial-velocity factor.

The projected world reference remains

```text
world_V_tcp(t) = world_J(q(t)) qdot(t)
```

so preserving `qdot(0)` also preserves the initial planned Cartesian twist.
The asynchronous solve may finish after the physical arm has moved again. The
existing controller continuity gate remains the independent activation check;
this change does not claim to remove planning latency or authorize blending.

## Current failure mechanism

The live runtime currently publishes `q_rad` but drops the actuator velocity
that is already present in the same `ArmExecutionResult`.

The point planner then manufactures `start_vel = Zero(7)` in
`GenerateTrajectory.cpp`, and GPMP2 applies finite-sigma priors to both the
initial position and the manufactured zero velocity. A competing objective can
therefore move `q(0)`.

The traced-path planner independently records waypoint zero as a rest index,
and `optimizeTaskTrajectory` also reinserts zero as a start rest state. Its
task-entry and final rest constraints have a different meaning and remain.

## Approved design

### Live request boundary

`PlanningRequest` gains an optional fixed-size `qdot_rad_s`. The optional form
prevents default construction from silently creating a valid-looking zero
measurement. `ValidatePlanningRequest` rejects a live request when velocity is
absent or non-finite.

`Runner` assigns `q_rad` and `qdot_rad_s` from the same `ArmExecutionResult`
before one publication to the existing latest-value slot. There is no new
thread, buffer, timestamp, or validity layer. A successful cyclic reply is the
measurement provenance already trusted by the controller; no invented quality
flag is added.

### Existing planner transport

The established `PlanningRequest -> PlannerRuntime arguments -> BridgeMain`
path remains. `PlannerRuntime` emits one explicit
`--start-velocity-deg-s J1..J7` argument from every valid live request.
`BridgeMain` accepts that flag but keeps it optional for standalone offline
use. The existing `--start-deg` and CSV position sources are unchanged.

This deliberately avoids a broader refactor of the typed-to-argument bridge.

### GPMP2 boundary

Both active optimizers receive an optional initial velocity. Before graph
construction they copy the supplied initial `Values`, then set `x0` to the
canonical measured position and, when present, `v0` to measured velocity. This
makes the initial linearisation feasible for GTSAM's equality factors.

They then use `gtsam::NonlinearEquality<gtsam::Vector>` for:

```text
x0 = q_plan
v0 = qdot_meas              only when velocity is present
```

This is an optimizer constraint, not a post-solve overwrite. No sigma is
tuned, and no downstream sample is repaired. The isolated hardware-free probe
against the repository's bundled GTSAM showed that this equality retains the
exact feasible value despite a competing soft prior.

For traced paths, waypoint zero is removed from `zero_velocity_indices`.
The task-entry and terminal rest indices remain soft zero-velocity constraints
with their existing semantics. Point-plan terminal rest also remains.

### Traced dynamic-duration boundary clarification

Each traced attempt solves from scratch at one duration `T`, then validates
the resulting dense trajectory. A longer-duration retry is permitted only
when the validation failure is exclusively joint-velocity or
joint-acceleration excess. Non-finite state, measured-start mismatch,
joint-position failure, modelled collision failure, IK or optimizer failure,
geometric/fidelity failure, or any other non-dynamic issue ends the solve
without retry. The existing alpha policy increases `T`, with at most three
total attempts; every retry rebuilds the scaled waypoint times, initial
`Values`, and velocity guesses before calling `optimizeTaskTrajectory` again.
No post-hoc velocity scaling is applied, so every attempted and final plan
retains the exact measured `q(0)` and `qdot(0)`. If supplied measured `qdot(0)`
already exceeds an effective planner joint-velocity limit, the request fails
before the first solve because duration cannot repair that boundary.

The recorded point failure and the traced dynamic-duration case are separate
evidence: point planning must not inherit traced retry policy, and a traced
retry must not hide a non-dynamic validation failure.

The unused `reOptimizeJointTrajectory` overload is outside this slice.

## Limiting cases and predictions

1. **Stationary live request:** measured `qdot_meas = 0`; both active planners
   emit exactly that zero at `t=0`.
2. **Moving live request:** nonzero measured velocity is preserved
   componentwise at `t=0`; substituting zero must fail verification.
3. **Offline request without velocity:** `q(0)` remains exact, no start-velocity
   equality or zero prior exists, and the optimizer chooses `v0` from its
   remaining factors and initial sketch.
4. **Continuous-joint representation:** an encoder-equivalent angle such as
   253 degrees is converted once to the existing principal coordinate; the
   equality applies to that coordinate.
5. **Traced path:** measured start velocity replaces only the old start-rest
   assumption; the arm still comes to rest at task entry and at the end.
6. **Delayed completion:** a later physical state can still differ from the
   immutable request snapshot; the controller gate may still reject it.

## Verification

One new hardware-free test executable owns this one contract. It covers point
and traced planning with velocity present and absent:

- nonzero supplied velocity must produce exact dense `q(0)` and `qdot(0)`;
- the existing factor-cost map must contain the start equalities when supplied;
- the absent case must contain no initial-velocity equality or prior;
- one `SolvePlanForRequest` case must prove the live typed request transports
  velocity through the existing argument/parser path;
- a nonzero-velocity mutation catches any reintroduced zero substitution.

The recorded failed run remains the pre-change characterization. After the
change, the same request is replayed offline into a fresh temporary artifact
directory. Its first joint state, projected Cartesian start, goal error, solve
time, joint-limit margin, and peak velocity are compared with the baseline.
The existing controller continuity gate is not loosened and remains independent
evidence at the planner/controller boundary.

No robot-facing executable is run.

## File and concept budget

Expected production files, limited to carrying or enforcing the same state:

- `Christian_control/contracts/PlanningRequest.h/.cpp`
- `Christian_control/runtime/Runner.cpp`
- `Christian_control/planning/src/PlannerRuntime.cpp`
- `Christian_control/planning/src/BridgeMain.cpp`
- `Christian_control/planning/src/PlanSolver.h/.cpp`
- `Christian_control/planning/src/PathAssembly.h/.cpp`
- `Christian_control/planning/optimisation/GenerateTrajectory.h/.cpp`
- `Christian_control/planning/optimisation/TrajectoryOptimization.h/.cpp`

Test/build files:

- one new focused planner test source;
- `Christian_control/planning/CMakeLists.txt` for that one target;
- existing request fixtures updated only where compilation or validity
  requires the new measured field.

Concepts added: one measured request field, one optional offline input, and
exact start equalities. Concepts removed: the manufactured point-plan zero
velocity, the traced-plan start-rest assumption, and finite-sigma initial
position priors. No class, manager, service, retry, fallback, safety gate, or
configuration key is added.

## Explicit non-goals

- No relaxation of the controller's Cartesian continuity tolerance.
- No blending, automatic replanning, trajectory slowing, or target change.
- No change to collision, joint-limit, task-entry-rest, or terminal-rest
  policy.
- No change inside the 500 Hz controller mathematics.
- No claim of physical execution improvement until a separately authorized
  supervised hardware run.
