# HumanSL dual-arm execution twin

**Date:** 2026-08-16  
**Status:** Approved design with external-review corrections; not yet implemented  
**Scope:** A dual-arm MuJoCo execution twin inside `HumanSL_MAIN`, sharing the
production C++ planning, reference, controller, software-safety, actuation, and
telemetry logic. No robot command or hardware validation is part of this work.

## 1. Outcome

HumanSL has a separate `humansl_sim` executable that is a trustworthy
pre-hardware testbed for controller and planner changes. It runs the complete
dual-arm path:

```text
panel goal
-> asynchronous GPMP2 solve
-> dense world Cartesian pose/twist projection
-> atomic dual-arm reference handoff
-> shared 500 Hz Cartesian execution core
-> shared limits and position-command integration
-> MuJoCo command adapter
```

The simulator does not recreate the controller by copying it. Hardware and
simulation compile and run the same hardware-independent C++ execution core.
Only sensing, clocking, startup/teardown, and command exchange have separate
hardware and simulation adapters.

Creating that shared core is a Level-2, behaviour-preserving refactor of the
hardware controller. The current hardware loop is Kortex-coupled and also owns
configuration reads, cycle sequencing, safety classification, actuation, and
log assembly. Extracting those responsibilities changes the hardware
executable even when the intended equations and limits do not change. The
refactored hardware path therefore has both an offline regression obligation
and a later supervised physical revalidation obligation.

The purpose is to let Christian implement and tune planner/controller ideas in
an interactive simulation, inspect plots, and then run a comprehensive offline
acceptance check before considering supervised hardware validation.

## 2. Explicit scope

Included:

- both Gen3 arms in one MuJoCo world;
- the production GPMP2 planning application, unchanged GPMP2 internals;
- world-frame trajectory projection and asynchronous replanning;
- the production world-frame Cartesian controller;
- the production reference-state, limit, integration, generic software-stop,
  and telemetry semantics;
- exact HumanSL joint ordering, kinematic model, Mount-to-base transforms, and
  configured end-effector/tool frames;
- a 500 Hz deterministic simulation/control clock;
- ideal and realistic Vicon modes;
- scripted, repeatable Mount disturbances;
- full control-panel launch, goal, scenario, tuning, viewer, and plot workflow;
- shared planner/MuJoCo static obstacles, implemented but disabled by default;
- hardware-parity and experiment tuning modes;
- targeted development tests and one comprehensive final acceptance suite.

Excluded:

- Kinova firmware or Kortex protocol emulation;
- identified Kinova actuator, friction, backlash, or motor dynamics;
- functional grippers, grasping, and object-manipulation physics;
- automatic claims that simulation proves physical safety or hardware
  performance;
- robot-facing execution.

The MuJoCo actuator model preserves the command path but remains generic.

## 3. Selected architecture

The selected approach is a shared execution kernel with explicit adapters.

```text
hardware controller                         humansl_sim
-------------------                         -----------
real Vicon adapter                          simulated Vicon adapter
Kortex feedback/command adapter              MuJoCo feedback/command adapter
Kortex takeover and restoration              simulation startup and reset
             \                               /
              +-- shared execution core ----+
                  world measurement
                  Cartesian reference
                  Cartesian controller
                  velocity/boundary limits
                  position integration
                  generic software stops
                  telemetry semantics
```

The simulation target must not link the Kortex hardware adapter, contain robot
addresses, or expose a runtime switch that could connect it to a Kinova. The
hardware executable remains separate and retains its Kortex-specific lifecycle.

Shared-library linkage proves common source, not behavioural equivalence with
a previously executed binary. Behavioural equivalence is established by the
pre-extraction characterization and replay gate in section 17. Physical
equivalence remains unproven until a separately authorized hardware campaign.

Two rejected alternatives remain documented:

1. Fake Kortex and Vicon servers would exercise more of the existing hardware
   binary, but require a large and misleading firmware/protocol emulation.
2. A separate simulation runner sharing only controller classes is initially
   cheaper, but duplicates timing, reference switching, safety sequencing, and
   integration—the behaviours the twin is intended to validate.

## 4. Shared per-arm execution contract

The hardware-free per-arm core receives one explicit input per 500 Hz cycle:

```text
ExecutionInput
  measured q_rad[7]
  measured qdot_rad_s[7]
  measured/feedback health facts
  coherent Mount pose/twist sample and age
  elapsed dt_s
  candidate world Cartesian trajectory, if one arrived
  external stop/fault injection facts
```

It owns persistent controller state, reference state, monitors, and position
integration state. It performs:

```text
measured arm + Mount state
-> measured world end-effector pose/twist and world Jacobian
-> one world Cartesian pose/twist reference
-> Cartesian PD/DLS/null-space law
-> raw joint velocity
-> velocity clamp and joint-boundary handling
-> integrated position request
-> generic software-stop decision
-> telemetry result
```

It returns an explicit result:

```text
ExecutionResult
  raw and limited joint velocity
  integrated position command
  measured and reference Cartesian state
  reference/planning events
  generic stop decision
  complete hardware-independent telemetry fields
```

Production configuration is supplied as an immutable execution-configuration
snapshot. The hardware executable constructs it from production constants.
`humansl_sim` uses the same snapshot in hardware-parity mode and an explicit
copy with logged temporary overrides in experiment mode.

No ordinary file, terminal, Vicon, planner, panel, or logging I/O enters this
per-cycle core.

## 5. Hardware and simulation boundaries

The hardware adapter retains:

- readiness and startup gates tied to real feedback;
- low-level servoing takeover and fixed measured-position hold;
- Kortex command IDs, acknowledgements, faults, mode checks, and communication;
- real scheduling and cyclic timing evidence;
- servoing restoration and hardware teardown.

It translates Kortex feedback into the shared execution input and translates
the shared position result into the cyclic command.

The MuJoCo adapter:

- validates and seeds both arm states;
- translates MuJoCo joint position/velocity into the same measured-state
  contract;
- applies both integrated position commands to generic position actuators;
- advances simulated time by exactly `0.002 s` after both arm cores have
  stepped, while permitting a fixed number of smaller internal MuJoCo
  integration substeps whose durations sum to one control period;
- reports contact and simulator numerical failures separately from production
  firmware facts.

Both arm commands are held constant across internal physics substeps. The
control/reference/Vicon clocks remain tied to the 2 ms control period, not the
smaller integrator step.

Kortex-only facts do not leak into the shared controller law. Simulation may
inject equivalent generic stop conditions for offline testing, but does not
claim to reproduce Kinova firmware.

## 6. Dual-arm scheduling and ownership

`humansl_sim` owns one execution core per arm, one MuJoCo model, one moving
Mount state, and one session stop flag. Each deterministic simulation tick is:

```text
1. read the right and left simulated arm states;
2. advance/sample the selected Mount/Vicon source;
3. step the right execution core;
4. step the left execution core;
5. resolve the shared-stop result;
6. apply both position commands;
7. advance MuJoCo by 2 ms;
8. publish non-blocking telemetry snapshots.
```

The two per-arm steps use the same tick time and coherent Mount sample. A stop
from either arm stops command progression for both arms, matching the existing
one-process/two-arm session intent.

## 7. Full planning and reference flow

The simulation session preserves the production process boundary: the panel
launches `humansl_sim` together with the same long-lived `planner_bridge`
process used by hardware. Planning requests and Cartesian results use the same
versioned serialized contracts and non-real-time reader/writer path. The
simulator does not link GPMP2 into its 500 Hz process.

The panel may submit right, left, or paired world-frame goals while simulation
is running. The planner worker processes each request outside the 500 Hz path
and captures one coherent snapshot:

- both measured joint configurations;
- the latest acceptable world-to-Mount pose, sequence, and timestamp;
- calibrated Mount-to-base transforms;
- production or experimental planning configuration;
- the enabled shared obstacle scene.

GPMP2 remains internally joint-space and returns timed `q(t), qdot(t)`. After
the existing validation and time scaling, the planner application projects
each dense sample using the frozen plan-time Mount pose:

```text
T_W_E(t) = T_W_M,plan T_M_B T_B_E(q(t))
V_W_E(t) = J_W(q(t)) qdot(t)
```

Only timed world-frame end-effector pose/twist and provenance cross into
control. Planned joint posture does not cross the boundary.

The result is published as a dual-arm trajectory pair with one trajectory ID.
A one-arm request pairs the planned trajectory with an explicit continuing
world hold for the other arm. Frame, timing, finite-value, start-continuity,
and freshness checks must pass before both sides activate on one common 500 Hz
cycle. The planner's inter-arm-clearance check must also pass before
publication, but its verdict applies only to GPMP2's internal planned joint
branches. It is not an activation-time guarantee about the link positions the
Cartesian controller will execute.

A failed, stale, or invalid replacement does not remove the active valid
trajectory or hold. Requests arriving during a solve are coalesced according
to the approved asynchronous-planning behaviour.

Live planner completion time is intentionally not part of deterministic
tracking evidence. Interactive sessions use the real asynchronous planner and
activate whenever its valid result arrives. Reproducible acceptance scenarios
first obtain a versioned output block from the real GPMP2 pipeline, then inject
that unchanged block at a scripted simulation tick. Planner-worker contract
tests separately exercise live solves, failure, and coalescing with controlled
completion events. Numeric tracking thresholds never depend on workstation
solve time or OS scheduling.

## 8. Controller equations and base motion

The shared controller keeps the existing approved world-frame law. Measured
world end-effector twist includes measured Mount motion:

```text
v_W_E = v_W_M
      + omega_W_M x (p_W_E - p_W_M)
      + (J_W qdot_measured)_linear

omega_W_E = omega_W_M
          + (J_W qdot_measured)_angular
```

The controller uses reference-minus-measurement pose and twist error:

```text
e_pose  = poseError(T_W_E,reference, T_W_E,measured)
e_twist = V_W_E,reference - V_W_E,measured
V_task  = Kp e_pose + Kd e_twist
qdot_raw = J_W,dls# V_task + N qdot_joint_limit
```

There is no separate explicit base-motion feedforward term. GPMP2 reference
twist enters only through Cartesian twist error.

## 9. Model and frame parity

MuJoCo uses the HumanSL arm geometry and explicit dual-arm Mount placement.
The required parity surface is:

- seven joints per arm in production order;
- revolute-axis direction and zero convention;
- joint bounds and velocity limits;
- world, Mount, right-base, left-base, and end-effector frame definitions;
- calibrated `T_M_B` for each arm;
- configured tool/TCP transform and collision geometry;
- units: metres, seconds, radians, metres/second, and radians/second.

The existing `msc_project` `pinch_site` is not silently treated as a HumanSL
tool frame. Its useful MuJoCo components may be migrated, but the resulting
model must be aligned to the production frame contract.

Final model-parity checks compare production Pinocchio and MuJoCo forward
kinematics over fixed and sampled joint states for both arms. Jacobian and
finite-difference checks independently validate frame direction and joint
ordering.

## 10. Simulation and Vicon clocks

Control and MuJoCo physics run deterministically at 500 Hz. Vicon behaviour is
selected independently.

### Ideal mode

- exact MuJoCo Mount pose and twist every `0.002 s`;
- no sampling latency, noise, occlusion, or dropout;
- intended as a mathematical and frame-debugging baseline.

### Realistic mode

- a simulated Vicon producer emits pose frames at 100 Hz;
- every frame has sequence, frame number, source/receive timestamps, reported
  latency, validity, and frame rate;
- configurable latency, pose noise, occlusion, and dropout are applied at the
  sensor boundary;
- the production Mount-twist estimator updates only when sequence advances;
- the 500 Hz execution core zero-order holds the coherent sample for the four
  intermediate control cycles while its age advances;
- realistic mode never reads MuJoCo Mount velocity directly.

The planner snapshots the newest coherent sample accepted by the same
freshness rules as execution.

Differentiating noisy 100 Hz pose and holding the estimate at 500 Hz is
expected to introduce visible twist steps/noise into the derivative term. This
is sensor-path evidence, not automatically a controller defect. Realistic-mode
telemetry and plots expose raw finite-difference twist, filtered twist, sample
sequence/age, Cartesian twist error, and resulting joint command so filtering
and control effects can be distinguished.

## 11. Scripted Mount motion

The first version supports repeatable scripted Mount motion only:

- static;
- translational sinusoidal disturbance;
- rotational sinusoidal disturbance;
- combined translation and rotation.

The panel exposes amplitude, frequency, axis, phase, duration, and random seed
where applicable. Defaults are conservative simulation values, not statements
about safe hardware motion.

Recorded Vicon replay and manual interactive Mount dragging are intentionally
deferred.

## 12. Obstacles and collision scene

Free space is the default. Shared-obstacle support exists behind an explicit
panel switch so it can be enabled for later planning and clearance work.

When enabled, one scene description supplies both:

- the planner collision representation; and
- corresponding MuJoCo visible/contact geometry.

This avoids displaying an obstacle that GPMP2 did not plan around, or planning
around an obstacle that MuJoCo does not contain. Executed contacts and
clearance are simulation monitoring evidence; they do not prove the
resolved-rate controller reproduced GPMP2's redundant joint branch.

GPMP2 collision, joint-limit, and inter-arm reports certify only its internal
`q(t)`. The pose/twist-only controller boundary deliberately discards planned
posture, and DLS plus joint-limit avoidance may select a different elbow
branch. Consequently:

- no planned-clearance result is described as executed clearance or safety;
- MuJoCo evaluates and logs executed link-to-link and link-to-obstacle
  clearance on the controller's actual simulated joint states;
- simulated contact can fail a scenario after the divergence is observed, but
  this is detection rather than a pre-execution guarantee;
- no planned `q(t)` or posture bias is added to the controller contract.

## 13. Panel workflow

The existing panel adds a separate Simulation session type. Simulation and
hardware sessions are mutually exclusive in panel state and launch controls.

The simulation surface provides:

- build and launch status for `humansl_sim`;
- start, pause, reset, and stop;
- MuJoCo viewer launch/attachment;
- right, left, and paired planner goals;
- mid-run asynchronous replanning;
- ideal/realistic Vicon selection;
- scripted Mount scenario controls;
- shared-obstacle enable/configuration;
- hardware-parity/experiment tuning selection;
- live high-level state and both-arm telemetry;
- run selection and post-run plot generation.

Reset clears active plans and controller/simulator state. It does not silently
resume or resubmit a previous trajectory.

## 14. Hardware-parity and experiment modes

### Hardware parity

- exact production controller, filter, limit, freshness, and stop
  configuration;
- simulation overrides are locked;
- the run preamble identifies the production configuration and model hashes.

The panel label means **production-code and configuration parity**, not proven
physical equivalence. Until the extracted hardware path completes separately
authorized robot validation, the panel and run preamble mark it as
`hardware refactor: offline-validated only`.

### Experiment

- starts from the production snapshot;
- permits clearly labelled temporary controller/filter/scenario overrides;
- does not silently write those values back into production configuration;
- logs every override and marks all plots as experiment-mode evidence.

Promoting an experimental value to production remains a separate explicit
configuration change and safety review.

Experiment mode supports controller-logic exploration and qualitative
stability analysis against the declared generic plant. Its Cartesian gains are
not hardware tuning results: MuJoCo position-actuator stiffness and damping are
chosen model dynamics, while the Kinova closes an unmodelled internal position
servo. Gain transfer requires separate physical identification or supervised
hardware tuning.

## 15. Failure behaviour

- Planner failure leaves the active valid trajectory or hold unchanged.
- Invalid, non-finite, wrongly framed, discontinuous, or stale trajectory
  replacements are rejected before activation.
- Brief Vicon staleness follows the production pause/hold behaviour.
- Prolonged Vicon staleness cancels active tracking, enters the approved world
  hold/re-anchor state, and requests replanning when permitted.
- A generic shared software-stop decision stops both simulated arms.
- A non-finite MuJoCo or controller state stops the simulation immediately.
- Simulator contact is logged and may fail a configured scenario, but is not
  misreported as a Kinova firmware fault.
- Reset is explicit and starts from a newly validated initial state.

A world-fixed hold can become kinematically infeasible during a sufficiently
large or long Mount disturbance. The first version does not invent an escape
planner or pass posture across the boundary. Null-space limit avoidance acts
while margin remains; if the integrated outward proposal reaches the existing
software boundary, the complete command frame holds and the shared session
stops with `joint_limit_warning`. The run is classified as an infeasible-hold
outcome. Automatic pre-boundary replanning is deferred unless separately
designed and approved.

## 16. Telemetry and plots

Hardware and simulation use the same semantic fields for:

- measured joint state;
- raw and limited joint velocity;
- integrated position command;
- measured/reference Cartesian pose and twist;
- world/Vicon freshness and Mount twist;
- trajectory IDs, activation, rejection, completion, cancellation, and replan;
- joint-limit margins, saturation, timing, and stop reason.

Simulation-only ground truth and contact fields are added in an explicitly
separate extension rather than masquerading as hardware measurements.

Every run records mode, code revision, model identifiers/hashes, configuration,
planner settings, Vicon settings, disturbance parameters, obstacle scene, and
random seed. The panel generates post-run plots for world pose error, twist,
joint commands/response, timing, Vicon age, limit margin, saturation,
clearance/contact, and planner/reference events.

Interactive viewer use and plots are the normal development workflow. The
full automated evidence suite is not run after every edit.

## 17. Verification strategy

Small targeted hardware-free tests are run during development so faults are
caught near the change. Comprehensive acceptance runs after the full pipeline
is assembled.

### Extraction regression gate

Before changing the hardware loop, the implementation freezes the exact source
revision and configuration and records pre-extraction characterization traces.
Because existing physical run logs are formats 9–11 and predate the current
format-13 world-Cartesian path, two evidence sources are required:

1. Usable historical robot CSV rows are converted into the subset of the new
   input contract they actually contain and characterize established
   integration, following-error, joint-boundary, timing, and stop semantics.
   Missing Cartesian-reference fields are never fabricated.
2. A hardware-free harness around the current pre-extraction format-13 code
   records complete synthetic per-cycle inputs and expected outputs for world
   measurement, reference transitions, Cartesian control, limiting,
   integration, and generic stop decisions.

After extraction, those same inputs are replayed through the shared core.
Discrete states, edge events, stop decisions, and limited commands must match
exactly; floating-point controller and kinematic fields use declared numerical
tolerances. This replay gate must pass before the hardware executable is
switched to the extracted core. Linking both targets to one library is an
additional structural check, not a substitute for replay equivalence.

Final acceptance covers:

1. both hardware and simulation targets link the same execution-core library;
2. production Pinocchio/MuJoCo model and TCP parity for both arms;
3. independent FK, Jacobian, and finite-difference validation of projected
   GPMP2 pose/twist;
4. paired atomic trajectory activation and one-arm-plan/other-arm-hold cases,
   with planner-path clearance labelled separately from executed simulation
   clearance;
5. 500 Hz control/physics and 100 Hz realistic-Vicon scheduling, zero-order
   hold, age, reset, and dropout behaviour;
6. static, translational, rotational, and combined scripted disturbances;
7. ideal versus realistic Vicon;
8. live planner solve/contract tests plus deterministic mid-run replacement,
   failure, invalid replacement, and coalescing using controlled completion
   events and planner outputs injected at scripted ticks;
9. injected stale Vicon, non-finite state, following error, limit, and stop
   cases;
10. panel build/launch/control/tuning/plot workflows;
11. C++ traces against the untouched Python reference implementation;
12. frozen-joint and measured-Mount-twist-disabled negative controls.

The final evidence reports world position/orientation mean, RMSE, and peak;
twist error; saturation; joint-limit margin; timing; collision clearance; and
stop/event outcomes.

Numerical pass thresholds are stated before the final suite runs. A threshold
may be derived from the independent Python baseline only when Python and C++
use the same robot model, actuator parameters, physics substeps, initial state,
reference, sensor mode, disturbance, and seed; those identities are recorded
with the result. Otherwise the Python run is a qualitative or algorithm-trace
comparison, not the numerical oracle. Thresholds also respect current
configured limits and declared scenario severity and are never chosen after
seeing the C++ result.

## 18. Evidence limits

Successful simulation verifies software architecture, frame/timing semantics,
and behaviour under the selected generic model. It does not establish:

- Kinova internal servo fidelity;
- unmodelled flex, backlash, cable loads, or wearable dynamics;
- real Vicon latency/noise/occlusion statistics unless configured from data;
- safe physical motion near a person;
- correct behaviour of firmware faults or Kortex communication;
- physical tracking performance.

In particular, simulated world-pose RMSE and peak error describe the selected
generic MuJoCo plant. They are useful for frame, timing, logic, and relative
scenario comparisons, but are not predictions of Kinova tracking error or
evidence that simulation-tuned gains will transfer.

Those remain separate supervised hardware-validation questions.

The implementation deliverable includes a hardware revalidation runbook for
the refactored executable, but does not execute it. Physical revalidation is a
later Level-2 operation requiring a fresh build audit, explicit authorization,
supervision, workspace/e-stop checks, conservative first targets, and review of
the resulting logs. Until then, the simulator and refactored hardware binary
remain offline-validated only.

## 19. Implementation boundary

Implementation begins only after Christian reviews this written specification
and approves an implementation plan. The implementation must preserve
unrelated working-tree changes, use test-first hardware-free development, and
must not run a robot-facing executable.
