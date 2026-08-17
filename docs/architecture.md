# Repository architecture

HumanSL is a research control stack for backpack-mounted Kinova Gen3 arms. Its
production objective is one world-frame Cartesian behavior: follow or hold an
end-effector pose in the room while the wearer and robot base move.

## Active components

| Component | Location | Owns | Does not own |
| --- | --- | --- | --- |
| Arm runtime | `Christian_control/basic_control/` | Vicon/robot snapshots, one 500 Hz Cartesian controller, limits, position-command integration, Kortex lifecycle, telemetry | GPMP2 solving |
| Planner | `Christian_control/planner_bridge/` | world-aware planning problem, GPMP2 joint solve, validation, dense world pose/twist projection, typed non-real-time worker | Kortex or the 500 Hz loop |
| Motion capture | `Christian_control/vicon/` | Vicon acquisition, snapshot validation, filtered Mount pose/twist publication | control decisions or robot commands |
| Shared contracts | `Christian_control/cartesian_contract/` | typed planning-request and world-Cartesian trajectory records; preview-only text adapters are separate | SDK or planner internals |
| Panel/tooling | `Christian_control/tools/panel/`, scripts | configuration editing, replay, plots, diagnosis | command generation in the cyclic thread |
| Models/config | `Christian_control/*/config/` | authoritative URDF and planner/goal/limit configuration | live state |

The inherited root planning/execution trees are frozen and absent from active
builds. There is no repository-root build; each active CMake project configures
independently.

## Current end-to-end flow

```text
Vicon thread                         Kortex feedback
  T_W_M, V_W_M, age, sequence          q, qdot
         \                               /
          +------ 500 Hz measurement ---+
                     T_W_E, J_W, V_W_E
                              |
                              | fresh startup/recovery edge
                              v
                    fixed-size request slot
                              |
                    typed request slot
                              |
                              v
                 one in-process planner worker per arm
                 newest pending request wins
                              |
          world-aware GPMP2 joint solve (internal only)
                              |
                validate + dense FK/Jqdot projection
                              |
              move typed WorldCartesianTrajectory
                              |
                    typed trajectory mailbox
                              |
                              v
 measured T_W_E/J_W/V_W_E -> Cartesian reference source
                              |
                  sole Cartesian pose/twist law
                              |
            raw qdot -> clamp -> joint boundary handling
                              |
                position integration -> Kortex send
```

The planner and controller use the same immutable `T_W_M` provenance for a
planning generation. World conversion matters on both sides:

- planner: it formulates the model, goal, obstacle grid, validation, and
  published end-effector trajectory in world frame;
- controller: it rebuilds measured `T_W_E`, `J_W`, and `V_W_E` every cycle from
  the latest Vicon Mount state and arm feedback.

The current tracked `Mount` segment is treated as the model Mount frame.
`T_M_B` is fixed by the mounted dual-arm URDF. A distinct marker-cluster
calibration is not yet represented and therefore remains a physical-validation
dependency.

## Planner/controller contract

GPMP2 remains internally joint-space. Its final validated and time-scaled
`q(t), qdot(t)` is projected outside GPMP2 through Pinocchio:

```text
T_W_E(t) = T_W_M T_M_B T_B_E(q(t))
V_W_E(t) = J_W(q(t)) qdot(t)
```

Only timed world pose, world twist, arrival eligibility, trajectory identity,
and planner Vicon sequence cross into control. No planned joint posture crosses
the boundary. This is why there is one explainable production controller rather
than a joint follower plus a Cartesian compensator.

Planning requests move the other way and contain only the fresh snapshot needed
to construct a new problem: request/arm identity, Vicon sequence/frame/time/age,
`T_W_M`, and measured seven-joint start state. They do not authorize robot
motion; the controller independently validates and splices returned trajectories.

## Controller equation

The runtime mirrors the Python simulation:

```text
e_pose = [p_W_E,d - p_W_E; log(R_W_E,d R_W_E^T)]
e_twist = V_W_E,d - V_W_E
V_task = Kp e_pose + Kd e_twist
qdot_raw = J_W^T (J_W J_W^T + lambda^2 I)^-1 V_task + qdot_null
```

Measured Mount twist contributes through rigid-body transport in measured
`V_W_E`, including `omega_W_M x (p_W_E - p_W_M)`. There is no separate
base-motion feedforward addition. Planned twist contributes through
`e_twist`; when the terminal reference is stationary, it is zero.

## Execution-core extraction status

The arm runtime's per-cycle command pipeline is extracted into one
hardware-independent static library, `humansl_execution_core`, whose entry
point is `ArmExecutionCore`
(`Christian_control/basic_control/src/ExecutionCore.h`). The hardware
`controller` executable and the hardware-free test probe link the same
library, so hardware and offline builds share one set of control
mathematics; a registered linkage test fails if a Kortex or Vicon SDK
symbol enters the core archive.

Evidence discipline: shared linkage plus the frozen pre-extraction
characterization replay establishes software equivalence — the reorganized
code computes the same commands as the pre-extraction code on recorded
inputs. It does not establish physical equivalence, which no offline test
can. The current hardware refactor is therefore **offline-validated only**
until a separately approved supervised run following
`Christian_control/docs/runbooks/execution-core-hardware-revalidation.md`;
that runbook is a procedure, not an authorization.

## Timing and failure ownership

- The 500 Hz thread performs fixed-size arithmetic, one Vicon-slot read, one
  request-slot publish on an edge, one Kortex exchange, and one log-slot push.
- Vicon SDK calls, GPMP2, terminal output, and CSV writes stay outside the
  cyclic path. The cyclic path performs only fixed-size request-slot publish;
  no planner serialization or file I/O occurs there.
- Each arm has at most one GPMP2 solve active. While it solves, newer pending
  requests coalesce latest-wins.
- A trajectory is visible to control only after the planner has completed its
  existing validation/projection path and moved a typed object into the
  mailbox. A failed solve publishes nothing.
- Brief Vicon staleness pauses reference time and decays the retained Mount
  twist from the greater of source-age excess and continuously invalid time.
  Fresh recovery before 200 ms resumes without catch-up.
- At 200 ms stale, the trajectory is cancelled. Fresh recovery captures a new
  world hold, invalidates pre-gap provenance, and emits one replan request.
- Every controller result enters one shared velocity-limit, joint-boundary,
  position-integration, following-error, fault, and teardown path.

## Runtime processes and artifacts

`planner_bridge/scripts/run_session.sh` starts one controller process after an
explicit `GO` gate. The controller owns one typed planner worker per selected
arm. Session folders retain planner diagnostics, controller output,
configuration snapshots, run CSVs, and process metadata; no production FIFO or
trajectory publication file is created.

The controller is the only robot-facing binary in this workflow. Planner,
Vicon, parsing, projection, and all CTest suites are hardware-free unless a
tool is explicitly documented otherwise.

## Documentation ownership

- Exact controller behavior and supervised use:
  `Christian_control/basic_control/README.md`.
- Supervised hardware revalidation of the extracted execution core:
  `Christian_control/docs/runbooks/execution-core-hardware-revalidation.md`.
- Current implementation design and acceptance criteria:
  `docs/superpowers/specs/2026-08-15-world-cartesian-planner-controller-design.md`.
- Detailed implementation sequence:
  `docs/superpowers/plans/2026-08-15-01-vicon-mount-twist.md` through
  `2026-08-15-04-asynchronous-replanning.md`.
- The older `docs/architecture_and_debugging_audit.md` body is historical
  evidence; its 2026-08-16 addendum identifies the architecture that supersedes
  its joint/world-hold split.
