# World-frame Cartesian planning and control

**Date:** 2026-08-15  
**Status:** Approved design; not yet implemented  
**Scope:** Planner application boundary, Cartesian reference contract,
world-state estimation, controller integration, and migration. GPMP2 library
internals are unchanged.

## 1. Outcome

The production system has one controller law: a world-frame Cartesian
pose/twist controller. GPMP2 may continue to optimise a timed joint trajectory
internally, but the planner application converts the validated result into a
timed world-frame end-effector pose/twist trajectory before publishing it.
Neither planned joint positions nor a planned posture cross into control.

This replaces the hard-to-explain production split between joint-trajectory
tracking and Cartesian world hold. Startup hold, trajectory tracking, final
hold, and replacement-plan handover are reference states handled by the same
Cartesian law, not separate controller modes.

## 2. Decisions and boundaries

- GPMP2 remains outside the 500 Hz control loop.
- GPMP2 itself is not modified. It still solves for `q(t), qdot(t)`.
- The planner application is world-aware through a fresh Vicon snapshot of
  `world_T_mount`, written mathematically as `T_W_M`.
- The controller receives only timed world-frame end-effector pose and twist.
- The controller follows the Python simulation law in
  `/home/christian/msc_project/controller/frames.py` and
  `/home/christian/msc_project/controller/reactive_controller.py`.
- Mount velocity contributes to the measured end-effector world twist. There
  is no separate explicit base-motion feedforward term.
- The existing Kinova velocity limits, joint-boundary handling, command
  integration, following-error checks, fault handling, stops, and teardown
  remain the sole actuation path.

This supersedes the rollback-era decision that a timed joint trajectory must
be the planner/controller boundary. The rollback's reason still stands: world
conversion belongs at a correctly defined planner boundary rather than in an
ad-hoc controller adapter.

## 3. Frames, units, and notation

| Symbol | Meaning |
|---|---|
| `W` | Vicon world frame |
| `M` | tracked robot Mount segment frame |
| `B` | Kinova model base frame |
| `E` | end-effector frame |
| `T_A_B` | pose of frame `B` expressed in frame `A` |
| `V_A_B` | spatial twist of `B` expressed in frame `A` |

The calibrated rigid transform `T_M_B` relates the tracked Mount to the model
base. Therefore

```text
T_W_B = T_W_M T_M_B
T_W_E(q) = T_W_B T_B_E(q)
```

Positions use metres, linear velocities metres/second, angular velocities
radians/second, joint positions radians, and joint velocities radians/second.
Quaternions are unit quaternions with an explicitly documented storage order
at each serialized boundary.

World-frame conversion matters in both halves of the system:

- the planner uses `T_W_M` at plan time to formulate and export a world-frame
  reference;
- the controller uses the latest `T_W_M`, its calibrated `T_M_B`, and measured
  Mount twist to calculate the actual end-effector pose and twist in `W`.

The planner snapshot and every published trajectory carry Vicon provenance so
the controller and logs can identify which world estimate generated a plan.

## 4. Architecture and data flow

```text
controller process
  Vicon acquisition thread
    -> atomic MountState {T_W_M, V_W_M, frame, sequence, timestamps, age, valid}
         |                               |
         | latest state (ZOH)            | immutable planning snapshot
         v                               v
  500 Hz controller                 non-real-time request writer
    measure T_W_E and V_W_E              |
    interpolate world reference           | request FIFO
    Cartesian PD + DLS + null space       v
    existing safety/actuation path   planner_bridge process
                                      asynchronous GPMP2 q(t), qdot(t)
                                      validate + time-scale
                                      dense FK/Jacobian projection
                                            |
                                            | Cartesian trajectory FIFO
                                            v
                                      non-real-time controller input thread
                                            |
                                            | atomic latest-valid trajectory
                                            v
                                      500 Hz controller
```

Planner and controller remain separate processes, preserving the existing
`planner_bridge`/FIFO boundary. The control loop publishes a fixed-size
planning request into a wait-free single-producer/single-consumer slot; a
non-real-time writer serializes the latest request, including measured joints
and the coherent Mount snapshot. The long-lived planner worker coalesces
pending requests and returns a complete Cartesian trajectory block. Its input
thread validates that block before atomically replacing the latest candidate.

Vicon I/O, planning, terminal I/O, pipe/file I/O, and ordinary logging remain
outside the 500 Hz thread. The cyclic path only reads immutable snapshots and
performs fixed-size copies and arithmetic.

## 5. Mount-state and velocity estimation

The Vicon thread publishes one coherent `MountState` containing pose, twist,
timing, sequence, age, and validity. Twist is estimated only when a new Vicon
frame advances. Re-reading a zero-order-held frame must never create another
finite-difference sample.

For consecutive valid frames `k-1` and `k`, with Vicon frame rate `f_V`:

```text
dt = (frame_k - frame_(k-1)) / f_V
v_W_M,raw = (p_W_M,k - p_W_M,k-1) / dt
omega_W_M,raw = Log(R_W_M,k R_W_M,k-1^T) / dt
```

The raw twist is filtered causally:

```text
alpha = 1 - exp(-dt / tau_filter)
V_W_M,k = alpha V_W_M,raw + (1-alpha) V_W_M,k-1
```

`tau_filter` is configurable and selected from Vicon replay evidence before
the estimate affects physical motion. The estimator resets across invalid,
out-of-order, discontinuous, or prolonged-stale intervals instead of
differentiating across them.

## 6. World-aware GPMP2 boundary

Each planning request captures one fresh `MountState` snapshot. The planning
application uses that fixed `T_W_M` (and calibrated `T_M_B`) consistently for
world goals, obstacles, and robot geometry during that solve. GPMP2 still
optimises and validates `q(t), qdot(t)` internally.

Only after final validation and time scaling, the planner densely samples the
continuous joint trajectory and projects every sample to:

```text
T_W_E,d(t) = FK_W(q(t); T_W_M at plan time)
V_W_E,d(t) = J_W(q(t); T_W_M at plan time) qdot(t)
```

Dense projection is required. Exporting sparse poses and independently
interpolating them in SE(3) can produce a path different from the GPMP2 joint
path between support states.

The controller-facing trajectory contains no `q_ref`, `qdot_ref`, planned
posture, elbow reference, or link reference. A reference sample contains:

```text
trajectory_id
planner_vicon_sequence
frame = WORLD
t_from_trajectory_start_s
position_W_E_m
orientation_W_E_quaternion
linear_velocity_W_E_mps
angular_velocity_W_E_radps
arrival_eligible
```

Trajectory publication is atomic. Invalid, incomplete, non-monotonic, stale,
or wrongly framed trajectories are rejected before they can replace the
active reference.

## 7. Controller law: exact simulation convention

At 500 Hz, measured world pose and world-rotated arm Jacobian are:

```text
T_W_E = T_W_B T_B_E(q_measured)
J_W = rotateJacobianToWorld(J_B, R_W_B)
```

The measured world twist includes Mount motion transported to the
end-effector and arm motion relative to the base:

```text
v_W_E = v_W_M
      + omega_W_M x (p_W_E - p_W_M)
      + (J_W qdot_measured)_linear

omega_W_E = omega_W_M
          + (J_W qdot_measured)_angular
```

Pose and twist errors are reference minus measurement:

```text
e_pose  = poseError(T_W_E,d, T_W_E)
e_twist = V_W_E,d - V_W_E
V_task  = Kp e_pose + Kd e_twist
```

The joint command is produced by the existing damped least-squares inverse
kinematics and null-space joint-limit avoidance:

```text
qdot_raw = J_W,dls# V_task + N qdot_joint_limit
```

Reference twist is present only through `e_twist`, exactly as in the Python
simulation. In particular, there is no additional `-V_base,E` term and no
second base-motion feedforward path.

## 8. Reference states and trajectory time

The same controller law runs in every state:

- **Startup hold:** capture the measured valid world pose and command zero
  reference twist.
- **Active trajectory:** interpolate the published world pose/twist at the
  reference clock.
- **Final hold:** retain the final world pose and command zero reference twist.
- **Plan replacement:** switch atomically to a validated trajectory whose
  handover sample satisfies continuity tolerances.

The reference clock advances only while the world estimate is sufficiently
fresh. It is not raw wall-clock time.

## 9. Vicon loss and recovery

Initial defaults preserve the current thresholds:

- fresh maximum age: `0.05 s`;
- prolonged-stale/re-anchor threshold: `0.20 s`.

For a brief stale interval, the trajectory clock pauses, the current world
pose reference is retained, measured pose is zero-order-held, and stale Mount
twist decays smoothly to zero. Recovery resumes smoothly from the paused
trajectory time; it does not catch up to wall clock.

For a prolonged stale interval, the active trajectory is cancelled. After a
fresh valid recovery, the system captures the current measured world pose as
a hold and requests a new plan. It does not resume the old plan from a base
state that may have moved unobserved.

Every transition and its freshness reason is logged. Corrections re-engage
continuously rather than stepping when Vicon returns.

## 10. Asynchronous replanning

Planning is event-driven and asynchronous, one solve per arm at a time. A
request uses the latest fresh world snapshot available when the solve starts.
If newer requests arrive during a solve, they are coalesced into the newest
pending request rather than queued without bound.

The old valid trajectory or hold remains active while a replacement is being
computed. A new trajectory takes effect only after frame, timing, validity,
freshness, and handover checks pass. Planner failure cannot remove the current
safe reference.

## 11. Safety invariants

Both trajectory following and holds pass through the unchanged chain:

```text
Cartesian reference
-> Cartesian feedback law
-> qdot_raw
-> velocity clamp and joint-boundary handling
-> position-command integration
-> command-lead/following-error checks
-> Kortex command
```

Kortex faults, startup gates, stop requests, cyclic timing checks, and teardown
continue to dominate references. No planner or Cartesian component may bypass
them. No robot-facing command is part of this migration unless separately
authorized.

## 12. Redundancy limitation

A 7-DoF arm has redundant joint postures for one end-effector pose. Once the
boundary exports only end-effector pose/twist, the controller is not required
to reproduce GPMP2's planned joint branch. GPMP2 collision and joint-limit
checks prove properties of its internal joint path; they do not, by
themselves, guarantee the whole-arm posture executed by resolved-rate control.

The existing null-space joint-limit avoidance remains, but no planned posture
is smuggled across the boundary. This accepted limitation must be stated in
the architecture and thesis rather than hidden. No offline rollout validator
is included in this approved migration.

## 13. Migration slices

1. **Observe Mount twist.** Add advancing-frame differentiation, filtering,
   snapshot metadata, replay tests, and logs. It must not affect commands yet.
2. **Expose world Cartesian plans.** Make the planner application world-aware,
   densely project validated GPMP2 output, and publish the explicit Cartesian
   contract while retaining the current controller path.
3. **Use one production Cartesian controller.** Wire the new reference into
   the simulation-matched law and the existing safety path; then remove the
   production joint-reference controller branch, `q_ref` tracking law, idle
   joint-hold workaround, and joint trajectory at the planner/controller
   boundary. GPMP2's internal joint trajectory remains.
4. **Add asynchronous replanning and dropout transitions.** Implement request
   coalescing, atomic handover, clock pause/resume, prolonged-stale
   cancellation, re-anchored hold, and replan request.

Each slice has a separate implementation plan, equations-to-symbol map,
hardware-free tests, and review gate. Existing unrelated working-tree changes
are preserved. No commits, pushes, or hardware runs occur without Christian's
explicit authorization.

## 14. Verification and acceptance

The fresh pre-change hardware-free baseline on 2026-08-15 is 35/35 complete
standalone-project tests passing:

- basic control: 14/14;
- planner bridge: 16/16;
- Vicon: 5/5.

The baseline was obtained by building each existing `build/` tree and running
its complete `ctest --output-on-failure` suite. Building the controller target
did not run it and no robot-facing command was executed.

New tests must independently cover:

- frame/sign correctness under pure Mount translation and rotation;
- measured end-effector world twist against the Python implementation on
  shared numeric fixtures;
- no derivative update for repeated Vicon frames;
- estimator reset and filter behaviour across invalid/out-of-order data;
- dense projected poses/twists against FK and `J qdot` at sampled GPMP2 states;
- contract rejection for wrong frame, timing, freshness, or non-finite data;
- one Cartesian law for startup, tracking, final hold, and replacement;
- reference-clock pause, smooth brief recovery, prolonged-stale cancellation,
  and replan request;
- asynchronous coalescing and atomic replacement;
- preservation of all existing safety and actuation checks.

The design is accepted as implemented only when production has one Cartesian
controller path, no planned joint posture crosses the boundary, planner output
is world-frame pose/twist with provenance, Mount velocity is used only in the
measured world twist as specified above, the relevant hardware-free suites
pass, and architecture documentation states the redundancy limitation.
