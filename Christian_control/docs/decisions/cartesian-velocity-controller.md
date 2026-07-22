# Cartesian velocity controller (damped least squares, VELOCITY mode)

Date: 2026-07-20
Status: superseded same day by `resolved-rate-position-integration.md` —
hardware proved low-level VELOCITY mode has no gravity compensation
(gravity-loaded joints drift and ignore commands). The Cartesian DLS
controller described here survives; only the actuation layer changed.
(Originally: accepted, superseding `single-loop-controller.md`.)

## Decision

Replace the joint-position cyclic loop with a Cartesian position
P-controller resolved to joint velocities, specified by Christian:

    e   = p_desired − p(q)
    v_d = Kp · e
    q̇   = Jpᵀ (Jp Jpᵀ + λ² I₃)⁻¹ v_d      (damped least squares)

streamed at 1 kHz as per-actuator **VELOCITY-mode** commands
(`src/control/Loop.cpp::RunCartesianVelocityLoop`). p_desired is typed on
stdin (x y z, meters, base frame); FK and the translational Jacobian come
from the **same** measured q each cycle (Pinocchio, `LOCAL_WORLD_ALIGNED`).
The DLS solve follows Pinocchio's official IK example: build JJᵀ, add λ² on
the diagonal, `LDLT` solve — no explicit inverse (`src/math/Dls.h`,
hardware-free-tested).

This reintroduces a Cartesian/reactive controller — the class removed
2026-07-17 after hardware faults (`reactive-control-removal.md`). The
reintroduction is Christian's explicit design decision; the differences
from the removed controller and the accepted risks are recorded here.

## Hardware findings this builds on (smoke tests, run 2026-07-20, deleted)

1. High-level `SendJointSpeedsCommand` works in SINGLE_LEVEL_SERVOING;
   stopped by the all-zero command.
2. Low-level per-actuator `ControlMode::VELOCITY` works: switch via
   `ActuatorConfig` (TCP, device ids 1–7) **after** a first holding frame,
   stream `set_velocity` + `set_position(measured)` frames, restore
   POSITION mode before leaving low-level. Filling the velocity field
   without the mode switch does nothing.
3. A crashed low-level run leaves the arm latched in LOW_LEVEL_SERVOING
   across sessions; high-level commands then fail with "Invalid command for
   the current servoing mode". Setting SINGLE_LEVEL explicitly recovers.

## Accepted risks (explicit design choices, this version)

- **The only limiting is a per-joint velocity clamp** (added same day):
  after the DLS solve, each q̇_i is independently clipped to
  ±`kModelVelocityLimitsDegS[i]` (79.6 deg/s joints 1–4, 69.9 deg/s joints
  5–7 — the URDF/model limits). No Cartesian velocity limiting, no
  acceleration limiting, no workspace clipping. Two consequences accepted:
  (1) when any joint saturates, the Cartesian direction of motion is
  distorted; (2) the clamp sits ABOVE the base's enforced 50 deg/s soft
  limit, whose behavior in VELOCITY mode is unverified — the clamp bounds
  the controller's worst case, it does not keep commands inside the base's
  own limit. A distant p_desired still commands large (now bounded)
  velocities; Kp and sensible typed targets remain the real speed control.
- **No reachability check** on typed targets: the controller pushes toward
  an unreachable target until the operator retargets/stops or the arm
  faults. This is the failure mode that contributed to the 2026-07-17
  removal; accepted consciously for this version.
- **Velocity-mode drift**: Kinova's trackers document joints drifting under
  gravity at commanded-zero velocity (kortex #42) — "hold" is now an active
  velocity command, not a position setpoint.
- Known real-time deviation: `convertJointAnglesToConfig` returns a small
  heap-allocated vector each cycle (Pinocchio boundary). Accepted for
  research-code simplicity; everything else in the loop is allocation-free
  (fixed-size Eigen, preallocated 6×nv Jacobian workspace, preallocated
  ring log).

## Safety mechanics kept

- Readiness gate before takeover; live fault → no takeover.
- First cyclic frame holds (positions = measured, velocities = 0) BEFORE
  the control-mode switch.
- **Zero-command settling phase** (added after a cycle-1 joint fault on
  hardware, 2026-07-20): the loop starts immediately after the VELOCITY
  mode switch — no 40 ms sleep, so the arm is never in low-level servoing
  without cyclic exchanges — and streams exactly-zero velocities for the
  first 100 cycles before p_desired is seeded from fresh feedback and the
  controller activates. Diagnostic property: a fault during settling is a
  Kinova mode/setup problem by construction (the controller has produced
  no command); a fault after a target is accepted implicates the
  controller, limits, or robot configuration.
- Stop policy unchanged: live actuator fault, non-JOINT_FAULT base fault,
  or leaving LOW_LEVEL_SERVOING stops the loop; latched JOINT_FAULT alone
  is a post-loop note.
- Shutdown on every exit path, each step best-effort, in this order: final
  zero-velocity frame → all actuators back to POSITION mode →
  SINGLE_LEVEL_SERVOING → RAII session close. First failure preserved.
- No printing/allocation/file I/O in the loop; decoded report after.

## Configuration

`Config.h`: `kKpCartesian` = 1.0 /s, `kDlsLambda` = 0.1,
`kEndEffectorFrame` = "EndEffector_Link" (flange — the robot's own TCP
report sits ~0.12 m further along tool z), `kCyclePeriod` = 1 ms.
Retired with the position loop: `kMaxCommandSpeedDegS`,
`kJointPositionLimitsDeg`, `MoveTowards`, joint-target parsing (git:
this commit's parent).
