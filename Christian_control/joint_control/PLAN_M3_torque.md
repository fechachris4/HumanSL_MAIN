# Milestone 3 — Torque control & full low-level control (plan)

**Date:** 2026-06-10 · **Status:** Proposed (next goal)

## Why / what exists already

The repo already has a working low-level **joint-torque** path (LOW_LEVEL servoing):
- `move.cpp:140 joint_impedance_control_single` — *named* impedance but is joint-space
  **computed-torque control**: reads cyclic feedback, computes
  `u = joint_impedance_controller(Dynamics, q,dq,ddq, q_d,dq_d,ddq_d, gains)` using the
  Pinocchio `M(q), C(q,dq), g(q)`, and sends `set_torque_joint(u[i])`.
- `move.cpp:69 joint_position_control_single` — the position counterpart (`set_position`).
- `joint_*_control_execution` wrap these in the 500 Hz loop over the `JointTrajectory` deque.

So "torque control" is **routing our MPC reference into the existing torque path** and
giving the seam a position/torque mode switch — not inventing torque control. Most of M3
is still laptop-testable with a toy/diagonal dynamics model; the real arm reuses the
existing Pinocchio `Dynamics` + `joint_impedance` path.

## Scope

**Laptop-testable now (this goal):**
1. Carry acceleration through the seam: add `ddq_des` to `JointSetpoint` (the MPC already
   computes `u0` = the optimal acceleration — expose it).
2. `JointDynamicsModel` interface: `M(q)`, `Cdq(q,dq)`, `g(q)`. Laptop impl
   `DiagonalRigidBody` (constant diagonal inertia, optional gravity term); robot impl is
   a thin adapter over the existing `Dynamics` (Pinocchio).
3. `ComputedTorqueController` (laptop analog of `joint_impedance_controller`):
   `tau = M(q)·(ddq_d + Kp·(q_d−q) + Kd·(dq_d−dq)) + Cdq + g`.
4. `TorqueSimBackend`: a real **dynamic** sim — forward dynamics
   `ddq = M⁻¹(tau − Cdq − g)`, integrated each tick. (This also closes the M1-review gap
   that the kinematic lag sim gives no confidence about dynamics.)
5. Mode switch: extend the toggle to `ControlMode { PositionSimple, PositionMpc,
   TorqueMpc }` (or compose a separate position/torque switch with the MPC on/off) —
   bumpless, like the M2 toggle.
6. Tests: computed-torque tracking converges in the dynamic sim; gravity compensation
   holds a configuration at rest; torque respects a torque limit; position↔torque switch
   is bumpless; the MPC reference + computed torque tracks a step.

**Robot-only (deferred until hardware):**
- The `JointTrajectory` adapter feeding the real `joint_position/impedance_control_execution`
  500 Hz thread, and enabling LOW_LEVEL torque mode via `ActuatorConfigClient` SetControlMode.
- Real gain tuning, dynamics identification, safety bring-up.

## TDD task outline

1. **`ddq_des` on `JointSetpoint`** + expose `u0` from the MPC. Test: MPC sets a finite
   `ddq_des`; position-mode users ignore it (no behavior change → existing tests stay green).
2. **`JointDynamicsModel` + `DiagonalRigidBody`.** Test: `M` SPD; `g` matches a known
   pendulum-ish closed form for the toy model; zero at a configured rest.
3. **`ComputedTorqueController`.** Test (open-loop against the model): with perfect model,
   `tau` produces exactly `ddq_d` (feedback-linearization identity) up to clamp.
4. **`TorqueSimBackend`.** Test: under gravity-only torque it falls; under
   gravity-compensation torque it holds; energy/limits sane.
5. **Closed loop:** `MpcJointController` → `ComputedTorqueController` → `TorqueSimBackend`
   converges to a joint goal and arrives at rest. Tune `Kp,Kd` (sweep, like M2's `w_v`).
6. **Mode switch** bumpless position↔torque; full harness green.
7. Docs + README + commit.

## Acceptance (M3 torque goal — done when)

1. Seam carries `ddq_des`; MPC exposes it.
2. `JointDynamicsModel` + `DiagonalRigidBody` + `ComputedTorqueController` +
   `TorqueSimBackend` implemented, Eigen-only.
3. Position/torque mode switch, bumpless.
4. Builds & runs on laptop, no robot; full harness exits 0 with the new torque tests.
5. Docs updated; committed.
6. A written mapping (in the README) from `ComputedTorqueController`/`TorqueSimBackend`
   to the existing `joint_impedance_control_single` + `Dynamics` for the robot bring-up.
