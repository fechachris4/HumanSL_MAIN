# joint_mpc — joint-control MPC seam (Milestones 1 & 2)

A minimal, **laptop-testable** joint-control abstraction for the Kinova Gen3, with an
Eigen-only MPC and a runtime on/off toggle, built so the real arm I/O drops in later.

It is deliberately **Eigen-only** (plus pthread): it does **not** include
`TrajectoryGeneration/utils.h` (which pulls in GTSAM/GPMP2/yaml-cpp), so it builds in
seconds and runs with **no robot and no heavy dependencies**.

## What's here

| File | Role |
|------|------|
| `include/joint_types.h` | `JointState`, `JointSetpoint`, `JointLimits` |
| `include/gen3_limits.h` | Gen3 limits, mirroring `config/joint_limits.yaml` |
| `include/joint_controller.h` | `JointController` — **the MPC seam** (`compute(state, goal) -> setpoint`) |
| `include/controllers.h` | `HoldController`, `RateLimitedController` (trivial controllers) |
| `include/mpc_controller.h` | **`MpcJointController`** — Eigen-only condensed-QP MPC (M2) |
| `include/switchable_controller.h` | **`SwitchableController`** — runtime simple↔MPC toggle, bumpless (M2) |
| `include/sim_backend.h` | `SimBackend` — first-order-lag kinematic arm; all velocity/position clamping |
| `include/setpoint_queue.h` | `SetpointQueue` — laptop mirror of the real `JointTrajectory` deque/replan seam |
| `include/control_driver.h` | `ControlDriver` — deterministic receding-horizon loop + CSV logging |
| `test/test_joint_control.cpp` | Standalone test harness (no gtest) — 30 checks |

## Build & test

```bash
cd TrajectoryRealTime/joint_mpc
cmake -S . -B build && cmake --build build
./build/test_joint_control      # prints PASS/FAIL per check, exits 0 if all pass
```

The harness writes `run_log.csv` / `mpc_run.csv` (`t, q*, qdes*, goal*`); plot them to
see the rate-limited ramp vs. the MPC's smooth, damped approach.

## The MPC (Milestone 2)

`MpcJointController` is an **Eigen-only condensed quadratic MPC — no external solver**.
Per joint it models a double integrator on its own reference state, minimizing
position-tracking + terminal + velocity-damping + effort cost over a horizon, solved as
a once-factorized SPD linear system (LLT) reused for all joints each tick. The first
optimal input is applied via semi-implicit Euler with velocity/position clamps. It
implements the **same** `compute(state, goal) -> JointSetpoint` contract, so the sim
backend, queue, and driver are unchanged.

Known limitation: it is a **feedforward optimal reference generator** — after `reset()`
seeds it from the measured state, it integrates its own reference and does not use
measured feedback, so there is no disturbance rejection (the convergence guarantee is on
the reference, not the measured arm). Closed-loop feedback MPC and hard in-QP constraints
(OSQP) are future upgrades. See `DESIGN_M2.md`.

## The toggle (Milestone 2)

`SwitchableController` owns the simple controller and the MPC and delegates `compute()`
to whichever is active. `set_use_mpc(on, state)` flips it and **bumplessly** resets the
newly-active controller to the measured state (position *and* velocity), so a mid-motion
switch produces no jump. This is what a "MPC on/off" voice/CLI command drives.

## How it reaches the real arm (Milestone 3)

Two paths already exist in `move.cpp`: position (`joint_position_control_single`) and
**torque** (`joint_impedance_control_single`, which is joint-space computed torque via the
Pinocchio `Dynamics` M/C/g). M3 routes this seam into them: a thin adapter maps
`SetpointQueue` onto the real `JointTrajectory` deque consumed at 500 Hz via the existing
`new_trajectory_ready`/`trajectory_mutex` replan seam, and a `ComputedTorqueController` +
dynamic `TorqueSimBackend` add a laptop-testable torque path with a position/torque mode
switch. See `Christian_control/joint_control/PLAN_M3_torque.md`.

See `Christian_control/joint_control/DESIGN.md`, `DESIGN_M2.md`, `PLAN.md`, and
`PLAN_M3_torque.md` for the full design.
