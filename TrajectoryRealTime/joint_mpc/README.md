# joint_mpc — joint-control MPC seam (Milestone 1)

A minimal, **laptop-testable** joint-control abstraction for the Kinova Gen3, built so a
Model Predictive Controller can be dropped in later without touching the robot I/O.

It is deliberately **Eigen-only** (plus pthread): it does **not** include
`TrajectoryGeneration/utils.h` (which pulls in GTSAM/GPMP2/yaml-cpp), so it builds in
seconds and runs with **no robot and no heavy dependencies**.

## What's here

| File | Role |
|------|------|
| `include/joint_types.h` | `JointState`, `JointSetpoint`, `JointLimits` |
| `include/gen3_limits.h` | Gen3 limits, mirroring `config/joint_limits.yaml` |
| `include/joint_controller.h` | `JointController` — **the MPC seam** (`compute(state, goal) -> setpoint`) |
| `include/controllers.h` | `HoldController`, `RateLimitedController` (trivial M1 controllers) |
| `include/sim_backend.h` | `SimBackend` — first-order-lag kinematic arm; all velocity/position clamping |
| `include/setpoint_queue.h` | `SetpointQueue` — laptop mirror of the real `JointTrajectory` deque/replan seam |
| `include/control_driver.h` | `ControlDriver` — deterministic receding-horizon loop + CSV logging |
| `test/test_joint_control.cpp` | Standalone test harness (no gtest) |

## Build & test

```bash
cd TrajectoryRealTime/joint_mpc
cmake -S . -B build && cmake --build build
./build/test_joint_control      # prints PASS/FAIL per check, exits 0 if all pass
```

Running the harness writes `run_log.csv` (`t, q*, qdes*, goal*`) for the end-to-end run,
which you can plot to see the controller converge.

## How MPC plugs in (Milestone 2)

Implement `JointController` as `MpcJointController` (double-integrator joint model,
tracking/effort/terminal cost, position/velocity/accel constraints, a QP solver such as
OSQP). It uses the **same** `compute(state, goal) -> JointSetpoint` contract — the sim
backend, queue, and driver are unchanged.

## How it reaches the real arm (Milestone 2)

A thin adapter (the only file that includes `utils.h`/GTSAM) maps `SetpointQueue` onto
the real `JointTrajectory` deque consumed by `joint_position_control_execution()` at
500 Hz, using the existing `new_trajectory_ready`/`trajectory_mutex` replan seam. The
core above is reused verbatim.

See `Christian_control/joint_control/DESIGN.md` and `PLAN.md` for the full design.
