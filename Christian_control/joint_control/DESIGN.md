# Joint-Control MPC Seam — Design Spec

**Date:** 2026-06-10
**Status:** Approved design (Milestone 1), revised after codebase inspection
**Author:** Christian Akabueze (with Claude)

## Problem & Motivation

The goal is a basic control app for Kinova Gen3 7-DoF arms that (eventually) accepts
voice commands and makes it easy to apply **Model Predictive Control (MPC)** to the
robot, starting with **joint control**. No robot hardware is available during initial
development, so the first deliverable must be **fully testable on a laptop**.

The existing `Christian_control/basic_control` tool is high-level, supervised,
point-to-point control via the Kortex `Base` API (`ExecuteAction`). It hands a goal to
the arm's own planner and waits — there is no per-tick command stream, so it is the
wrong layer for MPC. MPC must compute a command every control tick and stream it down.

The repo already contains the low-level half: `TrajectoryRealTime` runs a ~500 Hz joint
loop (`joint_position_control_execution()`) that consumes a `JointTrajectory` deque and
streams it to the arm, with built-in mid-execution replanning
(`new_trajectory_ready` / `trajectory_mutex`). That deque + replan seam is the natural
MPC injection point **at the robot**.

## Scope Decomposition

The full vision is three independent subsystems: (1) voice → command, (2) a control
abstraction MPC can target, (3) the MPC controller itself. They are technically
unrelated and must not be specced together.

**This spec covers Milestone 1 only: the control abstraction (joint-control seam),
delivered as a laptop-testable foundation.** Voice and the QP MPC plug into it later.

## Key Decisions

| Decision | Choice | Rationale |
|---|---|---|
| First milestone | Control abstraction | Keystone both voice and MPC depend on; testable without robot |
| Command level | Joint **position setpoints** (`q_des`, optional `dq_des`) | Matches existing 500 Hz loop, which consumes position waypoints; safest first rung |
| No-robot backend | **First-order-lag kinematic sim** | ~50 lines, no deps, enough to watch a controller converge and exercise limits |
| Integration target | The existing `TrajectoryRealTime` `JointTrajectory` deque/replan seam (via an adapter) | Compatible with the real control stack; the actual deployment path |
| M1 controller | **Trivial `RateLimitedController`** (QP MPC = Milestone 2) | De-risks plumbing independently of solver tuning |
| M1 concurrency | **Single-threaded, steppable** units | Deterministic, unit-testable; the threaded real-time wrapper is a robot-time concern |

## Critical Constraint Discovered During Inspection

`JointTrajectory` is declared in `TrajectoryGeneration/include/utils.h`, which
`#include`s GTSAM, GPMP2, yaml-cpp, and ezc3d. **Including it is not "Eigen-only."**

Therefore the laptop-testable core must **not** depend on `utils.h`. The core defines
its own minimal `Eigen::VectorXd`-based types and a self-contained setpoint queue.
A single thin **adapter** (the only file that includes `utils.h`/GTSAM) bridges the
core's setpoint stream to the real `JointTrajectory` deque — and is compiled **only**
in the full/robot build, never in the laptop build.

This yields two compile paths from one codebase:

- **Laptop build** (`TrajectoryRealTime/joint_mpc/`, own standalone `CMakeLists`,
  Eigen + pthread only): compiles and runs the harness + tests today.
- **Robot build** (future, M2): the adapter wires the same core into `main.cpp`'s real
  500 Hz execution thread via the replan seam.

The core and its setpoint contract are identical on both paths; only the backend swaps.

## Architecture — Units (all Eigen-only, header-only, single-threaded)

Location: `TrajectoryRealTime/joint_mpc/` (honours "inside TrajectoryRealTime").

### Shared types — `include/joint_types.h`
```cpp
struct JointState    { Eigen::VectorXd q, dq; double t = 0.0; };
struct JointSetpoint { Eigen::VectorXd q_des, dq_des; };
struct JointLimits   { Eigen::VectorXd q_lower, q_upper, dq_max; };
```

### Gen3 limits — `include/gen3_limits.h`
Hardcoded constants mirroring `config/joint_limits.yaml` (radians; joints 1/3/5/7 are
continuous → ±1e20; velocity ±0.8727 rad/s). Avoids a yaml-cpp dependency in the core.

### `JointController` interface — the MPC seam — `include/joint_controller.h`
```cpp
class JointController {
public:
    virtual ~JointController() = default;
    virtual JointSetpoint compute(const JointState& s,
                                  const Eigen::VectorXd& q_goal) = 0;
    virtual int horizon_steps() const { return 1; }
};
```
M1 implementations (`include/controllers.h`):
- `HoldController` — returns current `q` (safety baseline).
- `RateLimitedController` — steps `q` toward `q_goal` at a capped per-tick speed.

M2's `MpcJointController` implements the **same interface**.

### Simulated backend — `include/sim_backend.h`
`SimBackend::apply(const JointSetpoint& cmd, double dt)` integrates one tick of a
first-order lag `q += (clamp(q_des) - q) * (dt/tau)`, then enforces velocity and
position clamps from `JointLimits`, updating `q`/`dq`. `state()` returns `JointState`.
**All clamping lives here**, so the seam is protected regardless of controller bugs.

### Setpoint queue — `include/setpoint_queue.h`
Thread-safe `std::deque<JointSetpoint>`. `replace(horizon)` swaps the active horizon
atomically (receding-horizon replan); `next()` pops the front, or returns the last
element without removing it when only one remains (hold). The robot adapter mirrors
this contract onto the real `JointTrajectory`.

### Driver — `include/control_driver.h`
`ControlDriver` ties controller + queue + backend. `step()` runs one tick
(compute → replace queue → backend.apply). `run(seconds, q_goal, csv_path)` loops and
logs `t, q, q_des, q_goal` to CSV. Single-threaded and deterministic.

### Harness — `test/test_joint_control.cpp`
Standalone executable (matching the `test_kinova.cpp` precedent), minimal `check()`
assert helper (no gtest). Returns non-zero on any failure.

## Data Flow

```
q_goal -> controller.compute(state) -> JointSetpoint(s) -> SetpointQueue.replace()
                                                              |
                                              backend pops via next() each tick
                                                              |
                                       SimBackend.apply(cmd, dt): lag + clamps
                                                              |
                                              state() -> fed back to controller
```

At the robot (M2) the adapter replaces `SetpointQueue`/`SimBackend` with the real
`JointTrajectory` deque + `joint_position_control_execution()`; the controller is
unchanged.

## Safety & Error Handling

- Velocity + position clamping lives in the **backend** (protects sim and the seam).
- Driver holds the last valid setpoint and logs on any `compute()` failure or NaN.
- The robot path additionally keeps Kinova's own low-level limits.

## Testing (TDD)

Standalone assertions in the Eigen-only harness:

1. `SimBackend` converges to a constant goal within tolerance.
2. `SimBackend` velocity clamp caps per-tick motion to `dq_max * dt`.
3. `SimBackend` position clamp keeps `q` within `[q_lower, q_upper]`.
4. `RateLimitedController` reaches goal within the expected number of ticks.
5. `SetpointQueue` `next()` holds the last element when one remains; `replace()` swaps.
6. `ControlDriver.run()` converges end-to-end and writes a non-empty CSV.

## Out of Scope for Milestone 1 (YAGNI)

QP/optimizer, dynamics/torque control, Cartesian/task-space, voice, dual-arm, obstacle
avoidance, the threaded real-time loop, and the `JointTrajectory` robot adapter (M2).

## Acceptance Criteria (Milestone 1 "done")

1. This spec committed.
2. Implementation plan committed.
3. Core units + `RateLimitedController` + `SimBackend` + queue + driver implemented,
   Eigen-only, header-only.
4. Standalone harness **builds and runs on the dev Mac with no robot**.
5. All six tests above pass (harness exits 0).

## Roadmap (after M1)

- **M2:** `MpcJointController` — double-integrator joint model, tracking+effort+terminal
  cost, position/velocity/accel constraints, QP solver (OSQP recommended). Plus the
  `JointTrajectory` adapter + threaded real-time loop to drive the real arm. Both plug
  into the same `JointController` interface and setpoint contract.
- **M3:** Voice → `q_goal` (and named targets), wired to the same seam.
