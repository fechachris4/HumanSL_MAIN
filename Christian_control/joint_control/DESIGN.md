# Joint-Control MPC Seam — Design Spec

**Date:** 2026-06-10
**Status:** Approved design (Milestone 1)
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
MPC injection point.

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
| No-robot backend | **First-order-lag kinematic sim** | ~100 lines, no deps, enough to watch MPC converge and exercise limits |
| Structural approach | Build **inside `TrajectoryRealTime`**, reusing the `JointTrajectory` deque/replan seam | Compatible with existing codebase; the real deployment target |
| M1 controller | **Trivial `RateLimitedController`** (QP MPC = Milestone 2) | De-risks plumbing independently of solver tuning |

## The Tension and Its Resolution

Building inside `TrajectoryRealTime` (the chosen approach) normally means "needs robot
+ Vicon to run anything," which conflicts with the no-robot constraint. Resolved by a
**two-compile-path** design: the core code (controller, sim backend, loop) depends on
**only Eigen + the `JointTrajectory` struct** — never on `move.h`/Kortex/GTSAM.

- **Sim harness build** (new, Eigen-only): compiles and runs on the laptop today.
- **Full build** (existing CMake): a thin adapter wires the same controller into
  `main.cpp`'s real 500 Hz execution thread via the replan seam, for use at the robot.

The controller and the deque seam are identical in both paths; only the execution
backend swaps.

## Architecture — Four Units

### 1. `JointController` interface — the MPC seam
`TrajectoryRealTime/include/joint_controller.h` (Eigen-only)

```cpp
struct JointControlState { Eigen::VectorXd q, dq; double t; };
struct JointSetpoint     { Eigen::VectorXd q_des, dq_des; };   // position setpoints

class JointController {
public:
    virtual ~JointController() = default;
    // Compute the next setpoint(s) given current state and the goal.
    virtual JointSetpoint compute(const JointControlState& s,
                                  const Eigen::VectorXd& q_goal) = 0;
    virtual int horizon_steps() const = 0;   // 1 for trivial; N for MPC
};
```

M1 implementations:
- `HoldController` — returns current `q` (safety / baseline).
- `RateLimitedController` — steps `q` toward `q_goal` at a capped per-tick speed.
  Deterministic, trivially testable.

M2 implements `MpcJointController` against the **same interface**.

### 2. Receding-horizon loop
`TrajectoryRealTime/src/joint_mpc_loop.cpp`

Runs at the MPC rate (e.g. 50–100 Hz). Each iteration: read shared `q_cur`/`dq_cur`,
call `controller.compute()`, write the resulting horizon into the `JointTrajectory`
deque using the **existing** `new_trajectory_ready` / `trajectory_mutex` machinery
(no new synchronization). On `compute()` failure or NaN, log and hold the last setpoint.

### 3. Simulated execution backend
`TrajectoryRealTime/src/sim_execution.cpp`

Drop-in stand-in for `joint_position_control_execution()` that consumes the same
`JointTrajectory` deque but, instead of Kortex I/O, integrates a first-order lag:

```
q   += (q_cmd - q) * (dt / tau)
```

clamped to position/velocity limits from `config/joint_limits.yaml`, writing back the
shared `q_cur`/`dq_cur`. Depends only on Eigen + `JointTrajectory`. This is the
laptop-runnable "robot."

### 4. Standalone harness
`test_mpc_joint.cpp` (top-level, matching the `test_kinova.cpp` precedent)

Wires sim backend + loop + a controller + a goal. Runs N seconds, logs
`q`, `q_des`, `q_goal` to CSV, and **asserts** convergence and limit-respect. This is
the Milestone-1 acceptance test, green on the laptop with no robot.

## Data Flow

```
q_goal  ->  control loop (compute)  ->  JointTrajectory deque
                                          |
                          execution backend (sim OR real) pops at control_frequency
                                          |
                              applies setpoint -> updates q_cur/dq_cur
                                          |
                                   fed back to the loop
```

Identical in sim and on hardware; only unit #3 swaps.

## Safety & Error Handling

- **Velocity + position clamping lives in the execution backend**, so it protects both
  sim and the seam regardless of controller bugs. The real path additionally keeps
  Kinova's own low-level limits.
- Loop logs and **holds the last setpoint** on any `compute()` failure or NaN.
- Limits sourced from `config/joint_limits.yaml`.

## Testing (TDD)

Standalone assertions (no framework, matching repo style), all in the Eigen-only harness:

1. Lag sim converges to a constant goal within tolerance.
2. Velocity clamp caps per-tick motion.
3. `RateLimitedController` reaches goal within expected time.
4. Loop holds safely when the deque is exhausted.

## Out of Scope for Milestone 1 (YAGNI)

QP/optimizer, dynamics/torque control, Cartesian/task-space, voice, dual-arm, obstacle
avoidance. Each is a later milestone that plugs into this seam.

## Acceptance Criteria (Milestone 1 "done")

1. This spec committed.
2. Implementation plan written.
3. `JointController` + `RateLimitedController` + `SimExecutionBackend` +
   receding-horizon loop implemented, Eigen-only.
4. Standalone harness builds and runs on the dev Mac with no robot.
5. All four tests above pass.

## Roadmap (after M1)

- **M2:** `MpcJointController` — double-integrator joint model, tracking+effort+terminal
  cost, position/velocity/accel constraints, QP solver (OSQP recommended). Plugs into
  the same `JointController` interface. Optionally upgrade the model to the existing
  Pinocchio dynamics, and the sim backend to a dynamic sim.
- **M3:** Voice → `q_goal` (and named targets), wired to the same seam.
