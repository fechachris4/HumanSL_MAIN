# Single World Cartesian Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the production joint-reference/controller branch and idle-hold workaround with one world-frame Cartesian reference source and the Python simulation controller law.

**Architecture:** The controller first computes one immutable measured Cartesian state from measured joints plus the latest Mount pose/twist. A concrete Cartesian trajectory source then selects startup hold, active trajectory, or final hold; the sole controller law maps its world pose/twist error through existing DLS/null-space logic into the unchanged safety and actuation chain.

**Tech Stack:** C++17, Eigen, Pinocchio, existing Kortex-free controller tests, CMake/CTest.

## Global Constraints

- Exact law: `V_task = Kp*poseError(T_W_E,d,T_W_E) + Kd*(V_W_E,d-V_W_E)`.
- Measured twist: `v_W_E=v_W_M+omega_W_M x (p_W_E-p_W_M)+(J_W qdot)_lin`; `omega_W_E=omega_W_M+(J_W qdot)_ang`.
- No separate `-V_base,E` or other explicit feedforward path.
- Controller-facing reference is always `WORLD`; no frame selector, `q_ref`, `qdot_ref`, or posture.
- The source clock advances only on fresh world state; final hold uses zero reference twist.
- Preserve velocity clamps, joint-boundary handling, position integration, following-error/command-lead checks, faults, stops, and teardown.
- Keep the 500 Hz path allocation-, lock-, file-, terminal-, and pipe-I/O-free.
- Preserve unrelated dirty-tree work. Do not run robot-facing executables or commit without authorization.

---

### Task 1: Parse and atomically publish Cartesian trajectories

**Files:**
- Create: `Christian_control/basic_control/src/CartesianTrajectoryMailbox.h`
- Create: `Christian_control/basic_control/src/CartesianTrajectoryMailbox.cpp`
- Create: `Christian_control/basic_control/tests/test_cartesian_trajectory_input.cpp`
- Modify: `Christian_control/basic_control/src/Targets.h`
- Modify: `Christian_control/basic_control/src/Targets.cpp`
- Modify: `Christian_control/basic_control/CMakeLists.txt`

**Interfaces:**
- Consumes: `WorldCartesianTrajectoryAccumulator` from the shared contract.
- Produces:

```cpp
class CartesianTrajectoryMailbox {
public:
    void Publish(std::unique_ptr<WorldCartesianTrajectory> trajectory);
    std::unique_ptr<WorldCartesianTrajectory> Take();
};
void RunCartesianTrajectoryInputFromPipe(
    CartesianTrajectoryMailbox& mailbox, const std::atomic<bool>& stop,
    const std::string& pipe_path);
```

- [ ] **Step 1: Port the existing FIFO/partial-line tests to `CART_TRAJ_*`**

Test latest-wins replacement, writer disconnect/reopen, partial rows, overlong lines, invalid quaternion, wrong frame/version, early terminator, and stop/join without a writer. Assert only fully parsed and validated trajectories publish.

- [ ] **Step 2: Run the new input test and confirm the missing symbols**

Run: `cmake --build Christian_control/basic_control/build --target test_cartesian_trajectory_input -j2`

- [ ] **Step 3: Implement a latest-valid single-slot mailbox and non-RT parser**

Reuse the existing atomic pointer ownership pattern, but name the accepted bounded delete explicitly in comments. Increase line capacity enough for the 15-field Cartesian row while retaining a finite cap.

- [ ] **Step 4: Run input and contract tests**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^cartesian_trajectory_input$' --output-on-failure`

### Task 2: Split measurement from the sole Cartesian control law

**Files:**
- Modify: `Christian_control/basic_control/src/State.h`
- Modify: `Christian_control/basic_control/src/Frames.h`
- Modify: `Christian_control/basic_control/src/Controller.h`
- Modify: `Christian_control/basic_control/src/Controller.cpp`
- Modify: `Christian_control/basic_control/tests/test_frames.cpp`
- Modify: `Christian_control/basic_control/tests/test_controller.cpp`
- Modify: `Christian_control/basic_control/tests/reactive_fixtures.h`

**Interfaces:**
- Produces: `MeasuredCartesianState TrackingController::Measure(const RobotState&)` and `DesiredVelocity(const RobotState&, const MeasuredCartesianState&, const PoseReference&, double, ControllerStatus&)`.
- Removes: `JointReference`, `Reference::joint`, `joint_is_idle_hold`, `SolveJointTracking`, and controller dispatch.

- [ ] **Step 1: Add a shared numeric fixture matching Python `frames.py`**

Use nonzero Mount translation, Mount angular velocity, arm joint velocity, and an offset end effector. Assert C++ measured world pose, rotated Jacobian, transported linear velocity, and angular velocity match values generated independently by the Python implementation.

- [ ] **Step 2: Add a controller test that fails if base motion is explicit feedforward**

```cpp
const MeasuredCartesianState measured = controller.Measure(state_with_mount_twist);
const PoseReference ref{measured.ee_pose_world, Twist::Zero(), 1, true};
const auto qdot = controller.DesiredVelocity(state, measured, ref, dt, status);
const auto expected = SolveReactiveVelocityDetailed(
    measured.jacobian_world, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    -measured.ee_twist_world.linear_m_s,
    -measured.ee_twist_world.angular_rad_s, state.q_rad,
    configured_limit_rad, configured_zone_rad, configured_gains);
Check(Near(qdot, expected.qdot_task_rad_s + expected.qdot_null_rad_s),
      "Mount motion enters exactly once through measured twist error");
```

- [ ] **Step 3: Run `frames`, `reactive_law`, and `controller` to observe failure**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(frames|reactive_law|controller)$' --output-on-failure`

- [ ] **Step 4: Implement measurement and remove joint dispatch**

Add Mount linear/angular velocity and validity to `RobotState`. `Measure` composes `T_W_B`, invokes `world_frames::ArmControllerState` with measured `V_W_M`, and returns pose/twist/Jacobian. `DesiredVelocity` accepts one world `PoseReference`, computes reference-minus-measurement errors, and calls the unchanged reactive solver. Delete joint-following members and world-hold fallback branches from `TrackingController`.

- [ ] **Step 5: Re-run controller mathematics suites**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(frames|reactive_law|controller)$' --output-on-failure`

### Task 3: One Cartesian reference source for startup, trajectory, and final hold

**Files:**
- Create: `Christian_control/basic_control/src/CartesianReference.h`
- Create: `Christian_control/basic_control/src/CartesianReference.cpp`
- Create: `Christian_control/basic_control/tests/test_cartesian_reference.cpp`
- Modify: `Christian_control/basic_control/src/State.h`
- Modify: `Christian_control/basic_control/src/Runner.h`
- Modify: `Christian_control/basic_control/src/Runner.cpp`
- Modify: `Christian_control/basic_control/src/Config.h`
- Modify: `Christian_control/basic_control/CMakeLists.txt`

**Interfaces:**
- Consumes: `RobotState`, `MeasuredCartesianState`, `CartesianTrajectoryMailbox`, clamped `dt_s`.
- Produces:

```cpp
PoseReference CartesianReferenceSource::Get(
    const RobotState& state, const MeasuredCartesianState& measured,
    double dt_s, ControllerStatus& status);
```

- [ ] **Step 1: Write state-machine tests before implementation**

Cover: no Vicon startup gives a zero-error current-pose reference; first fresh sample captures a fixed world hold; a valid trajectory whose first pose is within existing arrival position/orientation tolerances activates at `t=0`; interpolation uses linear position/twist and shortest-hemisphere quaternion slerp; only the final sample is arrival-eligible; completion holds final pose with zero twist; wrong provenance/noncontinuous starts reject atomically; a fresh replacement cannot partially overwrite the active trajectory.

- [ ] **Step 2: Run the new test and confirm failure**

Run: `cmake --build Christian_control/basic_control/build --target test_cartesian_reference -j2`

- [ ] **Step 3: Implement the concrete source without virtual dispatch**

```cpp
enum class CartesianReferenceState { kAwaitingWorld, kHolding, kTracking };

class CartesianReferenceSource {
public:
    CartesianReferenceSource(CartesianTrajectoryMailbox& mailbox);
    PoseReference Get(const RobotState&, const MeasuredCartesianState&,
                      double dt_s, ControllerStatus&);
private:
    std::unique_ptr<WorldCartesianTrajectory> active_;
    world_frames::FramePose hold_world_;
    double trajectory_time_s_ = 0.0;
};
```

Use binary search over immutable points with no allocation. Use the existing arrival tolerances as strict splice tolerances so no new motion threshold is invented.

- [ ] **Step 4: Change Runner order to measurement → reference → sole law**

After the one BasePose slot read, copy/decay Mount twist into `RobotState`; call `controller.Measure(state)`, then source `Get`, then `controller.DesiredVelocity`. Preserve every line from non-finite handling through clamp, integration, Send, logging, and stop-priority resolution.

- [ ] **Step 5: Run reference, controller, actuation, and safety tests**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(cartesian_reference|frames|reactive_law|controller|control_logic|supervisor)$' --output-on-failure`

### Task 4: Switch production wiring and remove the old production concepts

**Files:**
- Modify: `Christian_control/basic_control/src/Main.cpp`
- Modify: `Christian_control/basic_control/src/Hardware.h`
- Modify: `Christian_control/basic_control/src/Hardware.cpp`
- Modify: `Christian_control/basic_control/tests/test_log_schema.cpp`
- Modify: `Christian_control/basic_control/CMakeLists.txt`
- Delete: `Christian_control/basic_control/src/JointTrajectory.h`
- Delete: `Christian_control/basic_control/src/JointTrajectory.cpp`
- Delete: `Christian_control/basic_control/tests/test_joint_trajectory.cpp`
- Delete: `Christian_control/basic_control/src/WorldHold.h`
- Delete: `Christian_control/basic_control/tests/test_world_hold.cpp` after Task 3's Cartesian hold tests pass.
- Modify: `Christian_control/planner_bridge/src/BridgeMain.cpp`
- Modify: `Christian_control/planner_bridge/scripts/run_session.sh`
- Modify: `Christian_control/planner_bridge/tests/test_run_session.sh`

**Interfaces:**
- Production accepts only `CART_TRAJ_*` blocks and invokes one Cartesian source/controller path.

- [ ] **Step 1: Change integration tests to expect Cartesian-only production**

Assert `TRAJ_BEGIN` is rejected, `CART_TRAJ_BEGIN` activates, startup/final hold still call the Cartesian law, and the session artifact contains a world Cartesian plan. Update the shell stub to emit the exact versioned Cartesian block.

- [ ] **Step 2: Run tests and confirm they expose current joint wiring**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(controller|cartesian_reference|cartesian_trajectory_input|log_schema)$' --output-on-failure`

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^(bridge_main|run_session)$' --output-on-failure`

- [ ] **Step 3: Wire the Cartesian mailbox/source and delete joint production code**

In `Main.cpp`, replace `JointTrajectoryMailbox/JointTrajectorySource` with Cartesian equivalents and update operator text. Make `planner_bridge` Cartesian output the only/default output and remove `TrajectoryEmit` joint formatting. Preserve the same FIFO path for compatibility unless a rename is needed to make logs unambiguous.

- [ ] **Step 4: Replace joint telemetry fields with world-reference evidence**

Append a new log format rather than reinterpreting old columns. Record trajectory ID, planner Vicon sequence, reference clock, reference world pose/twist, measured world pose/twist, freshness state, activation/rejection/completion edges, and replan edge. Keep requested raw qdot, clamped/sent/measured values and every safety field unchanged.

- [ ] **Step 5: Build controller but do not execute it; run all hardware-free tests**

Run: `cmake --build Christian_control/basic_control/build -j2`

Run: `ctest --test-dir Christian_control/basic_control/build --output-on-failure`

Run: `cmake --build Christian_control/planner_bridge/build -j2 && ctest --test-dir Christian_control/planner_bridge/build --output-on-failure`

Run: `rg -n 'JointReference|SolveJointTracking|joint_is_idle_hold|JointTrajectorySource|TRAJ_BEGIN' Christian_control/basic_control/src Christian_control/planner_bridge/src`

Expected: no production matches; historical docs may still describe the superseded design.

Review checkpoint: trace `PoseReference -> DesiredVelocity -> ClampJointVelocity -> PositionIntegration::Apply -> CyclicSession::Send` and verify no branch bypasses a safety or stop check.
