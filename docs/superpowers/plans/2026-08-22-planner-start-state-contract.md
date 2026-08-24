# Planner Start-State Contract Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. One sole implementation agent writes every production-code task; all other agents remain read-only reviewers. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every live point or traced plan start at the canonical measured joint position and the same-snapshot measured joint velocity, while an offline plan without velocity makes no zero-velocity assumption.

**Architecture:** Extend the existing fixed-size planning snapshot with measured joint velocity and carry it through the existing typed-request-to-argument planner entry. Seed GPMP2 state zero from the supplied measurement and use GTSAM nonlinear equality factors for the initial position and, when supplied, velocity. Remove the two old zero-start assumptions while retaining task-entry and endpoint rest constraints.

**Tech Stack:** C++17, Eigen, bundled GTSAM/GPMP2, CMake/CTest.

**Spec:** `docs/superpowers/specs/2026-08-22-planner-start-state-contract-design.md`

## Global Constraints

- `q(0) = WrapToPrincipalRad(q_meas)` for all seven Kortex-order joints.
- Every valid live `PlanningRequest` carries finite `qdot_meas` in rad/s from the same successful cyclic feedback reply as `q_meas`.
- Offline absence of velocity means no initial-velocity factor; it never means zero.
- Use `gtsam::NonlinearEquality<gtsam::Vector>` after making the initial `Values` feasible; do not tune a sigma or overwrite the solved trajectory.
- Keep the controller continuity gate, collision factors, joint limits, point endpoint rest, traced task-entry rest, and traced endpoint rest unchanged.
- Add one focused test executable only. Add no manager, service, retry, fallback, configuration key, or new safety gate.
- Do not run `controller` or any executable under `runtime/tools/`.
- Do not commit; Christian has not requested a commit.
- Preserve all unrelated working-tree changes.

---

### Task 1: Establish the failing start-state contract test

**Files:**
- Create: `Christian_control/planning/tests/test_planner_start_state.cpp`
- Modify: `Christian_control/planning/CMakeLists.txt`

**Interfaces:**
- Consumes: current `PlanningRequest`, `SolvePlanForRequest`, `SolveToPosition`, `SolveAlongPath`, and `TrajectoryResult::{trajectory_pos,trajectory_vel,start_costs}`.
- Produces: one CTest target named `planner_start_state` that fails until measured velocity transport and equality constraints exist.

- [ ] **Step 1: Register one test executable**

Add one target beside the other top-level planning tests:

```cmake
add_executable(test_planner_start_state tests/test_planner_start_state.cpp)
target_include_directories(test_planner_start_state PRIVATE src)
target_link_libraries(test_planner_start_state PRIVATE bridge_core)
set_target_properties(test_planner_start_state PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME planner_start_state COMMAND test_planner_start_state
         ${DH_PARAMS_TOOL_YAML} ${DH_PARAMS_FLANGE_YAML})
```

- [ ] **Step 2: Write the focused regression cases**

Use the known right-arm request from `session_194737` as the point fixture:

```cpp
const Eigen::Matrix<double, 7, 1> q_meas_deg =
    (Eigen::Matrix<double, 7, 1>() <<
        94.868255615234375, 103.06673431396484, 336.72296142578125,
        7.3997693061828613, 350.69995117187494, 354.3056640625,
        184.83963012695315).finished();
const Eigen::Matrix<double, 7, 1> qdot_meas_rad_s =
    (Eigen::Matrix<double, 7, 1>() <<
        0.01, -0.02, 0.03, -0.04, 0.05, -0.06, 0.07).finished();
```

The single executable must check:

```cpp
Check(MaxAbs(point.result.trajectory_pos.front() - q_plan) < 1e-12,
      "point q0 equals canonical measured q");
Check(MaxAbs(point.result.trajectory_vel.front() - qdot_meas_rad_s) < 1e-12,
      "point qdot0 equals measured qdot");
Check(point.result.start_costs.count("StartPosEquality") == 1,
      "point graph contains position equality");
Check(point.result.start_costs.count("StartVelEquality") == 1,
      "point graph contains velocity equality");
```

Repeat the same four assertions for a small reachable mount-frame circle passed through `SolveAlongPath`. For both point and path, call once with no offline velocity and assert:

```cpp
Check(result.start_costs.count("StartVelEquality") == 0,
      "offline plan without velocity has no start velocity equality");
Check(result.start_costs.count("StartVelPrior") == 0,
      "offline plan without velocity has no zero start prior");
```

Finally construct one valid `PlanningRequest` with the nonzero velocity, call `SolvePlanForRequest` with a temporary artifact root, and inspect its `request.args` and `joints.csv`: the velocity flag and seven values must be archived, and the first row must reproduce canonical `q0` and measured `qdot0`. Remove only the test's own named temporary directory on normal completion.

- [ ] **Step 3: Configure and run the test to prove the old behavior fails**

Run:

```bash
cmake -S Christian_control/planning -B Christian_control/planning/build
cmake --build Christian_control/planning/build --target test_planner_start_state -j2
ctest --test-dir Christian_control/planning/build -R '^planner_start_state$' --output-on-failure
```

Expected before production edits: compile failure because the velocity field/signatures do not exist, or behavioral failure because the old graph moves `q0` and manufactures zero `qdot0`. Record the exact failure.

---

### Task 2: Carry measured velocity through the existing live request path

**Files:**
- Modify: `Christian_control/contracts/PlanningRequest.h`
- Modify: `Christian_control/contracts/PlanningRequest.cpp`
- Modify: `Christian_control/runtime/Runner.cpp`
- Modify: `Christian_control/planning/src/PlannerRuntime.cpp`
- Modify: `Christian_control/planning/src/BridgeMain.cpp`
- Modify: `Christian_control/planning/src/PlanSolver.h`
- Modify: `Christian_control/planning/src/PlanSolver.cpp`
- Modify: `Christian_control/planning/tests/test_planning_attempt_recorder.cpp`

**Interfaces:**
- Produces: `PlanningRequest::qdot_rad_s` as an optional fixed-size rad/s vector whose absence is invalid for live requests; `PlanRequest::qdot_start_rad_s` as optional for the shared live/offline solver; optional start velocity passed into `SolveAlongPath`.
- Consumes: `ArmExecutionResult::state.qdot_rad_s`, already populated from the same cyclic feedback as `state.q_rad`.

- [ ] **Step 1: Add the non-zero-safe live request field and validation**

In `PlanningRequest.h` add:

```cpp
std::optional<Eigen::Matrix<double, 7, 1>> qdot_rad_s;
```

In `ValidatePlanningRequest` reject absence and non-finite values:

```cpp
if (!request.qdot_rad_s)
    return "qdot_rad_s must be present for a live planning request";
if (!request.qdot_rad_s->allFinite())
    return "qdot_rad_s must be finite";
```

- [ ] **Step 2: Publish position and velocity from one state snapshot**

Immediately beside the existing position assignment in `Runner.cpp`:

```cpp
request.q_rad = result.state.q_rad;
request.qdot_rad_s = result.state.qdot_rad_s;
```

Do not read feedback a second time and do not add a timestamp or validity flag.

- [ ] **Step 3: Carry velocity through the existing argument path**

In `PlannerRuntime.cpp`, after `--start-deg`, append:

```cpp
args.push_back("--start-velocity-deg-s");
for (int joint = 0; joint < 7; ++joint)
    args.push_back(Number((*request.qdot_rad_s)(joint) * 180.0 / M_PI));
```

`SolvePlanForRequest` already receives only validated live requests from `InProcessPlanner`; retain that architecture.

In `BridgeMain.cpp`:

```cpp
std::optional<std::array<double, 7>> start_velocity_deg_s;
```

Parse `--start-velocity-deg-s` as seven finite doubles, document it in the usage text, and convert it once without wrapping:

```cpp
std::optional<Eigen::Matrix<double, 7, 1>> qdot_start_rad_s;
if (parsed.start_velocity_deg_s) {
    Eigen::Matrix<double, 7, 1> qdot;
    for (int joint = 0; joint < 7; ++joint)
        qdot(joint) = (*parsed.start_velocity_deg_s)[joint] * kDegToRad;
    qdot_start_rad_s = qdot;
}
```

Reject an explicitly supplied non-finite value through the existing `ParseDouble` boundary. Do not require this flag for standalone offline use.

- [ ] **Step 4: Thread the optional velocity into both solver requests**

In `PlanSolver.h`:

```cpp
std::optional<Eigen::Matrix<double, 7, 1>> qdot_start_rad_s;
```

Add the same optional parameter immediately after `q_start_rad` in `SolveAlongPath`. Set/pass it at both `BridgeMain` call sites. In `PlanSolver.cpp`, forward it to the point free optimizer and the traced optimizer without modification.

- [ ] **Step 5: Keep the existing recorder fixture valid**

Set a finite nonzero `request.qdot_rad_s` in `test_planning_attempt_recorder.cpp`. Do not add another test case there; that test continues to own recording only.

- [ ] **Step 6: Build the affected request path**

Run:

```bash
cmake --build Christian_control/planning/build --target bridge_core test_planning_attempt_recorder test_planner_start_state -j2
```

Expected: request transport compiles; `planner_start_state` still fails because GPMP2 still uses soft/zero start factors.

---

### Task 3: Replace point-plan soft/zero starts with exact measured state

**Files:**
- Modify: `Christian_control/planning/optimisation/GenerateTrajectory.h`
- Modify: `Christian_control/planning/optimisation/GenerateTrajectory.cpp`
- Modify: `Christian_control/planning/optimisation/TrajectoryOptimization.h`
- Modify: `Christian_control/planning/optimisation/TrajectoryOptimization.cpp`

**Interfaces:**
- Consumes: `std::optional<gtsam::Vector> start_vel` from `SolveToPosition`.
- Produces: point results with exact state-zero position and conditional exact velocity; factor keys `StartPosEquality` and `StartVelEquality`.

- [ ] **Step 1: Change the point optimizer signatures**

Add `<optional>` and replace the mandatory vector with:

```cpp
const std::optional<gtsam::Vector>& start_vel
```

in the free function and `OptimizeTrajectory::optimizeJointTrajectory`. `GenerateTrajectory.cpp` forwards the optional value and deletes:

```cpp
const gtsam::Vector start_vel = gtsam::Vector::Zero(start_config.size());
```

Delete the comment claiming the shim supplies zero start velocity.

- [ ] **Step 2: Make the point initial values feasible and add equalities**

Include `gtsam/nonlinear/NonlinearEquality.h`. Before graph cost evaluation:

```cpp
gtsam::Values initial_values = init_values;
const gtsam::Symbol start_pos_key('x', 0);
const gtsam::Symbol start_vel_key('v', 0);
initial_values.update(start_pos_key, start_config);
if (start_vel)
    initial_values.update(start_vel_key, *start_vel);
```

At support state zero replace the two finite-sigma priors with:

```cpp
graph.add(gtsam::NonlinearEquality<gtsam::Vector>(key_pos, start_config));
factor_keys.push_back("StartPosEquality");
if (start_vel) {
    graph.add(gtsam::NonlinearEquality<gtsam::Vector>(key_vel, *start_vel));
    factor_keys.push_back("StartVelEquality");
}
```

Use `initial_values` consistently for initial graph error and as the optimizer input. Remove the now-unused start position noise model. Keep the endpoint zero-velocity `PriorFactor` and its existing noise model.

- [ ] **Step 3: Run the point-focused regression**

Run:

```bash
cmake --build Christian_control/planning/build --target test_planner_start_state -j2
ctest --test-dir Christian_control/planning/build -R '^planner_start_state$' --output-on-failure
```

Expected at this checkpoint: point present/absent cases pass; traced cases still fail on the old start-rest behavior.

---

### Task 4: Replace traced-plan start rest while retaining later rests

**Files:**
- Modify: `Christian_control/planning/src/PathAssembly.h`
- Modify: `Christian_control/planning/src/PathAssembly.cpp`
- Modify: `Christian_control/planning/optimisation/TrajectoryOptimization.h`
- Modify: `Christian_control/planning/optimisation/TrajectoryOptimization.cpp`

**Interfaces:**
- Consumes: optional measured `start_vel` passed by `SolveAlongPath`.
- Produces: exact traced `x0`, conditional exact traced `v0`, with `zero_velocity_indices` containing task entry and endpoint but not index zero.

- [ ] **Step 1: Remove the obsolete start-rest concept from path assembly**

Delete:

```cpp
assembled.zero_velocity_indices.push_back(0);
```

Update `PathAssembly.h/.cpp` comments so `zero_velocity_indices` names only task-entry and endpoint rest. Replace “stiff JOINT prior” wording with the exact measured-position equality. Do not change approach geometry or timing.

- [ ] **Step 2: Change the traced optimizer signature and initialization**

Add the optional start velocity immediately after `start_config` in `optimizeTaskTrajectory`. Copy and correct the initial values exactly as in Task 3:

```cpp
gtsam::Values initial_values = init_values;
initial_values.update(gtsam::Symbol('x', 0), start_config);
if (start_vel)
    initial_values.update(gtsam::Symbol('v', 0), *start_vel);
```

- [ ] **Step 3: Replace only state-zero factors**

Delete the unconditional `rest_indices.insert(0)`. Retain
`rest_indices.insert(total_time_step)` and all supplied task-entry/end indices.

At `i == 0`, add `StartPosEquality` and conditional `StartVelEquality` as in
Task 3. The subsequent rest-index block continues to add `RestVelPrior` only
for the task-entry and endpoint indices. Use corrected `initial_values` for
costs and optimization.

- [ ] **Step 4: Run the full focused test**

Run:

```bash
cmake --build Christian_control/planning/build --target test_planner_start_state -j2
ctest --test-dir Christian_control/planning/build -R '^planner_start_state$' --output-on-failure
```

Expected: all point/path and present/absent cases pass.

### Approved traced retry clarification

The traced optimizer now treats each duration as a fresh solve. It validates
at `T`, retries only for an exclusively dynamic velocity/acceleration excess,
and otherwise returns without retrying. A retry rebuilds scaled waypoint
times, initial `Values`, and velocity guesses before calling
`optimizeTaskTrajectory`; it never scales solved velocities post hoc. The
existing alpha policy remains bounded to three total attempts. Supplied
measured `qdot(0)` above an effective planner velocity limit fails before the
first solve. Every attempt retains exact measured `q(0)` and `qdot(0)`.
The point failure characterization remains separate from this traced retry
evidence.

---

### Task 5: Remove residue and verify independently

**Files:**
- Modify only files cited by concrete cleanup-review findings.
- Do not create another test or compatibility path.

**Interfaces:**
- Consumes: completed implementation and the pre-change `session_194737` artifacts.
- Produces: clean minimal diff and hardware-free evidence report.

- [ ] **Step 1: Search for the old assumptions**

Run scoped searches:

```bash
rg -n "start_vel = gtsam::Vector::Zero|zero_velocity_indices.push_back\(0\)|rest_indices.insert\(0\)|StartPosPrior|StartVelPrior|stiff JOINT prior|test_pose_noise_ordering" Christian_control/planning
rg -n "qdot_rad_s|start-velocity-deg-s|StartPosEquality|StartVelEquality" Christian_control/contracts Christian_control/runtime Christian_control/planning
```

Expected: no active point/path zero-start factory, soft start prior, start-rest insertion, or stale deleted-test reference remains. Classify historical/build-output hits rather than deleting them blindly.

- [ ] **Step 2: Replay the failed request offline**

Use the current hardware-free `planner_bridge` with the archived
`session_194737/.../request.args`, add
`--start-velocity-deg-s 0 0 0 0 0 0 0`, and redirect `--debug-dir` to a fresh
temporary directory. The zero is recorded evidence, not an assumption:
`loop_log_right_20260822_194738.csv` line 13885 is the controller-rejection
cycle at `time_s=27.6746`, with all seven `vel_j*` fields equal to zero.
Label this a stationary replay; the nonzero test fixture is the moving-replan
mutation evidence.

Compare:

```text
first q error, first qdot error, projected Cartesian start position/orientation,
final goal error, solve duration, minimum joint margin, peak |qdot|
```

The first position must equal the canonical request state within `1e-12` rad;
the first velocity must equal the supplied replay velocity within `1e-12`
rad/s. The controller's unchanged 1 mm / 0.001 rad activation threshold is an
independent downstream reference, not a value to modify.

- [ ] **Step 3: Build and run all affected hardware-free tests**

Inspect CTest registration again, then run:

```bash
cmake --build Christian_control/planning/build -j2
ctest --test-dir Christian_control/planning/build --output-on-failure
cmake --build Christian_control/runtime/build --target controller -j2
ctest --test-dir Christian_control/runtime/build --output-on-failure
```

Building `controller` is compile evidence only. Do not execute it.

- [ ] **Step 4: Run the read-only evidence and minimality reviews**

The evidence reviewer checks the measured predictions, exact equality, same-
snapshot provenance, point/path coverage, and whether a zero-substitution
mutation fails. The minimality reviewer returns exactly `CLEAN` or
`CLEANUP REQUIRED`, citing only removable branches, tests, abstractions,
comments, or old-path residue. The sole implementer performs one cleanup pass
if required, and the reviewer rechecks only those findings.

- [ ] **Step 5: Review the final diff without committing**

Run:

```bash
git diff --check
git diff --stat
git diff -- Christian_control/contracts/PlanningRequest.h \
  Christian_control/contracts/PlanningRequest.cpp \
  Christian_control/runtime/Runner.cpp \
  Christian_control/planning
```

Report production files/classes/concepts/tests added and removed, old-path
status, exact verification results, minimality verdict, replay limitations,
and `Hardware executed: no`.
