# Full Planner-to-Simulation Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect `humansl_sim` to the production external `planner_bridge` process with paired dual-arm requests, world-Cartesian results, atomic activation, live asynchronous replanning, and deterministic scripted-result acceptance tests.

**Architecture:** One versioned dual request captures both measured arm states and one coherent Mount snapshot. The planner keeps GPMP2 joint-space internals, validates planned paths, projects each selected arm to world pose/twist, and returns one paired result. A non-real-time simulation input path validates and publishes the complete pair; the 500 Hz coordinator either activates both references on one cycle or rejects both. Live planning is interactive; numeric acceptance injects provenance-checked planner outputs at fixed ticks.

**Tech Stack:** C++17, GPMP2/GTSAM, Pinocchio, existing Cartesian contract, named pipes/current planner worker process, CMake/CTest, shell integration tests.

## Global Constraints

- Plans 01–03 must be accepted first.
- `docs/engineering/humansl-engineering-contract.md` is binding for every task in this plan.
- **Topology change requiring Christian's explicit approval before Task 3 begins:** today `planner_bridge` accepts only `--arm right|left` (`BridgeMain.cpp:461`) and `run_session.sh` launches two independent single-arm planner processes on two independent FIFO pairs. The paired result in this plan requires one dual planner service handling both arms in one process, which restructures `run_session.sh`, the FIFO topology, and panel session handling. Cross-arm atomic activation does not exist today and is documented as absent (`InterArmDistance.h:18-23`); Task 4 builds it.
- GPMP2 internals remain unchanged and outside the 500 Hz process.
- Only timed world-frame end-effector pose/twist and provenance enter control.
- No planned `q(t)`, `qdot(t)`, posture, elbow, or null-space bias crosses the controller boundary.
- A planned inter-arm/collision check certifies only GPMP2's internal joint branches.
- Both references activate on one common control cycle or neither activates.
- A failed/rejected replacement leaves the active valid trajectory/hold unchanged.
- Pending requests coalesce to the newest request; no unbounded queue.
- Live solve completion time never drives numeric tracking thresholds.
- No robot-facing command or commit without explicit authorization.

---

### Task 1: Define versioned dual planning and trajectory contracts

**Files:**
- Create: `Christian_control/cartesian_contract/DualPlanningRequest.h`
- Create: `Christian_control/cartesian_contract/DualPlanningRequest.cpp`
- Create: `Christian_control/cartesian_contract/DualWorldCartesianTrajectory.h`
- Create: `Christian_control/cartesian_contract/DualWorldCartesianTrajectory.cpp`
- Create: `Christian_control/planner_bridge/tests/test_dual_cartesian_contract.cpp`
- Modify: `Christian_control/planner_bridge/CMakeLists.txt`
- Modify: `Christian_control/basic_control/CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
enum class ArmRequestMode { kPlan, kWorldHold };

struct DualPlanningRequest {
    std::uint64_t request_id;
    ArmRequestMode right_mode;
    ArmRequestMode left_mode;
    PlanningRequest right_snapshot;
    PlanningRequest left_snapshot;
    std::uint64_t right_goal_fnv1a64;
    std::uint64_t left_goal_fnv1a64;
};

struct PlannedClearanceSummary {
    bool evaluated;
    bool planned_paths_satisfy_required_clearance;
    double minimum_planned_clearance_m;
    std::string evidence_scope;
};

struct DualWorldCartesianTrajectory {
    std::uint32_t format_version;
    std::uint64_t request_id;
    std::uint64_t trajectory_id;
    WorldCartesianTrajectory right;
    WorldCartesianTrajectory left;
    PlannedClearanceSummary planned_clearance;
};

std::string SerializeDualPlanningRequest(const DualPlanningRequest&);
DualPlanningRequest ParseDualPlanningRequest(std::string_view);
void ValidateDualWorldCartesianTrajectory(
    const DualWorldCartesianTrajectory& trajectory);
```

- [ ] **Step 1: Write round-trip and rejection tests**

Cover right-only, left-only, paired goals, common request/trajectory IDs, one shared Mount sequence/timestamp across both `PlanningRequest` snapshots, expected goal-file digests, wrong frame/version, mismatched provenance, duplicate/missing arm blocks, nonmonotonic time, partial terminator, non-finite data, and planned-clearance labels.

- [ ] **Step 2: Run and observe missing contract**

Run: `cmake --build Christian_control/planner_bridge/build --target test_dual_cartesian_contract -j2`

- [ ] **Step 3: Implement bounded explicit serialization**

Reuse the current numeric precision, quaternion order, and line-length bounds. Require `right_snapshot.arm == kRight`, `left_snapshot.arm == kLeft`, and identical request/Mount provenance across snapshots. The planner reloads each planned arm's goal file and rejects the request if its FNV-1a digest differs from the captured digest. The non-planned arm carries a one-sample zero-twist world hold with explicit `ArmRequestMode::kWorldHold`; it does not carry a constant joint posture.

- [ ] **Step 4: Run contract tests in both builds**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^dual_cartesian_contract$' --output-on-failure`

Run: `ctest --test-dir Christian_control/basic_control/build -R '^dual_cartesian_contract$' --output-on-failure`

Review checkpoint: grep serialized fixtures and assert no joint trajectory/reference fields appear in output blocks.

### Task 2: Build paired requests from one coherent execution snapshot

**Files:**
- Create: `Christian_control/basic_control/src/DualPlanningRequestSlot.h`
- Create: `Christian_control/basic_control/src/DualPlanningRequestWriter.h`
- Create: `Christian_control/basic_control/src/DualPlanningRequestWriter.cpp`
- Create: `Christian_control/basic_control/tests/test_dual_planning_request.cpp`
- Modify: `Christian_control/simulation/src/DualSimulationRunner.h`
- Modify: `Christian_control/simulation/src/DualSimulationRunner.cpp`
- Modify: `Christian_control/basic_control/CMakeLists.txt`
- Modify: `Christian_control/simulation/CMakeLists.txt` (simulation-side sources/tests in this task)

**Interfaces:**
- Produces an SPSC latest-value slot from the simulation control owner to a non-real-time writer. One snapshot includes both measured joint states, current world TCP holds, and the same Mount sample/provenance.

- [ ] **Step 1: Write coherence/latest-wins tests**

Assert both arms and Mount share one cycle ID; a newer request replaces an unwritten pending request; a writer never combines one arm from request N with the other from N+1; stale/invalid Mount snapshots are rejected before serialization; stop joins with no reader.

- [ ] **Step 2: Run and observe missing slot/writer**

Run: `cmake --build Christian_control/basic_control/build --target test_dual_planning_request -j2`

- [ ] **Step 3: Implement fixed-size publication and non-RT serialization**

The control path copies only a fixed-size request record. Pipe open/write/retry and formatting happen in the writer thread. Preserve the current no-blocking control rule.

- [ ] **Step 4: Wire panel/simulator request edges without planning yet**

Expose a simulation command mailbox for right, left, or paired goals. On a right-only command, snapshot the left measured world TCP as a zero-twist hold, and vice versa.

- [ ] **Step 5: Run request and thread-shutdown tests**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(dual_planning_request|planning_request_writer)$' --output-on-failure`

Review checkpoint: inspect the 500 Hz path for pipe/file/string formatting, blocking waits, mutexes, or allocation.

### Task 3: Extend the planner worker to solve and return a pair

**Files:**
- Create: `Christian_control/planner_bridge/src/DualPlannerWorker.h`
- Create: `Christian_control/planner_bridge/src/DualPlannerWorker.cpp`
- Create: `Christian_control/planner_bridge/tests/test_dual_planner_worker.cpp`
- Modify: `Christian_control/planner_bridge/src/PlannerWorker.h`
- Modify: `Christian_control/planner_bridge/src/PlannerWorker.cpp`
- Modify: `Christian_control/planner_bridge/src/InterArmDistance.h`
- Modify: `Christian_control/planner_bridge/src/InterArmDistance.cpp`
- Modify: `Christian_control/planner_bridge/CMakeLists.txt`

**Interfaces:**
- Consumes `DualPlanningRequest` and current planner configuration/shared scene.
- Produces one `DualWorldCartesianTrajectory` or one paired failure report.

- [ ] **Step 1: Write paired solve/coalescing tests with controlled solvers**

Use solver fakes controlled by promises: request 1 starts; requests 2 and 3 arrive; only request 3 becomes next; one-arm solve failure rejects the pair; shutdown waits for active work; no more than one solve per arm runs at once.

- [ ] **Step 2: Write hold-arm and planned-clearance tests**

For a right-only request, project the right plan and emit the captured left world hold. Planned inter-arm checking uses right planned joints against the captured left joint state and labels the report `planned_joint_paths_only; tracking_error_not_modelled`.

- [ ] **Step 3: Implement paired worker orchestration**

Reuse the existing solve entry points `SolveToPosition` / `SolveAlongPath` (`Christian_control/planner_bridge/src/PlanSolver.h:51,86` — there is no function named `PlanWithGpmp2`), the existing validation and time scaling, and `ProjectWorldTrajectory` (`src/WorldTrajectoryProjection.h:15`); do not alter GPMP2 factors. Complete both arm results and planned-path report before serializing one pair.

- [ ] **Step 4: Run planner worker and existing planner suites**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^(dual_planner_worker|planner_worker|world_trajectory_projection|inter_arm_distance|plan_solver|path_validation)$' --output-on-failure`

Review checkpoint: a passing planned-clearance result is never named `executed`, `safe`, or `guaranteed` in code, report, tests, or UI-facing text.

### Task 4: Atomically stage and activate paired Cartesian references

**Files:**
- Create: `Christian_control/basic_control/src/DualCartesianTrajectoryMailbox.h`
- Create: `Christian_control/basic_control/src/DualCartesianTrajectoryMailbox.cpp`
- Create: `Christian_control/basic_control/src/DualReferenceCoordinator.h`
- Create: `Christian_control/basic_control/src/DualReferenceCoordinator.cpp`
- Create: `Christian_control/basic_control/tests/test_dual_reference_coordinator.cpp`
- Modify: `Christian_control/basic_control/src/CartesianReference.h`
- Modify: `Christian_control/basic_control/src/CartesianReference.cpp`
- Modify: `Christian_control/basic_control/src/ExecutionCore.h`
- Modify: `Christian_control/basic_control/src/ExecutionCore.cpp`
- Modify: `Christian_control/simulation/src/DualSimulationRunner.h`
- Modify: `Christian_control/simulation/src/DualSimulationRunner.cpp`
- Modify: `Christian_control/basic_control/CMakeLists.txt`
- Modify: `Christian_control/simulation/CMakeLists.txt` (simulation-side sources/tests in this task)

**Interfaces:**
- Produces a two-phase no-fail-after-prepare activation API:

```cpp
struct PreparedReference {
    std::unique_ptr<WorldCartesianTrajectory> trajectory;
    CartesianPose activation_pose_world;
};

struct ReferenceRejection { std::string reason; };
struct DualActivationResult {
    bool activated;
    std::uint64_t trajectory_id;
    std::string rejection_reason;
};

std::variant<PreparedReference, ReferenceRejection>
CartesianReferenceSource::Prepare(
    std::unique_ptr<WorldCartesianTrajectory>,
    const MeasuredCartesianState&, const RobotState&) const;

void CartesianReferenceSource::Commit(PreparedReference prepared) noexcept;

class DualReferenceCoordinator {
public:
    DualActivationResult PollPrepareAndCommit(
        ArmExecutionCore& right, ArmExecutionCore& left,
        const MeasuredCartesianState& right_measured,
        const MeasuredCartesianState& left_measured,
        const RobotState& right_state, const RobotState& left_state);
};
```

- [ ] **Step 1: Write all-or-neither activation tests**

Cover valid pair, invalid right, invalid left, wrong common ID, stale provenance, discontinuous start, replacement while tracking, one-arm hold, and mailbox latest-wins. Assert one rejection leaves both active IDs/clocks unchanged and one success changes both on the same cycle.

- [ ] **Step 2: Run and observe current independent activation**

Run: `cmake --build Christian_control/basic_control/build --target test_dual_reference_coordinator -j2`

- [ ] **Step 3: Split validation from mutation**

Make `Prepare` pure with respect to active reference state and allocate only outside the control step's critical arithmetic. Once both preparations succeed, `Commit` performs only noexcept ownership/state swaps before either reference is sampled for that cycle.

- [ ] **Step 4: Integrate coordinator before both core steps**

On each simulation tick: read both measured states, poll one complete pair, prepare both, commit both or reject both, then step right and left cores. Record a single paired activation/rejection event.

- [ ] **Step 5: Run reference and simulation suites**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(dual_reference_coordinator|cartesian_reference|execution_core)$' --output-on-failure`

Run: `ctest --test-dir Christian_control/simulation/build -R '^dual_simulation_runner$' --output-on-failure`

Review checkpoint: prove with an instrumented test that both arms first sample the new trajectory at identical `trajectory_time_s == 0`.

### Task 5: Add live and deterministic planner simulation modes

**Files:**
- Create: `Christian_control/simulation/src/PlannerResultSource.h`
- Create: `Christian_control/simulation/src/LivePlannerResultSource.cpp`
- Create: `Christian_control/simulation/src/ScriptedPlannerResultSource.cpp`
- Create: `Christian_control/simulation/src/SimPlannerIo.h`
- Create: `Christian_control/simulation/src/SimPlannerIo.cpp`
- Create: `Christian_control/simulation/tests/test_scripted_replanning.cpp`
- Create: `Christian_control/planner_bridge/tests/test_sim_full_pipeline.sh`
- Modify: `Christian_control/simulation/src/SimMain.cpp`
- Modify: `Christian_control/planner_bridge/scripts/run_session.sh`
- Modify: `Christian_control/planner_bridge/CMakeLists.txt`
- Modify: `Christian_control/basic_control/CMakeLists.txt`
- Modify: `Christian_control/simulation/CMakeLists.txt` (planner result sources and `test_scripted_replanning` live in the simulation project)

**Interfaces:**
- `LivePlannerResultSource` reads complete paired blocks from the real planner process.
- `ScriptedPlannerResultSource` loads provenance-checked paired blocks and publishes them at explicit control tick numbers.

- [ ] **Step 1: Write deterministic injection tests**

Inject trajectory A at tick 100 and B at tick 600; repeat twice; require identical activation ticks, trajectory IDs, reference clocks, controller traces, and final state. Add controlled invalid/failure events and coalescing order.

- [ ] **Step 2: Implement bounded non-RT planner I/O**

Keep pipe open/read/parse in a non-real-time thread. Publish only fully validated immutable pairs into the dual mailbox. On disconnect, retain current reference and retry outside control.

- [ ] **Step 3: Add full live pipeline shell test**

Launch headless `humansl_sim` and real `planner_bridge` against temporary FIFOs/configs, submit a small static-base dual goal, observe one paired result and atomic activation, then stop the process group. Stub only GPMP2 when the installed build is unavailable; the normal configured test must use the existing GPMP2 build.

- [ ] **Step 4: Separate numeric and live evidence**

Run live mode as contract/smoke evidence without fixed RMSE thresholds. Record one successful real planner block with code/model/config hashes; use scripted mode for numeric scenario acceptance.

- [ ] **Step 5: Run full hardware-free planner/simulation suites**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^(dual_planner_worker|sim_full_pipeline|inter_arm_distance)$' --output-on-failure`

Run: `ctest --test-dir Christian_control/simulation/build -R '^(scripted_replanning|dual_simulation_runner)$' --output-on-failure`

Run: `ctest --test-dir Christian_control/basic_control/build -R '^dual_reference_coordinator$' --output-on-failure`

Final gate: run one interactive live solve and one deterministic scripted replay in MuJoCo. Confirm both use the same serialized result contract and neither sends planned posture into the cores. Plan 05 may begin after paired activation and replanning evidence are accepted.
