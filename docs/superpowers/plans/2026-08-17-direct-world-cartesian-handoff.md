# Direct World-Cartesian Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with verification checkpoints.

**Goal:** Move production planning into the controller process and transfer the
existing `WorldCartesianTrajectory` by typed C++ ownership transfer, removing
production FIFO/file IPC without changing the 500 Hz execution law.

**Architecture:** `controller` owns one latest-value `PlanningRequestSlot`, one
non-real-time planner worker per selected arm, and the existing
`CartesianTrajectoryMailbox`. The worker calls the planner's typed solve API
and moves the returned `std::unique_ptr<WorldCartesianTrajectory>` into the
mailbox. The existing execution core remains the only cyclic command path.
The standalone `planner_bridge` CLI remains an offline preview wrapper, but
its text output is no longer a production controller interface.

**Tech Stack:** C++17, CMake, Eigen, Pinocchio, GPMP2/GTSAM, pthread, Python
panel/session tooling, CTest.

## Global Constraints

* `WorldCartesianTrajectory` remains the planner/controller boundary; planned
  GPMP2 joint states do not cross it.
* GPMP2 and all planner model/solve work remain outside the 500 Hz control
  loop.
* The planner remains the source of truth for trajectory construction and
  planner-side validity; the controller retains only live-state activation and
  hardware-safety checks.
* Preserve metres, radians, seconds, world-frame TCP pose/twist, quaternion
  `x y z w`, point order, timestamps, trajectory IDs, and Vicon provenance.
* Preserve Kortex takeover, velocity/joint limits, following-error, watchdog,
  fault, emergency-stop, and teardown behavior.
* No new generic bus, registry, factory, singleton, or message framework.
* No robot-facing command may be run; builds, tests, and offline preview only.
* Existing unrelated dirty changes remain untouched; do not commit or push.

---

### Task 1: Add a typed planner result API while preserving the preview CLI

**Files:**

- Create: `Christian_control/planner_bridge/src/PlannerRuntime.h`
- Create: `Christian_control/planner_bridge/src/PlannerRuntime.cpp`
- Create: `Christian_control/cartesian_contract/WorldCartesianTrajectoryWire.h`
- Create: `Christian_control/cartesian_contract/WorldCartesianTrajectoryWire.cpp`
- Modify: `Christian_control/planner_bridge/src/BridgeMain.cpp`
- Modify: `Christian_control/planner_bridge/src/BridgeMain.h`
- Modify: `Christian_control/planner_bridge/CMakeLists.txt`
- Test: `Christian_control/planner_bridge/tests/test_planner_runtime.cpp`

**Interfaces:**

```cpp
struct PlannerRuntimeConfig {
    std::string goal_file;
    std::string planner_config_file;
    std::string joint_limits_file;
    std::string right_dh_file;
    std::string left_dh_file;
    std::string runs_root;
};

struct PlannerSolveResult {
    int exit_code = 1;
    std::unique_ptr<WorldCartesianTrajectory> trajectory;
};

PlannerSolveResult SolveWorldTrajectory(
    const std::vector<std::string>& args,
    std::ostream& diagnostics);

PlannerSolveResult SolveWorldTrajectoryForRequest(
    const PlanningRequest& request,
    const PlannerRuntimeConfig& config,
    std::ostream& diagnostics);
```

- [ ] **Step 1: Write the failing typed-result test.** Construct a fixed
  `PlanningRequest` with seven known joint angles, an identity Mount pose,
  explicit temporary-safe planner paths already used by the planner tests, a
  nonzero trajectory ID, and a Vicon sequence. Call
  `SolveWorldTrajectoryForRequest`; assert exit code 0, a non-null result,
  `points.size() >= 2`, first time `0.0`, strictly increasing times, world
  pose/twist fields finite, final point arrival-eligible with zero twist, and
  IDs/provenance equal to the request.

- [ ] **Step 2: Run only the new test to confirm the expected missing-symbol
  or missing-API failure.**

  ```bash
  cmake --build Christian_control/planner_bridge/build --target test_planner_runtime -j2
  ctest --test-dir Christian_control/planner_bridge/build -R planner_runtime --output-on-failure
  ```

- [ ] **Step 3: Move the existing `RunBridge` solve/projection body behind
  `SolveWorldTrajectory`.** Preserve its existing argument parsing, planner
  checks, diagnostics, exit-code meanings, world-frame projection, and
  `ValidateWorldCartesianTrajectory` call. `SolveWorldTrajectoryForRequest`
  shall build the same explicit arguments the worker currently builds, using
  the request's q/world Mount/Vicon sequence/trajectory ID and the explicit
  runtime paths. It shall return the typed object, not a formatted block.

- [ ] **Step 4: Make `RunBridge` a thin offline-preview wrapper.** It calls
  `SolveWorldTrajectory`, formats the returned object only for its standalone
  stdout contract, and preserves the existing UI preview exit codes and
  diagnostics. No production controller code may call this formatter.

- [ ] **Step 5: Add the new source to the planner library target and run the
  targeted test.**

  ```bash
  cmake --build Christian_control/planner_bridge/build --target test_planner_runtime -j2
  ctest --test-dir Christian_control/planner_bridge/build -R planner_runtime --output-on-failure
  ```

- [ ] **Step 6: Run the affected existing planner tests.**

  ```bash
  ctest --test-dir Christian_control/planner_bridge/build -R 'bridge_main|plan_solver|world_trajectory_projection|circle_plan' --output-on-failure
  ```

No production FIFO is removed in this task; the old preview output remains
available until the in-process path is proven.

### Task 2: Add the concrete in-process planner worker and direct handoff test

**Files:**

- Create: `Christian_control/basic_control/src/InProcessPlanner.h`
- Create: `Christian_control/basic_control/src/InProcessPlanner.cpp`
- Modify: `Christian_control/basic_control/CMakeLists.txt`
- Test: `Christian_control/basic_control/tests/test_in_process_planner.cpp`

**Interfaces:**

```cpp
void RunInProcessPlanner(
    PlanningRequestSlot& requests,
    CartesianTrajectoryMailbox& trajectories,
    const PlannerRuntimeConfig& planner_config,
    const std::atomic<bool>& stop,
    PlannerSolveResult (*solve)(const PlanningRequest&,
                                const PlannerRuntimeConfig&,
                                std::ostream&));
```

- [ ] **Step 1: Write the failing handoff/lifetime test.** Start the worker
  with a deterministic request and a test-only function-pointer solve seam
  matching the interface above; the seam returns a known
  `WorldCartesianTrajectory` through the same concrete publish operation.
  Assert that the object taken from the mailbox has exactly the source ID,
  sequence, point values, ordering, and timestamps; mutate or destroy the
  source-side owner after publication and assert the mailbox object remains
  valid. Assert that setting `stop` before a pending result prevents a late
  publish and that worker join completes. The function pointer is a test seam,
  not a production abstraction: production passes the concrete
  `SolveWorldTrajectoryForRequest` function.

- [ ] **Step 2: Run the test and confirm it fails before the worker exists.**

  ```bash
  cmake --build Christian_control/basic_control/build --target test_in_process_planner -j2
  ctest --test-dir Christian_control/basic_control/build -R in_process_planner --output-on-failure
  ```

- [ ] **Step 3: Implement the smallest concrete worker loop.** Consume the
  existing latest request slot without blocking the cyclic thread, call
  `SolveWorldTrajectoryForRequest` outside the cyclic thread, publish only a
  non-null successful typed result, and check `stop` before publication.
  Diagnostics go to the existing non-cyclic output path; no planner call,
  allocation, sleep, or I/O is introduced into `Step`/`ResolveStop`.

- [ ] **Step 4: Run the direct-handoff test and the existing mailbox/reference
  tests.**

  ```bash
  ctest --test-dir Christian_control/basic_control/build -R 'in_process_planner|cartesian_reference|execution_core' --output-on-failure
  ```

### Task 3: Link planner code into the controller and wire lifecycle ownership

**Files:**

- Modify: `Christian_control/basic_control/CMakeLists.txt`
- Modify: `Christian_control/basic_control/src/Main.cpp`
- Modify: `Christian_control/basic_control/src/Runner.cpp`
- Modify: `Christian_control/basic_control/src/Config.h`
- Modify: `Christian_control/planner_bridge/CMakeLists.txt`
- Modify: `Christian_control/planner_bridge/src/PlannerWorker.cpp`
- Modify: `Christian_control/planner_bridge/src/PlannerWorker.h`

- [ ] **Step 1: Add the controller-side integration test/build gate before
  production wiring.** The test shall link the same planner runtime library
  that `controller` will link and assert the controller target pulls in the
  typed planner symbol while `humansl_execution_core` remains free of Kortex,
  Vicon, GPMP2, and GTSAM symbols.

- [ ] **Step 2: Configure one reusable planner runtime target.** Put the
  typed planner runtime and existing planner implementation in a library
  consumable by both the standalone preview executable and `controller`.
  When nested from `basic_control`, disable only planner-bridge test
  registration and standalone executable construction; retain the generated
  DH YAML dependencies needed by the controller build. Do not duplicate the
  planner source list in two targets.

- [ ] **Step 3: Add explicit runtime paths to the controller's compiled
  configuration.** Resolve goal, planner YAML, joint-limits YAML, both
  generated DH files, and runs root from repository/build paths, not the
  controller's current working directory. Keep the existing run-session
  artifact copies and binary freshness checks meaningful.

- [ ] **Step 4: In `RunOneArm`, construct the existing
  `PlanningRequestSlot` and `CartesianTrajectoryMailbox`, start exactly one
  `RunInProcessPlanner` thread, and pass the same request slot to
  `RunControlLoop`. Join the planner thread before destroying the mailbox,
  execution core, kinematics, or planner runtime objects. Preserve the
  current per-arm shared stop behavior for `--arm both`.

- [ ] **Step 5: Remove only the planning-request writer thread.** Keep the
  cyclic request publication into `PlanningRequestSlot`; delete FIFO creation,
  `RunPlanningRequestWriter`, writer joiner, request-pipe fields, and request
  serialization from the production controller path.

- [ ] **Step 6: Build and run the controller integration/linkage checks.**

  ```bash
  cmake --build Christian_control/basic_control/build --target controller test_in_process_planner execution_core_linkage -j2
  ctest --test-dir Christian_control/basic_control/build -R 'in_process_planner|execution_core|execution_core_linkage' --output-on-failure
  ```

### Task 4: Remove controller trajectory FIFO ingestion and duplicate structural validation

**Files:**

- Modify: `Christian_control/basic_control/src/CartesianTrajectoryMailbox.h`
- Modify: `Christian_control/basic_control/src/CartesianTrajectoryMailbox.cpp`
- Modify: `Christian_control/basic_control/src/CartesianReference.cpp`
- Modify: `Christian_control/basic_control/tests/test_cartesian_trajectory_input.cpp`
- Modify: `Christian_control/basic_control/CMakeLists.txt`
- Modify: `Christian_control/cartesian_contract/WorldCartesianTrajectory.h`
- Modify: `Christian_control/cartesian_contract/WorldCartesianTrajectory.cpp`

- [ ] **Step 1: Replace the FIFO parser test with a typed mailbox
  characterization test.** Assert one direct `Publish(std::move(...))` →
  `Take()` transfer, latest-unread replacement, exact point equality, and
  retire/reclaim behavior. Run it red against the old test target after
  changing the test expectation.

- [ ] **Step 2: Remove `RunCartesianTrajectoryInput`,
  `RunCartesianTrajectoryInputFromPipe`, POSIX headers, polling, read buffers,
  line limits, and FIFO-specific comments from the mailbox production files.
  Keep only the concrete typed mailbox and its non-cyclic reclamation path.

- [ ] **Step 3: Remove the controller's structural
  `ValidateWorldCartesianTrajectory` call from `TryActivate`. Preserve the
  existing live provenance and measured-start continuity checks and all
  status fields that explain activation/rejection.

- [ ] **Step 4: Move `FormatWorldCartesianTrajectoryBlock` and
  `WorldCartesianTrajectoryAccumulator` into an explicit preview-only
  `WorldCartesianTrajectoryWire.h/.cpp` pair. Link that pair only into the
  standalone planner CLI and preview tests. Keep
  `WorldCartesianTrajectory.h/.cpp` limited to typed data and planner-side
  contract validation, so the controller target cannot compile or call a
  trajectory serializer/deserializer.

- [ ] **Step 5: Run the mailbox, reference, characterization, and linkage tests.**

  ```bash
  cmake --build Christian_control/basic_control/build --target test_cartesian_trajectory_input test_cartesian_reference test_execution_characterization test_execution_core execution_core_linkage -j2
  ctest --test-dir Christian_control/basic_control/build -R 'cartesian_trajectory_input|cartesian_reference|execution_characterization|execution_core|execution_core_linkage' --output-on-failure
  ```

### Task 5: Remove production planner service/FIFO launch and update UI/session status

**Files:**

- Modify: `Christian_control/planner_bridge/src/PlannerWorker.cpp`
- Modify: `Christian_control/planner_bridge/src/PlannerWorker.h`
- Modify: `Christian_control/planner_bridge/src/main.cpp`
- Modify: `Christian_control/planner_bridge/scripts/run_session.sh`
- Modify: `Christian_control/planner_bridge/tests/test_run_session.sh`
- Modify: `Christian_control/tools/panel/session.py`
- Modify: `Christian_control/tools/panel/build.py`
- Modify: `Christian_control/tools/panel/plan.py`
- Modify: `Christian_control/tools/panel/tests/test_session.py`
- Modify: `Christian_control/tools/panel/tests/test_build.py`

- [ ] **Step 1: Add a shell regression test for the new single-process
  launcher.** Use stub controller/planner-runtime evidence to assert the
  launcher starts only `controller`, does not create/wait for target or
  request FIFOs, does not pass `--allow-stale`, and still propagates Enter and
  SIGINT through the existing session cleanup path.

- [ ] **Step 2: Remove `RunPlannerService`, `--serve`, request/trajectory pipe
  arguments, `WriteTrajectory`, planner publication markers, and FIFO-specific
  worker code from the production path. Keep the standalone preview entry
  point separate from session execution.

- [ ] **Step 3: Update `run_session.sh` to freshness-check the controller
  against both controller and planner sources, start only the controller, keep
  its session artifact/provenance handling, and wait for run-log data plus a
  controller telemetry activation edge instead of FIFO publication files.

- [ ] **Step 4: Update the panel session module and UI-facing diagnostics to
  describe one controller process. Preserve localhost-only start, typed `GO`,
  remote stop, stale-build refusal, and the offline preview subprocess path.

- [ ] **Step 5: Run the shell and panel tests.**

  ```bash
  ctest --test-dir Christian_control/planner_bridge/build -R run_session --output-on-failure
  python3 -m pytest -q Christian_control/tools/panel/tests/test_session.py Christian_control/tools/panel/tests/test_build.py Christian_control/tools/panel/tests/test_plan.py
  ```

### Task 6: Remove stale FIFO references, update documentation, and verify the complete migration

**Files:**

- Modify: `Christian_control/basic_control/README.md`
- Modify: `Christian_control/basic_control/src/Config.h`
- Modify: `Christian_control/basic_control/src/Main.cpp`
- Modify: `docs/architecture.md`
- Modify: relevant CTest descriptions and tests found by the final search

- [ ] **Step 1: Replace operator-facing topology text.** Document that the
  controller process owns the planner worker, the typed trajectory handoff,
  the worker's non-real-time timing, and the standalone preview distinction.

- [ ] **Step 2: Remove stale FIFO paths, pipe names, planner-subprocess claims,
  and duplicate parser descriptions from production docs and comments.

- [ ] **Step 3: Run repository searches proving production FIFO removal.**

  ```bash
  rg -n -g '!build/**' -g '!runs/**' -g '!third_party/**' \
    'mkfifo|planning_request_pipe|target_pipe|RunPlanningRequestWriter|RunCartesianTrajectoryInputFromPipe|RunPlannerService|published_.*\\.ok' \
    Christian_control docs README.md
  ```

  Expected: no production controller/session references; any remaining
  matches must be explicitly limited to historical documentation or the
  standalone offline-preview contract.

- [ ] **Step 4: Configure/build the affected projects and run all hardware-free
  suites.**

  ```bash
  cmake -S Christian_control/basic_control -B Christian_control/basic_control/build
  cmake --build Christian_control/basic_control/build -j2
  cmake -S Christian_control/planner_bridge -B Christian_control/planner_bridge/build
  cmake --build Christian_control/planner_bridge/build -j2
  ctest --test-dir Christian_control/basic_control/build --output-on-failure -j2
  ctest --test-dir Christian_control/planner_bridge/build --output-on-failure -j2
  python3 -m pytest -q Christian_control/tools/panel/tests
  ```

- [ ] **Step 5: Inspect the final diff and report exact files changed,
  production code removed, ownership/lifecycle behavior, preserved behavior,
  intentional behavior changes, test results, remaining panel/test failures,
  and the fact that no robot-facing command was executed.**

## Verification checklist

- [ ] Planner output and direct handoff preserve values, ordering, units,
  frames, timestamps, IDs, and provenance.
- [ ] No planner solve or FIFO/file operation occurs in the 500 Hz cycle.
- [ ] Controller runtime checks and hardware teardown remain active.
- [ ] Shutdown joins planner workers before dependent objects are destroyed.
- [ ] Production launcher/UI no longer starts a planner subprocess or waits on
  FIFO artifacts.
- [ ] Hardware-free C++/Python tests and link checks are fresh and recorded.
- [ ] No robot-facing command was run.
