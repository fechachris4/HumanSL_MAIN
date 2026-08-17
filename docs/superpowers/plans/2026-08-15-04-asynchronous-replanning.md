# Asynchronous Fresh-snapshot Replanning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replan outside 500 Hz from coherent fresh world snapshots, coalesce newer requests, atomically replace only valid trajectories, and implement brief/prolonged Vicon loss transitions.

**Architecture:** The cyclic loop publishes fixed-size planning requests to an SPSC slot. A non-real-time writer sends them over a request FIFO to one long-lived `planner_bridge` worker per arm; its reader coalesces requests latest-wins while the main worker solves sequentially and writes complete Cartesian blocks to the existing trajectory FIFO.

**Tech Stack:** C++17, POSIX FIFOs/poll, atomics/SPSC slots, existing planner and controller processes, CMake/CTest, shell session harness.

## Global Constraints

- One solve per arm at a time; pending requests coalesce to the newest.
- Request snapshot includes request ID, arm, measured `q` in Kortex order, `T_W_M`, Vicon sequence/frame/timestamps/age/validity.
- A solve starts only from a fresh valid snapshot (`age <= 0.05 s`).
- Brief stale: pause trajectory clock, retain current world pose reference, ZOH pose, and exponentially decay stale Mount twist to zero.
- Brief recovery resumes the paused trajectory with no wall-clock catch-up.
- Prolonged stale (`>= 0.20 s`): cancel trajectory; fresh recovery captures current world pose hold and requests a new plan.
- Old valid trajectory/hold remains active until a fully validated continuous replacement is available.
- No Vicon, GPMP2, pipe, file, terminal, log, lock, wait, or allocation work in the cyclic path.
- Preserve all Kinova safety/actuation behavior. No robot run or commit without separate authorization.

---

### Task 1: Fixed-size planning-request contract and wait-free handoff

**Files:**
- Create: `Christian_control/cartesian_contract/PlanningRequest.h`
- Create: `Christian_control/cartesian_contract/PlanningRequest.cpp`
- Create: `Christian_control/basic_control/src/PlanningRequestSlot.h`
- Create: `Christian_control/basic_control/tests/test_planning_request.cpp`
- Modify: both standalone `CMakeLists.txt` files.

**Interfaces:**
- Produces: `PlanningRequest`, `ValidatePlanningRequest`, `FormatPlanningRequestBlock`, `PlanningRequestAccumulator`, and single-producer/single-consumer `PlanningRequestSlot`.

- [ ] **Step 1: Write round-trip, rejection, and latest-wins tests**

```text
PLAN_REQUEST_BEGIN 1 <request_id> <arm> <vicon_sequence> <vicon_frame>
TIMING <receive_steady_s> <age_s>
WORLD_T_MOUNT <px> <py> <pz> <qx> <qy> <qz> <qw>
Q_RAD <q1> <q2> <q3> <q4> <q5> <q6> <q7>
PLAN_REQUEST_END
```

Reject non-finite values, non-unit quaternion, unknown arm/version, duplicate/missing rows, age outside `[0,0.05]`, sequence zero, and non-monotonic IDs at the worker boundary. Publish three requests before one `TakeLatest` and assert only the newest returns.

- [ ] **Step 2: Confirm tests fail before implementation**

Run: `cmake --build Christian_control/basic_control/build --target test_planning_request -j2`

- [ ] **Step 3: Implement the fixed-size protocol and triple-buffer slot**

Use the same ownership-transfer pattern as `BasePoseSlot`; the cyclic producer copies no vectors/strings and never frees memory.

- [ ] **Step 4: Run request tests under normal and a fresh ThreadSanitizer build**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^planning_request$' --output-on-failure`

Run:

```bash
planning_tsan_build="$(mktemp -d)"
cmake -S Christian_control/basic_control -B "$planning_tsan_build" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-fsanitize=thread -fno-omit-frame-pointer' \
  -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=thread'
cmake --build "$planning_tsan_build" --target test_planning_request -j2
ctest --test-dir "$planning_tsan_build" -R '^planning_request$' --output-on-failure
```

### Task 2: Controller-side request publication and non-RT FIFO writer

**Files:**
- Create: `Christian_control/basic_control/src/PlanningRequestWriter.h`
- Create: `Christian_control/basic_control/src/PlanningRequestWriter.cpp`
- Create: `Christian_control/basic_control/tests/test_planning_request_writer.cpp`
- Modify: `Christian_control/basic_control/src/CartesianReference.h`
- Modify: `Christian_control/basic_control/src/CartesianReference.cpp`
- Modify: `Christian_control/basic_control/src/Runner.h`
- Modify: `Christian_control/basic_control/src/Runner.cpp`
- Modify: `Christian_control/basic_control/src/Main.cpp`
- Modify: `Christian_control/basic_control/src/Config.h`

**Interfaces:**
- Consumes: `ControllerStatus::request_replan_edge`, current measured `q`, and the exact `BasePoseSample` read that cycle.
- Produces:

```cpp
void PlanningRequestSlot::Publish(const PlanningRequest& request);
bool PlanningRequestSlot::TakeLatest(PlanningRequest& request);
void RunPlanningRequestWriter(PlanningRequestSlot& slot,
                              const std::atomic<bool>& stop,
                              const std::string& request_pipe_path);
```

- [ ] **Step 1: Test startup and recovery request edges**

Assert exactly one request on first fresh world hold, none on repeated fresh cycles, one after prolonged-stale recovery, monotonically increasing request IDs, and exact q/Mount provenance from the triggering cycle. Test FIFO absence, late reader, disconnect/reopen, latest-wins writes, and prompt stop/join.

- [ ] **Step 2: Run targeted tests and observe missing writer/state transitions**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(cartesian_reference|planning_request|planning_request_writer)$' --output-on-failure`

- [ ] **Step 3: Publish requests only after control computation, before no I/O**

The loop performs one fixed-size `PlanningRequestSlot::Publish` on an edge. The writer thread owns `open/poll/write/close`, logs lifecycle messages, and cannot outlive the slot. Add per-arm request FIFO paths beside the existing trajectory FIFO path in `ArmConfig`.

- [ ] **Step 4: Verify no forbidden calls entered Runner's cycle**

Run targeted tests, then inspect the diff from the top of the normal-cycle loop through `CyclicSession::Send`; only fixed arithmetic/copies and slot publication are allowed.

### Task 3: Long-lived sequential planner worker with request coalescing

**Files:**
- Create: `Christian_control/planner_bridge/src/PlannerWorker.h`
- Create: `Christian_control/planner_bridge/src/PlannerWorker.cpp`
- Create: `Christian_control/planner_bridge/tests/test_planner_worker.cpp`
- Modify: `Christian_control/planner_bridge/src/BridgeMain.h`
- Modify: `Christian_control/planner_bridge/src/BridgeMain.cpp`
- Modify: `Christian_control/planner_bridge/src/main.cpp`
- Modify: `Christian_control/planner_bridge/CMakeLists.txt`

**Interfaces:**
- Adds: `planner_bridge --serve --arm <right|left> --request-pipe PATH --trajectory-pipe PATH`.
- Consumes valid `PlanningRequest`; invokes the existing one-solve function with request q and `T_W_M`.
- Produces complete `CART_TRAJ_*` blocks tagged with request/trajectory and planner Vicon sequence.

- [ ] **Step 1: Write a fake-solver concurrency test**

Use a condition-variable-controlled fake solve function: publish request 1 and block its solve; publish requests 2 and 3; release solve 1; assert calls are `[1,3]`, never concurrent, and outputs are ordered complete blocks. Assert solve failure leaves no partial block and does not consume request 3.

- [ ] **Step 2: Run the worker test and confirm failure**

Run: `cmake --build Christian_control/planner_bridge/build --target test_planner_worker -j2`

- [ ] **Step 3: Implement one explicit worker loop, not a manager/service hierarchy**

One reader thread parses request FIFO blocks into a latest-wins slot. The main thread takes one request, loads goal/config, solves synchronously, buffers/validates/projections, then writes one whole Cartesian block. While it solves, the reader may overwrite only the pending slot. Stop joins the reader before owned buffers disappear.

- [ ] **Step 4: Run worker, bridge, solver, and contract suites**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^(planner_worker|bridge_main|plan_solver|cartesian_contract|world_trajectory_projection)$' --output-on-failure`

### Task 4: Brief/prolonged Vicon loss and smooth recovery

**Files:**
- Modify: `Christian_control/basic_control/src/CartesianReference.h`
- Modify: `Christian_control/basic_control/src/CartesianReference.cpp`
- Modify: `Christian_control/basic_control/src/Runner.cpp`
- Modify: `Christian_control/basic_control/src/Config.h`
- Modify: `Christian_control/basic_control/tests/test_cartesian_reference.cpp`
- Modify: `Christian_control/basic_control/tests/test_controller.cpp`

**Interfaces:**
- Produces explicit freshness state and reference/replan edges; uses `0.05 s` fresh and `0.20 s` prolonged-stale defaults.

- [ ] **Step 1: Add deterministic time-series tests**

Drive samples at 2 ms: active trajectory to `t_ref=0.100`; stale for 0.10 s; assert `t_ref` remains 0.100 and pose reference unchanged; recover and assert next time is 0.102, not wall-clock 0.202. During stale, assert Mount twist multiplier is `exp(-(age-0.05)/tau_decay)` and continuous at the threshold.

Drive stale past 0.20 s; assert active trajectory is cancelled once, recovery captures measured world pose with zero twist, and one replan edge fires. Assert an old trajectory cannot resume.

- [ ] **Step 2: Run tests and observe current world-hold semantics fail**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(cartesian_reference|controller)$' --output-on-failure`

- [ ] **Step 3: Implement freshness transitions in the source and twist decay in Runner**

Use clamped control `dt` for state duration and reference clock; use measured sample age only for freshness classification. Never finite-difference in Runner. Keep ZOH `T_W_M`; decay only `V_W_M`. On fresh recovery after prolonged stale, measure before capturing the new hold.

- [ ] **Step 4: Run all basic-control hardware-free tests**

Run: `ctest --test-dir Christian_control/basic_control/build --output-on-failure`

### Task 5: Session orchestration and end-to-end hardware-free rehearsal

**Files:**
- Modify: `Christian_control/planner_bridge/scripts/run_session.sh`
- Modify: `Christian_control/planner_bridge/tests/test_run_session.sh`
- Modify: `Christian_control/basic_control/README.md`
- Modify: `docs/architecture.md`
- Modify: `docs/architecture_and_debugging_audit.md`

**Interfaces:**
- Starts one controller process and one planner worker per selected arm; preserves signal ownership and session artifacts.

- [ ] **Step 1: Extend shell stubs for request and trajectory FIFOs**

Assert controller SIGINT still reaches the real controller PID, each arm gets its own planner PID/FIFOs, initial request produces a Cartesian artifact, newer requests coalesce in the worker stub, planner failure sends no partial trajectory, and EXIT traps stop/join every child.

- [ ] **Step 2: Run the shell rehearsal and confirm old one-shot behavior fails expectations**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^run_session$' --output-on-failure`

- [ ] **Step 3: Launch planner workers before permitting trajectory exchange**

Keep the existing `GO` hardware authorization prompt and plain controller redirection that makes `$!` the actual process. Add planner PIDs to teardown; save every request, Cartesian plan, and diagnostic with arm/request IDs. Never invoke `controller` in this test.

- [ ] **Step 4: Full offline verification and safety review**

Run: `ctest --test-dir Christian_control/vicon/build --output-on-failure`

Run: `ctest --test-dir Christian_control/planner_bridge/build --output-on-failure`

Run: `ctest --test-dir Christian_control/basic_control/build --output-on-failure`

Run: `git diff --check`

Perform the complete `kinova-safe-cpp/references/safety-review.md` audit: frames/units/signs, first-cycle/stall timing, singular/non-finite behavior, all Kortex stop/teardown paths, SPSC ownership/lifetimes, schema/provenance, and proof that no robot-facing command was executed.

Review checkpoint: physical world-frame tracking, Vicon filter tuning, and dropout transitions remain unverified on hardware until Christian separately approves a supervised command with workspace/e-stop conditions.
