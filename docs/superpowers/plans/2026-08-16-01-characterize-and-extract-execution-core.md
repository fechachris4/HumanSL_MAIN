# Execution Core Characterization and Extraction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract one hardware-independent arm execution core from the current controller without changing its configured controller, reference, limiting, integration, or generic stop behaviour.

**Architecture:** Freeze pre-extraction evidence first. Introduce immutable configuration and explicit cycle input/result records, compose the existing controller/reference/actuation functions in `ArmExecutionCore`, then make the Kortex runner call that core while retaining Kortex takeover, exchange, acknowledgement, mode, fault, timing, logging, and teardown ownership.

**Tech Stack:** C++17, Eigen, Pinocchio, CMake/CTest, existing format 9–13 CSV tooling, AddressSanitizer/UndefinedBehaviorSanitizer.

## Global Constraints

- `docs/engineering/humansl-engineering-contract.md` is binding for every task in this plan (notably §8 forbidden 500 Hz dependencies, §9 separate command states, §12 characterize-before-refactor and mutation testing).
- This is a Level-2 refactor of the hardware executable; intended physical behaviour is unchanged.
- Preserve the pose/twist-only controller boundary; no `q_ref`, `qdot_ref`, planned posture, or null-space posture bias.
- Preserve `V_task = Kp*poseError + Kd*(V_reference-V_measured)` and the current Mount-twist construction.
- Preserve configured velocity limits, joint-boundary full-frame hold, position integration, following-error policy, stop priority, and teardown.
- Kortex takeover, mode/fault/acknowledgement handling, communication, and restoration remain outside the shared core.
- The 500 Hz path remains allocation-, lock-, file-, terminal-, planner-, Vicon-I/O-, and ordinary-log-I/O-free.
- Historical format 9–11 logs may prove only fields they contain; missing Cartesian inputs must never be fabricated.
- The hardware executable must not be run. Building and hardware-free tests are allowed.
- Preserve unrelated dirty-tree changes. Do not commit without Christian's explicit authorization.

---

### Task 1: Freeze pre-extraction characterization evidence

**Files:**
- Create: `Christian_control/basic_control/tests/execution_characterization.h`
- Create: `Christian_control/basic_control/tests/test_execution_characterization.cpp`
- Create: `Christian_control/basic_control/tests/fixtures/execution_preextract_v1.csv`
- Create: `Christian_control/basic_control/scripts/extract_execution_history.py`
- Create: `Christian_control/basic_control/tests/test_extract_execution_history.py`
- Modify: `Christian_control/basic_control/CMakeLists.txt`

**Interfaces:**
- Consumes: current `TrackingController`, `CartesianReferenceSource`, `PositionIntegration`, `ClampJointVelocity`, `ResolveStopPriority`, format 9–11 historical CSV rows.
- Produces:

```cpp
struct CharacterizationCycle {
    RobotState state;
    double dt_s;
    JointVector measured_deg;
    JointVector previous_command_deg;
    PoseReference reference;
};

struct CharacterizationExpected {
    MeasuredCartesianState measured;
    JointVector qdot_raw_rad_s;
    JointVector qdot_limited_deg_s;
    JointVector requested_position_deg;
    bool nonfinite;
    std::optional<int> joint_limit_warning_joint;
};
```

- [ ] **Step 1: Write the historical-log extraction tests**

Create temporary format-9 and format-11 CSV fixtures with known measured, requested, commanded, timing, following-error, and stop fields. Assert `extract_execution_history.py` emits only available named fields plus `source_log_format`, and omits Cartesian reference/twist fields rather than writing zeros.

- [ ] **Step 2: Run the Python tests and verify the extractor is missing**

Run: `python3 Christian_control/basic_control/tests/test_extract_execution_history.py -v`

Expected: FAIL because `extract_execution_history.py` does not exist.

- [ ] **Step 3: Implement the strict historical extractor**

Use `csv.DictReader`; require the log preamble and supported format; emit a versioned CSV with explicit empty fields. Reject unnamed columns, duplicate headers, and non-finite required joint measurements. Do not interpolate or synthesize missing values.

- [ ] **Step 4: Write the C++ pre-extraction characterization cases**

Cover fresh startup hold, active Cartesian reference, nonzero measured Mount twist, velocity saturation, bounded-joint outward proposal, non-finite controller output, final hold, brief stale pause, prolonged stale cancellation, and recovery replan edge. Follow the current `Runner.cpp` order explicitly — dt sampling and overrun counting; joint measurement from the previous exchange; the world/Vicon slot read with zero-order hold and stale-twist decay (`Runner.cpp:417-490`); `Measure`; reference `Get`; `DesiredVelocity`; the planning-request publish edge; the non-finite hold counter; `ClampJointVelocity`; integration `Apply`; and only then stop resolution (the Kortex send sits between integration and stop resolution in the hardware runner). Serialize inputs plus expected outputs at 17 significant digits.

- [ ] **Step 5: Build and run characterization before extraction**

Run: `cmake --build Christian_control/basic_control/build --target test_execution_characterization -j2`

Run: `ctest --test-dir Christian_control/basic_control/build -R '^execution_characterization$' --output-on-failure`

Expected: PASS on the current pre-extraction implementation and produce a stable `execution_preextract_v1.csv` whose header includes the Git revision plus SHA-256 hashes of every dirty source/configuration input used by the characterization, so an uncommitted working tree is identified exactly.

Review checkpoint: verify the fixture was produced before any production extraction and that physical logs are labelled partial evidence rather than format-13 proof.

### Task 2: Replace direct configuration reads with an immutable snapshot

**Files:**
- Create: `Christian_control/basic_control/src/ExecutionConfig.h`
- Create: `Christian_control/basic_control/src/ExecutionConfig.cpp`
- Create: `Christian_control/basic_control/tests/test_execution_config.cpp`
- Modify: `Christian_control/basic_control/src/Controller.h`
- Modify: `Christian_control/basic_control/src/Controller.cpp`
- Modify: `Christian_control/basic_control/src/CartesianReference.h`
- Modify: `Christian_control/basic_control/src/CartesianReference.cpp`
- Modify: `Christian_control/basic_control/src/Actuation.h`
- Modify: `Christian_control/basic_control/src/Actuation.cpp`
- Modify: `Christian_control/basic_control/CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
struct ExecutionConfig {
    ReactivePoseGains gains;
    JointVector velocity_limit_deg_s;
    JointVector software_limit_deg;
    double limit_avoid_zone_rad;
    double command_lead_limit_deg;
    double following_error_limit_deg;
    double world_fresh_max_age_s;
    double world_prolonged_stale_s;
    double arrival_position_tolerance_m;
    double arrival_orientation_tolerance_rad;
    double arrival_dwell_s;
    double target_hold_s;
    double null_ramp_duration_s;
    bool stop_on_fault;
    bool disable_following_error_stop;
    int nonfinite_stop_cycles;
    int overrun_stop_cycles;
};

ExecutionConfig ProductionExecutionConfig();
void ValidateExecutionConfig(const ExecutionConfig& config);
```

- [ ] **Step 1: Write a mapping test for every production value**

Assert every `ExecutionConfig` member equals the corresponding current `config::` constant, including per-joint vectors. Add rejection tests for non-finite gains, non-positive limits, invalid stale ordering, negative tolerances, and incorrect bounded-joint magnitudes.

- [ ] **Step 2: Run the test and observe missing types**

Run: `cmake --build Christian_control/basic_control/build --target test_execution_config -j2`

Expected: FAIL with missing `ExecutionConfig` symbols.

- [ ] **Step 3: Implement the snapshot and constructor injection**

Construct `TrackingController`, `CartesianReferenceSource`, and `PositionIntegration` from `const ExecutionConfig&`. Replace direct `config::` reads in their runtime methods with stored immutable values. Keep `ProductionExecutionConfig()` as the only mapping from compiled production constants.

- [ ] **Step 4: Run configuration and existing behaviour suites**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(execution_config|controller|cartesian_reference|control_logic|reactive_law)$' --output-on-failure`

Expected: PASS with unchanged characterization fixture output.

Review checkpoint: `rg -n 'config::' Controller.cpp CartesianReference.cpp Actuation.cpp` may match only construction-time or compile-time comments, not per-cycle behaviour reads.

### Task 3: Introduce the explicit arm execution contract

**Files:**
- Create: `Christian_control/basic_control/src/ExecutionCore.h`
- Create: `Christian_control/basic_control/src/ExecutionCore.cpp`
- Create: `Christian_control/basic_control/tests/test_execution_core.cpp`
- Modify: `Christian_control/basic_control/src/State.h`
- Modify: `Christian_control/basic_control/CMakeLists.txt`

**Interfaces:**
- Consumes: immutable `ExecutionConfig`, `CartesianTrajectoryMailbox`, measured `RobotState`, measured joint positions in continuous degrees, cycle `dt_s`, and generic adapter health facts.
- Produces:

```cpp
struct AdapterHealth {
    bool external_stop = false;
    bool live_fault = false;
    bool low_level_state_lost = false;
    bool stale_feedback = false;
    bool overrun_limit_reached = false;
};

struct ArmExecutionInput {
    RobotState state;
    JointVector measured_continuous_deg;
    double dt_s = 0.002;
    AdapterHealth adapter_health;
};

struct ArmExecutionResult {
    MeasuredCartesianState measured;
    PoseReference reference;
    ControllerStatus controller_status;
    JointVector qdot_raw_rad_s;
    JointVector qdot_limited_deg_s;
    PositionIntegration::ApplyStatus actuation_status;
    StopPriorityDecision stop;
};

class ArmExecutionCore {
public:
    ArmExecutionCore(const ExecutionConfig&, CartesianTrajectoryMailbox&,
                     PlanningArm arm);
    void Seed(const RobotState& measured);
    ArmExecutionResult Step(const ArmExecutionInput& input);
};
```

- [ ] **Step 1: Write core-order and persistence tests**

Assert `Seed` is required exactly once; `Step` order is measurement → world-freshness classification and stale-twist decay → reference → controller → non-finite handling → clamp → integration → stop decision (the core returns the integrated command and the stop decision separately so the hardware runner can keep its current send-then-resolve-stop order); previous integrated command persists; one mailbox trajectory activates atomically; an outward bounded-joint proposal holds all seven commands; adapter fault/low-level/stale facts retain current stop priority; no call allocates after seeding. No allocation-counter test pattern exists today — every current allocation-freedom claim is a comment — so this task adds a new counting `operator new/delete` test hook as new, reviewed infrastructure.

- [ ] **Step 2: Run the new core test and verify failure**

Run: `cmake --build Christian_control/basic_control/build --target test_execution_core -j2`

Expected: FAIL because `ExecutionCore` is missing.

- [ ] **Step 3: Implement the smallest composition of existing units**

Do not duplicate controller equations. `Step` calls the injected existing units and returns all intermediate evidence. Move the world-freshness classification and stale-twist decay currently inlined in `Runner.cpp:417-490` into the core — the input carries the latest coherent world sample inside `RobotState`, and the core owns deciding fresh/stale/prolonged and decaying the held twist, so the simulation never re-implements that logic. Keep Kortex frame IDs, acknowledgements, firmware fault banks, logging, sleeps, and communication out of this type. Note `PlanningArm` lives in `Christian_control/cartesian_contract/PlanningRequest.h`, not in `basic_control/src`.

- [ ] **Step 4: Replay the frozen characterization fixture**

Load `execution_preextract_v1.csv`; require exact discrete/event/stop matches and declared `1e-12`-scale tolerances for serialized floating-point fields where exact round-trip equality is not possible.

- [ ] **Step 5: Run the focused suite under ASan/UBSan**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(execution_core|execution_characterization|execution_config|controller|cartesian_reference|control_logic)$' --output-on-failure`

Run: `cmake -S Christian_control/basic_control -B /tmp/humansl-core-asan -DSANITIZER=address+undefined -DNO_VICON=ON && cmake --build /tmp/humansl-core-asan -j2 && ctest --test-dir /tmp/humansl-core-asan -R '^(execution_core|execution_characterization)$' --output-on-failure`

Review checkpoint: compare the fixture diff byte-for-byte for discrete fields and inspect every tolerated float difference.

### Task 4: Build one reusable execution-core library

**Files:**
- Modify: `Christian_control/basic_control/CMakeLists.txt`
- Modify: `Christian_control/basic_control/src/Runner.h`
- Modify: `Christian_control/basic_control/src/Runner.cpp`
- Modify: `Christian_control/basic_control/src/Main.cpp`
- Modify: `Christian_control/basic_control/tests/test_log_schema.cpp`

**Interfaces:**
- Produces CMake target `humansl_execution_core` containing `ExecutionCore`, `ExecutionConfig`, controller, reference, actuation, kinematics, dynamics, frame logic, Cartesian contracts, and hardware-independent safety policy.
- Hardware `controller` links `humansl_execution_core` plus Kortex-only runner/hardware/main/Vicon adapter sources.

- [ ] **Step 1: Add a link-structure test that initially fails**

Add a CTest script that requires both `controller` and a hardware-free probe target to link `humansl_execution_core`, and requires the core archive's undefined symbols to contain no `Kinova::Api` or Vicon SDK symbols.

- [ ] **Step 2: Split the CMake target without moving unrelated files**

Create the static library from current pure sources; give it only Eigen/Pinocchio and pthread dependencies. Link Kortex only to `controller`, `Hardware.cpp`, `Runner.cpp`, and hardware tools.

- [ ] **Step 3: Replace duplicated per-cycle logic in `Runner.cpp`**

Keep takeover T1–T6 unchanged (the labels in `Runner.cpp` run T1 readiness through T6 normal-control start). In normal control, assemble one `ArmExecutionInput`, call `core.Step`, map the result into the existing Kortex command and `LoopLogSample`, then run Kortex acknowledgement/mode/fault handling, send, sleep, report, and teardown in the current order.

- [ ] **Step 4: Build the hardware executable without running it**

Run: `cmake --build Christian_control/basic_control/build --target controller test_execution_core -j2`

Expected: build succeeds; no invocation of `controller` occurs.

- [ ] **Step 5: Run all hardware-free basic-control tests**

Run: `ctest --test-dir Christian_control/basic_control/build --output-on-failure`

Expected: all registered tests pass, including frozen replay equivalence and log schema.

Review checkpoint: trace `Kortex feedback → ArmExecutionInput → ArmExecutionCore::Step → integrated position → CyclicSession::Send` and verify takeover, command IDs, Kortex stop precedence, log semantics, and teardown were not moved into the core.

### Task 5: Document the offline-only hardware-refactor state

**Files:**
- Modify: `Christian_control/basic_control/README.md`
- Modify: `docs/architecture.md`
- Create: `Christian_control/docs/runbooks/execution-core-hardware-revalidation.md`

**Interfaces:**
- Produces an operator-visible statement that the extracted hardware path is offline-validated only and a future supervised validation procedure that is not executed by this plan.

- [ ] **Step 1: Write the runbook from current safety gates**

Name the exact build provenance checks, configuration snapshot, conservative initial hold/nearby target, workspace and e-stop confirmations, stop criteria, log fields, and post-run review required before broader motion. Do not include an authorization or claim that the run has occurred.

- [ ] **Step 2: Update architecture wording**

State that shared linkage plus replay establishes software equivalence evidence, not physical equivalence. Mark the current hardware refactor `offline-validated only` until a separately approved run.

- [ ] **Step 3: Validate documentation against symbols and tests**

Run: `rg -n 'ArmExecutionCore|humansl_execution_core|offline-validated only' Christian_control/basic_control/README.md docs/architecture.md Christian_control/docs/runbooks/execution-core-hardware-revalidation.md Christian_control/basic_control/src Christian_control/basic_control/CMakeLists.txt`

Run: `git diff --check`

Expected: names match source; no whitespace errors; no statement claims hardware validation.

Final gate: stop after offline verification and review. Do not proceed to any robot command. Plan 02 may begin only after this extraction diff is accepted.
