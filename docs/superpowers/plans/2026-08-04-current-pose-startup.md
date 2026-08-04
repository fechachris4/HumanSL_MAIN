# Current-Pose Startup for Basic Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow `Christian_control/basic_control` to take over from the arm's current measured pose, hold that pose during takeover, and then move to the configured Cartesian target using the existing bounded reference profile and safety guards.

**Architecture:** Separate the startup pose from the motion target. `Main.cpp` will retain readiness, hard-limit, and fresh-feedback checks but will remove the exact registered startup-pose gate and its compiled joint-branch dependency. `Runner.cpp` will continue to seed and hold the measured joint command through takeover; `PoseTargetSource` will be initialized with a profile whose start is the measured end-effector position and whose finish is `kFixedTargetM`, preserving the takeover orientation through the existing position-only target contract.

**Tech Stack:** C++17, Eigen, Pinocchio-backed `DualArmKinematics`, bundled Kortex 2.7.0 C++ API, CMake/CTest, existing hardware-free controller tests.

## Global Constraints

- Do not run any executable that can connect to or command the robot during implementation or verification.
- Preserve existing live-fault, feedback-freshness, following-error, command-rate, joint-boundary, reach, non-finite, overrun, acknowledgement, and teardown guards.
- Keep the current 500 Hz cyclic timing, 0.5 s takeover hold, position-integrator seeding, right-arm `base_link` frame, metres for Cartesian values, degrees for Kortex joint values, and Kortex actuator joint order.
- The startup path must command the measured joint position first; it must never send the configured Cartesian target as the first low-level position command.
- Remove the obsolete exact registered-pose gate, `ExperimentGate.h`, expected startup joint/quaternion configuration, and their telemetry/documentation. Do not replace them with a disabled or optional mode.
- Preserve unrelated uncommitted changes in the working tree; do not reset, checkout, or reformat unrelated files.
- Physical reachability, collision clearance, calibration, and hardware motion remain pending supervised validation even if offline tests pass.

---

### Task 1: Remove the experiment-specific startup contract

**Files:**
- Modify: `Christian_control/basic_control/src/Config.h`
- Modify: `Christian_control/basic_control/src/Main.cpp` (configuration echo only)
- Delete: `Christian_control/basic_control/src/ExperimentGate.h`
- Modify: `Christian_control/basic_control/README.md`
- Modify: `Christian_control/docs/code-read/Config.md`
- Modify: `Christian_control/docs/code-read/Main.md`

**Interfaces:**
- Removes `kExpectedStartupJointDeg`, `kExpectedStartupQuaternionXyzw`, `kStartupOrientationToleranceRad`, and `kStartupJointToleranceDeg`; none are needed for current-pose startup.
- Keeps `kFixedTargetM` as the terminal position target only; its comments and CSV key must no longer describe it as the required startup pose.

- [x] **Step 1: Add the failing configuration/telemetry test or characterization assertion.**

  Extend the hardware-free target/configuration coverage so the terminal target remains finite and no registered-startup fields are emitted. If the existing test executable has no suitable preamble seam, add the narrowest pure assertion to the existing log-compatibility test; do not test hardware connectivity.

- [x] **Step 2: Run the targeted test to verify the new contract is absent.**

  Run from `Christian_control/basic_control/build`:

  ```bash
  cmake --build . --target test_reactive_law test_control_logic
  ctest --output-on-failure -R 'reactive_law|control_logic'
  ```

  Expected: the new assertion fails until the obsolete startup contract is removed.

- [x] **Step 3: Remove the obsolete configuration and update documentation.**

  Delete the expected startup joint/quaternion constants and their tolerances, delete the exact-gate include/use, and remove the stale CSV preamble fields. Update README and code-read documents to describe current-pose startup with no alternate strict mode.

- [x] **Step 4: Run the targeted tests.**

  ```bash
  cmake --build . --target test_reactive_law test_control_logic
  ctest --output-on-failure -R 'reactive_law|control_logic'
  ```

  Expected: PASS, with no hardware process started.

- [x] **Step 5: Review the diff for scope.**

  Confirm only the obsolete startup contract and its documentation changed; do not alter safety thresholds, gains, profile limits, or target coordinates in this task.

---

### Task 2: Remove the mandatory registered-pose gate

**Files:**
- Modify: `Christian_control/basic_control/src/Main.cpp:318-445`
- Delete: `Christian_control/basic_control/src/ExperimentGate.h` if not already removed in Task 1
- Modify: `Christian_control/basic_control/tests/test_supervisor.cpp` only to remove obsolete gate coverage

**Interfaces:**
- Consumes: the fresh `initial` Kortex feedback snapshot and `DualArmKinematics::RightPoseAndJacobian`.
- Produces: startup continues after robot readiness and hard-limit checks; the current FK pose is passed to target-profile setup without comparing it to a compiled pose.

- [x] **Step 1: Write failing tests for the removed gate.**

  Remove or replace tests around the deleted startup-gate evaluator with a pure test that accepts two distinct finite startup poses as profile starts:

  ```cpp
  // A finite startup pose different from the old registered pose is accepted
  // as the profile start; no exact joint/pose comparison is performed.
  ```

  Continue covering non-finite measured feedback through the existing readiness/safety tests, but remove expectations tied only to the deleted registered-pose evaluator.

- [x] **Step 2: Run the tests and confirm the exact-gate removal is not implemented.**

  ```bash
  cmake --build . --target test_supervisor
  ctest --output-on-failure -R supervisor
  ```

  Expected: the new current-pose assertion fails before `Main.cpp` uses the measured FK pose as the profile start.

- [x] **Step 3: Delete the registered-pose block.**

  In `Main.cpp`, leave `RobotReadyForTakeover(initial, ...)`, `VerifyKinematicHardLimits(...)`, fault clearing, fresh feedback acquisition, and `EnsureJointLimits(...)` mandatory. Delete the `ExperimentStartupExpectation` construction, `EvaluateExperimentStartupGate(...)` call, rejection message, and registered-pose diagnostics. Compute the current FK pose once and pass its position to the target source.

- [x] **Step 4: Run all hardware-free supervisor tests.**

  ```bash
  cmake --build . --target test_supervisor
  ctest --output-on-failure -R supervisor
  ```

  Expected: PASS; no Kortex client is instantiated by the test process.

- [x] **Step 5: Review safety behavior.**

  Verify that a live fault, invalid feedback, failed hard-limit check, or failed configured firmware-limit verification still returns before `RunControlLoop`, and that no exact startup joint/pose comparison remains.

---

### Task 3: Initialize the target profile from the measured takeover pose

**Files:**
- Modify: `Christian_control/basic_control/src/Targets.h`
- Modify: `Christian_control/basic_control/src/Targets.cpp`
- Modify: `Christian_control/basic_control/src/Main.cpp:446-467`
- Test: `Christian_control/basic_control/tests/test_reactive_law.cpp` or a new focused target-source test registered in `Christian_control/basic_control/CMakeLists.txt`

**Interfaces:**
- Produces a `PoseTargetSource` constructor or factory that accepts an explicit `start_position_m` and terminal `PoseTarget`, with the existing `CartesianMotionLimits` and hold duration.
- The source returns the start position with zero twist until the first profile cycle is allowed, then returns a seventh-order rest-to-rest profile from `start_position_m` to `target.p_desired`.
- The source remains position-only: `rotation` is `nullopt`, so `TrackingController::Reset`’s takeover orientation is retained.

- [x] **Step 1: Add failing target-source tests.**

  Cover these cases:

  ```cpp
  // current start != terminal target: first reference is current start,
  // first moving sample has nonzero progress only after activation, and the
  // terminal sample equals the configured target with zero velocity.
  // current start == terminal target: no invalid profile and immediate arrival.
  // non-finite start/target: construction or profile creation is rejected.
  // a queued operator target starts from the previous terminal target, not
  // from the original startup pose.
  ```

  Assert metres, zero start/end velocity, monotonic progress along the straight Cartesian segment, and preserved `rotation == nullopt`.

- [x] **Step 2: Run the target-source tests and confirm they fail.**

  ```bash
  cmake --build . --target test_reactive_law
  ctest --output-on-failure -R reactive_law
  ```

  Expected: the new constructor/start-profile assertions fail because the source currently starts from the fixed target and does not own a startup profile.

- [x] **Step 3: Add the explicit startup-profile state.**

  Keep the existing terminal-target queue semantics. Add an initial profile built from the measured FK position to the configured fixed target, with a distinct phase/sequence policy that prevents stdin targets from bypassing it. The first post-takeover `Get` must return the profile's start sample (current measured position, zero velocity); subsequent samples follow the profile, then the existing arrival/dwell behavior applies. Ensure `OnArrived()` advances from the initial target exactly once and the first queued target still begins at the fixed target.

- [x] **Step 4: Pass the measured FK position into the source.**

  In `Main.cpp`, reuse the same fresh `initial` feedback and kinematics calculation that reports the current pose. Construct the target from `kFixedTargetM`, then construct the source with `ee_now.position` as the start. Remove `kMaxFixedTargetDistanceM` if it has no remaining independent safety purpose; it must not be used to reject a normal startup based on distance to the terminal target.

- [x] **Step 5: Run the target-source and controller tests.**

  ```bash
  cmake --build . --target test_reactive_law test_control_logic test_trajectory_profile
  ctest --output-on-failure -R 'reactive_law|control_logic|trajectory_profile'
  ```

  Expected: PASS, including the existing queue, arrival-edge, profile-limit, and parser coverage.

- [x] **Step 6: Review the first-command invariant.**

  Read `Runner.cpp` and confirm T3/T4/T5 still seed `commanded_deg` from fresh measured feedback and stream that hold before normal `PoseTargetSource::Get` output can command motion. The initial profile must not be sampled as the first takeover command.

---

### Task 4: Update telemetry, operator messaging, and documentation

**Files:**
- Modify: `Christian_control/basic_control/src/Main.cpp`
- Modify: `Christian_control/basic_control/README.md`
- Modify: `Christian_control/docs/code-read/Runner.md`
- Modify: `Christian_control/docs/code-read/Targets.md`
- Modify: `Christian_control/docs/code-read/Safety.md`
- Modify: `Christian_control/docs/decisions/whole-path-validation.md` to state that its registered starting pose is historical evidence, not a runtime prerequisite
- Test: `Christian_control/basic_control/tests/test_runlog_compat.py` if the CSV preamble changes

**Interfaces:**
- Telemetry distinguishes measured startup pose, configured terminal target, and active reference; it contains no strict-gate policy.
- Operator output states that takeover holds the current pose and then moves to the configured target; it no longer says the arm must already be at the target in normal mode.

- [x] **Step 1: Add/adjust log compatibility expectations.**

  Remove the obsolete `expected_startup_joint_deg`, `expected_startup_quaternion_xyzw`, and startup-tolerance preamble fields, while preserving compatibility for existing logs that contain them.

- [x] **Step 2: Update runtime banner and CSV preamble.**

  Print the measured FK startup position, measured joint vector, terminal target, and profile limits. Keep all values self-describing with units and frame names. Ensure the CSV records the actual starting pose independently from the terminal target.

- [x] **Step 3: Update the operator documentation.**

  Document the exact sequence: safety gates → low-level takeover hold at measured pose → bounded profile to target → terminal hold. State that no high-level prepositioning or special startup pose is required.

- [x] **Step 4: Run compatibility tests.**

  ```bash
  ctest --output-on-failure -R runlog_compat
  ```

  Expected: PASS with old logs still readable and new preambles correctly identified.

---

### Task 5: Build and perform offline verification

**Files:**
- Verify only; no source changes unless a test or build failure identifies a scoped issue.

- [x] **Step 1: Configure/build the affected target from the existing build setup.**

  First inspect the current build directory and CMake cache; do not delete it. Then run:

  ```bash
  cmake --build Christian_control/basic_control/build --target controller test_control_logic test_reactive_law test_trajectory_profile
  ```

  Expected: all requested targets compile; `controller` is built but never executed.

- [x] **Step 2: Run the full registered hardware-free suite.**

  ```bash
  ctest --test-dir Christian_control/basic_control/build --output-on-failure
  ```

  Inspect the CMake `add_test` definitions first and confirm no test opens a robot connection.

- [x] **Step 3: Review the final diff and safety paths.**

  Check startup, normal loop, target arrival, queued-target transition, operator stop, fault, following-error, stale feedback, exchange failure, exception, and teardown paths. Confirm units, frames, joint ordering, orientation preservation, and profile timing are explicit.

- [x] **Step 4: State hardware validation separately.**

  Before any supervised hardware run, confirm workspace clearance, e-stop availability, operator supervision, target coordinates, expected initial direction, and recovery/stop procedure. The first run should use a small displacement from the measured pose and verify the CSV’s measured-versus-commanded trace before attempting the full target.

---

## Self-review checklist

- The target is no longer used as the mandatory startup pose in normal mode.
- The measured startup pose is held before the reference profile can move the arm.
- The profile starts from the measured FK position and ends at `kFixedTargetM`.
- The takeover orientation remains the measured orientation because targets stay position-only.
- Historical offline experiment data is documented as evidence, not as a runtime gate.
- Existing safety guards and teardown behavior remain mandatory.
- Tests cover distinct start/target, zero-distance start/target, invalid values, profile limits, queue transitions, and CSV compatibility.
- No hardware command is part of implementation verification.
