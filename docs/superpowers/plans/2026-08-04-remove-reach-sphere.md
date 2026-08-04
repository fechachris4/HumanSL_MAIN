# Remove Reach-Sphere Screen and Telemetry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans (recommended) to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the conservative spherical reach screen and its `pd_beyond_reach` telemetry completely, while retaining finite-input validation and all independent controller safety guards.

**Architecture:** `ParsePoseTarget` will accept any finite Cartesian position in the declared right-arm `base_link` frame; it will no longer apply a geometric workspace screen or special-case the compiled target. The runtime CSV schema will move from format 6 to format 7 by removing `pd_beyond_reach`, and analysis code will continue reading historical logs without interpreting that obsolete field. No kinematics, DLS, motion-profile, joint-limit, fault, following-error, or teardown logic will be changed.

**Tech Stack:** C++17, Eigen, Kortex C++, CMake/CTest, Python 3, NumPy, existing CSV analysis tooling.

## Global Constraints

- Do not run any executable that can connect to or command the robot.
- Retain rejection of malformed, trailing, and non-finite target input.
- Retain the current Cartesian frame (`base_link`), metre units, target queue, profile limits, joint-rate clamp, DLS damping, following-error stop, joint-boundary guard, firmware-limit checks, fault handling, freshness checks, and teardown behavior.
- Removing the sphere must not be described as adding reachability, IK, collision, or path validation.
- Bump the runtime CSV schema from format 6 to format 7 because a data column is removed.
- Preserve compatibility with historical logs containing `pd_beyond_reach`; old logs must remain loadable and analyzable without the removed report/plot event.
- Preserve unrelated uncommitted changes in the dirty worktree; do not reset, checkout, or reformat unrelated files.
- Physical behavior remains unverified until a separately authorized supervised hardware run.

---

### Task 1: Remove the reach check from target parsing

**Files:**
- Modify: `Christian_control/basic_control/src/Config.h`
- Modify: `Christian_control/basic_control/src/Targets.h`
- Modify: `Christian_control/basic_control/src/Targets.cpp`
- Modify: `Christian_control/basic_control/tests/test_reactive_law.cpp`
- Modify: `Christian_control/basic_control/tests/test_dual_arm_model.cpp`

**Interfaces:**
- `ParsePoseTarget` continues to return `std::optional<PoseTarget>` and accepts exactly three finite coordinates.
- `kRightBaseOriginControlM`, `kReachRadiusM`, and `kReachMarginM` are removed because no production behavior consumes them.

- [x] **Step 1: Write the failing parser regression test.**

  Replace the current boundary assertions with a test that expects a finite target such as `"5.0 -4.0 3.0"` to be accepted, while malformed, trailing, and `nan`/`inf` inputs remain rejected. Keep the test in the existing hardware-free reactive-law executable.

- [x] **Step 2: Run the parser test and verify the expected failure.**

  ```bash
  cmake --build Christian_control/basic_control/build --target test_reactive_law
  ctest --test-dir Christian_control/basic_control/build --output-on-failure -R reactive_law
  ```

  Expected: the new finite far-target assertion fails because the sphere still rejects it.

- [x] **Step 3: Remove the parser sphere and constants.**

  Delete the fixed-target comparison, norm check, and reach-specific error message from `Targets.cpp`. Keep the finite-coordinate check immediately before constructing `PoseTarget`. Remove the three reach constants from `Config.h` and update the parser contract comment in `Targets.h` to say only “three finite `base_link` coordinates in metres.”

- [x] **Step 4: Remove the obsolete model-origin assertion.**

  Delete the `kRightBaseOriginControlM` lookup and zero-origin assertion from `test_dual_arm_model.cpp`; keep all URDF mount, frame, joint-order, and FK/Jacobian assertions.

- [x] **Step 5: Run the focused tests.**

  ```bash
  cmake --build Christian_control/basic_control/build --target test_reactive_law test_dual_arm_model
  ctest --test-dir Christian_control/basic_control/build --output-on-failure -R 'reactive_law|dual_arm_model'
  ```

  Expected: PASS.

---

### Task 2: Remove reach telemetry from runtime state and CSV format

**Files:**
- Modify: `Christian_control/basic_control/src/Hardware.h`
- Modify: `Christian_control/basic_control/src/Hardware.cpp`
- Modify: `Christian_control/basic_control/src/Runner.cpp`
- Modify: `Christian_control/basic_control/src/Runner.h` only if its comments mention the removed field
- Modify: `Christian_control/basic_control/tests/test_runlog_compat.py`

**Interfaces:**
- `LoopSample` no longer contains `pd_beyond_reach`.
- `WriteCsvHeader` and `WriteCsvRow` emit format 7 without the `pd_beyond_reach` column.
- All other columns retain their existing order relative to one another.

- [x] **Step 1: Add a schema regression test for format 7.**

  Extend `test_runlog_compat.py` with a format-7 sample/header that has exchange timestamps and no `pd_beyond_reach` column. Assert the analyzer loads it and that the format-7 timestamp contract is recognized. Keep format-6 samples to prove historical compatibility.

- [x] **Step 2: Run the new schema test and verify the expected failure.**

  ```bash
  ctest --test-dir Christian_control/basic_control/build --output-on-failure -R runlog_compat
  ```

  Expected: format 7 is currently treated as unsupported or the old runtime schema still exposes the field.

- [x] **Step 3: Remove the field from runtime state and writers.**

  Delete `LoopSample::pd_beyond_reach`, remove the `Runner.cpp` computation using `kRightBaseOriginControlM`/reach constants, remove the CSV header token and row value, and change all schema comments from format 6 to format 7 where they describe the current writer. Do not change neighboring field order.

- [x] **Step 4: Update the log-format contract.**

  Change `Main.cpp`’s preamble from `log_format = 6` to `log_format = 7`. Update `runlog.py` so formats 2 through 7 recognize exchange timestamps, while unknown format 8 remains unsupported. Update compatibility tests for the new range and retain old-format fallback cases.

- [x] **Step 5: Run focused build and tests.**

  ```bash
  cmake --build Christian_control/basic_control/build --target controller test_runlog_compat
  ctest --test-dir Christian_control/basic_control/build --output-on-failure -R runlog_compat
  ```

  Expected: PASS; `controller` is built but never executed.

---

### Task 3: Remove analyzer reporting and plot events

**Files:**
- Modify: `Christian_control/basic_control/scripts/analyze_run.py`
- Modify: `Christian_control/basic_control/tests/test_runlog_compat.py`
- Modify: `Christian_control/basic_control/README.md`
- Modify: `Christian_control/docs/code-read/Config.md`
- Modify: `Christian_control/docs/code-read/Runner.md`
- Modify: `Christian_control/docs/code-read/Hardware.md`

**Interfaces:**
- New format-7 analysis has no reach-sphere report and no orange “reference outside input sphere” plot event.
- Historical format-6 files remain readable; their obsolete `pd_beyond_reach` column is ignored rather than used to recreate the removed policy.

- [x] **Step 1: Add analyzer compatibility assertions.**

  Add a format-7 log with an intentionally present legacy `pd_beyond_reach` column and assert that analyzer output does not report “outside conservative input sphere” and that plotting does not add the removed reach event. Also retain the existing format-6 analysis tests.

- [x] **Step 2: Run the analyzer tests and verify the expected failure.**

  ```bash
  ctest --test-dir Christian_control/basic_control/build --output-on-failure -R runlog_compat
  ```

  Expected: the current analyzer still reports or plots the legacy reach field.

- [x] **Step 3: Delete reach-specific analysis code.**

  Remove the `pd_beyond_reach` summary block and the plot-event block from `analyze_run.py`. Do not change lag estimation, tracking masks, fault events, timing analysis, or other plots.

- [x] **Step 4: Update user-facing and code-read documentation.**

  Replace README statements that targets are rejected as “out-of-reach” with the finite-input contract and an explicit warning that no reachability or collision validation is performed. Remove the reach-sphere sections from the code-read docs and update CSV column-order notes to format 7.

- [x] **Step 5: Run compatibility tests.**

  ```bash
  ctest --test-dir Christian_control/basic_control/build --output-on-failure -R runlog_compat
  ```

  Expected: PASS for both historical and new-format analysis.

---

### Task 4: Full offline verification and safety review

**Files:**
- Verify the complete diff; no additional source changes unless a scoped test/build failure identifies one.

- [x] **Step 1: Search for stale reach-sphere dependencies.**

  ```bash
  rg -n -i 'kReachRadiusM|kReachMarginM|kRightBaseOriginControlM|pd_beyond_reach|0\.852|0\.902|conservative reach|input sphere' Christian_control/basic_control Christian_control/docs
  ```

  Expected: no production or documentation references remain except explicitly preserved historical-log compatibility text, if any.

- [x] **Step 2: Build all affected hardware-free and controller targets.**

  ```bash
  cmake --build Christian_control/basic_control/build --target controller test_control_logic test_reactive_law test_trajectory_profile test_process_lock test_supervisor test_dual_arm_model
  ```

  Expected: exit code 0; do not run `controller`, `clear_faults`, `set_joint_limits`, or any other hardware executable.

- [x] **Step 3: Run the complete registered test suite.**

  ```bash
  ctest --test-dir Christian_control/basic_control/build --output-on-failure
  ```

  Expected: all tests pass.

- [x] **Step 4: Review safety boundaries after removal.**

  Confirm that finite input validation remains, profiles still use configured speed/acceleration/jerk limits, joint velocity and position guards remain active, and faults/following error/stale feedback/communication failures still stop and restore servoing. Explicitly document that endpoint and path validity are now operator/planner responsibilities.

- [x] **Step 5: Review the final diff and report hardware status.**

  Run `git diff --check`, inspect all changed files, confirm the CSV schema bump and historical compatibility, and report that no hardware command was executed. Do not claim physical reachability or collision safety from these offline checks.
