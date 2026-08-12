# Planner–Controller Rollback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the planner and chicken-head controller as genuinely separate systems (revert commit `b3ec71e1`'s three architectural additions) without losing the useful work added afterwards (panel, build hygiene, in-flight Vicon work).

**Architecture:** This is a selective rollback, not new code: most files are restored byte-for-byte to their pre-merge commit `dc759bbb` via `git checkout`, five merged-in files are deleted, `CMakeLists.txt` loses exactly three registrations while keeping its hygiene, and the panel loses its replan/posture surface. Spec: `docs/superpowers/specs/2026-08-12-planner-controller-rollback-design.md`.

**Tech Stack:** C++17 / CMake / CTest (basic_control), Python 3 unittest (panel), bash test scripts (planner_bridge). Repo root: `/home/christian/Desktop/HumanSL_MAIN` — run all commands from there.

## Global Constraints

- NEVER run `controller`, `main`, `test_kinova`, or any Kortex-linked binary. Building them is fine and required; running them is forbidden. The CTest suites here are hardware-free (verified in Task 1 before running).
- Do not touch: `Christian_control/vicon/` (all uncommitted changes and untracked tests), `Christian_control/docs/superpowers/` (Vicon plans/specs), `Christian_control/planner_bridge/tests/test_run_session.sh` (uncommitted fix). They must appear IDENTICAL in `git status` before and after this plan.
- Exactly three commits, in Tasks 2, 6 and 8. No other commits, no push. Commit messages follow the repo's `area: lower-case summary` style. NEVER add a Co-Authored-By trailer (Christian's standing preference overrides any default).
- Keep `b3ec71e1`'s build hygiene: `-Wall -Wextra`, SYSTEM includes, `add_bundled_eigen_to`/`add_pinocchio_to`, the `src/Dynamics.cpp` source switch, and the `tests/test_main_args.cpp` warning fix.
- TDD exception (stated per superpowers/kinova workflow): a rollback restores previously-tested code, so the test artifact is the restored pre-merge suite itself. The discipline is: green baseline before edits, restored pre-merge tests pass UNMODIFIED after edits. Any old test needing edits to pass = behaviour changed = STOP and report.
- If a restored file triggers a `-Wall -Wextra` warning, re-apply ONLY the corresponding warning-fix hunk from `git diff dc759bbb b3ec71e1 -- <file>`; the hunk must be behaviour-neutral (const/reference/cast/unused-parameter). Anything non-neutral: STOP and report. List every re-application in the final report.

---

### Task 1: Green baseline (no edits)

**Files:** none modified. This task only records the starting state.

**Interfaces:**
- Consumes: current working tree (uncommitted panel + Vicon work present).
- Produces: a recorded baseline (test counts + pass/fail) later tasks compare against, and `baseline_status.txt` in a scratch dir for the Task 9 selectivity check.

- [ ] **Step 1: Record the untouchable files' state**

```bash
SCRATCH=$(mktemp -d)
echo "$SCRATCH" > /tmp/rollback_scratch_path.txt
git status --short > "$SCRATCH/baseline_status.txt"
git diff > "$SCRATCH/baseline_unstaged.patch"
cat "$SCRATCH/baseline_status.txt"
```

Expected: the status matches the spec's list (panel, vicon, gitignore, test_run_session.sh, docs).

- [ ] **Step 2: Confirm the CTest suites are hardware-free**

```bash
grep -n "add_test" Christian_control/basic_control/CMakeLists.txt Christian_control/planner_bridge/CMakeLists.txt
```

Expected: every registered test target compiles only `src/*.cpp` + `tests/*.cpp` sources (no Kortex `lib/release/libKortexApiCpp.a` in any `test_*` target — verify by reading each `test_*` block). If any test links Kortex, STOP and report before running anything.

- [ ] **Step 3: Build and run the basic_control suite**

```bash
cmake --build Christian_control/basic_control/build -j"$(nproc)" 2>&1 | tail -5
ctest --test-dir Christian_control/basic_control/build --output-on-failure 2>&1 | tail -15
```

Expected: build succeeds; all tests pass. Record the total test count (it will DROP by two in Task 5: `feasibility` and `trajectory_pose` go away).

- [ ] **Step 4: Run the panel suite**

```bash
python3 -m unittest discover -s Christian_control/tools/panel/tests -t . 2>&1 | tail -5
```

Expected: OK (200+ tests). Record the count.

- [ ] **Step 5: Run the bridge suite**

```bash
ctest --test-dir Christian_control/planner_bridge/build --output-on-failure 2>&1 | tail -10
bash Christian_control/planner_bridge/tests/test_run_session.sh 2>&1 | tail -5
```

Expected: all pass. If ANY baseline suite fails, STOP — report the failure to Christian before touching anything (a rollback on a red baseline can't prove behaviour preservation).

---

### Task 2: Commit 1 — the panel work, as-is

**Files:**
- Stage: `Christian_control/tools/control_panel.py`, `Christian_control/tools/panel/` (source + tests + static, NOT `__pycache__`), `docs/superpowers/specs/2026-08-11-control-panel-design.md`, `.gitignore`
- Never stage: anything under `Christian_control/vicon/`, `Christian_control/docs/superpowers/`, `Christian_control/planner_bridge/tests/test_run_session.sh`

**Interfaces:**
- Consumes: the uncommitted panel refactor already in the working tree.
- Produces: a clean commit boundary so Task 7's panel edits are reviewable on their own.

- [ ] **Step 1: Stage exactly the panel files**

```bash
git add Christian_control/tools/control_panel.py \
        "Christian_control/tools/panel/*.py" \
        Christian_control/tools/panel/tests \
        Christian_control/tools/panel/static \
        docs/superpowers/specs/2026-08-11-control-panel-design.md \
        .gitignore
git status --short
```

Expected: staged list contains ONLY the paths above; `__pycache__` is not staged (it is gitignored; if it appears, `git restore --staged` it). Vicon files and `test_run_session.sh` remain unstaged-modified/untracked.

- [ ] **Step 2: Commit**

```bash
git commit -m "tools: the control panel refactored into a package with tests"
```

- [ ] **Step 3: Verify the untouchables survived**

```bash
git status --short
```

Expected: vicon changes, `test_run_session.sh`, and `Christian_control/docs/superpowers/` still show exactly as in `baseline_status.txt` (minus the now-committed panel entries).

---

### Task 3: Restore basic_control to its pre-merge content

**Files:**
- Modify (restore from `dc759bbb`): 16 files, exact list in Step 1.
- Delete: `src/Feasibility.h`, `src/TrajectoryPoseSource.h`, `src/TrajectoryPoseSource.cpp`, `tests/test_feasibility.cpp`, `tests/test_trajectory_pose.cpp` (all under `Christian_control/basic_control/`).
- Do NOT touch: `tests/test_main_args.cpp` (keeps its warning fix), `CMakeLists.txt` (Task 4).

**Interfaces:**
- Consumes: git history — `dc759bbb` is the parent of the merge commit `b3ec71e1`; nothing after `b3ec71e1` modified basic_control, so current content == merge content and checkout is a faithful revert.
- Produces: pre-merge sources Task 4/5 build; `Reference` struct without `posture`; `State.h` without `BaseMotionSource`; log format 10.

- [ ] **Step 1: Restore the 16 files**

```bash
cd Christian_control/basic_control
git checkout dc759bbb -- \
    scripts/runlog.py \
    src/Config.h src/Controller.cpp src/Controller.h \
    src/Hardware.cpp src/Hardware.h src/Main.cpp src/ReactiveLaw.h \
    src/Runner.cpp src/Safety.h src/State.h \
    tests/test_controller.cpp tests/test_log_schema.cpp \
    tests/test_reactive_law.cpp tests/test_runlog_compat.py \
    tools/make_synthetic_log.cpp
cd ../..
```

- [ ] **Step 2: Delete the five merged-in files**

```bash
git rm Christian_control/basic_control/src/Feasibility.h \
       Christian_control/basic_control/src/TrajectoryPoseSource.h \
       Christian_control/basic_control/src/TrajectoryPoseSource.cpp \
       Christian_control/basic_control/tests/test_feasibility.cpp \
       Christian_control/basic_control/tests/test_trajectory_pose.cpp
```

- [ ] **Step 3: Verify the restore is exact**

```bash
git diff dc759bbb -- Christian_control/basic_control/src Christian_control/basic_control/scripts Christian_control/basic_control/tests Christian_control/basic_control/tools | grep "^diff" 
```

Expected: only `tests/test_main_args.cpp` differs from `dc759bbb` (its kept warning fix: `const std::string arm` instead of `const std::string& arm` in the range-for over the initializer list — a reference into a temporary list is what `-Wall` warns about). Any other file listed here means Step 1 missed it: fix before continuing.

---

### Task 4: CMakeLists — remove three registrations, keep all hygiene

**Files:**
- Modify: `Christian_control/basic_control/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 3's deletions (the build would fail on missing sources without this task).
- Produces: a target list Task 5 builds; test names `feasibility` and `trajectory_pose` cease to exist.

- [ ] **Step 1: Remove `src/TrajectoryPoseSource.cpp` from the controller target**

In the `add_executable(controller ...)` block, delete the single line:

```cmake
        src/TrajectoryPoseSource.cpp
```

- [ ] **Step 2: Remove the test_feasibility block**

Delete this whole block (comment included):

```cmake
# --- test_feasibility: hardware-free tests for the graded feasibility
#     supervision (src/Feasibility.h — pure Eigen). No Kortex, no
#     Pinocchio — safe to run anywhere. ---
add_executable(test_feasibility
        tests/test_feasibility.cpp
)
target_include_directories(test_feasibility PRIVATE src)
add_bundled_eigen_to(test_feasibility)
add_test(NAME feasibility COMMAND test_feasibility)
```

- [ ] **Step 3: Remove the test_trajectory_pose block**

Delete this whole block (it sits just before the closing `endif ()` of the Pinocchio-gated section):

```cmake
    # The pose-primary trajectory source: FK/Jacobian of the sampled nominal,
    # cross-checked against the model directly. No robot.
    add_executable(test_trajectory_pose
            tests/test_trajectory_pose.cpp
            src/TrajectoryPoseSource.cpp
            src/Targets.cpp
            src/JointTrajectory.cpp
            src/Kinematics.cpp
            src/Dynamics.cpp
    )
    target_include_directories(test_trajectory_pose PRIVATE src)
    add_pinocchio_to(test_trajectory_pose)
    add_test(NAME trajectory_pose COMMAND test_trajectory_pose)
```

- [ ] **Step 4: Confirm nothing else references the deleted files**

```bash
grep -n "TrajectoryPoseSource\|Feasibility\|test_trajectory_pose\|test_feasibility" Christian_control/basic_control/CMakeLists.txt
```

Expected: no output.

---

### Task 5: Build, warning contingency, full suite

**Files:**
- Possibly modify (warning fixes only, per Global Constraints): restored files that warn under `-Wall -Wextra`.

**Interfaces:**
- Consumes: Tasks 3–4.
- Produces: the "pre-merge rehearsal passes untouched" evidence for acceptance criterion 2.

- [ ] **Step 1: Incremental build (regenerates CMake automatically)**

```bash
cmake --build Christian_control/basic_control/build -j"$(nproc)" 2>&1 | grep -E "warning|error|Error" | head -30
```

Expected: no errors. Warnings, if any, must trace to restored files; apply the warning-fix rule from Global Constraints (re-apply only the matching hunk from `git diff dc759bbb b3ec71e1 -- <file>`), rebuild, repeat until quiet.

- [ ] **Step 2: Clean-build verification in a scratch directory**

```bash
SCRATCH=$(cat /tmp/rollback_scratch_path.txt)
cmake -S Christian_control/basic_control -B "$SCRATCH/bc_build" -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
cmake --build "$SCRATCH/bc_build" -j"$(nproc)" 2>&1 | grep -cE "warning" || true
```

Expected: configure succeeds; warning count 0 for first-party files. (Do NOT delete or modify the existing `build/` directory.)

- [ ] **Step 3: Run the restored suite**

```bash
ctest --test-dir Christian_control/basic_control/build --output-on-failure 2>&1 | tail -15
```

Expected: all pass; total = Task 1's count minus exactly two (`feasibility`, `trajectory_pose` gone). CRITICAL: the restored pre-merge tests were NOT edited (Task 3 checked them out verbatim). If any fails, do NOT edit the test — the rollback itself is wrong; diagnose and report.

- [ ] **Step 4: Bridge suite still green (planner untouched, prove it)**

```bash
ctest --test-dir Christian_control/planner_bridge/build --output-on-failure 2>&1 | tail -5
```

Expected: same results as Task 1.

---

### Task 6: Commit 2 — the rollback

**Files:** everything staged from Tasks 3–5.

- [ ] **Step 1: Review the staged diff once, whole**

```bash
git add Christian_control/basic_control
git status --short
git diff --cached --stat
```

Expected: only `Christian_control/basic_control/` paths; 5 deletions, ~17 modifications, no new files.

- [ ] **Step 2: Commit**

```bash
git commit -m "controller: restore the pre-merge planner-controller boundary

Reverts b3ec71e1's three additions - posture guidance, the pose-primary
TrajectoryPoseSource and the feasibility advisories - restoring dc759bbb
content, while keeping that commit's build hygiene (-Wall -Wextra, SYSTEM
includes, the CMake wiring functions, the src/Dynamics.cpp switch) and
the test_main_args warning fix. Log format returns to 10. The planner is
again the only place feasibility is judged; the controller tracks joint
trajectories and holds. Chicken-head and pose mode return with the Vicon
slice on this restored boundary (see
docs/superpowers/specs/2026-08-12-planner-controller-rollback-design.md)."
```

---

### Task 7: Panel adaptation — remove the replan/posture surface

**Files:**
- Modify: `Christian_control/tools/panel/config_file.py`, `telemetry.py`, `runs.py`, `static/panel.js`
- Modify (tests): `tests/test_config_file.py`, `tests/test_telemetry.py`, `tests/test_runs.py`, `tests/test_static.py` (only where they pin removed names)
- Do NOT touch: `static/readouts.js` (its replan band is null-tolerant and disappears on its own)

**Interfaces:**
- Consumes: Commit 2's `Config.h` (the removed knobs no longer exist to read or write).
- Produces: a panel whose settings, run view and summaries mention only features that exist.

- [ ] **Step 1: config_file.py — shrink the whitelist and thresholds**

In `KNOBS`, delete these eight entries: `kTrajectoryPosePrimary`, `kPostureEnabled`, `kPostureGain`, `kReplanSigmaMin`, `kReplanJointMarginDeg`, `kReplanPostureErrorDeg`, `kReplanPositionErrorM`, `kReplanAdviseCycles`.
Reword one surviving doc string:

```python
    "kNullSpaceEnabled": ("bool", "Null-space joint-limit avoidance on/off"),
```

In `THRESHOLDS`, delete the five `kReplan*` entries and reword:

```python
    "kControlDtS": ("double", "s", "control period"),
```

- [ ] **Step 2: telemetry.py — drop the two dead fields**

In `reset()`, remove `"posture_error_deg": None,` and `"replan_advised": None,` from `self._max`.
In `add()`, remove the tuples `("posture_error_deg", row),` and `("replan_advised", row),`.
In `snapshot()`, remove the `advised = self._max["replan_advised"]` line and the three output keys `posture_error_deg`, `replan_advised`, `replan_advised_max` (and the comment above them). Update the module docstring lines that mention posture and replan thresholds.

- [ ] **Step 3: runs.py — drop the summary column and counter**

Remove `"posture_error_deg",` from the column list, the `replan_advised_rows = 0` initialisation, the `if "replan_advised" in at ...` counting block, and the `"replan_advised_rows": replan_advised_rows,` output key.

- [ ] **Step 4: panel.js — drop the dead rows and bands**

In `errorRowSpecs()`: delete the four lookups `replanM`, `marginDeg`, `postureDeg`, `sigmaMin`; in the position row change the bands line to:

```javascript
      bands: band([[arrivalM, 'arrival', 1000]]),
```

Delete the whole Posture row object (`key: 'posture'` … `bands: band([[postureDeg, 'replan advised', 1]]),`). In the remaining rows (`margin`, `sigma` if present), remove any band that referenced a deleted lookup — check with:

```bash
grep -n "replanM\|marginDeg\|postureDeg\|sigmaMin\|kReplan" Christian_control/tools/panel/static/panel.js
```

Expected after edits: no output.

- [ ] **Step 5: Adapt the tests that pin removed names**

```bash
grep -rn "kReplan\|kPosture\|kTrajectoryPosePrimary\|posture_error\|replan_advised" Christian_control/tools/panel/tests/
```

For each hit: if it asserts a removed knob's presence/value, retarget the assertion to a surviving knob of the same type (bool → `kOrientationEnabled`, double → `kKpCartesian`, int → none survive, so delete int-specific cases); if it is a CSV fixture column, remove the column and its assertion; if it asserts the removed snapshot keys, remove the assertion. Do not weaken any test that checks surviving behaviour.

- [ ] **Step 6: Run the panel suite**

```bash
python3 -m unittest discover -s Christian_control/tools/panel/tests -t . 2>&1 | tail -5
```

Expected: OK. Count may drop slightly (deleted cases); it must not drop by more than the cases deliberately removed in Step 5 — compare with Task 1's count and account for every missing test by name.

---

### Task 8: Commit 3 — panel adaptation

- [ ] **Step 1: Stage and inspect**

```bash
git add Christian_control/tools/panel
git status --short && git diff --cached --stat
```

Expected: only `tools/panel/` paths.

- [ ] **Step 2: Commit**

```bash
git commit -m "tools: the panel stops offering the reverted controller features

Config.h no longer has the posture, pose-primary or replan knobs, so the
whitelist, thresholds, run-view rows and summaries stop mentioning them.
readouts.js needed no change - its replan band is null-tolerant."
```

---

### Task 9: Acceptance pass — the four criteria from the spec

**Files:** none modified (except the spec's Status line).

- [ ] **Step 1: Criterion 1 — the boundary reads back cleanly**

Read `Christian_control/basic_control/src/Main.cpp` and the `Reference` struct in `src/State.h`. Write the one-sentence description of what the program consumes and does. It must not need the words feasibility, replanning, or nominal posture. Verify "where is feasibility judged?" has one answer:

```bash
grep -rn -i "feasib\|replan" Christian_control/basic_control/src/ | wc -l
grep -rln -i "feasib" Christian_control/planner_bridge/ --include=*.cpp --include=*.h | head -3
```

Expected: first count 0; second lists planner files only.

- [ ] **Step 2: Criterion 2 — already proven in Task 5**

Confirm from the Task 5 record: restored pre-merge tests passed with zero edits; bridge suite unchanged. Restate the caveat in the report: this proves no-regression; criterion 1 proves the split.

- [ ] **Step 3: Criterion 3 — the panel survives being used**

Start the panel:

```bash
python3 Christian_control/tools/control_panel.py &
```

HARDWARE WARNING — the refactored panel CAN start controller sessions (unlike the first committed version). During this pass, NEVER touch any control that starts, runs or stops a session; those are hardware operations requiring Christian's explicit per-run authorization. The allowed clicks are exactly: view the settings list (it must show no posture/replan/pose-primary knobs), edit one value and restore it from `Config.h.panel.bak`, trigger a rebuild (compiling is fine), and open the summary of an existing run CSV. Via the Playwright browser or Christian by hand. All four actions must work. Kill the panel process afterwards.

- [ ] **Step 4: Criterion 4 — selectivity**

```bash
SCRATCH=$(cat /tmp/rollback_scratch_path.txt)
git status --short
diff <(git status --short | grep -E "vicon|test_run_session|docs/superpowers") \
     <(grep -E "vicon|test_run_session|docs/superpowers" "$SCRATCH/baseline_status.txt")
git log --oneline -3 --name-only | head -40
```

Expected: the diff is empty (untouchables identical); the three commits touch only panel, basic_control, panel again.

- [ ] **Step 5: Adversarial review (the one review motion-path code gets)**

Dispatch a code-reviewer subagent on `git show` of the rollback commit with the question: "does any hunk change behaviour relative to dc759bbb other than the documented warning fixes, and does any kept CMake hunk reference removed code?" Fix only confirmed findings; re-run Task 5 Step 3 if anything changes.

- [ ] **Step 6: Update the spec status and report**

Change the spec's `Status:` line to `implemented 2026-08-12`. Write the final report: outcome first; the three commits; added/removed production files, classes and concepts (removed: `Feasibility`, `TrajectoryPoseSource`, `PostureObjective`, `BaseMotionSource`/`BaseMotionEstimate`/`StaticBaseMotionSource`, `Reference.posture`, log format 11; added: nothing); every warning-fix re-application; the four criteria verdicts; and what remains unverified (physical behaviour — nothing was run on hardware).
