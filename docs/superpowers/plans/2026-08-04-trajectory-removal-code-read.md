# Trajectory Removal + Line-by-Line Controller Read — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove every trace of the trajectory-playback pipeline from `Christian_control/basic_control`, leave a fixed-target-only controller whose `main()` reads as configuration → fixed target pose → controller calculation → safety checks → robot command → logging → safe shutdown, and produce a per-file line-by-line plain-English read of the surviving code in `Christian_control/docs/code-read/`.

**Architecture:** Coordinated subagents with the main session as message hub. Phase 1 runs a Trajectory Surveyor and three Readers in parallel; their outputs are cross-checked by message into a removal contract. Phase 2 is a deletion-only Remover proven by build + tests. Phase 3 is a read-only Reviewer plus doc refresh. Phases run back-to-back without user checkpoints.

**Tech Stack:** C++17, CMake, ctest; Agent tool + SendMessage for coordination.

**Spec:** `docs/superpowers/specs/2026-08-04-trajectory-removal-code-read-design.md`

## Global Constraints

- NO binary is ever run against the arm; verification is compile + hardware-free tests only (`controller` is built, never executed).
- The Remover DELETES; it never redesigns. Excision-forced edits only (un-indent a former `if (playback)` body, drop an unused include).
- `Config.h` safety settings are untouchable: guards, joint limits, freshness gate, `kMaxFixedTargetDistanceM`, `kAllowUnverifiedActuators`. Only reference-source plumbing (`kReferenceSource`, `kTrajectory*` constants, stdin/file-watch config) may go.
- `TrajectoryExecution/` and `TrajectoryRealTime/` top-level projects are untouched (basic_control's borrow of `TrajectoryExecution/src/Dynamics.cpp` stays).
- Cross-check rule: a line is removed only if BOTH the Surveyor inventory and a Reader flag agree it is trajectory-only (or it is stdin/file-watch target plumbing per decision 2). Unresolved after one message round-trip → stays, reported to Christian.
- Commits: docs commits separate from the removal commit(s) so removal reverts independently. NO Co-Authored-By trailer.
- Any Reader flag of category `hides-work`, `mixed-jobs`, or `edit-hazard` on surviving code goes into the final report verbatim — never auto-fixed.

## File Structure

**Created (Phase 1, updated Phase 3):** `Christian_control/docs/code-read/<Name>.md` for each surviving source file: `Main.md`, `Config.md`, `Runner.md`, `Controller.md`, `Safety.md`, `ReactiveLaw.md`, `Hardware.md`, `Actuation.md`, `Kinematics.md`, `Targets.md`, `State.md`.
**Created (Phase 1):** `docs/superpowers/plans/2026-08-04-removal-contract.md` — the merged, agreed removal list.
**Deleted (Phase 2):** `basic_control/src/Trajectory.cpp`, `src/Trajectory.h`, `tests/test_playback.cpp`, `tests/test_trajectory_file.cpp`, `trajectories/` (3 CSVs), `TrajectoryGeneration/tools/` (plan_move.cpp, CMakeLists.txt, untracked build/); stdin + file-watch code paths in `src/Targets.cpp/.h`; playback branches in `src/Main.cpp`, `src/Runner.cpp`; trajectory targets/sources in `basic_control/CMakeLists.txt`.

---

### Task 1: Phase 1 — launch Surveyor + three Readers in parallel

**Files:**
- Create: `Christian_control/docs/code-read/*.md` (by the Readers)
- Read-only otherwise.

**Interfaces:**
- Produces: Surveyor returns a structured inventory (markdown list: file / lines / symbol / how-wired / depends-on). Each Reader returns a flag list: `file:line-range | category | one-line reason`, categories exactly `trajectory-only | unnecessary | hides-work | mixed-jobs | edit-hazard`. Task 2 consumes both.

- [ ] **Step 1: Launch all four agents in one message (parallel), run_in_background**

Surveyor prompt (agent type `Explore`):

```
Read-only survey of /home/christian/Desktop/HumanSL_MAIN. Inventory EVERY trajectory-playback touchpoint that exists to support trajectory playback in Christian_control/basic_control, plus the plan_move CLI in TrajectoryGeneration/tools. Cover: src/Trajectory.cpp/.h entirely; every line/branch in src/Main.cpp, src/Runner.cpp, src/Config.h, src/State.h, src/Hardware.* mentioning Trajectory/playback/kTrajectory*/kReferenceSource; CMakeLists.txt targets and source lists (controller target's Trajectory.cpp, test_playback, test_trajectory_file); tests/test_playback.cpp and tests/test_trajectory_file.cpp; trajectories/*.csv; TrajectoryGeneration/tools (plan_move.cpp, CMakeLists.txt, build/). Do NOT include TrajectoryExecution/ or TrajectoryRealTime/ themselves — but DO note where basic_control borrows from them so the borrow is preserved. For each item return: path, line range, symbol, how it is wired in (who calls/includes/links it), and whether anything non-trajectory depends on it. Also grep docs/ and Christian_control/ docs for references to these files. Return the inventory as a markdown list — it becomes a removal contract, so err toward listing a doubtful line with a note rather than omitting it.
```

Reader shared preamble (agent type `general-purpose`; each gets this plus its own file list):

```
You are one of three Readers producing a line-by-line plain-English read of Christian_control/basic_control for Christian, an MSc robotics student learning C++. For each assigned source file write /home/christian/Desktop/HumanSL_MAIN/Christian_control/docs/code-read/<Name>.md (create the directory if needed; <Name> = source file stem, e.g. Main.md covers Main.cpp). Follow REAL execution order from the file's entry points into each function it actually calls — not top-to-bottom declaration order. For every line or tight line-group: what it does in plain English, what it reads or changes, why it exists, and what might break if it changes. Explain non-obvious C++ simply (e.g. what std::optional or an atomic does) the first time it appears. Flag lines with exactly these categories: trajectory-only (exists solely for trajectory playback), unnecessary, hides-work (does more than it looks like), mixed-jobs (combines unrelated responsibilities), edit-hazard (useful but confusing enough that a future edit could cause unexpected behaviour). Mark flags inline in the doc AND return, as your final message, a structured flag list: one line per flag, format `file:startline-endline | category | one-line reason`. This code commands a physical Kinova Gen3 arm — do NOT build or run anything; read only, plus writing your docs.
```

- Reader 1 files: `src/Main.cpp` (entry point `main()`), `src/Config.h`.
- Reader 2 files: `src/Runner.cpp` + `Runner.h`, `src/Controller.cpp` + `Controller.h`, `src/Safety.cpp` + `Safety.h`, `src/ReactiveLaw.h` (entry: the functions Main.cpp calls, per its includes).
- Reader 3 files: `src/Hardware.cpp` + `Hardware.h`, `src/Actuation.cpp` + `Actuation.h`, `src/Kinematics.cpp` + `Kinematics.h`, `src/Targets.cpp` + `Targets.h`, `src/State.h`.

- [ ] **Step 2: Wait for all four completions (task notifications); collect Surveyor inventory + three flag lists**

Expected: 11 files exist under `Christian_control/docs/code-read/`; each Reader returned a non-empty flag list; Surveyor inventory covers at minimum: Trajectory.cpp/.h, both tests, 3 CSVs, plan_move, CMake references, Main.cpp playback block (~lines 250–300+), Runner.cpp playback notices.

### Task 2: Phase 1 — cross-check by message, write removal contract, commit docs

**Files:**
- Create: `docs/superpowers/plans/2026-08-04-removal-contract.md`

**Interfaces:**
- Consumes: Task 1's inventory + flag lists.
- Produces: the contract file — sections `Delete files`, `Excise line-ranges` (per file, with symbol names), `CMake edits`, `Targets simplification` (stdin/file-watch removal per decision 2), `Disputed — kept` (anything unresolved). Task 3 executes it verbatim.

- [ ] **Step 1: Diff inventory vs `trajectory-only` flags**

Mechanically compare: every inventory line-range should be matched by a Reader `trajectory-only` flag and vice versa. Build two mismatch lists: inventory-only (Surveyor saw it, Reader didn't flag it) and flag-only (Reader flagged it, Surveyor missed it).

- [ ] **Step 2: Resolve mismatches via SendMessage (one round-trip max)**

For each inventory-only item, SendMessage to the responsible Reader: "Surveyor lists <file:lines> (<symbol>) as trajectory-only because <how-wired>. You did not flag it. Confirm trajectory-only, or explain what else depends on it." For each flag-only item, SendMessage to the Surveyor agent: "Reader flagged <file:lines> as trajectory-only: <reason>. Your inventory missed it — confirm nothing non-trajectory depends on it." Items still disputed after replies go to `Disputed — kept`.

- [ ] **Step 3: Write the contract file with the agreed lists**

Include the stdin/file-watch removal items from Targets.* and Config.h under `Targets simplification` (these need no Surveyor agreement — they are decision 2, not trajectory code — but must carry the Reader's line numbers).

- [ ] **Step 4: Commit docs + contract**

```bash
git add Christian_control/docs/code-read/ docs/superpowers/plans/2026-08-04-removal-contract.md
git commit -m "docs: line-by-line code read of basic_control + trajectory removal contract"
```

Expected: commit contains 12 new files, no source changes.

### Task 3: Phase 2 — Remover executes the contract, proves it, commits

**Files:**
- Delete/Modify: exactly what `2026-08-04-removal-contract.md` lists.

**Interfaces:**
- Consumes: the contract file.
- Produces: a removal commit; build + ctest evidence pasted in its final message.

- [ ] **Step 1: Launch Remover agent (foreground, agent type `general-purpose`)**

Prompt:

```
Execute /home/christian/Desktop/HumanSL_MAIN/docs/superpowers/plans/2026-08-04-removal-contract.md exactly. Read it first, then: git rm the listed files (plus `git clean` nothing — the untracked TrajectoryGeneration/tools/build/ is removed with plain rm -rf); excise the listed line-ranges (deletion only — the sole allowed rewrites are un-indenting a conditional body whose condition was deleted, removing now-unused includes/variables the compiler names, and collapsing an if/else whose else branch was deleted); apply the listed CMake edits; apply the Targets simplification. NEVER touch items under 'Disputed — kept'. NEVER weaken Config.h safety settings (guards, limits, freshness gate, kMaxFixedTargetDistanceM, kAllowUnverifiedActuators). Then prove it: fresh configure + build in a NEW build dir (cmake -S Christian_control/basic_control -B Christian_control/basic_control/build-clean && cmake --build Christian_control/basic_control/build-clean -j) and ctest --test-dir Christian_control/basic_control/build-clean --output-on-failure. Build the controller binary but NEVER run it — it commands a physical robot arm. If the build fails, fix the excision (missing include, orphaned variable), never by reintroducing trajectory code. When green: git add -A the affected paths and commit as 'remove trajectory playback: fixed-target-only controller' (no Co-Authored-By trailer). Delete build-clean after ctest passes. Return: list of deleted files, per-file excised ranges, ctest summary line, commit hash.
```

- [ ] **Step 2: Verify Remover's evidence**

Run: `git -C /home/christian/Desktop/HumanSL_MAIN show --stat HEAD` and `grep -ri "trajector" Christian_control/basic_control/src Christian_control/basic_control/CMakeLists.txt || echo CLEAN`
Expected: stat shows only contract paths; grep prints `CLEAN` (comments referencing TrajectoryExecution's Dynamics borrow are allowed — if grep hits only those, that is CLEAN; anything else fails).

### Task 4: Phase 3 — Reviewer verifies the seven-step flow

**Files:** read-only.

**Interfaces:**
- Consumes: post-removal HEAD.
- Produces: PASS/FAIL verdict per verification item + any dangling references found. Task 5 consumes the verdict.

- [ ] **Step 1: Launch Reviewer agent (foreground, agent type `Explore`)**

Prompt:

```
Read /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/src/Main.cpp end-to-end at current HEAD, following execution from main(). Verdict on each, PASS/FAIL with evidence (line numbers): (1) the run path is literally: configuration → fixed target pose → controller calculation → safety checks → robot command → logging → safe shutdown, with no other reference source, no stdin/file-watch threads, no playback branch; (2) `grep -rn -i "trajector\|playback\|kReferenceSource" Christian_control/basic_control/src Christian_control/basic_control/tests Christian_control/basic_control/CMakeLists.txt` finds nothing except comments about the TrajectoryExecution Dynamics borrow; (3) CMake lists no deleted file and no test_playback/test_trajectory_file targets; (4) Config.h still contains kMaxFixedTargetDistanceM, the freshness gate, and the guard overrides unchanged (git diff HEAD~1 -- the Config.h safety block is empty outside reference-source plumbing). Read-only: do not build, run, or edit anything.
```

- [ ] **Step 2: If any FAIL → send the failure to a follow-up Remover run (SendMessage to the Remover agent with the Reviewer's evidence), then re-run Reviewer. Loop max twice; still failing → stop and report to Christian.**

### Task 5: Phase 3 — refresh code-read docs to final code, commit

**Files:**
- Modify: `Christian_control/docs/code-read/*.md`
- Delete: none (Trajectory.md was never created — Readers only covered surviving files).

- [ ] **Step 1: SendMessage to each Reader with the removal commit hash**

Message: "Removal commit <hash> is in. Update your docs/code-read files to match HEAD: delete sections for removed lines, fix all line references, drop trajectory-only flags (now moot), keep hides-work/mixed-jobs/edit-hazard flags that survive. Re-verify each doc's line numbers against the current file."

- [ ] **Step 2: Spot-check line references**

Run: for 3 random claims per doc, `sed -n '<line>p' <source>` and compare with the doc's description.
Expected: descriptions match actual lines.

- [ ] **Step 3: Commit**

```bash
git add Christian_control/docs/code-read/
git commit -m "docs: code read updated to post-removal controller"
```

### Task 6: Final report to Christian

- [ ] **Step 1: Write the final message: outcome first, complete sentences. Must contain: what was removed (files + line counts), the commit hashes (docs, removal, doc refresh), build/ctest evidence, the seven-step flow verdict, the full surviving-flag list (hides-work / mixed-jobs / edit-hazard) as decisions for Christian, and anything in `Disputed — kept`.**
