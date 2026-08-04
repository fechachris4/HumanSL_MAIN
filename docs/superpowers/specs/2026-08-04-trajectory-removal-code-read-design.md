# Trajectory removal + line-by-line controller read — design

**Date:** 2026-08-04
**Scope:** `Christian_control/basic_control` (and the `plan_move` CLI in `TrajectoryGeneration/tools`)

## Context

`basic_control` grew a trajectory-playback pipeline (CSV loader, 4-stage
validation, playback branches in `Main.cpp`/`Runner.cpp`, two tests, a
`plan_move` planner CLI) that was never run on hardware. Christian's goal is a
program a human can read and modify, with this exact flow:

configuration → fixed target pose → controller calculation → safety checks →
robot command → logging → safe shutdown

The trajectory code is dead weight against that flow and makes the run path
confusing. Alongside removal, Christian wants a durable line-by-line
plain-English read of the active controller code, flagging lines that are
unnecessary, trajectory-only, hiding important work, mixing jobs, or
edit-hazardous.

## Decisions (confirmed with Christian)

1. **Removal scope — everything in basic_control:** delete
   `src/Trajectory.cpp/.h`, the playback branches in `Main.cpp` and
   `Runner.cpp`, `tests/test_playback.cpp`, `tests/test_trajectory_file.cpp`,
   the `trajectories/*.csv` files, and the `plan_move` CLI (including its
   untracked `TrajectoryGeneration/tools/build/`). `TrajectoryExecution` (the
   separate top-level project whose `Dynamics.cpp` basic_control borrows) is
   untouched.
2. **Target source — fixed target only:** after removal the compiled target in
   `Config.h` is the only reference source. The stdin and file-watch input
   threads in `Targets.cpp` are deleted too; `kReferenceSource` and its
   validation disappear. Changing the target = edit `Config.h`, rebuild.
3. **Code-read output — one markdown doc per source file** under
   `Christian_control/docs/code-read/`, each walking its file in execution
   order: per line (or tight line-group), what it does, what it reads/changes,
   why it exists, what breaks if it changes, plus flags.

## Execution: coordinated agents (Approach A)

The session (me) is the message hub; agents exchange findings via relayed
messages, not shared files.

**Phase 1 — parallel survey + read**
- *Trajectory Surveyor*: exhaustive inventory of every trajectory touchpoint —
  source, headers, CMake targets/links, tests, CSVs, `plan_move`, docs
  references. Output: a structured removal list with, per item, how it is
  wired in and what depends on it.
- *Reader 1*: `Main.cpp` + `Config.h`. *Reader 2*: `Runner.cpp` +
  `Controller.*` + `Safety.*` + `ReactiveLaw.h`. *Reader 3*: `Hardware.*` +
  `Actuation.*` + `Kinematics.*` + `Targets.*` + `State.h`. Each follows real
  execution from its entry points, writes its `docs/code-read/` files, and
  returns a structured flag list (`trajectory-only`, `unnecessary`,
  `hides-work`, `mixed-jobs`, `edit-hazard`).
- **Cross-check by message:** Surveyor's inventory goes to each Reader; each
  Reader's `trajectory-only` flags go back against the inventory. Any line one
  side calls trajectory-only and the other doesn't is resolved by a follow-up
  message before Phase 2. The merged, agreed list is the removal contract.

**Phase 2 — removal**
- *Remover* agent executes the contract: delete files, excise branches, unwire
  CMake (`Trajectory.cpp` from the controller target, both test targets,
  `plan_move`), delete stdin/file-watch target paths, simplify `main()` to the
  seven-step flow. Deletion only — no rewrites beyond what excision forces
  (e.g. un-indenting a former `if (playback)` body).
- Proof: clean configure + build, all remaining tests pass.

**Phase 3 — review + doc refresh**
- *Reviewer* agent reads the post-removal `main()` path end-to-end and checks
  it is literally the seven-step flow with nothing extra; verifies no dangling
  references (grep for trajectory symbols); confirms build/tests.
- Readers' docs are updated to describe the final code (trajectory sections
  dropped, line references corrected).

## Safety invariant

- No binary is run against the arm at any point; verification is
  compile + hardware-free tests only.
- The Remover deletes; it does not redesign. Any Reader flag that is
  "confusing but load-bearing" (edit-hazard, hides-work, mixed-jobs on live
  code) is *reported* in the final summary for Christian's decision, never
  auto-fixed.
- `Config.h` safety settings (guards, limits, freshness gate,
  `kMaxFixedTargetDistanceM`) are not weakened or removed; only the
  reference-source plumbing goes.

## Error handling

- If excision breaks the build, the Remover fixes the excision (missing
  include, orphaned variable), never by reintroducing trajectory code.
- If a Reader and the Surveyor still disagree about a line after one
  round-trip, the line stays (removal is conservative) and the disagreement
  is surfaced to Christian.
- Git is the undo path: removal lands as its own commit(s), separate from the
  docs commits, so it can be reverted independently.

## Verification

1. `cmake --build` of `basic_control` succeeds from a clean configure.
2. `ctest` — remaining tests (control_logic, reactive_law, supervisor,
   dual_arm_model) pass; the two trajectory tests are gone from the list.
3. `grep -ri "trajector" Christian_control/basic_control/src` returns nothing;
   CMake has no reference to removed files.
4. Reviewer's read of `main()` matches the seven-step flow one-to-one.
5. `docs/code-read/` exists with one file per remaining source file, and its
   line references match the final code.
