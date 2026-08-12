# Selective rollback: restore the planner–controller split

Date: 2026-08-12
Status: implemented 2026-08-12 (commits 20a02b98, d04b2035, f099fa19);
criteria 1, 2, 4 verified; criterion 3 verified at API level, Christian's
browser click-through pending

## Goal

In Christian's words: **restore the planner and chicken-head controller as
genuinely separate systems, without losing the useful work added
afterwards.**

Unpacked:

- **Genuinely separate systems** — the planner (`planner_bridge` +
  `trajectory_generation`) is the only place feasibility is judged and the
  only producer of trajectories; the controller (`basic_control`) only
  tracks the references it is given or holds. Neither contains the other's
  reasoning.
- **Chicken-head controller** names the controller's role — holding the
  end-effector pose while the base moves. Per the pure-revert decision,
  the world-anchored implementation of that role is deliberately deferred
  to the Vicon slice; this rollback restores the separate system the role
  will be built on, it does not deliver the world-frame hold itself.
- **The useful work added afterwards** that must survive: the control
  panel (committed and uncommitted), the build hygiene from the merge
  commit (-Wall -Wextra, SYSTEM includes, collapsed Pinocchio/Eigen
  wiring), the in-flight Vicon work, and the bridge-test fix. What is
  removed — posture guidance, pose-primary following, feasibility
  advisories, the base-motion seam — is removed by Christian's explicit
  decision, not lost by accident.

The goal is achieved when all four acceptance criteria below hold, and not
achieved if any one of them fails.

## Problem

Commit `b3ec71e1` ("controller: pose-primary trajectory following, posture
guidance, feasibility advisories", 2026-08-11) moved three planner-flavoured
responsibilities into `Christian_control/basic_control`:

1. **Posture guidance** — the planner's nominal joint state enters the
   controller's null space as a secondary objective (`PostureObjective`).
2. **`TrajectoryPoseSource`** — a pose-primary mode in which the Cartesian
   law tracks FK(q_nom(t)), selected by `kTrajectoryPosePrimary`. Its
   slice-3 world-anchoring code (fed by the new `BaseMotionSource` seam)
   is also where chicken-head stabilisation was implemented.
3. **`Feasibility.h`** — runtime feasibility margins (smallest singular
   value, joint-limit margin, posture deviation, tracking error) and a
   debounced `replan_advised` advisory flag.

Christian wants the previous split restored: the planner alone judges and
produces feasible trajectories; the controller only tracks references or
holds. All three additions are reverted. The pose-mode / chicken-head
capability will be rebuilt later as its own approved slice (likely with the
Vicon integration); it is deliberately absent in the meantime.

Everything reverted was off or inert by default, so the command path on
hardware is bit-identical before and after this change.

## Decisions (made by Christian, 2026-08-12)

- **Scope:** revert all three additions of `b3ec71e1`, including the
  `BaseMotionSource` seam.
- **Feasibility:** delete entirely — no relocation to the planner, no
  telemetry-only demotion.
- **UI:** commit the uncommitted panel refactor first, then adapt the panel
  to the restored `Config.h` in a follow-up commit.
- **Pose mode:** pure revert now; rebuild later as an approved slice. The
  Cartesian law and its tests remain in the tree, unwired, per the standing
  engineering constraints.

## Commit sequence

### Commit 1 — panel refactor (existing uncommitted work, as-is)

Included: `Christian_control/tools/control_panel.py`,
`Christian_control/tools/panel/` (excluding `__pycache__`),
`docs/superpowers/specs/2026-08-11-control-panel-design.md`, and the
`.playwright-mcp/` line in `.gitignore`.

Excluded (separate in-flight work, left uncommitted and untouched): all
`Christian_control/vicon/` changes and tests, the Vicon plans/specs under
`Christian_control/docs/superpowers/`, and the
`planner_bridge/tests/test_run_session.sh` fix.

### Commit 2 — the rollback (all inside `basic_control`)

Deleted files:

- `src/Feasibility.h`
- `src/TrajectoryPoseSource.h`, `src/TrajectoryPoseSource.cpp`
- `tests/test_feasibility.cpp`
- `tests/test_trajectory_pose.cpp`

Restored to their pre-merge (`dc759bbb`) content:

- `src/Config.h` — removes `kPostureGain`, `kPostureEnabled`,
  `kTrajectoryPosePrimary`, the four `kReplan*` thresholds
  (`SigmaMin`, `JointMarginDeg`, `PostureErrorDeg`, `PositionErrorM`)
  and `kReplanAdviseCycles`.
- `src/ReactiveLaw.h` — null space back to the single limit-avoidance
  objective.
- `src/Controller.h`, `src/Controller.cpp` — remove posture/advisory wiring.
- `src/Runner.cpp`, `src/Main.cpp` — remove pose-source selection wiring.
- `src/Safety.h` — revert the commit's additions.
- `src/State.h` — remove `Reference.posture`, `BaseMotionSource`,
  `BaseMotionEstimate`, `StaticBaseMotionSource`, and the feasibility
  status fields.
- `src/Hardware.h`, `src/Hardware.cpp`, `scripts/runlog.py`,
  `tests/test_runlog_compat.py`, `tools/make_synthetic_log.cpp` — the
  writer's schema returns to its pre-merge form (format 9; the merge had
  taken it to 11). CORRECTION during implementation: eight real hardware
  CSVs from 2026-08-11 carry `log_format = 11` (the spec first claimed
  none existed — a faulty grep), so `runlog.py` keeps accepting format 11
  as a retired readable format; otherwise those logs would silently be
  analysed with the wrong clock. Decided by Christian at review.
- `tests/test_controller.cpp`, `tests/test_log_schema.cpp`,
  `tests/test_reactive_law.cpp` — back to the pre-merge cases.

Preserved deliberately (NOT reverted):

- `CMakeLists.txt` build hygiene from the same commit: `-Wall -Wextra` on
  first-party code, vendor headers as SYSTEM includes, the
  `add_bundled_eigen_to`/`add_pinocchio_to` wiring functions, and the
  switch to the local `src/Dynamics.cpp`. Only the three registrations of
  deleted code come out (`TrajectoryPoseSource.cpp` in the controller
  target, the `test_feasibility` block, the `test_trajectory_pose` block).
- `tests/test_main_args.cpp` keeps its current content: the merge commit's
  only change there is a one-line warning fix that belongs with the kept
  `-Wall -Wextra`.
- If restoring a file to `dc759bbb` content resurfaces a compiler warning
  that `b3ec71e1` had fixed, the warning fix (and only the warning fix) is
  re-applied from that commit's diff — it is part of the kept hygiene, and
  each such re-application is listed in the final report.
- Decided by Christian at the adversarial review: two further `b3ec71e1`
  hunks are kept — the `runlog.py`/`test_runlog_compat.py` format-11
  read-acceptance (see the correction above), and the `Runner.cpp` hunk
  moving the arrival prints out of the 500 Hz compute→Send window (the
  pure revert had put a terminal print back inside the control cycle).
  Deferred as future work, faithful to pre-merge behaviour: the
  `std::runtime_error`→"communication" exit-reason mislabelling and the
  stale synthetic-log format labels — both predate the merge.
- Every limiter, where it already lives: per-joint velocity clamp,
  software joint limits, command-lead limiting, and null-space
  limit-avoidance in the controller; the jerk penalty
  (`JerkPenaltyFactor`) and trajectory optimisation limits in the planner.
- The Cartesian PD law and its tests (unwired but present).
- `planner_bridge/` and `trajectory_generation/` — untouched.

### Commit 3 — panel adaptation

The panel must not display features that no longer exist, so the whole
replan/posture surface comes out, not just the three knobs first noticed:

- `tools/panel/config_file.py` — remove eight `KNOBS` entries
  (`kTrajectoryPosePrimary`, `kPostureEnabled`, `kPostureGain`, the four
  `kReplan*` thresholds, `kReplanAdviseCycles`) and the five `kReplan*`
  `THRESHOLDS` entries; reword the `kNullSpaceEnabled` doc (no longer
  "+ posture") and the `kControlDtS` consequence (no longer about the
  advisory count).
- `tools/panel/telemetry.py` — remove `posture_error_deg` and
  `replan_advised` from the window maxima and snapshot output.
- `tools/panel/runs.py` — remove `posture_error_deg` from the summary
  columns and the `replan_advised_rows` counter.
- `tools/panel/static/panel.js` — remove the `kReplan*` threshold lookups,
  the "replan advised" bands, and the Posture error row.
- `tools/panel/static/readouts.js` — unchanged: its replan-margin band is
  already null-tolerant and simply stops appearing.
- Panel tests (`test_config_file.py`, `test_telemetry.py`, `test_runs.py`,
  `test_static.py` as affected) — assertions that pin removed names move
  to surviving knobs (e.g. `kOrientationEnabled` for the bool write test);
  fixtures drop the removed columns.

## Acceptance criteria — what Christian wanted vs what was achieved

Each criterion maps to one of Christian's stated wants and is judged the
way a human engineer would judge it: by reading, using, or looking — not
by mechanical checks alone. All four must hold.

### 1. The split is back: the boundary reads back cleanly

Wanted: planner and controller with separate jobs.
Achieved when: reading the controller's entry point and its `Reference`
contract, its job can honestly be described in one sentence with no
planning vocabulary — "it receives a timed joint trajectory and produces
limited, integrated position commands, tracking or holding". If the
description needs the words feasibility, replanning, or nominal posture,
the criterion fails. The question "where would I change how feasibility
is judged?" must have exactly one answer: the planner. This is a reading
test; architecture is accepted by review because no grep measures
responsibility.

### 2. Behaviour is preserved: the pre-merge rehearsal passes untouched

Wanted: a rollback of structure, not of behaviour.
Achieved when: the hardware-free end-to-end path that existed before the
merge — plan through the bridge, validate, run the controller's offline
tests — passes with the same results, and every pre-merge test passes
WITHOUT being edited. Having to modify an old test to make it pass means
behaviour changed and the criterion fails.
Honest caveat for the report: because everything removed was switched off,
this proves "no regression"; only criterion 1 proves the split itself.

### 3. The UI is intact: the panel survives being used

Wanted: recent UI work not removed or broken.
Achieved when: the panel starts and a five-minute manual pass succeeds —
the settings list appears (minus the three dead knobs), a value can be
edited and restored from its backup, a rebuild can be triggered, and the
summary of an existing run CSV opens. Actually clicking through it is the
test; green unit tests alone do not satisfy this criterion.

### 4. The rollback is selective: everything else is untouched

Wanted: a selective rollback, not a full one.
Achieved when: one look at `git status` and the commit list shows the
Vicon work and the bridge-test fix exactly as they were, and the commits
touch only the panel, `basic_control`, and the panel knob catalogue.

## Implementation checklist (working tripwires, not acceptance)

Mechanical checks used while implementing; useful, but passing them does
not by itself mean the goal is achieved:

- Green baseline of all three suites (basic_control CTest, panel Python
  tests, bridge tests) BEFORE any edit; same suites after each commit.
- Clean-build configure + compile, warnings visible, zero first-party
  warnings.
- Grep for removed symbols over `basic_control/src` and `tools/panel`
  returns nothing.
- `git diff dc759bbb` over restored files shows only the deliberately
  preserved build-hygiene differences.
- `test_log_schema` asserts the format-10 column set.
- Safety-review checklist plus one adversarial review of the rollback
  diff, since it touches motion-path files.
- No robot-facing binary is executed. Building is verification; running
  is not.

## What is lost until future slices (stated honestly)

Pose-primary trajectory following, null-space posture guidance, runtime
feasibility margins and the `replan_advised` advisory, and the
`BaseMotionSource` seam. Chicken-head stabilisation is aspirational until
the Vicon slice rebuilds it on the restored boundary. Because every removed
feature was disabled by default, no currently-exercised behaviour changes.
