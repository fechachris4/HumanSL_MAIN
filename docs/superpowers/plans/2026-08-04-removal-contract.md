# Removal contract — trajectory playback + fixed-target-only simplification

Agreed by cross-check: Surveyor inventory vs Reader flag lists, mismatches
resolved by message 2026-08-04. Line numbers refer to current HEAD
(af3563d6). All paths relative to repo root unless noted. Execute top to
bottom; the paired-deletion notes matter (a constant and its last reader must
go in the same change).

## 1. Delete files (git rm)

- `Christian_control/basic_control/src/Trajectory.h`
- `Christian_control/basic_control/src/Trajectory.cpp`
- `Christian_control/basic_control/tests/test_playback.cpp`
- `Christian_control/basic_control/tests/test_trajectory_file.cpp`
- `Christian_control/basic_control/trajectories/first_run_home_j4j6_4deg_10s.csv`
- `Christian_control/basic_control/trajectories/hold_home_10s.csv`
- `Christian_control/basic_control/trajectories/next_move.csv` (directory then empty — remove it)
- `Christian_control/basic_control/trajectories/` (whole directory)
- `TrajectoryGeneration/tools/plan_move.cpp`
- `TrajectoryGeneration/tools/CMakeLists.txt`
- `Christian_control/docs/decisions/trajectory-playback.md`
- `TrajectoryGeneration/tools/build/` — UNTRACKED: plain `rm -rf`, not git rm. `TrajectoryGeneration/tools/` is then empty — remove the directory.

## 2. Excise — `Christian_control/basic_control/src/Main.cpp`

- 36: `#include "Trajectory.h"`.
- 68-71: the four preamble echo lines (`reference_source`, `trajectory_file`, `playback_kp`, `start_mismatch_limit_deg`).
- 81-92 (partial): in `WriteConfigLines`, the `use_fixed_target` echo line and the `if (config::kUseFixedTarget)` guard around the fixed-target/rpy echo lines — delete the guard + `use_fixed_target` line, keep the fixed-target/rpy echoes unconditional (they describe the only remaining target source).
- 123: bump `# log_format = 5` → `# log_format = 6` (9 CSV columns removed; see §5).
- 224-237: the whole `InputThreadJoiner` struct (sole instantiation dies with the stdin thread). KEEP `#include <thread>` — line 322's `sleep_for` needs it.
- 250-255: `playback`/`operator_targets` booleans, the two-value validity throw, the `kTrajectoryFile` throw. (Package with every other `playback` reader below: 262, 350, 417, 424, 477, 550, 567.)
- 257-294: the whole pre-hardware load+validate block. KEEP helpers `PrintJointHeader`/`PrintRow` (160-175) — `PrintRobotState` (191-193) still calls them.
- 350-407: `if (playback) {...}` start-state gate + URDF FK cross-check.
- 417-423: playback banner `if` branch; collapse the `else {` at 424, un-indent 425-435; inside it delete the `kUseFixedTarget` guard at 425 around the interactive prompt — the prompt text itself is stdin-only and is DELETED with the stdin thread (the operator no longer types targets).
- 415: `PoseTargetStore pose_targets;` (store class deleted, §4).
- 476: `TrajectorySource* trajectory_source = nullptr;`.
- 477-484: the `if (playback)` TrajectorySource construction; collapse `} else {` at 485, un-indent 486-542. KEEP line 475 (`std::unique_ptr<ReferenceSource> reference;`).
- 491: freshness-gate guard on `kUseFixedTarget` becomes unconditional — THE GATE ITSELF STAYS (kMaxFixedTargetDistanceM check).
- 540-541: drop the first constructor argument — `std::make_unique<PoseTargetSource>(fixed_target)` (see §4).
- 547-570: the source-comment (547-548) and the `if (!playback && kUseFixedTarget)` / `else if (!playback)` structure: fixed-target path becomes unconditional; the else branch (stdin thread start, 567-570) deleted; `std::thread input_thread` at 549 deleted.
- 571: `InputThreadJoiner` instantiation; 582-583: the explicit join. Line 581's `g_stop = true` may stay (harmless) or go if orphaned.
- 605-615: playback outcome report.
- 621-626: delete `playback_refused` (621-622) and its term in the exit expression; final expression keeps `result.reason == LoopStop::kUserStop && !result.faults_observed`.

## 3. Excise — other basic_control sources

**`src/Config.h`**
- 30-35: reference-source doc comment, `kReferenceSource`, `kTrajectoryFile`.
- 74-77 region: `kUseFixedTarget` and its comment (the fixed target is now unconditional). KEEP the freshness-gate constants (`kMaxFixedTargetDistanceM`) and every safety constant.
- 230-236: playback comment block, `kPlaybackKp`, `kStartMismatchLimitDeg` (paired with Controller.cpp:62-66 and Main.cpp playback blocks — same change).
- 238-248: trajectory-gates comment + `kTrajectoryVelGateFactor`, `kTrajectoryAccelLimitDegS2`, `kTrajectoryPosLimitDeg` incl. closing `};` at 248.

**`src/Controller.h`** — comment edits only: line 5 (joint channel → Trajectory.h) and 12-13 (drop the Trajectory.h mentions).

**`src/Controller.cpp`**
- 10: `#include "Trajectory.h"`.
- 62-66: the joint-channel comment + `if (reference.joints) return JointTrackVelocity(..., config::kPlaybackKp);`.

**`src/Runner.cpp`**
- 70-73: `ref_deg`/`playback_t_s`/`playback_state` copies in FillSample.
- 248-257: the two playback console notices incl. the line-248 comment. KEEP the comment at 402 (TrajectoryExecution borrow attribution).

**`src/State.h`**
- 10: the `│ Trajectory.h ... │` diagram row.
- 56-65: ControllerStatus playback telemetry (comment 56-58, `q_ref_deg`, `playback_t_s`, `playback_state`, `playback_done_edge`, `playback_refused_edge`).
- 94-100: `JointReference` struct + its comment.
- 103-105: the "joint references win" prose; 109: `std::optional<JointReference> joints;`. KEEP 102 and 106 (reword 102's "at most one channel set" to describe the pose-only struct); KEEP the `Reference` struct and `pose`.
- 120: comment-only edit — drop the "trajectory start gate" mention. KEEP `class ReferenceSource` entirely.

**`src/Hardware.h`**
- 211: the `//   ref_j1..7, playback_t_s, playback_state,` column-order doc line.
- 214: `(130 columns)` → `(121 columns)`.
- 215-221: extend the format-history paragraph with the format-6 change (columns removed) — excision-forced doc edit.
- 286-292: the field comment block (286-289) and `ref_deg`/`playback_t_s`/`playback_state` fields (290-292).

**`src/Hardware.cpp`**
- 335-336: the `ref_j<i>` header loop. 337 is MIXED — trim to `csv << ",command_frame_id,feedback_frame_id";`.
- 383-384: the `ref_deg` row loop. 385-386 are MIXED — trim to `csv << "," << s.command_frame_id << "," << s.feedback_frame_id;`.
- KEEP the comment at 185 (TrajectoryExecution attribution).

## 4. Targets simplification (fixed-target-only, decision 2)

**`src/Targets.h`**
- 3: stale file-watch mention in the header comment.
- 41-62: the whole `PoseTargetStore` comment + class.
- 64-68: `RunPoseTargetInput` declaration + comment.
- 70-79: prune the `PoseTargetSource` class comment — drop 71-73 and 76 (store-sequence prose).
- 84: the `const PoseTargetStore& store` ctor parameter (85 keeps its default-arg text); 92: `store_` member; 94: `baseline_sequence_` member. Survivors: 80-83, 85, 87 (Reset override — base is pure virtual; body becomes empty), 88-89, 91, 93, 95.

**`src/Targets.cpp`**
- 29-58: `ParsePoseTarget` (only caller is the stdin thread). Its declaration in Targets.h goes too if separately declared.
- 60-80: all three `PoseTargetStore` methods.
- 82-116: `RunPoseTargetInput` — the entire stdin thread.
- 118-122: the file-watch tombstone comment (its subject no longer exists in any form).
- Ctor: delete 129 (store param) and the `store_(store),` initializer on 131 (keep `initial_target_(std::move(initial_target))`); keep 128, 130, 132.
- Reset: delete 136-138; keep the empty override shell 134-135, 139.
- Get: delete 144 (store snapshot) and 150-152 (store branch); edit 153 `else if (initial_target_)` → `if (initial_target_)`; prune comment 154-157 (store-sequence rationale now false); keep 141-143, 145, 146-149, 158-161.
- KEEP `RotationFromRpy` (19-27) — the compiled fixed target's rpy path (Main.cpp:535) uses it.

## 5. CMake — `Christian_control/basic_control/CMakeLists.txt`

- 69: remove `src/Trajectory.cpp` from the `controller` target. KEEP line 74 (`TrajectoryExecution/src/Dynamics.cpp`) and 86 (`TrajectoryExecution/include`) — and the same borrows at 131/137, 281/288.
- 78-80: comment edit — drop "Trajectory" from the src/ roster sentence.
- 231-242: the whole `test_trajectory_file` target + `add_test`.
- 244-257: the whole `test_playback` target + `add_test`.
- `enable_testing()` (203) and the cppcheck glob target (31-40) stay untouched.

## 6. Documentation edits

- `Christian_control/basic_control/README.md`: 15-16 (`kReferenceSource` selection), 20-26 (pluggable-sources prose), 37-48 (whole trajectory section), 72 (Trajectory roster row), 120 (`kReferenceSource` check reminder), 139 + 158-160 (reference-source branching prose) — rewrite each spot to describe the single fixed-target flow; deletion-first, minimal new prose.
- `Christian_control/docs/decisions/compiled-config.md:91`: remove the dangling cross-link line to trajectory-playback.md.
- `Christian_control/docs/decisions/qdot-limit-raise.md:28`: remove the now-false plan_move sentence.
- KEEP: `fault-handling-hardening.md:10` (TrajectoryExecution citation), root `README.md:68`, `scripts/plot_run.py:64` comment (harmless), everything in `docs/superpowers/`.

## 7. Disputed — kept

(none — all cross-check items resolved)

## Safety reminders for the executor

- Never touch: guards, joint limits, freshness gate (`kMaxFixedTargetDistanceM`), `kAllowUnverifiedActuators`, `kStopOnFault`, `kQdotLimitDegS`, `kModelVelocityLimitsDegS`, `kFollowingErrorLimitDeg`, `kCommandLeadLimitDeg`, overrun/non-finite stop constants.
- Build the controller binary; NEVER run it.
- Allowed non-deletion edits are exactly the ones listed: line trims (Hardware.cpp 337/385-386), `else if`→`if` (Targets.cpp:153), collapse/un-indent of emptied conditionals, comment prunes, the log_format 5→6 bump + column-count/format-history text, README rewording, and removing now-unused includes/variables the compiler names.
