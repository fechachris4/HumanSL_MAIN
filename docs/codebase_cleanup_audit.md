# Codebase cleanup audit

Audit-only pass, 2026-08-07, at HEAD `f23719ad`. No production code was
modified; this file is the only artifact. Method: one lead pass over the
build files, README and entry points; three read-only subagents (B — build
and launch path, C — configuration, prints and dead code, S — safety gates
and termination); every subagent claim that carries a recommendation was
re-verified by the lead against the working tree before it was written down
here. Both hardware-free test suites were run as the evidence baseline:
**basic_control 11/11 passed (2.6 s), planner_bridge 16/16 passed (21 s)** —
all 27 tests green before any cleanup begins.

Two findings dominate everything else in this document, so they come first.

**The repository root build cannot configure at all.** Root
`CMakeLists.txt:118` calls `add_subdirectory(TrajectoryRealTime)`, but that
directory has no `CMakeLists.txt` — commit `0161f39b` ("Remove
TrajectoryRealTime module") deleted it without updating the root file.
Independently, `TrajectoryExecution/CMakeLists.txt:9-10` lists
`../Christian_control/basic_control/src/Connect.cpp`/`.h` as sources; those
files were folded into `Hardware.cpp/.h` by commit `b25b5bc8` and no longer
exist. Both facts verified directly on the tree. A fresh `cmake -S . -B
build` fails before generating anything, which means the root `main` and
`test_vicon` targets — the upstream Vicon demo the README presents as the
project's headline — have been unbuildable for weeks. The existing `build/`,
`cmake-build-*` directories are stale caches, not evidence to the contrary.
The active layer is unaffected: `basic_control` and `planner_bridge` are
standalone CMake projects and are the two builds that actually work.

**The controller's pose channel is unreachable in the shipped binary.**
`Main.cpp:466-468` wires exactly one reference source,
`JointTrajectorySource`. Every return path of its `Get`
(`Targets.cpp:279-303`) sets `reference.joint` — including the idle case,
which returns a joint-space hold at `hold_q_rad_` (line 283), never an empty
reference. `TrackingController::DesiredVelocity` (`Controller.cpp:69-81`)
therefore takes the `if (reference.joint)` early return on every cycle, and
the pose branch below it — the reactive law, the null-space joint-limit
avoidance (`kLimitAvoidGain`/`kLimitAvoidZoneDeg`), and the
arrival/non-arrival monitors — never executes. The lead confirmed this
independently rather than taking the subagent's word: `reference.pose` is
written nowhere in production code, and the idle path was read line by line.
The consequence for safety architecture: during joint-trajectory execution —
the only motion path today — there is **no graded push-back** as a bounded
joint (2/4/6) approaches its software limit. The protection is the hard
hold-and-stop in `PositionIntegration::Apply` plus the firmware JOINT_LIMIT
bank. The soft mechanism this project's own instructions rank first exists,
is tuned, and is disconnected. Reconnecting it (or explicitly deciding not
to) is a controller-behaviour decision that belongs to Christian, not to a
cleanup pass — it is recorded here as the highest-value *investigate*, not
as a cleanup item.

---

## 1. Current system and runtime path

The repository is two systems sharing one tree.

The **inherited upstream system** lives at the root: `main.cpp` (the
Vicon-driven dual-arm tube-handoff state machine the README describes),
`test_vicon.cpp`, plus the `TrajectoryGeneration`, `TrajectoryExecution`,
`TrajectoryRealTime` and `ViconDataStream` directories and the root
`config/` files. It is currently unbuildable (see above). The active layer
reaches into it for source files only: `TrajectoryExecution/src/Dynamics.cpp`
(the Pinocchio model wrapper) is compiled directly into the controller and
into `bridge_core`, and planner_bridge compiles six `TrajectoryGeneration`
source files directly. Neither active project links the upstream *library
targets* — they bypass them by naming the `.cpp` files.

The **active system** is `Christian_control/`:

- `basic_control/` — the 500 Hz hardware controller. Runtime path, traced
  function by function: `Main.cpp:560 main()` installs SIGINT/SIGTERM
  handlers into one shared `std::atomic<bool>`; `ParseMainArgs` reads
  `--arm`/`--log`; per arm, `ProcessLock` (advisory lock per robot IP) and
  the Pinocchio `Dynamics` model load happen before any hardware contact;
  `RunOneArm` (Main.cpp:317) then opens the two Kortex sessions
  (`Connect`, TCP 10000 + UDP 10001), clears faults, runs the three
  pre-motion gates (`RobotReadyForTakeover`, `VerifyKinematicHardLimits`,
  `EnsureJointLimits`), opens the run CSV and its writer thread, spawns the
  target-pipe reader thread, and calls `RunControlLoop` (Runner.cpp:85).
  Inside: takeover T2-T5 (servoing-mode guard, seed read, 25-cycle hold at
  the measured pose, controller/integrator seeding), then the T6 loop:
  `reference.Get()` → `DesiredVelocity()` → `ClampJointVelocity()` →
  `PositionIntegration::Apply()` → `CyclicSession::Send()`
  (Hardware.cpp:266 → `base_cyclic_->Refresh(command_, 0)` at line 280 —
  the actual robot command, once per 2 ms cycle) → log push → stop
  arbitration → `sleep_until` the next grid point.
- `planner_bridge/` — the offline GPMP2 planner. `main.cpp` is a 7-line
  trampoline into `RunBridge` (BridgeMain.cpp:498): parse args, convert the
  goal into the `mount` frame, read the start state from the newest run CSV,
  solve (point goal or Cartesian path), validate, and only on success emit a
  `TRAJ_BEGIN … TRAJ_END` block on stdout — which the session script pipes
  into the controller's per-arm FIFO. The controller side validates every
  block again (`ValidateJointTrajectory`) before it can become motion.

Control frequency is 500 Hz from the single source `config::kControlDtS =
0.002` (Config.h:147); the planner's dense validation deliberately re-states
0.002 s in `planner.yaml` with a comment acknowledging the duplication.

## 2. Entry points, executables and shell scripts

Buildable targets today: `basic_control` produces `controller` (the only
binary that commands the arm), 11 diagnostic/offline tools (`clear_faults`,
`probe_direction`, `set_joint_limits`, `read_safety_limits`,
`print_joint_positions`, `print_end_effector_pose`,
`compare_end_effector_pose`, `print_tool_configuration`,
`make_synthetic_log`, `expand_run_poses`, `print_dual_arm_fk`) and 10-11
test binaries; `planner_bridge` produces `planner_bridge`,
`generate_dh_params` (build-time DH generation from the URDF),
`probe_path_reachability`, and its test set. Unbuildable (root chain):
`main`, `test_vicon`, and the `TrajectoryGeneration` / `TrajectoryExecution`
/ `ViconDataStream` library targets. Not built by anything, anywhere:
root `test_kinova.cpp` and `test_task_impedance.cpp` — no CMakeLists in the
tree references them, and `test_kinova.cpp` reads a `dh_params.yaml` that
does not exist in the repo, so it is stale as well as orphaned.
`TrajectoryRealTime/joint_mpc/` is a deliberately standalone project
referenced by nothing else.

**`planner_bridge/scripts/run_session.sh`** (read line by line by the lead;
the full annotation is in the audit working notes and summarised here) is
*not* a wrapper CLion could replace. What it actually does, block by block:
resolves repo-relative binary paths; refuses to run a `controller` or
`planner_bridge` binary older than its newest source (`fresh_or_die`);
refuses a generated `dh_params_*.yaml` older than the URDF — both are
real staleness gates that catch a forgotten rebuild before it reaches
hardware; prints the supervised-session checklist and requires a typed `GO`
(the human-authorization gate the project's hardware rules require);
creates a per-session artifact directory and copies `goal.yaml` /
`planner.yaml` into it; writes `session.json` (git revision, dirty flag,
binary and URDF SHA-256s, run-log paths) on *every* exit path via an EXIT
trap; starts the controller with a **plain `>` redirect instead of process
substitution** — a measured, documented workaround: with `> >(tee log)`,
`$!` is the tee subshell, and the trap's `kill -INT` never reaches a moving
arm; waits for each arm's run log to exist *and contain data rows* before
letting the bridge read it as a start state; solves to a file first and
copies into the FIFO only if the bridge exited 0, so no byte reaches the
controller from a failed plan. Skipping the script loses the staleness
gates, the authorization gate, and the session evidence bundle — the C++
safety kernel itself (signal handler, `ServoingGuard`) does not live in the
script, so a direct CLion run still stops safely, but a report-quality
experiment record does not happen by itself.

**`tests/test_run_session.sh`** rehearses the script against stub binaries
(a Python stub, deliberately, because a backgrounded bash stub in a
non-interactive shell inherits SIGINT ignored and cannot exercise the real
signal path). It pins exactly the three load-bearing properties: SIGINT
reaches the real controller PID; pipe bytes are identical to the saved plan
file; a failing bridge sends nothing. It is wired into **no** ctest/CI —
verified by grep — and must be run by hand today.

The Python scripts in `basic_control/scripts/` are operator analysis tools
(`analyze_run.py`, `plot_run.py`, `plot_move.py`, `plot_joint.py`,
`runlog.py`, `measure_delay.py`); `gen_reactive_fixtures.py` is
build-adjacent (regenerates the cross-validation fixtures `test_reactive_law`
consumes). The MATLAB `show_frames.m` carries a stale usage comment
describing the retired `"WORLD x y z"` pipe grammar (the current wire format
is TRAJ blocks only); the frame math itself is fine.

## 3. Terminal-output inventory

Counts per file (grep, active layer): Main.cpp 32, Safety.cpp 21,
Runner.cpp 19, BridgeMain.cpp 10, Targets.cpp 5, Hardware.cpp 3,
FramePrint.h 2, PathValidation.cpp 1, planner main.cpp 1; tools/ carry 95
across 10 files (fine — they are interactive diagnostics).

Classification: the Main/Safety prints are lifecycle status (connect, gate
PASS/FAIL, arm summary) and fatal/stop reports (`PrintStopReport`, one
decoded line per stop reason). Targets/JointTrajectory prints are actionable
rejections (bad wire block, with the offending line). No temporary-debug
prints, no duplicates, and no stale prints were found in the sampled set.

**The 500 Hz loop body is clean**: every print reachable inside it is
edge-triggered (arrival, fault-bank change — capped, trajectory
activate/reject/complete) or rate-limited by `kStatusPrintPeriodS` (1 s
status line). No unconditional per-cycle print exists. One latent defect
(S2): the arrival/non-arrival prints at Runner.cpp:388-404 sit **before**
`cyclic.Send()` in the cycle, contradicting the file's own "no print
between compute and Send" rule that the other three print sites obey. It is
harmless today only because those edges live in the dead pose channel; it
becomes real the day the pose channel is reconnected.

## 4. Configuration and command-line argument inventory

`Config.h` (~45 constants) was inventoried symbol by symbol with consumer
greps. The result, compressed: nearly everything is live and correctly
single-sourced. The exceptions:

- `kAcceptOrientationTargets` (Config.h:370) — **zero consumers anywhere**.
  Its check site was deleted with the pose-target grammar in commit
  `7dd50d0d`; the constant and its "Stage 1.6 gate" comment describe a
  rejection path that no longer exists.
- `kLoopLogPrefix` (Config.h:377) — zero consumers; superseded by the
  per-arm `ArmConfig::log_prefix`.
- `kFixedTargetUseRpy` / `kFixedTargetRpyRad` — consumers only in the CSV
  preamble writer behind a provably-false branch (`static_assert`ed false).
  The comment says exactly this; accurate, deliberate schema preservation.
- `kReferenceFrame` has zero basic_control consumers but eight in
  planner_bridge — genuinely live, just cross-project.
- The `kLimitAvoid*`, `kArrival*`, `kOrientationEnabled`-family constants
  are live *in code* but sit in the unreachable pose channel (finding S1) —
  configured, tested, and currently without effect on the robot.

`planner.yaml` is the best config surface in the repo: `LoadPlannerConfig`
requires exact keys on every table, so nothing can be silently ignored, and
`EffectiveConfigText` echoes every field into the run record. No unused
fields exist there — structurally guaranteed. `goal.yaml` is strict-parsed
per arm block; its `session_arms:` key is read only by `run_session.sh` via
awk, documented as such. `dual_arm_mounting.yaml` is consumed only by
`test_dual_arm_mounting` as the URDF cross-check — that is its declared job.
The root `config/` files (`joint_limits.yaml`, `parameters.yaml`,
`task_parameters.conf`, `GEN3_With_GRIPPER_DYNAMICS.urdf`) are read only by
the unbuildable/orphaned root sources; none is read by the active layer,
which uses `TrajectoryGeneration/config/joint_limits.yaml` (a byte-identical
but independent copy — duplication finding below) and its own URDF.

Command-line surfaces: controller `--arm right|left|both` (required),
`--log PATH` (rejected with `--arm both`); planner_bridge `--arm`, `--goal`,
`--goal-file`, `--state-csv`, `--start-deg` (test-only), `--dh`,
`--joint-limits`, `--planner-config`, `--runs-root`, `--box`, `--help`, with
a documented exit-code contract (0/1/2/3/4); tools share `--ip` with the
right-arm default. All parsed strictly; unknown flags are refused by name.

Duplication found: **(a)** `joint_limits.yaml` exists as two byte-identical,
independently-maintained copies (root and TrajectoryGeneration) with no sync
mechanism; **(b)** joint velocity limits exist as three unreconciled
numbers — controller clip 45 °/s uniform (marked TEMPORARY), planner pacing
~79.6/69.9 °/s from joint_limits.yaml, and 1.3963/1.2218 rad/s hardcoded in
`test_dual_arm_model.cpp` — so the planner can pace a trajectory the
controller will only accept because validation re-checks against the
stricter clip late, after a full solve (the planner.yaml comment concedes
this); **(c)** robot IPs are correctly single-sourced in Config.h but
re-declared in the orphaned root sources, where `test_kinova.cpp:27` labels
`.9` as the *right* arm — wrong under the active convention, harmless only
because the file is never built; **(d)** `GEN3_DUAL_URDF_PATH` is computed
independently by both active CMakeLists (acknowledged in a comment);
**(e)** 500 Hz appears twice by design, with the duplication documented at
the second site.

## 5. Safety-gate and termination inventory

The full gate table (subagent S, spot-verified) — invariant, threshold,
response, class:

| Gate | Invariant | Threshold (source) | Response | Class |
|---|---|---|---|---|
| `RobotReadyForTakeover` (Safety.cpp:81) | no live fault before takeover | any live fault bit | refuse run, decoded bank text | pre-motion rejection |
| `VerifyKinematicHardLimits` (Hardware.cpp:177) | compiled clip ≤ robot's hard speed limit | `kQdotLimitDegS` vs reported | refuse run | pre-motion rejection |
| `EnsureJointLimits` (Hardware.cpp:90) | firmware j2/4/6 bands match config (don't survive power cycle) | `kJointLimitWarnDeg`/`ErrorDeg` | correct, re-read, refuse on mismatch | pre-motion rejection |
| `ProcessLock` | one process per arm IP | flock | refuse, name the IP | pre-motion rejection |
| FK finiteness, CSV/dir creation, FIFO type (Main.cpp) | run evidence and model sanity before motion | — | refuse run | pre-motion rejection |
| Wire grammar + `ValidateJointTrajectory` (JointTrajectory.cpp) | every point in pos/vel limits, monotone time, finite | `kJointSoftwareLimitDeg`, `kQdotLimitDegS` | reject the block, keep running previous reference | pre-motion rejection (per block) |
| Activation splice guard (Targets.cpp) | trajectory starts where the arm is | 2.0° (`kTrajStartToleranceDeg`) | reject block, print worst joint | pre-motion rejection (per block) |
| Takeover hold (Runner.cpp:192-291) | arm quiet for 25 cycles before control | exact cycle count, static_assert | throw, no controller motion ever computed | pre-motion rejection |
| Cartesian following error (Safety.cpp:33) | \|cmd−meas\| per joint | 3° (`kFollowingErrorLimitDeg`) | stop loop | hard stop |
| Joint-tracking following error (Controller.h:54) | wrapped ref-vs-meas; NaN fails toward stopping | 8° (`kTrajFollowingErrorStopDeg`) | stop loop | hard stop |
| Left low-level servoing | mode never silently changes | state ≠ LOW_LEVEL | stop loop | hard stop |
| Live fault (`kStopOnFault`) | no live actuator/base fault | any bit | stop loop (record-only if override set) | hard stop |
| Software joint boundary (Actuation.cpp:102-129) | j2/4/6 never commanded further outward | Table-39 − 2°, capped by firmware warn | **hold prior safe frame, then stop** — the unsafe increment is never transmitted | hard stop (with command filtering first) |
| Stale cyclic ack (Freshness.h) | per-actuator command_id advances | 25 cycles = 50 ms | stop loop | hard stop |
| Non-finite output (Runner.cpp:379) | finite before integration | 3 consecutive (`kNonFiniteStopCycles`); each held-at-zero meanwhile | soft → hard | soft anomaly escalating |
| Cycle overrun (Runner.cpp:340) | 500 Hz cadence | raw dt > 3 ms, 10 consecutive | soft → hard | soft anomaly escalating |
| Kortex/comm + catch-all exceptions (Runner.cpp:594-627) | nothing escapes without restore | — | stop, D1/D2 restore still runs | hard stop |

Stop arbitration (`ResolveStopPriority`): following-error >
left-low-level > fault(∧policy) > joint-limit-warning > stale-feedback; the
consecutive-cycle counters are checked after the switch, so effectively
lowest. On every stop the Runner **breaks and stops sending** — because this
is a position integrator, the last setpoint stops advancing and the arm's
own low-level servo brings it to rest; no supervisory ramp exists. The one
partially graceful path is the software-boundary gate, which filters the
unsafe command before stopping. The graded-response mechanisms the project
prefers exist in two places: the (dead) null-space avoidance, and
planner_bridge's validation reports.

Timing verified: `sleep_until` grid with reset-not-burst on stall;
`ClampedCycleDt` caps integration at 2× nominal while **overrun detection
uses the raw unclamped dt** — two different dt values for two purposes,
correct individually, undocumented as a pair (S8). Teardown D1 (explicit
`Restore`) runs after every catch path, D2 (destructor) retries only if D1
failed; every destructor on the shutdown path is exception-safe (verified:
`noexcept` Restore, try/catch in `~LoopLogWriter`, `~Channel`). No reachable
C++ exit path skips the log drain; SIGKILL is the only gap and is inherent
to the OS. For `--arm both`, one stop flag is shared — either arm's failure
stops both, matching the stated preference.

planner_bridge's rejections are mostly exemplary: `ValidatePlannedPath`
reports every measured quantity (clearance, error percentiles, headroom)
before its verdict; the time-scaling retry slows a dynamically-infeasible
trajectory instead of refusing it; the IK-seed quality report explicitly
does *not* refuse a plan for poor initialisation. Bare refusals that remain
justified: missing start state, strict config parsing, box-outside-grid
(shrinking a safety volume silently would be the dangerous direction). One
improvement candidate: the path-IK failure names the failing sample but not
how far off it was (S13).

## 6. Dead, duplicated or obsolete code candidates

Evidence-backed, with callers shown in the findings table:

- **Dead by unreachability (large)**: the pose channel of
  `TrackingController::DesiredVelocity` and everything only it uses —
  reactive law invocation, null-space limit avoidance, arrival monitors
  (finding S1). *Not* a deletion candidate: this is hardware-lesson code the
  project may want to reconnect; it is a **decision** candidate.
- **Dead constants**: `kAcceptOrientationTargets`, `kLoopLogPrefix`
  (C-DEAD-1/2); unused public accessor
  `DualArmKinematics::right_base_frame_id()` (C-DEAD-3; the private member
  is live, only the accessor is uncalled).
- **Orphaned files**: root `test_kinova.cpp`, `test_task_impedance.cpp` —
  in no build, and the former reads a config file that does not exist.
- **Dangling build references**: root `add_subdirectory(TrajectoryRealTime)`
  and `TrajectoryExecution`'s `Connect.cpp/.h` source entries (B1/B2).
- **Unbuildable upstream targets**: `main`, `test_vicon`,
  `ViconDataStream` — their fate is a scope decision (is the upstream Vicon
  demo still wanted?), not a mechanical cleanup.
- **Duplicated data**: the two `joint_limits.yaml` copies; the three-way
  velocity-limit inconsistency; the twice-computed URDF path.
- **Checked and ruled out** (recorded so nobody re-litigates them):
  `FramePrint.h` is live (Main.cpp:215, print_dual_arm_fk);
  `PointBaseToMount`/`ToolPoseInMount`/`MountFromBase` are live across both
  projects; the `Targets.h`/`JointTrajectory.h` exports are the shared wire
  contract, live on both sides.

## 7. Comments and documentation

The sampled comment blocks are overwhelmingly current and load-bearing —
the dense Config.h/Hardware.h/Runner.h commentary explains genuinely
non-obvious behaviour and should stay. Specific actions:

- **Move**: the CSV log-format changelog (Hardware.h:236-244, formats 4→9)
  is pure history; excerpt to `docs/decisions/` and leave a one-line
  "current format is 9; see WriteCsvRow" pointer (C-DOC-1).
- **Fix (stale)**: `show_frames.m`'s usage comment documents the retired
  pose-line pipe grammar (B11). `kAcceptOrientationTargets`' comment
  describes a rejection path that no longer exists (goes away with the
  constant).
- **Update (now-misleading)**: `Controller.h`'s and `State.h`'s header
  diagrams present the pose channel as a live peer of the joint channel.
  After S1, a new engineer reading them would misjudge what the binary
  actually does. One added sentence — "the pose channel is currently
  unreachable; JointTrajectorySource is the only wired source" — fixes the
  misdirection without deleting the (accurate) mechanism description.
  (Lead finding L2.)
- **Keep as-is**: the "TEMPORARY 45" velocity note (accurate, points at its
  decision record); the `kFixedTarget*` legacy-schema comment (accurate);
  `Hardware.cpp:218`'s explains-an-absence note (cross-link rather than
  move).

No comment was found that *incorrectly* makes an old implementation appear
mandatory — every historical claim checked against git survived.

## 8. Accidental complexity and architecture problems

- **The broken root build is the largest piece of accidental complexity**:
  four upstream directories, two orphaned test files, four config files and
  a README all describe a system that cannot currently be built, while the
  two real projects live one directory down. A new engineer's first hour is
  spent discovering this. Whatever is decided about the Vicon demo, the tree
  should stop pretending the root build works.
- **basic_control CMake boilerplate**: the include-dirs / RPATH /
  link-options triple is pasted ~9 times across Pinocchio-linking targets;
  the root file's own `add_project_executable()` shows the factoring pattern
  was known. A 10-line function removes ~100 lines and one class of
  copy-paste error (B7).
- **`-w` everywhere**: root and basic_control both suppress all warnings;
  ViconDataStream uses `-Wall -Wextra` — inconsistent even internally. For a
  hardware-commanding codebase, `-w` on first-party sources is a real cost;
  narrowing it is worthwhile but should be done incrementally (B6).
- **Class/abstraction health is good**: no factories, no managers, no
  gratuitous interfaces. `ReferenceSource` (one implementation today) is a
  seam, not ceremony — it is how the pose channel returns. The plain-struct
  + free-function style the goal asks for is already the house style.
- **The genuinely architectural problems** are the two recorded above and
  in §5: the dead-but-tuned soft-safety layer (S1), and the
  planner-paces-fast / controller-clips-slow velocity split (C-DUP-3) that
  turns a planning-time property into a late validation failure.
- Out of scope but adjacent: the sim/hardware unification decision
  (`docs/superpowers/specs/2026-08-07-shared-robot-io-boundary-design.md`)
  will move several of these boundaries; slices below are chosen so none
  conflicts with it.

## 9. Findings table

Deduplicated across the three subagents (B/C/S prefixes) plus lead findings
(L). Merged: B1≡C-BUG-1; B4≡C-DEAD-4/5.

| ID | file:symbol | evidence | current purpose | class | risk | conf | smallest verification |
|---|---|---|---|---|---|---|---|
| B1 | root CMakeLists:118 `add_subdirectory(TrajectoryRealTime)` | dir has no CMakeLists (verified); deleted by `0161f39b`, root never updated | none — dangling | remove | low: root build already cannot configure | high | `cmake -S . -B /tmp/cfg` fails naming this line |
| B2 | TrajectoryExecution/CMakeLists:9-10 `Connect.cpp/.h` | files absent (verified); folded into Hardware by `b25b5bc8` | none — dangling sources | remove entries | low: target unreachable anyway | high | same configure attempt after B1 |
| B3 | root `main`/`test_vicon` targets + ViconDataStream | unbuildable via B1/B2; not referenced by active layer (grep) | upstream Vicon demo | investigate (scope decision) | medium: README's headline demo | high on status / low on fate | ask: is the upstream demo retired? |
| B4 | root `test_kinova.cpp`, `test_task_impedance.cpp` | in no CMakeLists (grep, verified by lead); test_kinova reads nonexistent `dh_params.yaml` | none | remove after confirmation | low | high | `grep -rln test_kinova --include=CMakeLists.txt .` |
| B6 | `-w` at root:8 and basic_control:15 | direct read; ViconDataStream contradicts with `-Wall -Wextra` | warning suppression | replace incrementally | medium: may surface many warnings at once | high | scratch build with `-Wall`, count |
| B7 | basic_control CMakeLists, 9× boilerplate triple | pattern repeats lines 83-352 | per-target plumbing | replace with function | low: behaviour-preserving | high | diff `compile_commands.json` before/after |
| B8 | `GEN3_DUAL_URDF_PATH` defined twice | both CMakeLists, comment acknowledges | one URDF path | document | low | high | n/a |
| B9 | `RUNS_ROOT_DIR` baked absolute | CMakeLists:558-562, intentional per comment | run output root | document (README note) | low | high | n/a |
| B10 | `tests/test_run_session.sh` unwired | grep: no add_test/CI (lead-verified) | pins the SIGINT-delivery safety property | keep + wire into ctest | low (stub binaries only) | high | `ctest -N` shows it after wiring |
| B11 | `show_frames.m` header comment | describes retired pipe grammar; Targets.cpp accepts TRAJ blocks only | stale usage doc | document (fix comment) | low | high | textual contradiction, already shown |
| C1 | Config.h:370 `kAcceptOrientationTargets` | grep: declaration only (lead-verified); use site deleted in `7dd50d0d` | orphaned gate | remove | low: unread | high | repo grep stays zero; build + 27 tests |
| C2 | Config.h:377 `kLoopLogPrefix` | grep: declaration only (lead-verified); superseded by `ArmConfig::log_prefix` | superseded | remove | low | high | same |
| C3 | Kinematics.h:182 `right_base_frame_id()` | grep: no external caller; member itself live | unused accessor | remove | low | high | grep + build + tests |
| C6 | root vs TrajectoryGeneration `joint_limits.yaml` | diff empty; two independent files | duplicated limits table | replace with one copy (or generate) | low; confirm planner reads the surviving path | high | planner_bridge tests after change |
| C7 | 45 °/s clip vs ~79.6/69.9 planner pacing vs test's 1.3963 rad/s | three sources, planner.yaml comment concedes late failure | unreconciled velocity envelopes | investigate | medium: motion-path pacing | high | decide once `qdot-limit-raise.md` is resolved |
| C8 | Main.cpp:107 `kFixedTargetUseRpy` branch | static_assert'd false; CSV schema preservation, comment accurate | legacy schema field | keep (documented) | none | high | n/a |
| C9 | Hardware.h:236-244 format changelog | inline history, formats 4→9 | log archaeology | move to docs/decisions | low | med | cosmetic |
| S1 | Controller.cpp pose branch + Targets.cpp:283 idle hold | `reference.pose` never written; idle path returns joint hold (lead-verified line-by-line) | reactive law, null-space avoidance, arrival monitors — all unreachable | investigate (decision: reconnect or retire) | acting is controller behaviour — Christian's call; *not acting* leaves no graded limit avoidance on the live path | high | temporary print in pose branch; confirm silent on a real run |
| S2 | Runner.cpp:388-404 prints before Send | contradicts the file's own rule obeyed at 515-536 | arrival notices | replace (move below Send) when pose channel returns | low now (dormant via S1) | high | reorder + overrun counter as backstop |
| S3 | Actuation.cpp:102-129 boundary hold+stop | holds prior safe frame, then stops | only live j2/4/6 protection | document (pair with S1 decision) | low (read-only) | high | cross-ref S1 |
| S5 | Config.h guard overrides (4×, all false) | echoed to CSV preamble, loud at loop start | escape hatches | keep | none while false | high | pre-session grep for `= true` |
| S8 | raw-dt overrun vs clamped-dt integration | Runner.cpp:340 vs Actuation.h:28 | two dts, two purposes | document (one cross-ref comment) | low | med | comment only |
| S13 | PlanSolver.cpp:159 IK failure text | names failing sample, not residual | reachability gate | document (add residual to message) | low | med | message change + bridge tests |
| L1 | both test suites | 27/27 pass at HEAD (run by lead) | cleanup baseline | keep (rerun after every slice) | — | high | `ctest` in both build dirs |
| L2 | Controller.h / State.h header diagrams | present pose channel as live peer; S1 shows it is not | orientation for new readers | document (one-sentence status note) | low | high | textual |

## 10. Ranked cleanup plan

Small, independent slices, ranked by confidence × value, deletion before
architecture. After every slice: rebuild both projects, rerun both suites
(27 tests), nothing else.

1. **Dead-symbol deletion** — remove C1, C2, C3; fix B11 and the
   `show_frames.m` comment; add the L2 one-sentence status notes.
   Pure deletion + comment edits; zero behaviour change possible.
2. **Root build honesty** — remove the dangling B1/B2 references, and put
   the B3/B4 question to Christian: if the upstream Vicon demo is retired,
   delete `main.cpp`, `test_vicon.cpp`, `test_kinova.cpp`,
   `test_task_impedance.cpp`, `ViconDataStream/`, the root config files and
   the README's demo sections (or move the lot to an `attic/`); if kept, fix
   the root build properly. Either way the tree stops lying about what
   builds.
3. **CMake boilerplate** — B7 function extraction in basic_control;
   verify by diffing `compile_commands.json` for identical flags.
4. **Wire `test_run_session.sh` into ctest** (B10) so the script's safety
   property is regression-tested automatically.
5. **Docs moves** — C9 changelog excerpt to docs/decisions.
6. **Config data dedup** — C6 single `joint_limits.yaml`.
7. **Decision slices (Christian first)** — S1 (reconnect the soft
   limit-avoidance for the joint path, or record its retirement), C7
   (velocity-envelope reconciliation, naturally paired with the existing
   `qdot-limit-raise.md` decision), S2 (print reorder, only meaningful with
   S1), B6 (incremental warning re-enable).

**Recommended first three slices: 1, 2, 3.** Slice 1 is pure
high-confidence deletion with a mechanical verification. Slice 2 removes the
single biggest source of new-engineer confusion and needs only a yes/no from
Christian on the demo's fate. Slice 3 is behaviour-preserving simplification
with a byte-level check. All three leave controller behaviour, safety
thresholds and hardware paths untouched, and none conflicts with the pending
sim/hardware unification design.
