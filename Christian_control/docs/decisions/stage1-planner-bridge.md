# Decision: `planner_bridge` — Stage 1 stop-and-replan as a separate process

2026-08-05. `Christian_control/planner_bridge/` is a new, standalone CMake
project: one solve of the existing GPMP2 optimizer (`TrajectoryGeneration/`)
per invocation, turned into a short list of Cartesian waypoints on the
controller's existing stdin target grammar. It is Stage 1 of the
stop-and-replan design in `../thesis/replanning-motivation.md` (§4,
"Stage 1 — stop-and-replan (baseline)").

## Why a separate process, not a new controller

`../thesis/replanning-motivation.md` §5 states the failure-ownership
argument this follows exactly: replanning sits *above* the safety kernel,
never inside it. `basic_control` keeps its ONE controller and ONE
reference source (`src/Controller.h`); `planner_bridge` is a new *source*
of stdin targets, exactly like a human typing `x y z` — it is built,
linked, and tested independently of `basic_control` and never touches
Kortex (`gtsam`/`gpmp2`/`Eigen` only; the only `Kortex` tokens anywhere
under `planner_bridge/` are C++ comments documenting "Kortex actuator
order", not an SDK dependency). If the bridge process crashes, hangs, or
is never started, the controller's existing behaviour is unchanged: it
holds the last valid target, or holds its compiled terminal target
forever. There is no code path by which a planner failure can degrade
controller safety below what `whole-path-validation.md` and the 2026-08-04
hardening already guarantee.

## Base-flip and tool-matching (measured numbers)

`../thesis/replanning-motivation.md` §7 records the model-agreement check
this bridge is built on: the planner's DH chain and the controller's URDF
chain agree to 0.36 mm mean / 0.49 mm max after removing one fixed
base-frame rotation (a 180° flip of the DH base frame relative to
`base_link`, Kinova Table 94 convention). `PlannerModel.cpp` applies that
flip directly — `DhBaseInBaseLink()` returns
`gtsam::Pose3(gtsam::Rot3::Rx(M_PI), Point3(0,0,0))`, used as
`ArmModel::createArmModel`'s `base_pose` — rather than converting outputs
after the fact. `tests/test_planner_model.cpp` re-proves agreement on this
branch's build: **0.445 mm worst-case** tool-position disagreement against
`tools/AnalyticalKinematics.cpp` over the FK-agreement sweep (asserted
`< 1 mm`; measured directly via `ctest -R planner_model -V`, 2026-08-05).

Tool matching: the planner's own DH table (`TrajectoryGeneration/config/
dh_params.yaml`) targets the flange, not the controller's configured tool
frame. `config/dh_params_tool.yaml` is a tool-matched copy — every row
identical except joint 7's `d`, changed from the flange offset to
`-0.2874 m` (flange `-0.1674` minus the `0.12 m` `ConfiguredTool_Link`
offset read from `ControlConfig::GetToolConfiguration`, matching
`basic_control`'s `kRightEndEffectorFrame`). This is a decision, not a
derived constant: §7 notes the two tool points differ by design
(~20 mm, planner grasp site vs. controller tool frame) and one had to be
chosen for the interface. The bridge always loads
`config/dh_params_tool.yaml` by default (`BridgeMain.cpp`
`DefaultDhPath()`), never the untouched planner original.

> **Superseded (2026-08-05, later the same day):** the hand-authored DH
> YAMLs described above no longer exist. `dh_params_tool.yaml` is now
> GENERATED from `GEN3_dual_mounted.urdf` at build time
> (`planner_bridge/tools/generate_dh_params.cpp`), and the flange-targeted
> planner original plus root `config/dh_params.yaml` were deleted. See
> `generated-dh-params.md`. The tool-frame *decision* recorded here still
> stands — the generator derives d7 to the same `ConfiguredTool_Link`
> frame; only the maintenance mechanism changed.

## Waypoint transport: stdin FIFO, and its Stage 1 limitation

**Superseded 2026-08-05 by `stage15-bridge-workflow.md`.** Everything in
this section describes the Stage 1 transport as originally shipped:
interactive stdin, held open by an operator-managed FIFO
(`exec 3>fifo`), position-only lines. Stage 1.5 deletes interactive
stdin entirely, replaces the operator-held FIFO with a controller-owned,
self-reopening named pipe (`config::kTargetPipePath`), and extends the
line grammar to optionally carry orientation (parsed, not yet consumed).
The waypoint cap, exit-code contract, and validation-before-write
guarantee described below are unchanged. See
`stage15-bridge-workflow.md` for the current design.

The bridge writes plain `%.6f %.6f %.6f` lines to `targets`
(`Waypoints.cpp` `FormatTargetLine`) — the exact grammar
`Targets.cpp`'s `ParsePoseTarget` already accepts (position-only, no
orientation; `input >> x >> y >> z`). In the hardware runbook this stream
is a named FIFO the operator creates (`mkfifo`), piped into
`./controller`'s stdin in one terminal while `planner_bridge` writes to it
from another. No new controller-side parsing was added — the bridge is
just another writer on an interface that already existed.

**Stage 1 limitation, stated plainly:** the controller does not execute
the planner's joint-space path. It receives a handful of Cartesian
waypoints and reaches each one via the reactive law's own DLS/task-space
tracking, moving all 7 joints in whatever way the reactive law resolves at
runtime. The path GPMP2 optimized for obstacle clearance and joint limits
is NOT the path the arm actually flies between those waypoints — only the
waypoints themselves are guaranteed to lie on the planned path. Obstacle
and joint-limit margins used when placing waypoints must be generous
enough to cover the reactive controller's deviation from the planned
inter-waypoint path, not just the sampled points. This is the
specific gap that Stage 2 (replan-while-moving) and Stage 3
(receding-horizon) are motivated to close (`../thesis/
replanning-motivation.md` §4).

## The ≤8 waypoint cap

`Waypoints.cpp` `SampleCartesianWaypoints` thins the optimizer's support
states to at most `max_count` points (default 8) at least
`min_spacing_m` apart (default 0.05 m), always keeps the final goal, and
evicts trailing kept intermediates that fall within spacing of the goal
so the last pair still meets the guarantee. The cap of 8 is not
arbitrary: it is exactly `PoseTargetMailbox::kCapacity` in
`basic_control/src/Targets.h:43`, the controller's fixed-size SPSC queue.
A ninth waypoint would either block the bridge's single write or be
silently unqueueable by `Targets.cpp`'s `Enqueue` (`write - read >=
kCapacity` rejects); capping in the bridge, before any target is written,
keeps the failure a normal "shorter plan" case instead of a queue-full
rejection mid-stream.

## SDF grid volume and `--box` rejection

`WorldSdf.cpp`'s grid is fixed: origin `(-1.2, -1.2, -0.4)` m, cell 0.04 m,
60×60×50 cells, covering `x [-1.2, 1.2]`, `y [-1.2, 1.2]`, `z [-0.4, 1.6]`
m in `base_link` (`WorldSdf.h` `WorldGridBounds()`). z was widened from
40 to 50 cells (`-0.4..1.16` m to `-0.4..1.6` m) because the zero-config
tool sits at `z = 1.3073` m (measured 2026-08-05) — inside the old grid's
top face, leaving essentially no headroom for an obstacle near the tool's
own working height. gpmp2's `SignedDistanceField::getSignedDistance`
returns zero obstacle cost for any query outside the grid — "no
obstacle" — with no warning. `BridgeMain.cpp` therefore rejects a
`--box` whose center ± half-extent is not fully contained in
`WorldGridBounds()` before the solve ever starts (exit 1, diagnostics
state the checked volume), rather than silently planning through an
obstacle gpmp2 could not see.

## `RunBridge` exit-code contract

`BridgeMain.h` / `BridgeMain.cpp`:

| exit | meaning |
| --- | --- |
| 0 | targets emitted (one line per waypoint, buffered and flushed only after validation passes) |
| 1 | bad arguments |
| 2 | start state unavailable (bad `--state-csv`, or no complete `meas_j1..7` row) |
| 3 | solve failed (model load or optimizer threw / returned an empty trajectory) |
| 4 | validation rejected the plan (a support state exceeds the joint-2/4/6 software limits — `Waypoints.cpp`'s `ValidationLimitsDeg()`, pinned by test to `config::kJointSoftwareLimitDeg`, Config.h:168: 126.9/145.0/118.0 deg, tighter for j2 than its 130 deg firmware warn limit) |

All output is buffered in an `std::ostringstream` and written to `targets`
only once, after validation fully passes (`RunBridge`, `BridgeMain.cpp`)
— every non-zero exit path writes nothing to `targets`, so a failed bridge
run can never leave a partial or malformed target stream for the
controller to read. During the solve, the legacy optimizer's own
`std::cout` chatter is redirected into `diagnostics` via the RAII
`CoutRedirectGuard` and restored on scope exit (including through an
exception), so that chatter can never land in the real binary's `targets`
stream (`std::cout`, piped to the controller over the FIFO).

## Two things the plan didn't foresee

1. **`planner_bridge/src/GenerateTrajectory.cpp` is a new forwarding
   shim, not existing legacy code.** `GenerateTrajectory.h` declared a
   free function `optimizeJointTrajectory` that was never defined
   anywhere in the tree — the only implementation was the class method
   `OptimizeTrajectory::optimizeJointTrajectory`
   (`TrajectoryOptimization.h`). This shim forwards to that class with
   zero start velocity and class-default `target_dt`/tolerances, and
   additionally times the call to fill `TrajectoryResult::
   optimization_duration`, which the class method computes internally
   but discards (`TrajectoryOptimization.cpp:561` — a local duration
   variable is computed and never assigned to the returned struct,
   leaving the field default-constructed/indeterminate).
   `initiation_duration` is set to zero rather than left indeterminate.
   The shim was first added inside the legacy `TrajectoryGeneration/`
   tree; on 2026-08-05 Christian ruled it should live with the bridge
   instead, so it sits in `planner_bridge/src/` and compiles only into
   `bridge_core`. The legacy tree remains untouched by this branch.
2. **`LD_LIBRARY_PATH` test-environment workaround.** All six
   `planner_bridge` ctest targets that link `bridge_core`
   (`planner_model`, `world_sdf`, `plan_solver`, `waypoints`,
   `start_state`, `bridge_main`) set `ENVIRONMENT
   "LD_LIBRARY_PATH=${TP_LIB}"` in `CMakeLists.txt`, because the
   third-party `gtsam`/`gpmp2` `.so` files under `third_party/lib` lack
   `RUNPATH` entries pointing at their sibling libraries. Without this,
   the test binaries build but fail to start (dynamic linker cannot find
   the sibling `.so`s). This is a property of the vendored third-party
   binaries, not of code written for this task; it is worked around at
   the test level here the same way `../pinocchio-libcoal-fix.md`
   documents an equivalent gap elsewhere.

## Validated state (2026-08-05, this branch)

- 6/6 `ctest --test-dir Christian_control/planner_bridge/build` PASS,
  hardware-free (`cmake --build` + `ctest`, this session).
- `test_planner_model`: 0.445 mm worst-case FK disagreement (< 1 mm gate).
- `test_plan_solver`: representative run — 3.2 mm final goal error,
  862 ms solve (both machine- and start/goal-dependent; the task brief's
  range across runs was 3.2–4.9 mm / ~886 ms).
- `test_bridge_main`: end-to-end CLI run from `--start-deg 0 0 0 0 0 0 0`
  to `--goal 0.15 0.075 1.207` exits 0 and emits 1–8 lines that all
  round-trip through the real `ParsePoseTarget`.
- Hardware validation — the arm actually tracking bridge-emitted
  waypoints — is **PENDING** and requires Christian's explicit per-run
  authorization. Nothing in this branch has been run against the robot.
