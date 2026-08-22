# Obstacle-Aware Graded Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` to implement this plan task-by-task. One implementation agent owns all writes; the minimality and evidence reviewers remain read-only. After every task, the writer performs the accepted cleanup before the next task begins.

**Goal:** Produce a validated exact or shortened trajectory for every request that the bounded GPMP2 search can solve, then prove the selected hand physically moves on a good trajectory in a supervised run.

**Architecture:** Keep one GPMP2 solver and one dense validation path. Replace the anonymous union scene with named per-object fields, try a fixed deterministic set of terminal branches and route seeds, repair only dynamic excess by bounded from-scratch duration re-solves, and emit one honest `REACHED`, `GOAL_BLOCKED`, or `FAILED` outcome. Collision and dynamic constraints disqualify candidates rather than prematurely ending the request; exact-route exhaustion always falls through to shortened search.

**Tech Stack:** C++17, Eigen, bundled GTSAM/GPMP2 and Pinocchio, yaml-cpp, CMake/CTest, Python 3/pytest and the existing run-artifact plotting path.

**Spec:** `docs/superpowers/specs/2026-08-22-obstacle-aware-graded-planner-design.md`

## Global Constraints

- Preserve exact `q(0)` and supplied `qdot(0)` on every solve. Never replace measured state or manufacture zero velocity.
- Use the existing dense trajectory sampling at `path_following.validation_dt_s`, including both endpoints. Add no second validator or reconstructed wire-format gate.
- Hard executable conditions are finite timed states, exact-start numerical integrity, configured prohibited-pair/self-collision clearance, effective joint/dynamic limits after bounded repair, and honest terminal classification.
- IK failure, optimizer convergence/cost, route-seed failure, intermediate path deviation and acceleration before duration repair are candidate evidence, not independent whole-request rejection gates.
- Keep the approved obstacle-only bypass strength (`0.1 * scene sigma`) and three duration attempts. Do not add generic retries.
- Use one writer. Reviewers never edit. Each replacement task removes its old production path, test registration, config key, include, comment and call site in the same commit.
- Add one planner-behavior test executable. Extend existing tests where ownership already exists; do not create source-text/order tests or duplicate micro-tests.
- Preserve all unrelated dirty-tree work. Never run a robot-facing binary until the final supervised task and its exact authorization gate.

---

### Task 1: Make physical and effective limits explicit

**Files:**
- Modify: `Christian_control/planning/config/joint_limits.yaml`
- Modify: `Christian_control/planning/optimisation/utils.h`
- Modify: `Christian_control/planning/optimisation/utils.cpp`
- Modify: `Christian_control/planning/src/PlanSolver.cpp`
- Modify: `Christian_control/planning/src/PlanSolver.h`
- Modify: `Christian_control/planning/tools/probe_path_reachability.cpp`
- Modify: `Christian_control/planning/tests/test_joint_type_contract.cpp`

**Interface:** Replace the two-value `createJointLimits()` pair with one plain value:

```cpp
struct PlannerJointLimits {
    JointLimits position_rad;
    JointLimits hardware_velocity_rad_s;
    JointLimits effective_velocity_rad_s;
    JointLimits hardware_acceleration_rad_s2;
    JointLimits effective_acceleration_rad_s2;
};

PlannerJointLimits createJointLimits(const std::string& config_path);
```

- [ ] Change `acceleration_limits` in `joint_limits.yaml` to the Kinova Gen3 7-DoF hardware table: joints 1–4 `[-5.2, 5.2] rad/s²`, joints 5–7 `[-10.0, 10.0] rad/s²`. Add `acceleration_planner_fraction: 1.0` beside the existing velocity fraction. Keep the 7-DoF grouping explicit in comments and cite the official Gen3 guide.
- [ ] Parse and range-check both fractions in `createJointLimits()`: finite and in `(0, 1]`. Derive effective values componentwise. Delete the current `vel_limits.upper(j) * 2.0` acceleration fabrication from `PlanSolver.cpp`.
- [ ] Carry both hardware and effective vectors in `PlanJointLimits`/artifacts so a run can state what came from Kinova and what the planner enforced. Do not add a controller acceleration constant that has no consumer.
- [ ] Extend the existing joint-limit test with exactly two assertions: the hardware 1–4/5–7 acceleration split and `effective = fraction * hardware`. The panel does not currently expose acceleration limits, so do not add unused panel handling.
- [ ] Run:

```bash
cmake -S Christian_control/planning -B Christian_control/planning/build
cmake --build Christian_control/planning/build --target test_joint_type_contract -j2
ctest --test-dir Christian_control/planning/build -R '^joint_type_contract$' --output-on-failure
```

- [ ] Cleanup scan:

```bash
rg -n "vel_limits\.upper\(j\) \* 2\.0|No acceleration table exists|planning acceleration evidence|57\.3" Christian_control/planning Christian_control/panel
```

Expected active-code result: no fabricated acceleration or old 1.0 rad/s² hardware claim.

- [ ] Commit only Task 1 files:

```bash
git commit -m "planner: derive effective Gen3 dynamic limits"
```

---

### Task 2: Replace the combined SDF with named prohibited-pair fields

**Files:**
- Modify: `Christian_control/planning/src/StaticScene.h`
- Modify: `Christian_control/planning/optimisation/GenerateArmModel.h`
- Modify: `Christian_control/planning/optimisation/GenerateArmModel.cpp`
- Modify: `Christian_control/planning/src/PlannerModel.h`
- Modify: `Christian_control/planning/src/PlannerModel.cpp`
- Modify: `Christian_control/planning/src/MountSdf.h`
- Modify: `Christian_control/planning/src/MountSdf.cpp`
- Modify: `Christian_control/planning/src/PlannerConfig.cpp`
- Modify: `Christian_control/planning/config/planner.yaml`
- Modify: `Christian_control/panel/scene_config.py`
- Modify: `Christian_control/panel/static/scene.js`
- Modify: `Christian_control/panel/static/run.html`
- Modify: `Christian_control/planning/tests/test_mount_sdf.cpp`
- Modify: `Christian_control/planning/tests/test_scene_config.cpp`
- Modify: `Christian_control/panel/tests/test_scene_config.py`

**Interfaces:**

```cpp
enum class CollisionSphereGroup {
    kMountInterface, kProximalArm, kUpperArm, kForearm, kTool
};

struct NamedStaticObstacle {
    std::string id;
    bool enabled = false;
    StaticObstacleGeometry geometry;
    std::vector<CollisionSphereGroup> permitted_sphere_groups;
};

struct ObstacleQuery {
    double clearance_m;
    Eigen::Vector3d outward_normal_mount;
};

ObstacleQuery QueryStaticObstacle(const StaticObstacleGeometry& geometry,
                                  const Eigen::Vector3d& sphere_center_mount,
                                  double sphere_radius_m);

struct NamedObstacleField {
    std::string id;
    StaticObstacleGeometry geometry;
    gpmp2::SignedDistanceField sdf;
    std::unique_ptr<gpmp2::ArmModel> participating_arm;
    std::vector<std::size_t> participating_sphere_indices;
};

std::vector<NamedObstacleField> MakeNamedObstacleFields(
    const GridGeometry&, const PlannerModel&,
    const std::vector<NamedStaticObstacle>&);
```

- [ ] Return the authored sphere vector and a parallel group vector from `GenerateArmModel`; assign the first base-origin sphere to `mount_interface`, remaining link-0 spheres to `proximal_arm`, link 2 to `upper_arm`, link 4 to `forearm`, and link 6/tool spheres to `tool`. Keep group ownership beside sphere authorship.
- [ ] Implement analytic box and finite-vertical-cylinder signed clearance and outward normal in `QueryStaticObstacle()`. Use deterministic x/y/z and object-id tie breaking. Reuse it for dense checks and bypass evidence.
- [ ] Build one SDF and one filtered `gpmp2::ArmModel` per enabled object, reusing the existing FK model and copying only prohibited spheres. Delete `MakeMountSdf()` and its anonymous `min()` union implementation in this task.
- [ ] Replace `obstacles.epsilon_dist_m` with two strict keys in `planner.yaml`: `minimum_clearance_m: 0.05` and `preferred_clearance_m: 0.10`. Require `minimum <= preferred`. Add required `permitted_sphere_groups: []` to every scene object; unknown group names fail with the object id.
- [ ] Extend the panel's existing scene parser/editor to round-trip only the five declared group names. Do not add FK, SDF, collision decisions or inferred permissions to the browser.
- [ ] Extend `test_mount_sdf` to independently check box/cylinder clearance and normals at faces, caps, corners and inside points, then check that a permitted mount-interface sphere is absent while a prohibited proximal sphere remains. Extend the existing C++ and Python scene-config tests with one valid round trip and one unknown-group rejection.
- [ ] Run:

```bash
cmake --build Christian_control/planning/build --target test_mount_sdf test_scene_config -j2
ctest --test-dir Christian_control/planning/build -R '^(mount_sdf|scene_config)$' --output-on-failure
python3 -m pytest Christian_control/panel/tests/test_scene_config.py -q
```

- [ ] Cleanup scan:

```bash
rg -n "MakeMountSdf|epsilon_dist_m|minimum signed distance to all enabled|combined.*SDF" Christian_control/planning Christian_control/panel
```

Expected active-code result: zero old combined-SDF/config references. Historical docs may remain only when explicitly dated.

- [ ] Commit:

```bash
git commit -m "planner: model named obstacle pairs"
```

---

### Task 3: Use one dense point/path validator and one outcome vocabulary

**Files:**
- Rename: `Christian_control/planning/src/PathValidationReport.h` -> `Christian_control/planning/src/PlanValidationReport.h`
- Rename: `Christian_control/planning/src/PathValidationReport.cpp` -> `Christian_control/planning/src/PlanValidationReport.cpp`
- Rename: `Christian_control/planning/src/ValidatePath.h` -> `Christian_control/planning/src/ValidatePlan.h`
- Rename: `Christian_control/planning/src/ValidatePath.cpp` -> `Christian_control/planning/src/ValidatePlan.cpp`
- Modify: `Christian_control/planning/src/PlanSolver.h`
- Modify: `Christian_control/planning/src/PlanSolver.cpp`
- Modify: `Christian_control/planning/src/BridgeMain.cpp`
- Modify: `Christian_control/planning/src/PlannerRuntime.h`
- Modify: `Christian_control/planning/src/PlannerRuntime.cpp`
- Modify: `Christian_control/planning/src/PlanDebugDump.h`
- Modify: `Christian_control/planning/src/PlanDebugDump.cpp`
- Modify: `Christian_control/planning/src/PlanningAttemptRecorder.cpp`
- Modify: `Christian_control/planning/CMakeLists.txt`
- Delete: `Christian_control/planning/tests/test_plan_verdict.cpp`
- Delete: `Christian_control/planning/tests/check_solver_scene_ownership.cmake`

**Interfaces:**

```cpp
enum class PlanStatus { kReached, kGoalBlocked, kFailed };

struct PlanValidationInputs {
    const CartesianPath* desired_task_path = nullptr;
    gtsam::Pose3 requested_terminal_mount;
    gtsam::Pose3 candidate_terminal_mount;
    PlanStatus intended_status = PlanStatus::kReached;
    // measured start, dt, named fields, self-collision pairs, effective limits
};

struct PlanValidationReport {
    bool executable = false;
    std::string failure_reason;
    // finite/start, named pair clearance, self clearance, joint/dynamic,
    // terminal and optional trace-quality measurements
};

PlanValidationReport ValidatePlan(const PlannerModel&,
                                  const TrajectoryResult&,
                                  double duration_s,
                                  const PlanValidationInputs&);
```

- [ ] Move the existing uniform dense-view construction into `ValidatePlan()`. Sample at `0, dt, ... , T` and always include `T` once. Point and trace call this same function before world projection.
- [ ] Evaluate every named prohibited sphere/object pair with `QueryStaticObstacle()` and the configured hard minimum; record the worst pair id, sphere index and time. Evaluate existing self-collision pairs separately.
- [ ] Compute componentwise joint position, velocity and finite-difference acceleration ratios using Task 1's effective vectors. A dynamic excess returns a candidate disposition `needs_longer_duration`; it is not yet a whole-request result.
- [ ] Compute final FK translation/orientation error. `REACHED` requires 1 mm/0.01 rad to the requested pose; `GOAL_BLOCKED` requires the final FK to match the declared shortened candidate within those same numerical tolerances and reports shortfall to the request.
- [ ] Replace `PlanVerdict`, `PlanOutcome::ok`, `PathPlanOutcome::ok`, and stringly `meta.status = "ok"` with `PlanStatus`. Only reached/blocked statuses may own a trajectory. A failed solve returns no default/stale trajectory.
- [ ] Delete the verdict test and regex source-ownership CMake test. Their observable replacements live in the behavior fixture in Task 6; do not preserve compatibility aliases.
- [ ] Run the current focused start-state and recorder tests to characterize the mechanical migration:

```bash
cmake --build Christian_control/planning/build --target test_planner_start_state test_planning_attempt_recorder -j2
ctest --test-dir Christian_control/planning/build -R '^(planner_start_state|planning_attempt_recorder)$' --output-on-failure
```

- [ ] Cleanup scan:

```bash
rg -n "PlanVerdict|DecidePlanVerdict|PathValidationReport|ValidatePlannedPath|\.ok\b|status = \"ok\"|solver_scene_ownership" Christian_control/planning
```

Expected active-code result: zero old verdict, bool outcome, point-only validation or source-text test references.

- [ ] Commit:

```bash
git commit -m "planner: unify dense validation outcomes"
```

---

### Task 4: Generate deterministic terminal branches and exact terminal solves

**Files:**
- Modify: `Christian_control/planning/optimisation/TrajectoryInitiation.h`
- Modify: `Christian_control/planning/optimisation/TrajectoryInitiation.cpp`
- Modify: `Christian_control/planning/src/PathIk.h`
- Modify: `Christian_control/planning/src/PathIk.cpp`
- Modify: `Christian_control/planning/optimisation/TrajectoryOptimization.h`
- Modify: `Christian_control/planning/optimisation/TrajectoryOptimization.cpp`
- Modify: `Christian_control/planning/src/PlanSolver.h`
- Modify: `Christian_control/planning/src/PlanSolver.cpp`
- Modify: `Christian_control/planning/tests/test_planner_start_state.cpp`

**Interfaces:**

```cpp
struct TerminalIkCandidate {
    Eigen::Matrix<double, 7, 1> q_rad;
    std::uint64_t stream_id;
    int attempt_index;
    double position_error_m;
    double orientation_error_rad;
};

std::vector<TerminalIkCandidate> SolveTerminalIkCandidates(
    const PlannerModel&, const gtsam::Pose3& target_mount,
    const Eigen::Matrix<double, 7, 1>& q_start_rad,
    const PathIkJointLimits&, std::uint64_t effective_seed,
    std::size_t maximum_candidates = 3);
```

- [ ] Replace point initialization's repeated identical `solveIK` loop with fixed stream ids and fixed attempt counts. Remove wall-clock expiry from candidate existence in `PathIk`; retain a recorded solve duration only as telemetry. Order accepted, joint-space-distinct branches by wrapped displacement from measured start then stream id; retain at most three.
- [ ] Add an optional `terminal_config` equality argument to both GPMP2 solve functions. When present, seed the final `Values` with that exact configuration and add `NonlinearEquality<Vector>(x_N, terminal_config)`. Do not add a stiff prior beside it.
- [ ] Delete the now-unused `reOptimizeJointTrajectory()` overload and `InitializeTrajectory::initJointTrajectoryFromTarget()` path after their sole production caller is replaced. Do not retain the old single-terminal initializer as a fallback.
- [ ] Keep trace intermediate pose factors soft. The requested final trace pose still has terminal IK candidates and exact final joint equality; final FK, not IK residual, classifies `REACHED`.
- [ ] Extend `test_planner_start_state` with one mutation-sensitive case: nonzero measured start remains exact while a distinct exact terminal branch is also exact. Do not add another executable.
- [ ] Run:

```bash
cmake --build Christian_control/planning/build --target test_planner_start_state -j2
ctest --test-dir Christian_control/planning/build -R '^planner_start_state$' --output-on-failure
```

- [ ] Cleanup scan:

```bash
rg -n "kAnchorIkTimeBudgetS|steady_clock::now\(\) < deadline|initJointTrajectoryFromTarget\(|StartPosPrior|Goal.*Prior" Christian_control/planning
```

Expected: no wall-clock-controlled candidate pool, old single-terminal initializer, unused re-optimizer, or soft prior duplicating an exact terminal equality.

- [ ] Commit:

```bash
git commit -m "planner: solve deterministic terminal branches"
```

---

### Task 5: Add bounded normal/bypass routes and dynamic duration repair

**Files:**
- Modify: `Christian_control/planning/src/PlanSolver.h`
- Modify: `Christian_control/planning/src/PlanSolver.cpp`
- Modify: `Christian_control/planning/src/PathAssembly.h`
- Modify: `Christian_control/planning/src/PathAssembly.cpp`
- Modify: `Christian_control/planning/optimisation/TrajectoryOptimization.h`
- Modify: `Christian_control/planning/optimisation/TrajectoryOptimization.cpp`
- Modify: `Christian_control/planning/src/PlanDebugDump.h`
- Modify: `Christian_control/planning/src/PlanDebugDump.cpp`

**Interfaces:**

```cpp
enum class RouteHypothesis { kNormal, kPositiveBypass, kNegativeBypass };

struct CandidateEvidence {
    PlanStatus terminal_kind;
    std::size_t terminal_branch;
    RouteHypothesis route;
    int duration_attempt;
    double duration_s;
    double scene_collision_sigma;
    double solve_time_s;
    PlanValidationReport validation;
    std::string disposition;
};
```

- [ ] For each terminal branch, solve the normal seed first. Only when dense failure contains prohibited scene collision, identify the first blocking named object/sphere with Task 2's query and create the two opposing tangential bypass seeds. Point bypass adds one Cartesian midpoint and solves it to joint space; trace bypass offsets only the colliding seed interval before continuation IK.
- [ ] Pass scene fields as a collection to optimization and add one obstacle factor set per field. Pass self-collision sigma separately. Normal route uses configured scene sigma; both bypass routes use `0.1 * scene sigma`; neither changes self-collision weight.
- [ ] After one solve, if and only if geometry/terminal/numerics are valid and velocity or acceleration exceeds its effective vector, compute

```cpp
alpha = max(max_j velocity_ratio_j,
            sqrt(max_j acceleration_ratio_j));
next_duration_s = current_duration_s * alpha;
```

Rebuild waypoint times, initial values and velocity guesses, then solve the same candidate from scratch. Maximum three total duration attempts. Never post-scale stored qdot.
- [ ] Evaluate all entered routes for a branch. Rank point routes by `min(clearance, preferred_clearance_m)`, duration, then integrated joint travel. Rank trace routes by common-parameter RMS, maximum deviation, then capped clearance. Move to the next terminal branch only when the current branch has no validated route.
- [ ] Append every attempt to one `candidate_attempts.csv`; do not create per-retry directories or a retry manager.
- [ ] Run the start-state test and an offline replay of the existing point request with its copied config:

```bash
ctest --test-dir Christian_control/planning/build -R '^planner_start_state$' --output-on-failure
replay_dir=$(mktemp -d)
mapfile -t replay_args < runs/2026-08-22/session_194737/plans/right_request_1_goal_1/request.args
Christian_control/planning/build/planner_bridge "${replay_args[@]}" --debug-dir "$replay_dir"
```

Redirect replay output to a new temporary directory under `/tmp`; do not overwrite archived evidence.

- [ ] Cleanup scan:

```bash
rg -n "trajectory_vel\[.*\] /=|post-hoc|uniformly time-scaled|shared.*collision_sigma|generic retry" Christian_control/planning
```

Expected: no post-hoc derivative scaling and no shared scene/self escalation.

- [ ] Commit:

```bash
git commit -m "planner: search bounded obstacle routes"
```

---

### Task 6: Fall through to bounded shortened terminals and prove behavior

**Files:**
- Modify: `Christian_control/planning/src/PlanSolver.h`
- Modify: `Christian_control/planning/src/PlanSolver.cpp`
- Modify: `Christian_control/planning/src/BridgeMain.cpp`
- Modify: `Christian_control/planning/src/PlannerRuntime.h`
- Modify: `Christian_control/planning/src/PlannerRuntime.cpp`
- Modify: `Christian_control/planning/src/PlanningAttemptRecorder.cpp`
- Modify: `Christian_control/planning/src/PlanDebugDump.h`
- Modify: `Christian_control/planning/src/PlanDebugDump.cpp`
- Create: `Christian_control/planning/tests/test_obstacle_aware_planner.cpp`
- Modify: `Christian_control/planning/CMakeLists.txt`

- [ ] Implement the exact control flow without an early IK shortcut:

```text
search exact terminals and routes
if validated exact exists: REACHED
else generate/search shortened terminals and routes
if validated shortened exists: GOAL_BLOCKED
else: FAILED with no trajectory
```

- [ ] Generate requested-orientation shortened targets at 16 fixed fractions from requested terminal toward measured-start TCP, nearest first, plus blocking-object outward projection and two tangent targets. Reuse Task 4's fixed IK streams. Retain at most three distinct terminals. Rank exact-orientation tier by position shortfall; if empty, rank orientation then position. Record both residuals and label the selected result `best_validated_bounded_candidate`.
- [ ] Apply the same normal/bypass/duration search to shortened terminals. A shortened trace reports terminal shortfall and never claims rejoin. Only `REACHED` and `GOAL_BLOCKED` project and publish a world trajectory.
- [ ] Add one behavior executable containing five scenario fixtures from spec section 9: obstructed exact point route; detoured/rejoined trace; exact IK but all exact routes fail then shortened succeeds; no shortened route returns failed/no trajectory; nonzero moving start with acceleration duration repair. Recompute terminal FK and primitive clearance independently in the test; do not call the outcome helper as its own oracle.
- [ ] Make the test mutation-sensitive to: collision changed to warning, exact-route failure skipping shortened search, path deviation becoming rejection, start state replaced, and acceleration repair removed. One case per distinct contract; no combinatorial defensive cases.
- [ ] Run:

```bash
cmake --build Christian_control/planning/build --target test_obstacle_aware_planner -j2
ctest --test-dir Christian_control/planning/build -R '^obstacle_aware_planner$' --output-on-failure
```

- [ ] Cleanup scan:

```bash
rg -n "PlanVerdict|bool ok|closest reachable|globally unreachable|path.*REJECT|collision.*WARNING|ValidateJointPath" Christian_control/planning
```

Classify historical-doc hits; active production/config/tests must have zero superseded semantics.

- [ ] Commit:

```bash
git commit -m "planner: return graded executable outcomes"
```

---

### Task 7: Benchmark, visualize, document, and perform the final lean cleanup

**Files:**
- Modify: `Christian_control/planning/scripts/plot_plan.py`
- Modify: `Christian_control/runtime/README.md`
- Modify: `Christian_control/README.md`
- Modify: `Christian_control/docs/motion-limits-map.md`
- Modify: `docs/intent/story.md` in its own commit

- [ ] Replay the archived point and traced requests plus the five fixture scenes. Save one benchmark CSV with request id, phase, branch, route, duration attempt, disposition, solve time, capped clearance, terminal residual and trace metrics.
- [ ] Produce the requested visuals from existing artifacts: requested/planned TCP with obstacle geometry, clearance against hard/preferred lines, planned/measured q and qdot with start mismatch, and candidate success/latency. Use the existing visualization path; do not duplicate FK or collision math in the panel.
- [ ] Report success rate, p50/p95 latency, actual/max solve count, route diversity and which bounded behaviors contributed a selected trajectory. Do not simplify bypass weighting or acceleration repair in this implementation; record evidence for a later decision.
- [ ] Update active docs to the one validator, three outcomes, hard/effective dynamic limits and exact supervised completion rule. Update `docs/intent/story.md` with the approved outcome and why, cite the raw prompt log, show Christian the story diff, and commit it separately.
- [ ] Run all affected hardware-free checks:

```bash
cmake -S Christian_control/planning -B Christian_control/planning/build
cmake --build Christian_control/planning/build -j2
ctest --test-dir Christian_control/planning/build --output-on-failure
cmake -S Christian_control/runtime -B Christian_control/runtime/build
cmake --build Christian_control/runtime/build -j2
ctest --test-dir Christian_control/runtime/build --output-on-failure
python3 -m pytest Christian_control/panel/tests -q
git diff --check
```

- [ ] Minimality reviewer returns `CLEAN` only after inspecting the replacement ledger and these scoped scans:

```bash
rg -n "MakeMountSdf|PlanVerdict|DecidePlanVerdict|PathValidationReport|ValidatePlannedPath|epsilon_dist_m|bool ok|solver_scene_ownership" Christian_control/planning Christian_control/panel
rg -n "GOAL_BLOCKED|REACHED|FAILED|minimum_clearance_m|preferred_clearance_m|hardware_acceleration|effective_acceleration" Christian_control/planning Christian_control/panel Christian_control/runtime
```

- [ ] Commit docs/plot cleanup separately from `docs/intent/story.md`; never bundle unrelated dirty files.

---

### Task 8: Supervised physical motion proof

**No source edits during a motion attempt.**

- [ ] Confirm the final implementation commit is cleanly built and identify the exact controller binary hash, selected arm and IP from the current runtime configuration.
- [ ] Open the existing panel with:

```bash
python3 Christian_control/panel/control_panel.py
```

This is not robot-facing. Verify the local panel loads before starting hardware.
- [ ] Immediately before hardware execution, state: exact `run_session.sh` command, arm/IP, small clear-space point target, expected direction/distance/duration, effective velocity/acceleration limits, stop conditions and recovery path. Obtain Christian's confirmation that he is present, workspace is clear and e-stop is reachable.
- [ ] Run only the authorized command, initially one arm and fixed Mount:

```bash
Christian_control/planning/scripts/run_session.sh --arm left --mount fixed
```

Use the panel to send the agreed small point target. Stop on planner `FAILED`, continuity rejection, fault, unexpected direction, following error, stale world state or Christian's stop command. Do not improvise another target.
- [ ] Completion requires archived evidence of `REACHED` or honest `GOAL_BLOCKED`, trajectory activation, nonzero measured joint/TCP motion, accepted exact start splice, and bounded tracking. Render the final requested/planned/measured and dynamic/clearance plots.
- [ ] Only after clear-space motion succeeds and the observed rig geometry plus permitted pairs match `planner.yaml`, request a separate exact authorization for one modelled-obstacle motion.
- [ ] Do not mark the goal complete until the selected hand physically moves with a good validated trajectory. If it does not, retain the run artifacts, diagnose the observed boundary, and continue the same goal.
