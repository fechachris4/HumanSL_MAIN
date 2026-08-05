# Stage 2: Joint-Trajectory Following Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The controller executes TrajectoryGeneration's timed joint trajectory (positions + velocities from the GPMP2 solve) instead of re-planning Cartesian waypoint hops with its own profile; the waypoint/profile machinery is swapped out.

**Architecture:** The bridge stops sampling 8 Cartesian waypoints and instead emits the validated solve's support states as a framed block of timed joint rows over the existing FIFO. The controller parses and validates the block off the RT thread, then a `JointTrajectorySource` samples it at 500 Hz via cubic Hermite interpolation and a joint-space tracking law (`qdot_cmd = qdot_ref + Kp_j (q_ref − q_meas)`) feeds the **unchanged** actuation path: per-joint velocity clip → position integration → lead clamp. Safety guards (servoing guard, fault policy, per-joint clip, following-error stop) are not weakened; a start-state tolerance gate refuses any trajectory that does not begin at the measured arm position.

**Tech Stack:** C++17, Eigen, existing `basic_control` build (`Christian_control/basic_control/build`), existing `planner_bridge` build, ctest. No new dependencies.

## Global Constraints

- Never run `controller` or any Kortex-linked binary as a test step; hardware runs need Christian's explicit per-run authorization (project `CLAUDE.md`).
- Wire format units: **seconds, degrees, degrees/second** (operator-facing units, matching telemetry `meas_j*` and Kortex actuator order). Internal storage: radians (matching `RobotState`).
- The per-joint velocity clip (`config::kQdotClipDegS`, verify exact name in `Config.h` before use), position integration, lead clamp, servoing guard, and fault handling are **not modified** by any task.
- Real-time loop rules: no allocation, locks, or I/O in the 500 Hz path except the accepted bounded exception documented in Task 3 (one pointer exchange + one bounded `delete` per plan activation).
- The final deletion task (Task 6) is gated on a supervised hardware run of the new path; do not execute it until Christian says the run happened.
- Follow existing test style: standalone `main()` + `assert`, `#undef NDEBUG` first line, registered in the module's `CMakeLists.txt` with `add_test`.

## File Structure

- `Christian_control/basic_control/src/JointTrajectory.h/.cpp` (new): wire grammar, accumulator, validation, Hermite sampling. Pure — no robot, no I/O.
- `Christian_control/basic_control/src/Targets.h/.cpp` (modify): input thread learns the block grammar; single-slot trajectory handoff.
- `Christian_control/basic_control/src/State.h` (modify): `Reference` gains an optional joint reference.
- `Christian_control/basic_control/src/Controller.cpp/.h` (modify): joint-space tracking branch.
- `Christian_control/basic_control/src/Main.cpp` (modify): construct the new source; hold-at-measured-q startup.
- `Christian_control/planner_bridge/src/TrajectoryEmit.h/.cpp` (new): format the solve result as a block.
- `Christian_control/planner_bridge/src/BridgeMain.cpp` (modify): emit block instead of waypoints.
- Deleted in Task 6 (post-hardware-gate): `TrajectoryProfile.h`, `PoseTargetSource`, Cartesian waypoint sampling, their tests.

---

### Task 1: JointTrajectory grammar, accumulator, and validation (controller side, pure)

**Files:**
- Create: `Christian_control/basic_control/src/JointTrajectory.h`
- Create: `Christian_control/basic_control/src/JointTrajectory.cpp`
- Test: `Christian_control/basic_control/tests/test_joint_trajectory.cpp`
- Modify: `Christian_control/basic_control/CMakeLists.txt` (register test; add `JointTrajectory.cpp` to the controller target's source list)

**Interfaces:**
- Produces:
```cpp
struct JointTrajectoryPoint {
    double t_s;
    Eigen::Matrix<double, 7, 1> q_rad;
    Eigen::Matrix<double, 7, 1> qdot_rad_s;
};
struct JointTrajectory {
    std::vector<JointTrajectoryPoint> points; // strictly increasing t_s, first t_s == 0
};
// Wire grammar (degrees on the wire):
//   TRAJ_BEGIN <count>
//   <t_s> <q1..q7 deg> <v1..v7 deg/s>     (count rows, 15 fields each)
//   TRAJ_END
class JointTrajectoryAccumulator {
public:
    // Feed one line. Returns a completed trajectory on TRAJ_END, nullopt otherwise.
    // Any malformed line resets the accumulator and sets `error` (never silently skips).
    std::optional<JointTrajectory> Feed(const std::string& line, std::string& error);
};
// nullopt = valid. Checks: >=2 points, strictly increasing t, first t == 0, all finite,
// every q within limits_low/high_deg, every |v| <= vel_limit_deg_s, and the implied
// average velocity between adjacent points <= vel_limit_deg_s per joint.
std::optional<std::string> ValidateJointTrajectory(
    const JointTrajectory& traj,
    const Eigen::Matrix<double, 7, 1>& limits_low_deg,
    const Eigen::Matrix<double, 7, 1>& limits_high_deg,
    const Eigen::Matrix<double, 7, 1>& vel_limit_deg_s);
```

- [ ] **Step 1: Write the failing test**

```cpp
// tests/test_joint_trajectory.cpp
#undef NDEBUG
#include <cassert>
#include <string>
#include "JointTrajectory.h"

static JointTrajectory FeedBlock(const std::vector<std::string>& lines) {
    JointTrajectoryAccumulator acc;
    std::string error;
    std::optional<JointTrajectory> out;
    for (const auto& l : lines) { out = acc.Feed(l, error); assert(error.empty()); }
    assert(out.has_value());
    return *out;
}

int main() {
    // Happy path: 3 points, degrees on the wire, radians in memory.
    const auto traj = FeedBlock({
        "TRAJ_BEGIN 3",
        "0    0 0 0 0 0 0 0    0 0 0 0 0 0 0",
        "1.0  10 0 0 0 0 0 0   10 0 0 0 0 0 0",
        "2.0  20 0 0 0 0 0 0   0 0 0 0 0 0 0",
    });
    assert(traj.points.size() == 3);
    assert(std::abs(traj.points[1].q_rad(0) - 10.0 * M_PI / 180.0) < 1e-12);
    assert(std::abs(traj.points[1].qdot_rad_s(0) - 10.0 * M_PI / 180.0) < 1e-12);

    // A malformed row resets and reports; the next block still works.
    {
        JointTrajectoryAccumulator acc; std::string error;
        assert(!acc.Feed("TRAJ_BEGIN 2", error) && error.empty());
        assert(!acc.Feed("0 1 2 nonsense", error).has_value());
        assert(!error.empty());
    }
    // Row count mismatch: TRAJ_END before <count> rows is an error.
    {
        JointTrajectoryAccumulator acc; std::string error;
        acc.Feed("TRAJ_BEGIN 2", error);
        acc.Feed("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", error);
        assert(!acc.Feed("TRAJ_END", error).has_value());
        assert(!error.empty());
    }

    // Validation.
    Eigen::Matrix<double, 7, 1> lo = Eigen::Matrix<double, 7, 1>::Constant(-180.0);
    Eigen::Matrix<double, 7, 1> hi = Eigen::Matrix<double, 7, 1>::Constant(180.0);
    Eigen::Matrix<double, 7, 1> vmax = Eigen::Matrix<double, 7, 1>::Constant(45.0);
    assert(!ValidateJointTrajectory(traj, lo, hi, vmax).has_value());
    // Position outside limits rejected.
    {
        auto bad = traj; bad.points[2].q_rad(0) = 200.0 * M_PI / 180.0;
        assert(ValidateJointTrajectory(bad, lo, hi, vmax).has_value());
    }
    // Stated velocity above the clip rejected.
    {
        auto bad = traj; bad.points[1].qdot_rad_s(0) = 90.0 * M_PI / 180.0;
        assert(ValidateJointTrajectory(bad, lo, hi, vmax).has_value());
    }
    // Implied velocity above the clip rejected (20 deg in 0.1 s = 200 deg/s).
    {
        auto bad = traj; bad.points[2].t_s = 1.1;
        assert(ValidateJointTrajectory(bad, lo, hi, vmax).has_value());
    }
    // Non-monotonic time rejected.
    {
        auto bad = traj; bad.points[2].t_s = 0.5;
        assert(ValidateJointTrajectory(bad, lo, hi, vmax).has_value());
    }
    return 0;
}
```

- [ ] **Step 2: Register the test in `Christian_control/basic_control/CMakeLists.txt`** following the exact pattern of `test_control_logic` (same includes/link line), then configure and build:
`cmake --build Christian_control/basic_control/build --target test_joint_trajectory -j`
Expected: compile FAILURE — `JointTrajectory.h` does not exist. That is this cycle's red.

- [ ] **Step 3: Implement `JointTrajectory.h/.cpp`** — accumulator states: `kIdle → kCollecting(count, rows) → emit on TRAJ_END`. Parse rows with the same strict finite-number discipline as `ParsePoseTarget` (reject trailing garbage). Convert deg → rad on parse. Keep every function free of I/O and allocation beyond the returned vector.

- [ ] **Step 4: Build and run:** `ctest --test-dir Christian_control/basic_control/build -R joint_trajectory --output-on-failure`
Expected: PASS. Then run the full controller suite; all existing tests stay green.

- [ ] **Step 5: Commit** `git commit -m "controller: joint-trajectory wire grammar, accumulator, validation"`

---

### Task 2: Hermite sampling

**Files:**
- Modify: `Christian_control/basic_control/src/JointTrajectory.h/.cpp`
- Test: extend `Christian_control/basic_control/tests/test_joint_trajectory.cpp`

**Interfaces:**
- Produces:
```cpp
// Cubic Hermite between the two bracketing points (positions + stated
// velocities); clamps: t <= 0 returns the first point, t >= last returns the
// last point with zero velocity. `complete` true once t >= last t.
struct JointTrajectorySample {
    Eigen::Matrix<double, 7, 1> q_rad;
    Eigen::Matrix<double, 7, 1> qdot_rad_s;
    bool complete;
};
JointTrajectorySample SampleJointTrajectory(const JointTrajectory& traj, double t_s);
```

- [ ] **Step 1: Write the failing test** (append to `main()`):

```cpp
    // Endpoint interpolation: at support times, exactly the support values.
    {
        const auto s0 = SampleJointTrajectory(traj, 0.0);
        assert(std::abs(s0.q_rad(0)) < 1e-12 && !s0.complete);
        const auto s1 = SampleJointTrajectory(traj, 1.0);
        assert(std::abs(s1.q_rad(0) - 10.0 * M_PI / 180.0) < 1e-9);
        // Midpoint lies between the bracketing supports and velocity is finite.
        const auto sm = SampleJointTrajectory(traj, 0.5);
        assert(sm.q_rad(0) > 0.0 && sm.q_rad(0) < 10.0 * M_PI / 180.0);
        // Past the end: last position, zero velocity, complete.
        const auto se = SampleJointTrajectory(traj, 99.0);
        assert(std::abs(se.q_rad(0) - 20.0 * M_PI / 180.0) < 1e-9);
        assert(se.qdot_rad_s.norm() == 0.0 && se.complete);
        // Hermite consistency: derivative at t=1 approximates the stated velocity.
        const double h = 1e-5;
        const auto a = SampleJointTrajectory(traj, 1.0 - h);
        const auto b = SampleJointTrajectory(traj, 1.0 + h);
        assert(std::abs((b.q_rad(0) - a.q_rad(0)) / (2 * h) - s1.qdot_rad_s(0)) < 1e-3);
    }
```

- [ ] **Step 2: Build, run, verify it fails** with `SampleJointTrajectory` undefined.
- [ ] **Step 3: Implement** standard cubic Hermite basis on each segment: with `s = (t−t0)/(t1−t0)`, `q(s) = h00 q0 + h10 (t1−t0) v0 + h01 q1 + h11 (t1−t0) v1`; derivative analogously. Linear scan for the bracketing segment (≤ ~21 points; no allocation).
- [ ] **Step 4: Build and run to green**, full suite green.
- [ ] **Step 5: Commit** `git commit -m "controller: Hermite sampling of joint trajectories"`

---

### Task 3: Input thread learns the block grammar; single-slot handoff

**Files:**
- Modify: `Christian_control/basic_control/src/Targets.h/.cpp`
- Test: extend `Christian_control/basic_control/tests/test_control_logic.cpp` (which already exercises `RunPoseTargetInputFromFd` over a real pipe fd — follow its existing fixture pattern exactly)

**Interfaces:**
- Consumes: `JointTrajectoryAccumulator`, `ValidateJointTrajectory` (Task 1).
- Produces:
```cpp
// Single-slot, latest-wins handoff. Producer (input thread) validates the
// block BEFORE publishing; the RT consumer only ever sees valid trajectories.
// Take() is one atomic exchange; the consumer deletes the previous active
// trajectory when it swaps a new one in — one bounded free per plan
// activation, the accepted RT exception recorded in the plan header.
class JointTrajectoryMailbox {
public:
    void Publish(std::unique_ptr<JointTrajectory> traj); // input thread
    std::unique_ptr<JointTrajectory> Take();             // RT thread; nullptr if none
private:
    std::atomic<JointTrajectory*> slot_{nullptr};
};
// The fd input loop now feeds every line to a JointTrajectoryAccumulator.
// Completed blocks are validated (limits from config) and published; any
// accumulator or validation error is written to stderr with the offending
// line, and the accumulator resets. Pose lines ("x y z") remain accepted
// during the transition and go to the existing PoseTargetMailbox.
void RunTargetInput(PoseTargetMailbox& pose_mailbox,
                    JointTrajectoryMailbox& traj_mailbox,
                    const std::atomic<bool>& stop, int input_fd);
// Pipe wrapper gains the same extra parameter:
void RunTargetInputFromPipe(PoseTargetMailbox&, JointTrajectoryMailbox&,
                            const std::atomic<bool>& stop,
                            const std::string& pipe_path);
```

- [ ] **Step 1: Write the failing test** — in the existing input-thread fixture, write a valid `TRAJ_BEGIN 2 … TRAJ_END` block into the pipe, join the thread via `stop`, assert `mailbox.Take()` returns a 2-point trajectory and a second `Take()` returns nullptr. Then write one valid block followed by a newer valid block; assert `Take()` yields the **newer** one (latest wins). Then write a block whose row count mismatches; assert `Take()` is nullptr (invalid never published) and that a subsequent valid pose line `"0.1 0.2 0.3"` still reaches the pose mailbox (grammar error does not kill the thread).
- [ ] **Step 2: Run to verify failure** — `RunTargetInput` does not exist yet; keep the old entry points delegating to the new one so existing tests compile unchanged.
- [ ] **Step 3: Implement.** `Publish`: `delete slot_.exchange(raw)` is wrong (drops latest-wins ordering under race) — producer exchanges and deletes the *displaced* pointer itself: `delete slot_.exchange(traj.release())`. `Take`: `std::unique_ptr<JointTrajectory>(slot_.exchange(nullptr))`. Validation limits: read the joint position limits and velocity clips from `Config.h` (use the exact constant names found there — verify before writing; do not invent names).
- [ ] **Step 4: Build, run to green**, full suite green (existing `RunPoseTargetInputFromFd` callers updated).
- [ ] **Step 5: Commit** `git commit -m "controller: input thread accepts framed joint-trajectory blocks"`

---

### Task 4: JointTrajectorySource and the joint tracking branch

**Files:**
- Modify: `Christian_control/basic_control/src/State.h` (`Reference` gains `std::optional<JointReference> joint;` where `JointReference { Eigen::Matrix<double,7,1> q_rad; Eigen::Matrix<double,7,1> qdot_rad_s; }`)
- Create: `JointTrajectorySource` in `Christian_control/basic_control/src/Targets.h/.cpp`
- Modify: `Christian_control/basic_control/src/Controller.cpp/.h`
- Modify: `Christian_control/basic_control/src/Config.h` — add:
```cpp
// Joint-space tracking gain on (q_ref - q_meas), 1/s.
inline constexpr double kKpJointTracking = 5.0;
// A trajectory whose first point is farther than this from the measured
// position (any joint) is rejected at activation — the splice guard.
inline constexpr double kTrajStartToleranceDeg = 2.0;
// Joint-space following-error stop: measured vs reference, any joint.
inline constexpr double kTrajFollowingErrorStopDeg = 8.0;
```
- Test: extend `Christian_control/basic_control/tests/test_control_logic.cpp`

**Interfaces:**
- Consumes: `SampleJointTrajectory` (Task 2), `JointTrajectoryMailbox::Take` (Task 3).
- Produces:
```cpp
// Startup: holds the measured takeover q (zero-velocity joint reference).
// Each RT cycle: Take() a newly published trajectory; if its first point is
// within kTrajStartToleranceDeg of q_meas on every joint, activate it and
// start its clock at 0; otherwise report the rejection in ControllerStatus
// and keep holding. While active, Get returns the Hermite sample; on
// complete, holds the final point. Never blocks, never allocates (Take is
// an exchange; rejection deletes the bounded block — same accepted
// exception as Task 3).
class JointTrajectorySource {
public:
    JointTrajectorySource(Eigen::Matrix<double, 7, 1> hold_q_rad,
                          JointTrajectoryMailbox& mailbox);
    Reference Get(const RobotState& state, double dt_s, ControllerStatus& status);
};
// Controller: when reference.joint is set, the command is
//   qdot_cmd = qdot_ref + config::kKpJointTracking * (q_ref - q_meas)
// flowing into the existing clip -> integration path. If any
// |q_ref - q_meas| exceeds kTrajFollowingErrorStopDeg, the controller
// requests stop through the existing stop policy (do not invent a new stop
// path — use the same mechanism the Cartesian following-error rule uses;
// read Safety.h/StopPriority.h first).
```

- [ ] **Step 1: Write the failing tests** (hardware-free, synthetic `RobotState`): (a) with no trajectory published, `Get` returns a joint reference equal to the hold pose with zero velocity; (b) publish a trajectory starting at the measured q → activated; sampling advances with accumulated `dt_s`; after the final time the reference holds the last point and `status` reports arrival; (c) publish a trajectory starting 5° away → rejected, hold reference unchanged, status carries the rejection; (d) controller math: given `reference.joint` with `q_ref − q_meas = 0.1 rad` on joint 1 and `qdot_ref = 0`, the pre-clip command is `0.1 * kKpJointTracking` rad/s on joint 1, zero elsewhere; (e) following-error: `q_ref − q_meas` of 10° triggers the stop request.
- [ ] **Step 2: Run to verify failures** (types/methods missing).
- [ ] **Step 3: Implement** source and controller branch. The Cartesian branch stays untouched and is selected exactly as before when `reference.pose` is set; `Reference` carries one or the other, never both.
- [ ] **Step 4: Build, run to green**, full suite green.
- [ ] **Step 5: Commit** `git commit -m "controller: joint trajectory source and tracking law behind existing clips"`

---

### Task 5: Bridge emits the timed block

**Files:**
- Create: `Christian_control/planner_bridge/src/TrajectoryEmit.h/.cpp`
- Modify: `Christian_control/planner_bridge/src/BridgeMain.cpp`
- Modify: `Christian_control/planner_bridge/CMakeLists.txt` (add `TrajectoryEmit.cpp` to `bridge_core`)
- Test: extend `Christian_control/planner_bridge/tests/test_bridge_main.cpp` (it already compiles controller sources — add `JointTrajectory.cpp` to its source list and parse the bridge's output with the controller's own accumulator: the wire contract is then tested end-to-end)

**Interfaces:**
- Consumes: `TrajectoryResult::trajectory_pos/trajectory_vel` (`TrajectoryGeneration/include/utils.h:83`), solve timing from `PlanSolver.cpp` (`total_time_sec`, exposed as below), `JointTrajectoryAccumulator` (Task 1) in the test.
- Produces:
```cpp
// Formats the validated solve as the TRAJ_BEGIN/TRAJ_END block, radians in,
// degrees out, t_i = i * (total_time_sec / (n_states - 1)). Returns the
// full block text; writes nothing itself (RunBridge owns buffering).
std::string FormatTrajectoryBlock(const std::vector<gtsam::Vector>& pos_rad,
                                  const std::vector<gtsam::Vector>& vel_rad_s,
                                  double total_time_sec);
// PlanSolver.h: PlanOutcome gains `double total_time_sec;` filled by
// SolveToPosition from its existing local of the same name.
```

- [ ] **Step 1: Write the failing test** — run `RunBridge` with the existing proven-solvable goal/start; feed every emitted line to a `JointTrajectoryAccumulator`; assert exactly one complete block, `points.size() >= 2`, first `t_s == 0`, strictly increasing `t_s`, and `ValidateJointTrajectory` under 45 deg/s clips passes; assert the first point matches the start state within 1e-6 rad. Keep the existing waypoint assertions for now on the `--emit-waypoints` flag (added this task as the transition escape hatch, default OFF).
- [ ] **Step 2: Run to verify failure** (block absent — output is still waypoint lines).
- [ ] **Step 3: Implement** `FormatTrajectoryBlock`, plumb `total_time_sec` through `PlanOutcome`, switch `RunBridge`'s default output to the block (`--emit-waypoints` restores the old lines; `--emit-orientation` becomes meaningful only with `--emit-waypoints`).
- [ ] **Step 4: Build both projects, run both full ctest suites to green.**
- [ ] **Step 5: Commit** `git commit -m "bridge: emit timed joint-trajectory block; waypoints behind a flag"`

---

### Task 6: Wire Main, retire the Cartesian waypoint path — GATED

**Do not start until Christian confirms a supervised hardware run of Tasks 1–5 succeeded.**

**Files:**
- Modify: `Christian_control/basic_control/src/Main.cpp` (construct `JointTrajectoryMailbox` + `JointTrajectorySource` with the measured takeover q; startup banner and CSV preamble say `startup_hold = measured_q`)
- Delete: `TrajectoryProfile.h`, `PoseTargetSource` + `PoseTargetMailbox` + pose-line parsing in `Targets.h/.cpp`, `tests/test_trajectory_profile.cpp`, pose-target cases in `test_control_logic.cpp`, `Waypoints.cpp` sampling + `--emit-waypoints`/`--emit-orientation` in the bridge (bridge `Waypoints.cpp` retains only `ValidateJointPath`— rename file to `PathValidation.cpp` in the same commit so the name matches the job)
- Modify: `Christian_control/basic_control/README.md`, `Christian_control/docs/decisions/` — new record `stage2-joint-trajectory-following.md` stating what replaced what and why, superseding the relevant parts of `stage15-bridge-workflow.md`; note explicitly that this deliberately reinstates planner-owned timing, reversing the 2026-08-04 playback removal, with the splice guard + on-receipt validation as the new safety argument.

- [ ] **Step 1:** Wire `Main.cpp`, run full suites of both projects to green.
- [ ] **Step 2:** Delete the listed code, fix compilation, run both suites to green.
- [ ] **Step 3:** Update README + decision record.
- [ ] **Step 4:** Commit in two commits: `"controller: joint-trajectory source is the only motion path"` then `"docs: stage 2 decision record"`.

## Verification at the end of Tasks 1–5 (pre-hardware)

- Both ctest suites green.
- Offline end-to-end: `planner_bridge --start-deg 0 0 0 0 0 0 0` emits a block the controller's own parser accepts and validates (proven by the Task 5 test).
- `run_session.sh --dry-run` still passes its gates (script unchanged — same pipe, same one-shot bridge invocation).
- First supervised run: goal a few centimetres from the arm's pose; expect visibly continuous motion with no per-waypoint stops.
