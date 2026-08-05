# Stage 1 Planner Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A hardware-free `planner_bridge` executable that solves a GPMP2 plan from the arm's last logged state to a goal position and emits ≤8 validated Cartesian waypoints as `x y z` lines that `basic_control`'s existing stdin target input accepts unchanged.

**Architecture:** New standalone CMake project `Christian_control/planner_bridge/` compiling the needed `TrajectoryGeneration/` sources directly (no root-build dependency, no Kortex linkage anywhere). The planner is constructed with a flipped base pose so it plans directly in `base_link` coordinates, and with a tool-matched DH table (d7 extended by the 0.12 m ConfiguredTool offset instead of the 0.14 m grasp site). Every solve is cross-checked waypoint-by-waypoint against the controller's independent `AnalyticalKinematics` FK before anything is emitted. The controller is NOT modified: transport is a FIFO redirected to its stdin, operated per the runbook.

**Tech Stack:** C++17, Eigen, GTSAM/GPMP2 (`third_party/lib`), Pinocchio libs (linked transitively via `utils.cpp`→`Jacobian.h`), system yaml-cpp, CTest.

## Global Constraints

- NEVER link `libKortexApiCpp.a` into any target of this plan; the bridge must build and run with no robot present.
- Never run `controller`, root `main`, or any Kortex-linked binary as a test step. Hardware trials are a separate supervised operation requiring Christian's explicit authorization (project CLAUDE.md).
- Units: joint angles radians internally, degrees only at the telemetry-CSV boundary (`meas_j*` columns are degrees, shifted to ±180°); positions metres; frame is right-arm `base_link`; joint order is Kortex actuator order 1..7.
- The DH↔URDF base transform is a fixed rotation of π about x (measured 2026-08-05, residual 0.49 mm max over 501 configs — see `Christian_control/docs/thesis/replanning-motivation.md` §7).
- Waypoint count per plan ≤ 8 (`PoseTargetMailbox::kCapacity`, `Targets.h:43`).
- Joint position validation limits (joints 2/4/6, degrees): |q2| ≤ 130, |q4| ≤ 145, |q6| ≤ 118 — the controller's `kJointLimitWarnDeg` (`Config.h:160`). Joints 1/3/5/7 are continuous, no position check.
- Compiler flags: `-Wall -Wextra`; fix warnings in new code, never silence globally.

## File Structure

```
Christian_control/planner_bridge/
  CMakeLists.txt              — standalone project, bridge + tests
  config/dh_params_tool.yaml  — DH copy with d7 = -0.2874 (tool-matched)
  src/PlannerModel.h/.cpp     — DH load, ArmModel with flipped base, FK
  src/WorldSdf.h/.cpp         — empty-world / one-box SDF construction
  src/PlanSolver.h/.cpp       — init + optimize wrapper → TrajectoryResult
  src/Waypoints.h/.cpp        — sample, validate, format target lines
  src/StartState.h/.cpp       — read meas_j* from controller run CSV
  src/BridgeMain.h/.cpp       — RunBridge(args…) — testable main body
  src/main.cpp                — thin wrapper calling RunBridge
  tests/test_planner_model.cpp
  tests/test_world_sdf.cpp
  tests/test_plan_solver.cpp
  tests/test_waypoints.cpp
  tests/test_start_state.cpp
  tests/test_bridge_main.cpp
Christian_control/docs/decisions/stage1-planner-bridge.md   (Task 8)
```

---

### Task 1: Project skeleton that builds and links everything

**Files:**
- Create: `Christian_control/planner_bridge/CMakeLists.txt`
- Create: `Christian_control/planner_bridge/src/main.cpp` (stub)
- Create: `Christian_control/planner_bridge/config/dh_params_tool.yaml`

**Interfaces:**
- Produces: a configured build directory where later tasks add sources/tests; the `dh_params_tool.yaml` path consumed by `LoadPlannerModel` (Task 2).

- [ ] **Step 1: Write the config file** — copy `TrajectoryGeneration/config/dh_params.yaml` verbatim EXCEPT the last joint's `d`:

```yaml
# Joint 7 entry only — all other entries copied unchanged from
# TrajectoryGeneration/config/dh_params.yaml
  - joint_id: 7
    joint_name: "Actuator7_to_configured_tool"
    joint_type: "continuous"
    a: 0.0
    alpha: 3.141592653589793
    d: -0.2874   # -0.1674 (flange) - 0.12 (ConfiguredTool_Link offset,
                 # read from ControlConfig::GetToolConfiguration 2026-08-05;
                 # matches basic_control's kRightEndEffectorFrame)
    theta_offset: 3.141592653589793
```

- [ ] **Step 2: Write the stub main**

```cpp
// src/main.cpp
#include <iostream>
int main() { std::cout << "planner_bridge stub\n"; return 0; }
```

- [ ] **Step 3: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
project(planner_bridge CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(TP_LIB "${HUMANSL_ROOT}/third_party/lib")

# TrajectoryGeneration sources compiled directly — this project must not
# depend on the (currently broken) root build. utils.cpp pulls
# TrajectoryExecution's Jacobian, hence Jacobian.cpp and Pinocchio below.
set(TG_SOURCES
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/utils.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/GenerateArmModel.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/TrajectoryInitiation.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/TrajectoryOptimization.cpp
    ${HUMANSL_ROOT}/TrajectoryExecution/src/Jacobian.cpp
)

add_library(bridge_core STATIC ${TG_SOURCES})
target_include_directories(bridge_core PUBLIC
    ${HUMANSL_ROOT}/TrajectoryGeneration/include
    ${HUMANSL_ROOT}/TrajectoryExecution/include
    ${HUMANSL_ROOT}/third_party/include
    ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(bridge_core PUBLIC
    ${TP_LIB}/libgpmp2.so
    ${TP_LIB}/libgtsam.so
    ${TP_LIB}/libmetis-gtsam.so
    ${TP_LIB}/libpinocchio_default.so
    yaml-cpp
    pthread
)
set_target_properties(bridge_core PROPERTIES BUILD_RPATH "${TP_LIB}")

add_executable(planner_bridge src/main.cpp)
target_link_libraries(planner_bridge PRIVATE bridge_core)
set_target_properties(planner_bridge PROPERTIES BUILD_RPATH "${TP_LIB}")

enable_testing()
```

- [ ] **Step 4: Configure and build from a fresh directory**

Run: `cmake -S Christian_control/planner_bridge -B Christian_control/planner_bridge/build && cmake --build Christian_control/planner_bridge/build -j"$(nproc)"`
Expected: builds; `./Christian_control/planner_bridge/build/planner_bridge` prints `planner_bridge stub`. If the linker reports missing GTSAM-dependent symbols (e.g. tbb or boost), inspect `ls third_party/lib` and add the specific `.so` to `target_link_libraries` — do not add `-Wl,--allow-shlib-undefined` here; the bridge actually calls this code at runtime, unlike basic_control's lazy-linked Pinocchio usage.

- [ ] **Step 5: Commit**

```bash
git add Christian_control/planner_bridge
git commit -m "planner_bridge: standalone build skeleton over TrajectoryGeneration"
```

---

### Task 2: PlannerModel — flipped base, tool-matched DH, FK cross-checked against the controller

**Files:**
- Create: `Christian_control/planner_bridge/src/PlannerModel.h`, `src/PlannerModel.cpp`
- Test: `Christian_control/planner_bridge/tests/test_planner_model.cpp`
- Modify: `Christian_control/planner_bridge/CMakeLists.txt` (add sources + test target)

**Interfaces:**
- Consumes: `createDHParams(const std::string&)` (`TrajectoryGeneration/include/utils.h:179`), `forwardKinematics(dh, q, base_pose)` (`utils.h:187`), `ArmModel::createArmModel(base_pose, dh)` (`GenerateArmModel.h:24`).
- Produces:

```cpp
// PlannerModel.h
#pragma once
#include <memory>
#include <string>
#include <Eigen/Dense>
#include "GenerateArmModel.h"
#include "utils.h"

// The DH base frame is rotated pi about x relative to base_link (measured
// 2026-08-05, thesis notes §7). Building the gpmp2 arm with this base pose
// makes every planner-side quantity come out directly in base_link.
gtsam::Pose3 DhBaseInBaseLink();

struct PlannerModel {
    DHParameters dh;
    gtsam::Pose3 base_pose;                      // = DhBaseInBaseLink()
    std::unique_ptr<gpmp2::ArmModel> arm_model;  // collision-sphere model
};

// yaml_path: config/dh_params_tool.yaml (d7 tool-matched, -0.2874).
PlannerModel LoadPlannerModel(const std::string& yaml_path);

// Tool position in base_link, metres. q_rad: Kortex actuator order.
Eigen::Vector3d ToolPositionInBaseLink(const PlannerModel& model,
                                       const Eigen::Matrix<double, 7, 1>& q_rad);
// Full pose counterpart (rotation used for the goal-pose prior).
gtsam::Pose3 ToolPoseInBaseLink(const PlannerModel& model,
                                const Eigen::Matrix<double, 7, 1>& q_rad);
```

- [ ] **Step 1: Write the failing test** — the acceptance criterion IS the model-agreement measurement, now as a permanent test:

```cpp
// tests/test_planner_model.cpp
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include "PlannerModel.h"
#include "AnalyticalKinematics.h"  // basic_control/tools — the controller's URDF chain

int main(int argc, char** argv) {
    assert(argc == 2 && "usage: test_planner_model <dh_params_tool.yaml>");
    const PlannerModel model = LoadPlannerModel(argv[1]);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-2.0, 2.0);
    double worst_mm = 0.0;
    for (int trial = 0; trial < 200; ++trial) {
        Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
        if (trial > 0)
            for (int j = 0; j < 7; ++j) q[j] = dist(rng);
        const Eigen::Vector3d p_dh = ToolPositionInBaseLink(model, q);
        const Eigen::Vector3d p_urdf = AnalyticalForwardKinematics(q).translation();
        worst_mm = std::max(worst_mm, (p_dh - p_urdf).norm() * 1000.0);
    }
    std::printf("worst tool-position disagreement: %.3f mm\n", worst_mm);
    assert(worst_mm < 1.0 && "DH and URDF chains must agree under 1 mm");
    return 0;
}
```

CMake addition (same file, after `enable_testing()`):

```cmake
target_sources(bridge_core PRIVATE src/PlannerModel.cpp)
add_executable(test_planner_model
    tests/test_planner_model.cpp
    ${HUMANSL_ROOT}/Christian_control/basic_control/tools/AnalyticalKinematics.cpp
)
target_include_directories(test_planner_model PRIVATE
    src ${HUMANSL_ROOT}/Christian_control/basic_control/tools)
target_link_libraries(test_planner_model PRIVATE bridge_core)
set_target_properties(test_planner_model PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME planner_model COMMAND test_planner_model
         ${CMAKE_CURRENT_SOURCE_DIR}/config/dh_params_tool.yaml)
```

- [ ] **Step 2: Run to verify it fails** — `cmake --build … && ctest --test-dir … -R planner_model` → FAIL: `PlannerModel.h` does not exist yet (compile error is the expected failure mode here).

- [ ] **Step 3: Implement**

```cpp
// PlannerModel.cpp
#include "PlannerModel.h"
#include <cmath>

gtsam::Pose3 DhBaseInBaseLink() {
    return gtsam::Pose3(gtsam::Rot3::Rx(M_PI), gtsam::Point3(0, 0, 0));
}

PlannerModel LoadPlannerModel(const std::string& yaml_path) {
    PlannerModel model;
    model.dh = createDHParams(yaml_path);
    model.base_pose = DhBaseInBaseLink();
    ArmModel factory;
    model.arm_model = factory.createArmModel(model.base_pose, model.dh);
    return model;
}

gtsam::Pose3 ToolPoseInBaseLink(const PlannerModel& model,
                                const Eigen::Matrix<double, 7, 1>& q_rad) {
    return forwardKinematics(model.dh, gtsam::Vector(q_rad), model.base_pose);
}

Eigen::Vector3d ToolPositionInBaseLink(const PlannerModel& model,
                                       const Eigen::Matrix<double, 7, 1>& q_rad) {
    return ToolPoseInBaseLink(model, q_rad).translation();
}
```

- [ ] **Step 4: Run to verify it passes** — expected `worst tool-position disagreement:` ≤ ~0.5 mm, PASS. If it reports ~metres, the base flip direction is wrong — check `Rx(M_PI)` against the zero-config sign pattern in thesis notes §7 before touching anything else.

- [ ] **Step 5: Commit** — `git commit -m "planner_bridge: PlannerModel with base flip, FK agreement test vs controller chain"`

---

### Task 3: WorldSdf — empty world and one analytic box

**Files:**
- Create: `src/WorldSdf.h`, `src/WorldSdf.cpp`; Test: `tests/test_world_sdf.cpp`; Modify: `CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
// WorldSdf.h
#pragma once
#include <optional>
#include <Eigen/Dense>
#include <gpmp2/obstacle/SignedDistanceField.h>

struct AxisAlignedBox {           // metres, base_link
    Eigen::Vector3d center;
    Eigen::Vector3d half_extent;
};

// Grid covering the right-arm workspace: origin (-1.2,-1.2,-0.4),
// cell 0.04 m, 60x60x40 cells. Cells hold distance to the box surface
// (negative inside); with no box, a uniform large free distance.
gpmp2::SignedDistanceField MakeWorldSdf(const std::optional<AxisAlignedBox>& box);
```

- [ ] **Step 1: Write the failing test**

```cpp
// tests/test_world_sdf.cpp
#include <cassert>
#include <cmath>
#include "WorldSdf.h"
int main() {
    const auto empty = MakeWorldSdf(std::nullopt);
    assert(empty.getSignedDistance(gtsam::Point3(0.4, 0.0, 0.4)) > 1.0);

    AxisAlignedBox box{{0.4, 0.0, 0.4}, {0.1, 0.1, 0.1}};
    const auto world = MakeWorldSdf(box);
    assert(world.getSignedDistance(gtsam::Point3(0.4, 0.0, 0.4)) < 0.0);   // inside
    const double near = world.getSignedDistance(gtsam::Point3(0.4, 0.0, 0.55));
    assert(near > 0.0 && near < 0.1);                                      // 5 cm off face
    assert(world.getSignedDistance(gtsam::Point3(-0.8, -0.8, 0.0)) > 0.5); // far
    return 0;
}
```

CMake: `target_sources(bridge_core PRIVATE src/WorldSdf.cpp)`, add `test_world_sdf` executable + `add_test(NAME world_sdf …)` following the exact pattern of `test_planner_model` (include dir `src`, link `bridge_core`, `BUILD_RPATH "${TP_LIB}"`).

- [ ] **Step 2: Run to verify it fails** (compile error: header missing).

- [ ] **Step 3: Implement**

```cpp
// WorldSdf.cpp
#include "WorldSdf.h"
#include <vector>
#include <gtsam/base/Matrix.h>

namespace {
double BoxSignedDistance(const Eigen::Vector3d& p, const AxisAlignedBox& box) {
    const Eigen::Vector3d d = (p - box.center).cwiseAbs() - box.half_extent;
    const Eigen::Vector3d outside = d.cwiseMax(0.0);
    const double inside = std::min(d.maxCoeff(), 0.0);
    return outside.norm() + inside;   // standard AABB SDF
}
}  // namespace

gpmp2::SignedDistanceField MakeWorldSdf(const std::optional<AxisAlignedBox>& box) {
    const gtsam::Point3 origin(-1.2, -1.2, -0.4);
    const double cell = 0.04;
    const int nx = 60, ny = 60, nz = 40;
    // gpmp2 layout: one z-slice per matrix; matrix rows = y, cols = x.
    std::vector<gtsam::Matrix> field(nz, gtsam::Matrix(ny, nx));
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const Eigen::Vector3d p = origin + Eigen::Vector3d(i, j, k) * cell;
                field[k](j, i) = box ? BoxSignedDistance(p, *box) : 10.0;
            }
    return gpmp2::SignedDistanceField(origin, cell, field);
}
```

- [ ] **Step 4: Run to verify it passes.** If `getSignedDistance` disagrees with the analytic value near the box, the row/col-vs-x/y convention is flipped — check `third_party/include/gpmp2/obstacle/SignedDistanceField.h`'s accessor before changing the loop.

- [ ] **Step 5: Commit** — `git commit -m "planner_bridge: analytic empty/box world SDF"`

---

### Task 4: PlanSolver — init + optimize from measured state to goal position

**Files:**
- Create: `src/PlanSolver.h`, `src/PlanSolver.cpp`; Test: `tests/test_plan_solver.cpp`; Modify: `CMakeLists.txt` (same pattern)

**Interfaces:**
- Consumes: `PlannerModel`, `MakeWorldSdf`; `InitializeTrajectory(dh).initJointTrajectoryFromTarget(start_conf, end_pose, base_pose, total_time_step)` (`TrajectoryInitiation.h`); `optimizeJointTrajectory(arm_model, sdf, init_values, target_pose, start_config, pos_limits, vel_limits, total_time_step, total_time_sec)` (`GenerateTrajectory.h:32`); `createJointLimits(path)` (`utils.h:181`) with `TrajectoryGeneration/config/joint_limits.yaml`.
- Produces:

```cpp
// PlanSolver.h
#pragma once
#include <string>
#include "PlannerModel.h"
#include "WorldSdf.h"

struct PlanRequest {
    Eigen::Matrix<double, 7, 1> q_start_rad;  // Kortex order
    Eigen::Vector3d goal_position_m;          // base_link
    std::optional<AxisAlignedBox> obstacle;
};

struct PlanOutcome {
    bool ok = false;
    std::string error;                 // set when !ok
    TrajectoryResult result;           // trajectory_pos: radians, Kortex order
    double final_goal_error_m = 0.0;   // FK(last waypoint) vs requested goal
};

// joint_limits_yaml: TrajectoryGeneration/config/joint_limits.yaml.
// Goal orientation = tool orientation at q_start (the controller is
// position-only and preserves takeover orientation; the pose prior is soft).
PlanOutcome SolveToPosition(const PlannerModel& model, const PlanRequest& request,
                            const std::string& joint_limits_yaml);
```

- [ ] **Step 1: Write the failing test**

```cpp
// tests/test_plan_solver.cpp
#include <cassert>
#include <cstdio>
#include "PlanSolver.h"

int main(int argc, char** argv) {
    assert(argc == 3 && "usage: test_plan_solver <dh_tool.yaml> <joint_limits.yaml>");
    const PlannerModel model = LoadPlannerModel(argv[1]);
    PlanRequest request;
    request.q_start_rad = Eigen::Matrix<double, 7, 1>::Zero();
    const Eigen::Vector3d start = ToolPositionInBaseLink(model, request.q_start_rad);
    request.goal_position_m = start + Eigen::Vector3d(0.15, 0.10, -0.10);
    const PlanOutcome outcome = SolveToPosition(model, request, argv[2]);
    assert(outcome.ok && "solve must succeed in an empty world");
    assert(!outcome.result.trajectory_pos.empty());
    // First support state is the start; last reaches the goal.
    assert((outcome.result.trajectory_pos.front() -
            gtsam::Vector(request.q_start_rad)).norm() < 1e-3);
    std::printf("final goal error: %.1f mm, %lld ms solve\n",
                outcome.final_goal_error_m * 1000.0,
                static_cast<long long>(outcome.result.optimization_duration.count()));
    assert(outcome.final_goal_error_m < 0.03 && "goal within 3 cm");
    return 0;
}
```

CMake: add `test_plan_solver` (same pattern), `add_test(NAME plan_solver COMMAND test_plan_solver <dh_params_tool.yaml path> ${HUMANSL_ROOT}/TrajectoryGeneration/config/joint_limits.yaml)` using `${CMAKE_CURRENT_SOURCE_DIR}`/`${HUMANSL_ROOT}` for the two paths.

- [ ] **Step 2: Run to verify it fails** (header missing).

- [ ] **Step 3: Implement**

```cpp
// PlanSolver.cpp
#include "PlanSolver.h"
#include <algorithm>
#include "GenerateTrajectory.h"
#include "TrajectoryInitiation.h"

PlanOutcome SolveToPosition(const PlannerModel& model, const PlanRequest& request,
                            const std::string& joint_limits_yaml) {
    PlanOutcome outcome;
    try {
        const auto [pos_limits, vel_limits] = createJointLimits(joint_limits_yaml);
        const gtsam::Pose3 start_pose = ToolPoseInBaseLink(model, request.q_start_rad);
        const gtsam::Pose3 goal_pose(start_pose.rotation(),
                                     gtsam::Point3(request.goal_position_m));
        // Slow by construction: Stage 1 plans are executed via profiled
        // point-to-point moves, so plan duration only shapes the solve.
        const double distance_m =
            (request.goal_position_m - start_pose.translation()).norm();
        const size_t total_time_step = 20;
        const double total_time_sec = std::max(4.0, distance_m / 0.05);

        InitializeTrajectory initializer(model.dh);
        const gtsam::Values init_values = initializer.initJointTrajectoryFromTarget(
            gtsam::Vector(request.q_start_rad), goal_pose, model.base_pose,
            total_time_step);
        const auto sdf = MakeWorldSdf(request.obstacle);
        outcome.result = optimizeJointTrajectory(
            *model.arm_model, sdf, init_values, goal_pose,
            gtsam::Vector(request.q_start_rad), pos_limits, vel_limits,
            total_time_step, total_time_sec);
        if (outcome.result.trajectory_pos.empty()) {
            outcome.error = "optimizer returned an empty trajectory";
            return outcome;
        }
        Eigen::Matrix<double, 7, 1> q_final(outcome.result.trajectory_pos.back());
        outcome.final_goal_error_m =
            (ToolPositionInBaseLink(model, q_final) - request.goal_position_m).norm();
        outcome.ok = true;
    } catch (const std::exception& exception) {
        outcome.error = exception.what();
    }
    return outcome;
}
```

- [ ] **Step 4: Run to verify it passes.** This is also the gpmp2-internal-FK spot check: the goal factor uses gpmp2's own `Arm` FK while `final_goal_error_m` is computed with `utils.cpp` DH FK — a pass proves they agree. If the 3 cm assert fails with a small consistent offset, print `ToolPositionInBaseLink` vs `createPoseFromConf(*model.arm_model, q_final).translation()` (`utils.h:152`) to see which side moved; do not loosen the tolerance without understanding the source.

- [ ] **Step 5: Commit** — `git commit -m "planner_bridge: PlanSolver wraps init + GPMP2 optimize, goal-error checked"`

---

### Task 5: Waypoints — sample, validate, format

**Files:**
- Create: `src/Waypoints.h`, `src/Waypoints.cpp`; Test: `tests/test_waypoints.cpp`; Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `PlannerModel`, `ToolPositionInBaseLink`; `ParsePoseTarget(line, error)` (`basic_control/src/Targets.h:34`) in the test only.
- Produces:

```cpp
// Waypoints.h
#pragma once
#include <optional>
#include <string>
#include <vector>
#include "PlannerModel.h"

// Controller-side acceptance rules (Config.h:160 kJointLimitWarnDeg):
// joints 2/4/6 within ±130/145/118 deg; 1/3/5/7 continuous. Returns an
// error description, or nullopt when every support state passes.
std::optional<std::string> ValidateJointPath(
    const std::vector<gtsam::Vector>& trajectory_pos);

// Cartesian tool positions of the support states, thinned to at most
// max_count points at least min_spacing_m apart. The final point is
// always included; the first (current position) is always dropped.
std::vector<Eigen::Vector3d> SampleCartesianWaypoints(
    const PlannerModel& model, const std::vector<gtsam::Vector>& trajectory_pos,
    std::size_t max_count = 8, double min_spacing_m = 0.05);

// "x y z" with 6 decimals — the exact grammar ParsePoseTarget accepts.
std::string FormatTargetLine(const Eigen::Vector3d& position_m);
```

- [ ] **Step 1: Write the failing test**

```cpp
// tests/test_waypoints.cpp
#include <cassert>
#include "Waypoints.h"
#include "Targets.h"  // basic_control — ParsePoseTarget round-trip

int main(int argc, char** argv) {
    assert(argc == 2);
    const PlannerModel model = LoadPlannerModel(argv[1]);

    // Straight-line joint path: q2 sweeps 0 -> 1 rad over 21 support states.
    std::vector<gtsam::Vector> path;
    for (int i = 0; i <= 20; ++i) {
        gtsam::Vector q = gtsam::Vector::Zero(7);
        q(1) = 0.05 * i;
        path.push_back(q);
    }
    assert(!ValidateJointPath(path).has_value());

    const auto waypoints = SampleCartesianWaypoints(model, path);
    assert(!waypoints.empty() && waypoints.size() <= 8);
    for (std::size_t i = 1; i < waypoints.size(); ++i)
        assert((waypoints[i] - waypoints[i - 1]).norm() >= 0.05 - 1e-9);
    Eigen::Matrix<double, 7, 1> q_last(path.back());
    assert((waypoints.back() - ToolPositionInBaseLink(model, q_last)).norm() < 1e-9);

    // Limit violation: q4 at 150 deg exceeds the 145 deg warn limit.
    auto bad = path;
    bad.back()(3) = 150.0 * M_PI / 180.0;
    assert(ValidateJointPath(bad).has_value());

    // Every emitted line must be accepted verbatim by the controller parser.
    for (const auto& waypoint : waypoints) {
        std::string error;
        assert(ParsePoseTarget(FormatTargetLine(waypoint), error).has_value());
    }
    return 0;
}
```

CMake: `test_waypoints` additionally compiles `${HUMANSL_ROOT}/Christian_control/basic_control/src/Targets.cpp` and adds include dir `${HUMANSL_ROOT}/Christian_control/basic_control/src`. If `Targets.cpp` drags in a controller-only header that won't compile standalone, STOP and report — do not copy the parser; the round-trip against the real parser is the point of this test.

- [ ] **Step 2: Run to verify it fails** (header missing).

- [ ] **Step 3: Implement**

```cpp
// Waypoints.cpp
#include "Waypoints.h"
#include <cmath>
#include <cstdio>

std::optional<std::string> ValidateJointPath(
    const std::vector<gtsam::Vector>& trajectory_pos) {
    constexpr double kLimitDeg[7] = {0, 130.0, 0, 145.0, 0, 118.0, 0};
    for (std::size_t s = 0; s < trajectory_pos.size(); ++s)
        for (int j : {1, 3, 5}) {
            const double q_deg = trajectory_pos[s](j) * 180.0 / M_PI;
            if (std::abs(q_deg) > kLimitDeg[j]) {
                char buffer[128];
                std::snprintf(buffer, sizeof buffer,
                              "support state %zu: joint %d at %.1f deg exceeds ±%.0f",
                              s, j + 1, q_deg, kLimitDeg[j]);
                return std::string(buffer);
            }
        }
    return std::nullopt;
}

std::vector<Eigen::Vector3d> SampleCartesianWaypoints(
    const PlannerModel& model, const std::vector<gtsam::Vector>& trajectory_pos,
    std::size_t max_count, double min_spacing_m) {
    std::vector<Eigen::Vector3d> waypoints;
    if (trajectory_pos.size() < 2) return waypoints;
    Eigen::Matrix<double, 7, 1> q(trajectory_pos.front());
    Eigen::Vector3d last_kept = ToolPositionInBaseLink(model, q);
    for (std::size_t s = 1; s + 1 < trajectory_pos.size(); ++s) {
        q = trajectory_pos[s];
        const Eigen::Vector3d p = ToolPositionInBaseLink(model, q);
        if ((p - last_kept).norm() >= min_spacing_m &&
            waypoints.size() + 1 < max_count) {   // reserve one slot for the goal
            waypoints.push_back(p);
            last_kept = p;
        }
    }
    q = trajectory_pos.back();
    waypoints.push_back(ToolPositionInBaseLink(model, q));
    return waypoints;
}

std::string FormatTargetLine(const Eigen::Vector3d& position_m) {
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%.6f %.6f %.6f",
                  position_m.x(), position_m.y(), position_m.z());
    return std::string(buffer);
}
```

- [ ] **Step 4: Run to verify it passes.**

- [ ] **Step 5: Commit** — `git commit -m "planner_bridge: waypoint sampling, warn-limit validation, target-line round-trip"`

---

### Task 6: StartState — last measured configuration from the controller's run CSV

**Files:**
- Create: `src/StartState.h`, `src/StartState.cpp`; Test: `tests/test_start_state.cpp`; Modify: `CMakeLists.txt` (same pattern; no extra deps)

**Interfaces:**
- Consumes: the log_format 8 CSV written by `basic_control` (`Hardware.cpp:361` `WriteCsvHeader`): columns `meas_j1..meas_j7`, degrees shifted to ±180°.
- Produces:

```cpp
// StartState.h
#pragma once
#include <optional>
#include <string>
#include <Eigen/Dense>

struct StartStateResult {
    Eigen::Matrix<double, 7, 1> q_rad;  // Kortex order
    std::string error;                  // set when read failed
};

// Reads the header to locate meas_j1..meas_j7 by NAME (column positions
// are not stable across log_format revisions), then returns the last
// complete data row converted to radians. nullopt value + error on any
// missing column, short row, or non-finite value.
std::optional<Eigen::Matrix<double, 7, 1>> ReadLatestMeasuredQ(
    const std::string& csv_path, std::string& error);
```

- [ ] **Step 1: Write the failing test**

```cpp
// tests/test_start_state.cpp
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include "StartState.h"

int main() {
    const char* path = "test_start_state_tmp.csv";
    {
        std::ofstream csv(path);
        csv << "time_s,dt_s,meas_j1,extra,meas_j2,meas_j3,meas_j4,"
               "meas_j5,meas_j6,meas_j7\n";
        csv << "0.001,0.002,10,99,20,30,40,50,60,70\n";
        csv << "0.003,0.002,11,99,21,31,41,51,61,71\n";
        csv << "0.005,0.002,12,99";  // torn final row must be ignored
    }
    std::string error;
    const auto q = ReadLatestMeasuredQ(path, error);
    assert(q.has_value() && error.empty());
    assert(std::abs((*q)[0] - 11.0 * M_PI / 180.0) < 1e-12);
    assert(std::abs((*q)[6] - 71.0 * M_PI / 180.0) < 1e-12);

    const auto missing = ReadLatestMeasuredQ("does_not_exist.csv", error);
    assert(!missing.has_value() && !error.empty());
    std::remove(path);
    return 0;
}
```

- [ ] **Step 2: Run to verify it fails** (header missing).

- [ ] **Step 3: Implement**

```cpp
// StartState.cpp
#include "StartState.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

namespace {
std::vector<std::string> SplitCsv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}
}  // namespace

std::optional<Eigen::Matrix<double, 7, 1>> ReadLatestMeasuredQ(
    const std::string& csv_path, std::string& error) {
    error.clear();
    std::ifstream csv(csv_path);
    if (!csv) { error = "cannot open " + csv_path; return std::nullopt; }

    std::string line;
    if (!std::getline(csv, line)) { error = "empty file"; return std::nullopt; }
    const auto header = SplitCsv(line);
    std::array<int, 7> column{};
    int highest = 0;
    for (int j = 0; j < 7; ++j) {
        const std::string name = "meas_j" + std::to_string(j + 1);
        const auto it = std::find(header.begin(), header.end(), name);
        if (it == header.end()) { error = "missing column " + name; return std::nullopt; }
        column[j] = static_cast<int>(it - header.begin());
        highest = std::max(highest, column[j]);
    }

    std::optional<Eigen::Matrix<double, 7, 1>> latest;
    while (std::getline(csv, line)) {
        const auto fields = SplitCsv(line);
        if (static_cast<int>(fields.size()) <= highest) continue;  // torn row
        Eigen::Matrix<double, 7, 1> q_deg;
        bool valid = true;
        for (int j = 0; j < 7 && valid; ++j) {
            try { q_deg[j] = std::stod(fields[column[j]]); }
            catch (const std::exception&) { valid = false; }
            if (valid && !std::isfinite(q_deg[j])) valid = false;
        }
        if (valid) latest = q_deg * (M_PI / 180.0);
    }
    if (!latest) error = "no complete data row";
    return latest;
}
```

- [ ] **Step 4: Run to verify it passes.**

- [ ] **Step 5: Commit** — `git commit -m "planner_bridge: start state from telemetry CSV by column name"`

---

### Task 7: BridgeMain — CLI wiring and offline end-to-end test

**Files:**
- Create: `src/BridgeMain.h`, `src/BridgeMain.cpp`; Modify: `src/main.cpp`; Test: `tests/test_bridge_main.cpp`; Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: everything above.
- Produces:

```cpp
// BridgeMain.h
#pragma once
#include <iosfwd>
#include <string>
#include <vector>

// Runs one plan: resolves the start state, solves, validates, and writes
// one target line per waypoint to `targets` (the stream the operator
// connects to the controller's stdin). Diagnostics go to `diagnostics`
// (stderr in main). Returns a process exit code: 0 emitted, 1 bad
// arguments, 2 start-state unavailable, 3 solve failed, 4 validation
// rejected the plan. NOTHING is written to `targets` on any non-zero path.
int RunBridge(const std::vector<std::string>& args, std::ostream& targets,
              std::ostream& diagnostics);
```

CLI grammar (document in `--help` text inside `RunBridge`):

```
planner_bridge --goal X Y Z (--state-csv PATH | --start-deg J1..J7)
               [--dh PATH] [--joint-limits PATH]
               [--box CX CY CZ HX HY HZ]
```

Defaults: `--dh` → `config/dh_params_tool.yaml` resolved relative to the executable's directory via `/proc/self/exe`; `--joint-limits` → `TrajectoryGeneration/config/joint_limits.yaml` resolved the same way (`../../..` up to the repo root from `build/`). Both stated in `--help`.

- [ ] **Step 1: Write the failing test**

```cpp
// tests/test_bridge_main.cpp
#include <cassert>
#include <sstream>
#include "BridgeMain.h"
#include "Targets.h"

int main(int argc, char** argv) {
    assert(argc == 3 && "usage: test_bridge_main <dh_tool.yaml> <joint_limits.yaml>");
    std::ostringstream targets, diagnostics;
    const std::vector<std::string> args = {
        "--goal", "0.20", "0.35", "0.90",
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2]};
    const int exit_code = RunBridge(args, targets, diagnostics);
    assert(exit_code == 0);

    std::istringstream lines(targets.str());
    std::string line, error;
    int count = 0;
    while (std::getline(lines, line)) {
        assert(ParsePoseTarget(line, error).has_value());
        ++count;
    }
    assert(count >= 1 && count <= 8);

    // Bad arguments produce exit code 1 and NO target output.
    std::ostringstream empty_targets, ignored;
    assert(RunBridge({"--goal", "not-a-number"}, empty_targets, ignored) == 1);
    assert(empty_targets.str().empty());
    return 0;
}
```

CMake: `test_bridge_main` compiles `BridgeMain.cpp` via `target_sources(bridge_core PRIVATE src/BridgeMain.cpp)` and, like `test_waypoints`, additionally compiles `${HUMANSL_ROOT}/Christian_control/basic_control/src/Targets.cpp` with include dir `${HUMANSL_ROOT}/Christian_control/basic_control/src`.

Note on the goal point: `(0.20, 0.35, 0.90)` must be reachable from zero config — before finalizing the test, print `ToolPositionInBaseLink(model, 0)` once and pick a goal ~15–20 cm from it; adjust the literal here and re-commit if needed.

- [ ] **Step 2: Run to verify it fails** (header missing).

- [ ] **Step 3: Implement** — `RunBridge` parses args positionally after each flag (`std::stod` in try/catch → exit 1 with usage on `diagnostics`); start state from `--start-deg` (degrees→radians) or `ReadLatestMeasuredQ` (exit 2 on failure); `LoadPlannerModel` + `SolveToPosition` (exit 3, error text to diagnostics); `ValidateJointPath` (exit 4); `SampleCartesianWaypoints` + `FormatTargetLine` lines to `targets`, one per waypoint, then a diagnostics summary: waypoint count, solve ms (`result.optimization_duration`), final goal error mm. `main.cpp` becomes:

```cpp
#include <iostream>
#include <vector>
#include "BridgeMain.h"
int main(int argc, char** argv) {
    return RunBridge(std::vector<std::string>(argv + 1, argv + argc),
                     std::cout, std::cerr);
}
```

- [ ] **Step 4: Run the whole suite** — `ctest --test-dir Christian_control/planner_bridge/build` → all 6 tests PASS. Also run the built binary once by hand with `--start-deg 0 0 0 0 0 0 0 --goal …` and eyeball the stderr summary (this runs no robot code — Global Constraints).

- [ ] **Step 5: Commit** — `git commit -m "planner_bridge: CLI main, offline end-to-end target emission"`

---

### Task 8: Decision record, README, and the supervised hardware runbook

**Files:**
- Create: `Christian_control/docs/decisions/stage1-planner-bridge.md`
- Modify: `Christian_control/basic_control/README.md` (new "Planner bridge (Stage 1)" section near the existing target-input documentation)

**Interfaces:** none — documentation of what Tasks 1–7 built, validated against the final source.

- [ ] **Step 1: Write the decision record** covering: why a separate process (safety kernel untouched — cite the replanning thesis notes §5); the base-flip and tool-matching decisions with the measured numbers (§7 of the thesis notes); waypoint transport via stdin FIFO and its Stage 1 limitation (reactive joint path between Cartesian waypoints is not the planned joint path — margins must cover it); the ≤8 waypoint cap (mailbox capacity); exit-code contract of `RunBridge`.

- [ ] **Step 2: Write the runbook section** in the README:

```markdown
## Planner bridge (Stage 1) — supervised runs only

Offline check (no robot, safe anytime):
    ./planner_bridge --start-deg 0 0 0 0 0 0 0 --goal 0.2 0.35 0.9

Hardware run (requires explicit authorization, operator present, e-stop
in reach; the controller consumes bridge waypoints only after reaching
its compiled terminal target):
    mkfifo /tmp/bridge_targets
    ./controller --log < /tmp/bridge_targets        # terminal 1
    ./planner_bridge --state-csv <today's run csv> \
        --goal <x y z> > /tmp/bridge_targets        # terminal 2, repeat per replan
Stop-and-replan: wait for hold (arrival messages in terminal 1), then
re-run the bridge with a new goal; each run reads the fresh CSV state.
```

- [ ] **Step 3: Verify every cited name, path, flag, and value** against the built source (workflow step 7 for documentation).

- [ ] **Step 4: Commit** — `git commit -m "docs: stage1 planner bridge decision record and runbook"`

---

## Verification of the whole plan (run after Task 8)

1. `cmake --build Christian_control/planner_bridge/build -j"$(nproc)"` — zero warnings in new files.
2. `ctest --test-dir Christian_control/planner_bridge/build` — 6/6 PASS (none touch hardware).
3. Confirm `grep -r Kortex Christian_control/planner_bridge/` matches nothing.
4. Hardware validation (arm actually tracking bridge waypoints) is PENDING and requires Christian's explicit per-run authorization — report it as such, never as done.
