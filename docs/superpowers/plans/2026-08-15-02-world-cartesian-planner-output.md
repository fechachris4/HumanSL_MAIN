# World-aware GPMP2 Cartesian Output Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Formulate the planner application in Vicon world from a supplied `T_W_M`, keep GPMP2 internally joint-space, and export the final validated path as timed world-frame end-effector pose/twist.

**Architecture:** The planner constructs its GPMP2 arm model and SDF in world coordinates using one immutable Vicon snapshot. A shared Eigen-only wire contract validates and formats projected samples; every final dense `q,qdot` state is transformed through Pinocchio FK and `J_W qdot` after time scaling and validation.

**Tech Stack:** C++17, Eigen, Pinocchio adapter, GTSAM/GPMP2, YAML, CMake/CTest.

## Global Constraints

- Do not modify GPMP2 or GTSAM internals.
- Every quantity paired in one solve—model, goal, path, obstacle, SDF, FK validation—must be in Vicon world `W`.
- `T_W_B = T_W_M T_M_B`; `T_W_E = T_W_B T_B_E`; rotate both Jacobian blocks by `R_W_B`.
- Output contains no planned joints or posture.
- Output sample units are seconds, metres, unit quaternion `x y z w`, metres/second, radians/second.
- Projection happens after the final validation/time-scaling pass and keeps every final dense state.
- A trajectory carries `trajectory_id`, `planner_vicon_sequence`, `frame=WORLD`, monotonic relative time, and terminal `arrival_eligible`.
- Preserve the user's pending planner-limit/validation changes. Do not run a robot executable or commit without authorization.

---

### Task 1: Shared Cartesian trajectory contract

**Files:**
- Create: `Christian_control/cartesian_contract/WorldCartesianTrajectory.h`
- Create: `Christian_control/cartesian_contract/WorldCartesianTrajectory.cpp`
- Create: `Christian_control/planner_bridge/tests/test_cartesian_contract.cpp`
- Modify: `Christian_control/planner_bridge/CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
struct WorldCartesianTrajectoryPoint {
    double t_from_start_s = 0.0;
    Eigen::Vector3d position_world_m = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_world = Eigen::Quaterniond::Identity();
    Eigen::Vector3d linear_velocity_world_m_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_world_rad_s = Eigen::Vector3d::Zero();
    bool arrival_eligible = false;
};
struct WorldCartesianTrajectory {
    std::uint64_t trajectory_id = 0;
    std::uint64_t planner_vicon_sequence = 0;
    std::vector<WorldCartesianTrajectoryPoint> points;
};
std::optional<std::string> ValidateWorldCartesianTrajectory(
    const WorldCartesianTrajectory& trajectory);
std::string FormatWorldCartesianTrajectoryBlock(
    const WorldCartesianTrajectory& trajectory);
class WorldCartesianTrajectoryAccumulator {
public:
    std::optional<WorldCartesianTrajectory> Feed(const std::string& line,
                                                 std::string& error);
    bool Collecting() const noexcept;
};
```

- [ ] **Step 1: Write parser/formatter rejection tests first**

```cpp
WorldCartesianTrajectory traj;
traj.trajectory_id = 9;
traj.planner_vicon_sequence = 42;
WorldCartesianTrajectoryPoint first;
first.t_from_start_s = 0.0;
WorldCartesianTrajectoryPoint last;
last.t_from_start_s = 0.002;
last.arrival_eligible = true;
traj.points = {first, last};
Check(!ValidateWorldCartesianTrajectory(traj).has_value(), "valid world contract");
const std::string block = FormatWorldCartesianTrajectoryBlock(traj);
Check(block.rfind("CART_TRAJ_BEGIN 1 9 42 WORLD 2\n", 0) == 0,
      "versioned header");
```

Reject fewer than two points, nonzero first time, non-increasing time, non-finite values, quaternion norm outside `1e-3`, non-`WORLD` frame, `arrival_eligible=true` before the final point, a final nonzero reference twist, excessive declared count, early/missing terminator, and trailing tokens.

- [ ] **Step 2: Run the test and confirm the contract is absent**

Run: `cmake --build Christian_control/planner_bridge/build --target test_cartesian_contract -j2`

- [ ] **Step 3: Implement the exact wire grammar**

```text
CART_TRAJ_BEGIN 1 <trajectory_id> <planner_vicon_sequence> WORLD <count>
<t> <px> <py> <pz> <qx> <qy> <qz> <qw> <vx> <vy> <vz> <wx> <wy> <wz> <arrival_eligible>
<one row for each remaining point>
CART_TRAJ_END
```

Set the block cap to 20,000 points, parse complete blocks off the real-time path, and keep the first and final samples exactly. Formatting uses sufficient precision (`std::setprecision(17)`) to avoid unit-quaternion drift caused by six-decimal text.

- [ ] **Step 4: Run the new test and `git diff --check`**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^cartesian_contract$' --output-on-failure`

### Task 2: World-frame transform boundary and dynamic SDF geometry

**Files:**
- Modify: `Christian_control/basic_control/src/Config.h`
- Modify: `Christian_control/planner_bridge/src/PathFrames.h`
- Modify: `Christian_control/planner_bridge/src/PathFrames.cpp`
- Modify: `Christian_control/planner_bridge/src/PlannerModel.h`
- Modify: `Christian_control/planner_bridge/src/PlannerModel.cpp`
- Modify: `Christian_control/planner_bridge/src/WorldSdf.h`
- Modify: `Christian_control/planner_bridge/src/WorldSdf.cpp`
- Modify: `Christian_control/planner_bridge/tests/test_path_assembly.cpp`
- Modify: `Christian_control/planner_bridge/tests/test_planner_model.cpp`
- Modify: `Christian_control/planner_bridge/tests/test_world_sdf.cpp`
- Modify: `Christian_control/planner_bridge/tests/test_grid_coverage.cpp`

**Interfaces:**
- Consumes: immutable `Eigen::Isometry3d world_T_mount`.
- Produces:

```cpp
Eigen::Isometry3d PoseToWorld(const Eigen::Isometry3d& pose,
                             config::ReferenceFrame frame,
                             const Eigen::Isometry3d& world_T_mount);
CartesianPath PathToWorld(const CartesianPath& path,
                          const Eigen::Isometry3d& world_T_mount);
PlannerModel LoadPlannerModel(const std::string& yaml_path, bool has_tool,
                              const Eigen::Isometry3d& world_T_mount);
gtsam::Pose3 ToolPoseInWorld(
    const PlannerModel&, const Eigen::Matrix<double, 7, 1>& q_rad);
GridGeometry WorldGridGeometry(const Eigen::Isometry3d& world_T_mount);
```

- [ ] **Step 1: Add failing tests using a translated and rotated Mount**

Use `T_W_M = Translation(1,2,3) * Rz(90 deg)` and assert a Mount point `(1,0,0)` becomes world `(1,3,3)`, both orientation and path samples rotate, the GPMP2 model base equals `T_W_M * T_M_DHroot`, and FK agrees with `T_W_M * ToolPoseInMount` from the pre-change implementation.

For the SDF, transform all eight corners of the measured mount-frame grid bounds, build a world-axis-aligned grid at `kGridCellM`, and assert every transformed corner plus the existing 0.05 m collision-envelope margin is queryable.

- [ ] **Step 2: Run targeted tests and confirm mount-only assumptions fail**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^(path_assembly|planner_model|world_sdf|grid_coverage)$' --output-on-failure`

- [ ] **Step 3: Implement world as a first-class declared frame**

Extend `config::ReferenceFrame`/names with `kWorld`/`"world"`. Replace `PoseToMount/PathToMount` with functions requiring `world_T_mount`; convert mount/base inputs through `T_W_M`, and pass world inputs unchanged. Rename model accessors so comments and types no longer claim Mount output.

Build an axis-aligned world SDF around the transformed mount-workspace corners:

```cpp
struct GridGeometry {
    Eigen::Vector3d origin_world_m;
    int nx, ny, nz;
    double cell_m;
};
GridGeometry WorldGridGeometry(const Eigen::Isometry3d& world_T_mount);
gpmp2::SignedDistanceField MakeWorldSdf(
    const GridGeometry&, const std::optional<AxisAlignedBox>& box_world);
```

- [ ] **Step 4: Run all frame/model/SDF tests**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^(cartesian_path|path_assembly|planner_model|world_sdf|grid_coverage|sphere_scaling|gpmp2_urdf_validation)$' --output-on-failure`

### Task 3: Dense FK/Jacobian projection after final validation

**Files:**
- Create: `Christian_control/planner_bridge/src/WorldTrajectoryProjection.h`
- Create: `Christian_control/planner_bridge/src/WorldTrajectoryProjection.cpp`
- Create: `Christian_control/planner_bridge/tests/test_world_trajectory_projection.cpp`
- Modify: `Christian_control/planner_bridge/src/PlanSolver.h`
- Modify: `Christian_control/planner_bridge/src/PlanSolver.cpp`
- Modify: `Christian_control/planner_bridge/CMakeLists.txt`

**Interfaces:**
- Consumes: final dense `trajectory_pos`, `trajectory_vel`, `total_time_sec`, world `PlannerModel`, trajectory/provenance IDs.
- Produces:

```cpp
WorldCartesianTrajectory ProjectWorldTrajectory(
    const PlannerModel& model,
    const std::vector<gtsam::Vector>& position_rad,
    const std::vector<gtsam::Vector>& velocity_rad_s,
    double total_time_s, std::uint64_t trajectory_id,
    std::uint64_t planner_vicon_sequence);
```

- [ ] **Step 1: Write independent FK and `J qdot` fixture tests**

```cpp
const auto projected = ProjectWorldTrajectory(model, q, qdot, 0.004, 12, 99);
const auto base = pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
    Eigen::Matrix<double,7,1>(q[1]), model.end_effector_frame, model.left_arm);
const Eigen::Matrix3d R_W_B = model.world_T_mount.linear() *
    pinocchio_kinematics_adapter::MountFromBase(model.left_arm).linear();
Check(Near(projected.points[1].linear_velocity_world_m_s,
           R_W_B * base.jacobian.topRows<3>() * EigenQdot(qdot[1])),
      "projected linear twist is independent J_W qdot evidence");
```

Also test angular twist, left/right arm frames, timestamps, quaternion normalization/hemisphere continuity, and final zero twist/arrival eligibility.

- [ ] **Step 2: Confirm projection tests fail before implementation**

Run: `cmake --build Christian_control/planner_bridge/build --target test_world_trajectory_projection -j2`

- [ ] **Step 3: Return final dense joints from path solving, then project**

Replace `PathPlanOutcome::emitted_block` with the final time-scaled `TrajectoryResult`; keep validation on the exact final dense artefact. `ProjectWorldTrajectory` must iterate every state—no independent SE(3) path synthesis and no decimation.

- [ ] **Step 4: Run projection, solver, and validation suites**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^(world_trajectory_projection|plan_solver|path_validation|circle_plan)$' --output-on-failure`

### Task 4: Make `planner_bridge` accept a world snapshot and emit Cartesian output

**Files:**
- Modify: `Christian_control/planner_bridge/src/BridgeMain.cpp`
- Modify: `Christian_control/planner_bridge/src/BridgeMain.h`
- Replace: `Christian_control/planner_bridge/src/TrajectoryEmit.h`
- Replace: `Christian_control/planner_bridge/src/TrajectoryEmit.cpp`
- Modify: `Christian_control/planner_bridge/tests/test_bridge_main.cpp`
- Modify: `Christian_control/planner_bridge/scripts/run_session.sh` only to preserve the old joint execution path until Slice 3 switches both ends together.

**Interfaces:**
- Adds one-shot arguments `--world-mount-pose-m-quat PX PY PZ QX QY QZ QW`, `--vicon-sequence N`, and `--trajectory-id N`.
- Produces only `CART_TRAJ_*` when `--output world-cartesian` is requested; the temporary default remains joint output until Slice 3.

- [ ] **Step 1: Extend bridge tests with a non-identity world snapshot**

Assert output begins with the versioned Cartesian header, contains no `q1..q7`, and each first/last pose equals independent Pinocchio FK transformed by the supplied `T_W_M`. Assert missing/non-unit/non-finite snapshot arguments produce exit 1 and no stdout.

- [ ] **Step 2: Run `bridge_main` and confirm argument/output failures**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^bridge_main$' --output-on-failure`

- [ ] **Step 3: Thread the immutable snapshot through parsing, world model/SDF construction, solve, projection, and buffered output**

Keep optimizer chatter redirected to diagnostics and preserve the all-or-nothing stdout contract. Echo `planner_vicon_sequence`, `T_W_M`, frame, trajectory ID, sample count, and duration to diagnostics.

- [ ] **Step 4: Verify the complete planner suite**

Run: `cmake --build Christian_control/planner_bridge/build -j2 && ctest --test-dir Christian_control/planner_bridge/build --output-on-failure`

Run: `git diff --check`

Review checkpoint: search emitted rows and public contract headers to prove no `q_ref`, `qdot_ref`, or posture field crosses the new Cartesian output boundary.
