# Dual-Arm MuJoCo Execution Twin Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Kortex-free `humansl_sim` target that runs two instances of the shared execution core against an exact-frame dual-arm MuJoCo model with ideal world-state feedback.

**Architecture:** Reuse audited `msc_project` assets and backend patterns, but make HumanSL's dual-arm URDF, Mount-to-base calibration, joint order, limits, and configured TCPs authoritative. One deterministic coordinator reads both arms, steps both cores on the same 2 ms control tick, applies both integrated position commands, and advances MuJoCo using one or more fixed physics substeps.

**Tech Stack:** C++17, MuJoCo 3.10.0 vendored in `third_party/` (lib, headers, and licenses copied from the official `mujoco==3.10.0` wheel; see `third_party/MUJOCO_PROVENANCE.md`), Eigen, Pinocchio, `humansl_execution_core`, CMake/CTest, optional GLFW MuJoCo passive viewer. GLFW dev is NOT installed system-wide: the viewer requires `libglfw3-dev` (ask Christian before installing) or a FetchContent build; headless is the default and the only mode tests use.

## Global Constraints

- Plan 01's replay gate and core extraction must be accepted first.
- `docs/engineering/humansl-engineering-contract.md` is binding for every task in this plan.
- Simulation sources live in their own CMake project (`Christian_control/simulation/CMakeLists.txt`), matching the repo's per-component pattern (`basic_control`, `planner_bridge`, `vicon`); it obtains `humansl_execution_core` via `add_subdirectory` of `../basic_control`. There is deliberately no top-level CMakeLists.
- `humansl_sim` must not link Kortex, contain robot IPs, or share a runtime hardware/simulation switch.
- Control period is exactly `0.002 s`; fixed internal physics substeps may divide that period.
- MuJoCo position actuators are generic and are not a model of Kinova internal servo dynamics.
- Use exact HumanSL right/left joint ordering, axes, limits, Mount transforms, and configured TCPs.
- No functional gripper or object manipulation.
- Ideal mode supplies exact Mount pose/twist on every 500 Hz control tick.
- Free space is the default; shared obstacles are implemented in Plan 03.
- No robot-facing executable may be run. Do not commit without explicit authorization.

---

### Task 1: Record model and dependency provenance

**Files:**
- Create: `Christian_control/simulation/README.md`
- Create: `Christian_control/simulation/model/model_provenance.yaml`
- Create: `Christian_control/simulation/tests/test_model_provenance.py`
- Create: `Christian_control/simulation/CMakeLists.txt` (new standalone simulation project; registers the provenance test)

**Interfaces:**
- Produces a machine-readable record of source URDF, source `msc_project` MJCF/assets, MuJoCo version, right/left TCP names, and Mount transforms.

- [x] **Step 1: Write a provenance test**

Require absolute repository-relative source paths, SHA-256 hashes for the HumanSL URDF and every copied mesh/MJCF source, explicit units, `world`, `mount`, `right_base`, `left_base`, right TCP, and left TCP names (right is the configured tool frame `ConfiguredTool_Link`; left is the bare flange `leftEndEffector_Link` — different physical points by design, never compared as equivalents). Reject `pinch_site` as either production TCP.

- [x] **Step 2: Run the test and verify missing provenance**

Run: `python3 Christian_control/simulation/tests/test_model_provenance.py`

Expected: FAIL because the provenance file is absent.

- [x] **Step 3: Audit the vendored MuJoCo and source assets without installing**

MuJoCo 3.10.0 is already vendored (2026-08-17) in `third_party/lib/libmujoco.so.3.10.0` (+ `libmujoco.so`/`.so.3` symlinks) and `third_party/include/mujoco/`; provenance and library hash are in `third_party/MUJOCO_PROVENANCE.md`. A smoke program has compiled and run against it.

Run: `ls third_party/lib/libmujoco.so* third_party/include/mujoco/mujoco.h third_party/MUJOCO_PROVENANCE.md`

Run: `sha256sum third_party/lib/libmujoco.so.3.10.0 Christian_control/basic_control/config/GEN3_dual_mounted.urdf /home/christian/msc_project/sim/assets/kinova_gen3/gen3.xml`

Record the discovered paths/versions and asset licenses. Do not download or install a second model or MuJoCo.

- [x] **Step 4: Write provenance and rerun the test**

Expected: PASS and every named file exists.

Review checkpoint: HumanSL frames/TCPs are authoritative; `msc_project` is a source of simulator mechanics, not kinematic truth.

### Task 2: Build the exact-frame dual-arm MJCF

**Files:**
- Create: `Christian_control/simulation/model/humansl_dual_gen3.xml`
- Create: `Christian_control/simulation/model/assets/` populated only with audited required meshes
- Create: `Christian_control/simulation/src/ModelContract.h`
- Create: `Christian_control/simulation/src/ModelContract.cpp`
- Create: `Christian_control/simulation/tests/test_model_contract.cpp`
- Modify: `Christian_control/simulation/CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
struct ArmModelIds {
    std::array<int, 7> joint_qpos_adr;
    std::array<int, 7> joint_dof_adr;
    std::array<int, 7> actuator_id;
    int base_body_id;
    int tcp_site_id;
};

struct DualModelContract {
    int mount_body_id;
    ArmModelIds right;
    ArmModelIds left;
};

DualModelContract ValidateAndResolveModel(const mjModel& model);
```

- [x] **Step 1: Write failing name/order/range tests**

Load the MJCF and require one moving Mount body; fourteen uniquely prefixed joints and actuators; correct continuous versus bounded joints; exact bounded ranges; right/left bases under Mount; explicit production TCP sites; no accidental duplicate names; `nq`, `nv`, and `nu` consistent with the contract.

- [x] **Step 2: Run the contract test and observe missing model**

Run: `cmake --build Christian_control/simulation/build --target test_sim_model_contract -j2`

Expected: FAIL because the simulation model/contract does not exist.

- [x] **Step 3: Create the dual-arm MJCF**

Port only necessary Gen3 inertial, visual, collision, joint, and actuator definitions from the audited source. Place both chains using the mount→base transforms read from the production URDF via `DualArmKinematics::MountFromBase(Arm)` (`Christian_control/basic_control/src/Kinematics.cpp:262`) — there are no `T_M_B_*` constants in code; the human-readable numbers in `config/dual_arm_mounting.yaml` are documentation, not the authority. Generate the MJCF placement numbers with a script from those URDF/Pinocchio values, never hand-typed (hand-carried rounding already cost ~2e-5 m in the DH-parameter work). TCP sites come from the production end-effector frames, which are asymmetric by design: right = `ConfiguredTool_Link` (configured tool), left = `leftEndEffector_Link` (bare flange — there is no left tool transform; never invent one). Do not use `pinch_site`. Prefix every right/left object name.

- [x] **Step 4: Implement strict runtime contract resolution**

Resolve by exact names once at startup. Throw with the missing/mismatched name, expected range/order, and observed model value. Never search by body index assumptions during control.

- [x] **Step 5: Run contract and MuJoCo load smoke tests**

Run: `ctest --test-dir Christian_control/simulation/build -R '^sim_model_contract$' --output-on-failure`

Expected: PASS with no warning about implicit free joints, missing inertials, or invalid ranges.

Review checkpoint: visually inspect the loaded static model and compare both base/TCP frame axes against `print_dual_arm_fk`; do not infer correctness from appearance alone.

### Task 3: Cross-validate MuJoCo and production Pinocchio kinematics

**Files:**
- Create: `Christian_control/simulation/tests/test_model_parity.cpp`
- Create: `Christian_control/simulation/tests/model_parity_cases.h`
- Modify: `Christian_control/simulation/CMakeLists.txt`

**Interfaces:**
- Consumes: `DualArmKinematics`, `DualModelContract`, fixed joint vectors.
- Produces parity evidence for base pose, TCP pose, joint axis/order, and finite-difference translational/angular Jacobian.

- [x] **Step 1: Add independent fixed cases**

Use home, retracted, asymmetric bounded-joint, continuous-joint wrap, and seeded random configurations. For each arm compare `T_W_TCP` from MuJoCo site state against production Pinocchio FK after applying the same `T_W_M` and `T_M_B`.

- [x] **Step 2: Add finite-difference Jacobian checks**

Perturb one joint at a time by `1e-7 rad`; compare MuJoCo TCP position/rotation-log derivatives with the production world Jacobian column. Fail with arm, joint, state, expected vector, and actual vector.

- [x] **Step 3: Run and observe frame/model discrepancies**

Run: `ctest --test-dir Christian_control/simulation/build -R '^sim_model_parity$' --output-on-failure`

Expected before corrections: any axis/TCP/base discrepancy fails explicitly.

- [x] **Step 4: Correct only model/frame definitions**

Do not add numerical offsets in the test or controller. Fix MJCF body transforms, axes, joint zero, or TCP placement and document each correction in `model_provenance.yaml`.

- [x] **Step 5: Re-run parity suite**

Required tolerances: position `<=1e-8 m`, orientation-log norm `<=1e-8 rad`, finite-difference Jacobian `<=1e-6` per component unless a larger tolerance is justified by a recorded MuJoCo precision experiment before results are viewed.

Review checkpoint: verify a deliberately swapped joint or inverted axis makes the test fail.

### Task 4: Implement the MuJoCo command/feedback adapter

**Files:**
- Create: `Christian_control/simulation/src/MujocoBackend.h`
- Create: `Christian_control/simulation/src/MujocoBackend.cpp`
- Create: `Christian_control/simulation/tests/test_mujoco_backend.cpp`
- Modify: `Christian_control/simulation/CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
struct SimArmFeedback {
    RobotState right;
    RobotState left;
    JointVector right_continuous_deg;
    JointVector left_continuous_deg;
    double simulation_time_s;
};

class MujocoBackend {
public:
    MujocoBackend(const std::string& model_path, double control_dt_s,
                  int physics_substeps);
    SimArmFeedback Seed(const CartesianPose& world_T_mount);
    SimArmFeedback Exchange(const JointVector& right_position_deg,
                            const JointVector& left_position_deg,
                            const CartesianPose& world_T_mount,
                            const Twist& world_V_mount);
};
```

> Superseded 2026-08-17 by the mocap Mount decision: `Exchange` takes no
> `world_V_mount`. A mocap body is welded to world and has no velocity the
> physics can see, so the plant had no honest use for a twist. Mount twist
> is a sensing quantity and belongs to whatever describes the Mount's
> motion, alongside the pose it is the derivative of — see
> `Christian_control/simulation/src/MujocoBackend.h` for the shipped
> signature. The feedback record above also lost its `RobotState` fields
> (the core owns that boundary). Read this block as the original sketch,
> not as the interface.

- [x] **Step 1: Write seed/exchange/substep tests**

Assert seed returns model keyframe state without advancing time; one exchange advances exactly 0.002 s; substeps are positive and divide the period exactly; controls remain constant across substeps; feedback ordering matches the contract; non-finite commands and state stop before another physics step.

- [x] **Step 2: Run and confirm missing backend**

Run: `cmake --build Christian_control/simulation/build --target test_mujoco_backend -j2`

- [x] **Step 3: Implement the adapter with generic position actuators**

Set `model.opt.timestep = control_dt_s / physics_substeps`; write the fourteen actuator controls once; call `mj_step` exactly `physics_substeps` times; recover continuous measured degrees near the previous integrated command for continuous joints.

- [x] **Step 4: Run backend tests for one and four substeps**

Run: `ctest --test-dir Christian_control/simulation/build -R '^mujoco_backend$' --output-on-failure`

Expected: deterministic byte-identical state traces for repeated runs on the same build and substep setting.

Review checkpoint: actuator `kp/kv` values and substep count appear in run provenance and are not described as Kinova parameters.

### Task 5: Add the dual-arm simulation coordinator and executable

**Files:**
- Create: `Christian_control/simulation/src/SimulationConfig.h`
- Create: `Christian_control/simulation/src/DualSimulationRunner.h`
- Create: `Christian_control/simulation/src/DualSimulationRunner.cpp`
- Create: `Christian_control/simulation/src/SimMain.cpp`
- Create: `Christian_control/simulation/tests/test_dual_simulation_runner.cpp`
- Create: `Christian_control/simulation/tests/test_sim_no_kortex.cmake`
- Modify: `Christian_control/simulation/CMakeLists.txt`

**Interfaces:**
- Consumes: two `ArmExecutionCore` instances, one `MujocoBackend`, one ideal Mount source, and one shared stop flag.
- Produces CMake targets `humansl_sim`, `test_dual_simulation_runner`, and a Kortex-link exclusion test.

```cpp
struct DualSimulationCycle {
    ArmExecutionResult right;
    ArmExecutionResult left;
    SimArmFeedback next_feedback;
    bool shared_stop;
};

class DualSimulationRunner {
public:
    void Start();
    DualSimulationCycle Step();
    void Reset();
};
```

- [x] **Step 1: Write cycle-order/shared-stop tests**

Assert both cores consume the same tick and Mount state; both commands are computed before physics advances; either arm's stop prevents both commands from advancing; reset clears references/integrators and requires a fresh seed; ideal Mount pose/twist is exact every tick.

- [x] **Step 2: Implement the deterministic coordinator**

Keep all viewer, file, planner, and panel operations outside `Step`. Use one thread for deterministic physics/control ordering. Publish only immutable non-blocking telemetry snapshots to observers.

- [x] **Step 3: Add headless CLI and optional passive viewer**

Support `--headless`, `--duration-s`, `--physics-substeps`, and `--viewer`; default to headless for tests. Viewer rendering may lag or drop frames but must never alter simulation/control time.

- [x] **Step 4: Prove the target cannot link Kortex**

Use a CTest script over the link command and `nm -uC`/`ldd` output. Fail if `KortexApi`, `Kinova::Api`, a robot address, or hardware `Runner.cpp`/`Hardware.cpp` appears in `humansl_sim` dependencies.

- [x] **Step 5: Build and run hardware-free smoke**

Run: `cmake --build Christian_control/simulation/build --target humansl_sim test_dual_simulation_runner -j2`

Run: `ctest --test-dir Christian_control/simulation/build -R '^(dual_simulation_runner|sim_no_kortex|sim_model_parity|mujoco_backend)$' --output-on-failure`

Run: `Christian_control/simulation/build/humansl_sim --headless --duration-s 2 --physics-substeps 4`

Expected: both arms hold their initial world TCP poses for 1000 control cycles, no Kortex symbol/address appears, and the run exits without contact, limit, or non-finite stop.

Final gate: inspect the viewer manually only after headless tests pass. No robot process is started. Plan 03 may begin after model parity and ideal-mode hold are accepted.

---

## Completion evidence (2026-08-17, evidence class: simulation)

- All six CTest suites pass: `sim_model_contract`, `sim_model_parity`,
  `mujoco_backend`, `dual_simulation_runner`, `sim_no_kortex`,
  `sim_model_provenance`.
- Headless hold: `humansl_sim --headless --duration-s 2 --physics-substeps 4`
  completed 1000 of 1000 cycles with no stop and no contact. Max estimated
  TCP position error 0.548 mm (both arms), max orientation error
  0.093 mrad (right) / 0.063 mrad (left), zero overruns, zero contacts.
- Determinism: two consecutive identical runs produced byte-identical
  output.
- Kortex-free proof: `sim_no_kortex` CTest inspects the link line and
  undefined symbols; no Kortex symbol, robot address, or hardware
  Runner/Hardware dependency in `humansl_sim`.
- Outstanding: manual viewer inspection by Christian (final gate step);
  headless evidence does not substitute for it.
