# Shared controller across hardware and MuJoCo — proposed boundary

Status: proposal, nothing implemented. Written 2026-08-07 after reading the
four existing pieces. Every number below was read out of the repositories or
measured by a probe recorded in the evidence section; none is recalled.

## 1. What actually exists today

Four things were inspected.

**The hardware controller** is `Christian_control/basic_control`, in this
repository. It opens two Kortex sessions to one Gen3, runs a 500 Hz cyclic
position loop, and drives one arm per process. Its control law is
`ReactiveLaw.h`; its composition is `Controller.h`; its kinematics adapter is
`DualArmKinematics` in `Kinematics.h`, which loads the dual-arm URDF into
Pinocchio.

**The planner** is `Christian_control/planner_bridge`, also here. It solves in
GPMP2, validates, and writes a joint-trajectory text block into a named pipe
the controller reads.

**The C++ MuJoCo port** is `/home/christian/msc_project/cpp`. It is a separate
repository. It already contains the abstraction this task is asking for:
`src/sim/Backend.h` declares `PlantBackend` with exactly `Takeover`,
`Exchange`, `Release`, and `MujocoBackend` implements it. Its header comment
already says "shared by MuJoCo and any future hardware backend".

**The Python MuJoCo simulation** is `/home/christian/msc_project` proper. The
C++ port is a one-to-one transliteration of it — `controller/backend.py`
declares the same three-method Protocol, and nearly every C++ header names the
Python file it mirrors.

So the boundary you are asking for was designed once already, on the
simulation side, and the hardware side has never been fitted to it. That is
the single most useful fact in this document: the work is not to invent an
interface, it is to decide which of two existing controller cores becomes the
shared one, and then to fit the other side's plant access behind
`PlantBackend`.

### The uncomfortable part

There is not one controller core with two backends missing. There are **two
independent controller cores** that implement the same family of equations and
disagree in ways that matter:

| | hardware (`basic_control`) | simulation (`msc_project`) |
|---|---|---|
| arms per process | one, fixed at construction | both, in one runner |
| Cartesian frame | the controlled arm's own `base_link` | world |
| torso | does not exist; URDF root is `mount` | mocap body, moves, drives the mount calibration |
| null-space objective | deadband limit avoidance, `LimitAvoidanceVelocity` | centring to joint midpoint, `JointCentering` |
| null-space projector | damped, `λ = 0.1` | undamped pseudo-inverse |
| `kp_position` | 10.0 | 2.0 |
| joint wrap | required — Kortex reports `[0, 360)` | not required — MuJoCo `q` is continuous |
| reference channels | pose **or** joint trajectory | pose only |
| human safety filter | none | `SafetyFilter.h`, cylinder around the wearer |

`ReactiveLaw.h`'s own header comment documents the first two deviations as
deliberate hardware changes. So "run the existing controller core unchanged"
is not yet a well-posed goal — it becomes one only once you name which core.
Section 7 puts that choice to you.

## 2. The smallest interface

Everything above the plant already speaks in fixed-size joint vectors. The
whole of the hardware-versus-simulation difference collapses into three
operations, which is what both existing `PlantBackend` declarations found:

```
class RobotIo {
 public:
  virtual ~RobotIo() = default;

  // Acquire control and return the first state sample. The plant is now
  // holding its measured position; no motion has been commanded.
  virtual RobotState Takeover() = 0;

  // Apply one complete position command and return the next state sample.
  // The implementation owns whether that means stepping the simulator or
  // waiting for a cyclic reply.
  virtual RobotState Exchange(const JointCommand& command) = 0;

  // Relinquish control. Idempotent; safe on every exit path.
  virtual void Release() = 0;

  // Static facts the controller must not guess: which arms are present,
  // the nominal cycle period, joint limits, and command bounds.
  virtual const RobotIoCapabilities& capabilities() const = 0;
};
```

Three of those four are already implemented on both sides. `capabilities()` is
the only addition, and it exists because the two plants currently obtain the
same facts by different routes: the simulation reads joint ranges out of the
MuJoCo model at construction (`MujocoBackend::BuildArmPipelineSetup`), while
the hardware takes them from compile-time constants in `Config.h`
(`kJointSoftwareLimitDeg`, `kQdotLimitDegS`) and verifies them against the
robot in `Connect::EnsureJointLimits`. One accessor lets the shared pipeline
be built the same way for both.

### Why nothing else needs to cross

A control law that works from `q`, `q̇`, a target pose and a Jacobian does not
care what produced `q`. Everything the hardware has that MuJoCo has not —
Kortex sessions, servoing-mode switching, fault banks, command-acknowledgement
freshness, joint-limit threshold writes, the process lock — produces no value
the controller reads. It either succeeds silently or it stops the run. So all
of it belongs *inside* `HardwareRobotIo`, surfaced as at most a status field
on `RobotState` (`ok`, plus a reason) rather than as new interface methods.

Symmetrically, everything MuJoCo has that the hardware has not — position
actuators, contacts, the mocap torso, scene geometry, the viewer — either
belongs inside `MujocoRobotIo` or stays in simulation-only code above it. You
said those may remain simulation-specific and they should.

### Two contract details that will bite if left implicit

**Units and winding.** The interface must be radians, and it must state a
winding convention. Hardware reports position in degrees on `[0, 360)`;
MuJoCo reports radians, continuous and signed. `Runner.cpp:148` already
converts degrees to radians at the feedback boundary, and `State.h`'s
`WrappedJointError` exists solely because the winding was left unspecified
downstream. Fix it at the boundary: `RobotIo` delivers **signed radians
wrapped to (−π, π]**, `HardwareRobotIo` does the wrap and keeps the raw
`[0, 360)` value in its own log column (the run log already has
`measraw_j1..7` for exactly this). Then `WrappedJointError` becomes belt-and-
braces rather than load-bearing.

**Command semantics.** Both plants take an absolute joint *position*
setpoint, not a velocity, so `JointCommand` is a position command and the
integrator (`PositionIntegration` / `PositionIntegrator`) stays above the
boundary. That is already true on both sides and is worth writing down so it
does not drift.

## 3. Requested mappings

### 3.1 Canonical joint names

I propose the URDF names as canonical, because the URDF is already the
authority for the planner, the DH generation and the hardware FK, and because
every other name in the system is derivable from it by a rule.

| canonical (URDF) | hardware | Pinocchio joint | MJCF today | MJCF via URDF attach |
|---|---|---|---|---|
| `Actuator1` … `Actuator7` | Kortex actuator 0…6 | same name | `right_joint_1` … `_7` | `<prefix>Actuator1` … `7` |
| `leftActuator1` … `leftActuator7` | Kortex actuator 0…6 (other process) | same name | `left_joint_1` … `_7` | `<prefix>leftActuator1` … `7` |

The MJCF column is the one place a translation table exists today, and it is
the one the URDF-attach route removes. Note that the hardware index is
per-arm: both arms number their actuators 0…6, and which physical arm that is
is decided by which IP the process connected to (`kRightRobotIp`
`192.168.1.10`, `kLeftRobotIp` `192.168.1.9`).

### 3.2 Hardware joint indices

Kortex exposes each arm as seven actuators addressed two different ways:

- **Cyclic feedback and command**: `feedback.actuators(i)` and
  `command.actuators(i)` for `i` in `[0, 6]`, where `i` is joint number minus
  one. `Runner.cpp:148-149` reads position and velocity this way;
  `CyclicSession::Send` writes setpoints the same way. Units are degrees and
  degrees per second, position on `[0, 360)`.
- **Configuration RPCs**: `DeviceConfig`/`ActuatorConfig` address a
  **1-based** device identifier equal to the joint number.
  `tools/set_joint_limits.cpp:52` states this explicitly: "1-based joint
  number == device identifier".

That off-by-one between the two addressing schemes is a real trap and belongs
in the `HardwareRobotIo` documentation, not in the shared controller.

Joint 6's configuration service is currently wedged on this robot — config
RPCs time out while the joint servoes normally. That is a live hardware fault,
not an interface concern, but it means `HardwareRobotIo`'s capability query
cannot assume every configuration read will answer.

### 3.3 MuJoCo qpos/qvel indices

These must never be hard-coded and are not today. Both implementations resolve
them by name at construction and store them:

- Python: `sim/world.py:69-70` builds `qpos_adrs` and `dof_adrs` from
  `model.jnt_qposadr[joint_id]` and `model.jnt_dofadr[joint_id]`, with
  `joint_id` from `mj_name2id(mjOBJ_JOINT, f"{side}_joint_{index}")`.
- C++: `MujocoBackend.h:98-101` holds `qpos_adrs_`, `dof_adrs_`, `ctrl_adrs_`
  and `joint_id_` as `DualArm<std::array<int, 7>>`.

Because every Gen3 joint is a MuJoCo hinge (one `qpos`, one `qvel` each),
`qpos` and `qvel` addresses coincide, and in a URDF-derived model with no
free joints they are simply `0…6` for the right arm and `7…13` for the left,
in declaration order. I measured that: loading the canonical URDF gives
`jnt_qposadr = [0, 1, …, 13]` with all fourteen joints of type hinge. Keep
resolving by name anyway — the moment the scene gains a floating base or a
gripper, the positional coincidence ends.

### 3.4 Pinocchio joint indices

Pinocchio does **not** use a flat 7-per-arm layout, because Kinova's joints
1, 3, 5 and 7 are `continuous` in the URDF and Pinocchio represents a
continuous joint as the pair `(cos θ, sin θ)`. So the dual model has
`nq = 22` and `nv = 14`, asserted in `test_dual_arm_model.cpp:62-64` along
with `njoints == 15` (fourteen movable joints plus Pinocchio's universe).

Per arm the configuration sizes are `{2, 1, 2, 1, 2, 1, 2}` —
`DualArmKinematics::kJointConfigurationSizes` — summing to eleven `q` slots
and seven `v` slots. `Kinematics.cpp:78-88` resolves the index of each by
name through `model.getJointId(name)` and then `model.idx_qs` / `model.idx_vs`,
and `test_dual_arm_model.cpp` checks the stored mapping against Pinocchio's
own. Nothing is hard-coded and nothing should become hard-coded; the shared
interface should expose the seven-wide Kortex-ordered view and let the adapter
own the scatter into `q_full`.

The simulation side does not have this problem, and that is itself evidence of
divergence: `PinModel` parses the **MJCF**, where the same joints are plain
hinges, so its `q` is a flat `Vector7`. Under the shared model both sides
would parse the URDF and both would see `nq = 22`.

### 3.5 Base and tool frames

| role | URDF (canonical) | hardware use | MJCF today |
|---|---|---|---|
| rig root | `mount` | `config::kReferenceFrame = kMount` | *(no counterpart; `torso` mocap body sits above the arms instead)* |
| right base | `base_link` | `kRightBaseFrame` | `right_base_link` |
| left base | `leftbase_link` | `kLeftBaseFrame` | `left_base_link` |
| right flange | `EndEffector_Link` | — | `right_pinch_site` |
| right tool | `ConfiguredTool_Link` | `kRightEndEffectorFrame` | *(no counterpart)* |
| left flange | `leftEndEffector_Link` | `kLeftEndEffectorFrame` | `left_pinch_site` |

Two facts here matter more than the table.

First, **`pinch_site` is the flange, not the tool.** The URDF's `EndEffector`
joint offsets `(0, 0, −0.0615250)` with `rpy (π, 0, 0)` from `Bracelet_Link`;
the MJCF's `pinch_site` is at `pos (0, 0, −0.061525)` with `quat (0, 1, 0, 0)`,
which is the same 180° rotation about x. They are the same point. The hardware
right arm, however, controls `ConfiguredTool_Link`, which sits a further
0.12 m along the flange z — measured off the robot itself via
`GetToolConfiguration` on 2026-08-05. So the simulation is today controlling a
point 120 mm from the one the hardware controls. That is the same
tool-versus-flange class of error recorded against the left-arm bring-up.

Second, **the URDF has no torso.** Its root is `mount`, and `Config.h:74-79`
explicitly anticipates the gap: "A room frame belongs ABOVE it as
`T_room_mount`, identity while the rig is bolted to a bench and supplied by
motion capture once it is worn. That transform does not exist yet." The
simulation's `torso` mocap body *is* that missing frame. Under the shared
model the composition becomes
`T_world_torso · T_torso_mount · T_mount_base`, where the URDF owns the last
factor, the scene owns the first two, and `MountCalibration` carries what the
backend measured. That lines up cleanly, which is a good sign for the design.

But the two disagree numerically today, and the URDF must win:

| | URDF `mount → base` | MJCF `torso → base` |
|---|---|---|
| base separation | 0.113415 m | 0.2 m |
| mount tilt | ±1.2085 rad (69.24°) | ±0.845708 rad (48.46°) |
| offset from parent | `(0, ∓0.0567075, 0)` | `(−0.16, ∓0.1, 0.14)` |

Adopting the URDF moves each simulated base 0.0433 m in y and rotates it
0.363 rad — about 21°. The simulation's existing tuning, goldens and
trajectory configurations will shift. That is a consequence to plan for, not a
blocker, and it is the correct direction: `dual_arm_mounting.yaml` is the
declared source of truth and is enforced against the URDF by
`test_dual_arm_mounting.cpp`. Its own header is honest that the numbers are
inherited rather than surveyed, so both models are provisional — but only one
of them should be provisional at a time.

### 3.6 Controller inputs

| | hardware | simulation |
|---|---|---|
| state | `RobotState{q_rad[7], qdot_rad_s[7], t_s}` | `ArmControllerState{joints, ee_pose_world, ee_twist_world, jacobian_world, link_safety_points}` ×2 |
| target | `Reference{optional<PoseReference>, optional<JointReference>}` | `WorldTarget{pose_world, twist_world}` ×2 |
| timing | `dt_s`, measured and clamped by the Runner | `dt_s` |
| extra | — | `DualArmHumanSafetyStates` |

The shapes differ, but note *where*: the simulation resolves FK, the Jacobian
and the world-frame pose **before** the controller (`Frames.cpp`), while the
hardware resolves them **inside** it (`Controller.cpp` calling
`DualArmKinematics`). The simulation's split is the better one for this
architecture, because it puts one pure-Eigen controller above a kinematics
layer that both plants share, and it is the split `State.h`'s own header
diagram already describes.

### 3.7 Controller outputs

| | hardware | simulation |
|---|---|---|
| law output | `Eigen::Matrix<double,7,1>` desired `q̇` (rad/s), pre-clamp | `Vector7` `qdot_raw` per arm |
| then | `ClampJointVelocity` → `PositionIntegration::Apply` | speed clip → safety projection → `PositionIntegrator` |
| plant command | `JointVector setpoints_deg` (degrees, wrapped `[0, 360)`) | `JointPositionCommand{DualArm<Vector7> position_rad}` |
| telemetry | `ControllerStatus` (~20 fields) + `LoopLogSample` (141 CSV columns) | `ControlTrace` (18 fields) + `RunnerCycle` |

Both end at an absolute joint position command, which is what makes the
three-method interface sufficient. The degree conversion and the `[0, 360)`
wrap are hardware wire format and belong inside `HardwareRobotIo`.

### 3.8 Control and integration frequency

Both sides already run at **500 Hz**, and this is the least contentious part
of the whole design.

- Hardware: `config::kControlDtS = 0.002`, so `kControlFrequencyHz = 500`, one
  Kortex `Refresh` exchange per cycle.
- Simulation: `control.toml` sets `run.nominal_dt_s = 0.002`; both backends
  assign it straight to `model.opt.timestep` (`world.py:38`,
  `MujocoBackend.cpp:33`) and call `mj_step` **once** per exchange
  (`world.py:191`, `MujocoBackend.cpp:186`).

So the current MuJoCo integration frequency equals the control frequency
exactly, with a substep ratio of one. I would make that ratio explicit in
`MujocoRobotIo` rather than leave it implied — `control_dt / mj timestep`,
defaulting to 1 — because a 2 ms MuJoCo timestep is coarse for contact-rich
scenes, and the day you need a 0.5 ms integrator you want to shrink the
timestep and loop four `mj_step` calls per `Exchange`, not change the control
rate. That is a change inside the backend and invisible above it, which is
exactly the property the boundary is supposed to buy.

## 4. Can the MuJoCo port consume the URDF directly?

**Yes, with a mechanical build step and no second kinematic model.** This was
measured, not assumed. MuJoCo 3.10.0 is what the project has installed.

**Evidence 1 — the only blocker is mesh URIs.** Loading
`GEN3_dual_mounted.urdf` unmodified fails with
`Error opening file 'package://kortex_description/.../spherical_wrist_1_link.STL'`.
MuJoCo does not resolve ROS `package://` URIs. With those prefixes stripped
and a `<mujoco><compiler meshdir="…"/></mujoco>` element added, the same file
loads: `nq = 14, nv = 14, njnt = 14`, all hinges, joint names exactly
`Actuator1…7` and `leftActuator1…7`, and `jnt_limited` false/true/false/true/
false/true/false per arm — matching the URDF's continuous/revolute pattern.
`balanceinertia` turned out **not** to be needed.

**Evidence 2 — `fusestatic="false"` preserves every frame that is named.**
By default MuJoCo welds fixed-joint links away, and the frames the controller
and planner depend on vanish. With `fusestatic="false"` the model carries 21
bodies including `mount`, `base_link`, `EndEffector_Link`,
`ConfiguredTool_Link`, `leftbase_link` and `leftEndEffector_Link`. No sites
need to be invented; the tool frame can be read from `xpos`/`xmat` of the body
that the URDF already defines.

**Evidence 3 — the URDF can be attached into a scene.** MuJoCo 3.x accepts a
URDF as a child model: `<asset><model name="srl" file="…urdf"/></asset>` plus
`<attach model="srl" body="mount" prefix="srl_"/>` inside a `torso` mocap
body compiles and yields the fourteen expected prefixed joints. So the scene
XML keeps the torso, floor, targets, contacts and actuators, and the arms come
from the URDF. That is exactly the split you asked for.

**Evidence 4 — there is no second model in substance, only in encoding.** At
`q = [0.11, −0.22, 0.33, −0.44, 0.55, −0.66, 0.77]` the URDF's
`EndEffector_Link` and the existing MJCF's `pinch_site`, both expressed in
`base_link`, differ by **7.4 µm and 10.4 µrad**. The two files describe the
same arm; the residual is consistent with the ~2 × 10⁻⁵ m floor that URDF
`rpy` rounding already imposes on the generated DH parameters. `gen3.xml` is a
redundant re-encoding, and retiring it as a kinematic authority loses nothing
measurable.

**What the URDF genuinely cannot supply**, and must stay in the scene overlay:
actuators (`nu = 0` from URDF — MuJoCo ignores URDF `<transmission>`), the
mocap torso, contact exclusions, floor and target markers, and visual assets.
All of these are simulation-specific by your own scoping, so this is not a
gap.

**Recommended mechanism.** Generate the MuJoCo-consumable copy at build time,
the way `dh_params_tool.yaml` is already generated from the same URDF. A small
tool reads `GEN3_dual_mounted.urdf`, rewrites the mesh paths to the vendored
`third_party/ros_kortex` meshes, injects the `<mujoco><compiler>` element, and
writes `GEN3_dual_mounted.mujoco.urdf` into the build directory. The scene XML
attaches that. The canonical URDF is never edited, the Pinocchio path is
untouched and cannot regress, and the generated file inherits the existing
"never hand-edit, never commit" contract. The alternative — adding the
`<mujoco>` element to the canonical URDF directly — is one fewer moving part
but puts a MuJoCo-specific element in the file Pinocchio parses, and I have
not verified that `urdfdom` tolerates it. If you prefer that route it needs
that check first.

## 5. Proposed interface boundary

```
                    ┌──────────────────────────────────────────┐
                    │  planner  (planner_bridge, GPMP2)        │
                    │  URDF ─▶ DH ─▶ solve ─▶ validate ─▶ emit  │
                    └────────────────────┬─────────────────────┘
                                         │ joint trajectory
                    ┌────────────────────▼─────────────────────┐
                    │  reference sources                       │
                    │  pose targets · joint trajectories       │
                    └────────────────────┬─────────────────────┘
                                         │ Reference
                    ┌────────────────────▼─────────────────────┐
                    │  kinematics  (Pinocchio, from the URDF)   │
                    │  FK · Jacobian · frame composition        │
                    └────────────────────┬─────────────────────┘
                                         │ ControllerState
                    ┌────────────────────▼─────────────────────┐
                    │  control law  (pure Eigen, no I/O)        │
                    │  PD · DLS · null space · safety filter    │
                    └────────────────────┬─────────────────────┘
                                         │ q̇
                    ┌────────────────────▼─────────────────────┐
                    │  actuation  (clamp · integrate · lead)    │
                    └────────────────────┬─────────────────────┘
                                         │ JointCommand (rad)
   ══════════════════════════════════════▼══════════════════════════  RobotIo
        Takeover() · Exchange(command) → RobotState · Release()
   ═════════════════╤═══════════════════════════════════╤════════════
                    │                                   │
        ┌───────────▼─────────────┐        ┌────────────▼────────────┐
        │  HardwareRobotIo        │        │  MujocoRobotIo          │
        │  Kortex sessions        │        │  mjModel / mjData       │
        │  servoing guard         │        │  actuators, contacts    │
        │  joint-limit gates      │        │  scene, mocap torso     │
        │  fault + freshness      │        │  N × mj_step per cycle  │
        │  deg ↔ rad, wrap        │        │  viewer hooks           │
        └─────────────────────────┘        └─────────────────────────┘
```

The line sits below actuation and above the plant. It is one line, crossed by
two data types and three methods.

## 6. Which files move behind the boundary

**Into `HardwareRobotIo`** — from `Christian_control/basic_control/src/`:

- `Hardware.h/.cpp` entirely: `Connect` (the two Kortex sessions),
  `CyclicSession` (the command frame and its id stamping), `read_feedback`,
  and both startup gates `EnsureJointLimits` / `VerifyKinematicHardLimits`.
  The run-log machinery in the same file (`LoopLog`, `LoopLogWriter`,
  `WriteCsvRow`) is *not* plant access and should split out rather than move.
- `Safety.h/.cpp` — the servoing-mode guard and fault decoding.
- `Freshness.h`, `StopPriority.h` — cyclic acknowledgement tracking and stop
  arbitration.
- `ProcessLock.h/.cpp` — one process per robot IP.
- The takeover and teardown halves of `Runner.cpp`: steps T1–T6 and D1–D2 as
  documented in `Runner.h`, plus the degree/radian and winding conversion at
  lines 148-149 and 301-303.

**Into `MujocoRobotIo`** — from `/home/christian/msc_project/cpp/src/sim/`:

- `MujocoBackend.h/.cpp` becomes the implementation nearly as-is; it already
  has the right shape.
- `Backend.h` is promoted to the shared interface header (with
  `capabilities()` added), or is replaced by it.

**Stays above the boundary, shared** — these are the files that become the one
controller:

- law: `ReactiveLaw.h` (hardware) or `ReactiveController.h/.cpp` +
  `SafetyFilter.h/.cpp` (simulation) — §7 decides which.
- composition: `Controller.h/.cpp` or `Servo.h/.cpp`.
- actuation: `Actuation.h/.cpp` or `PositionActuation.h/.cpp`.
- types: `State.h` and `core/Types.h` merge into one shared header.
- kinematics: `Kinematics.h/.cpp` (`DualArmKinematics`) and
  `PinModel.h/.cpp` + `Frames.h/.cpp` merge into one URDF-backed layer.
- references: `Targets.h/.cpp`, `JointTrajectory.h/.cpp`, `TargetSource.h`,
  `Arrival.h`.
- cycle ordering: the middle of `Runner.cpp` / `control/Runner.cpp`.

**Stays simulation-specific, above the boundary** — `render/`, `sim/Motion`,
`sim/Targets`, `sim/TargetMotion`, `sim/TargetTrajectory`, `sim/DesiredPos`,
`scene.xml`, the actuator block, the viewer entry points.

**Retired** — `sim/assets/kinova_gen3/gen3.xml` as a kinematic authority (it
stays only if you want a standalone single-arm scene), and the
`{side}_joint_{n}` naming it forces.

**Duplicated today, needs a decision** — `msc_project/planning/` and
`msc_project/cpp/src/planning/` overlap with `Christian_control/planner_bridge`.
That is a second consolidation and I would keep it out of this one.

## 7. Options

Ranked easiest to hardest. All four assume the URDF-generation step of §4,
which is common to every route and low-risk.

**A. Fit `HardwareRobotIo` behind the simulation's existing `PlantBackend`,
and adopt the simulation controller as the shared core.**
*What it does:* keeps `msc_project/cpp` as the trunk, writes one new backend
implementing three methods against Kortex, and moves the hardware's plant
access behind it.
*Difficulty:* moderate. The interface exists; the work is the Kortex
implementation plus the takeover sequence.
*Advantage:* smallest amount of new design; the dual-arm, world-frame,
torso-aware structure is the one you actually want long-term.
*Risk:* the simulation controller has never driven hardware. It lacks the
joint-trajectory channel the planner emits into, the winding handling,
the deadband limit avoidance and the damped projector — all four are
hardware lessons paid for in this repository, and all four would have to be
ported *into* it. It also has no run log; the 141-column CSV is the evidence
base for the thesis.

**B. Fit `MujocoRobotIo` behind a new interface extracted from the hardware
controller, and adopt the hardware controller as the shared core.**
*What it does:* keeps `basic_control` as the trunk, extracts `RobotIo` from
`Runner.cpp`'s hardware half, and wraps `MujocoBackend` to implement it.
*Difficulty:* moderate, comparable to A but distributed differently — more
extraction, less new implementation.
*Advantage:* the hardware-hardened behaviour, the planner integration and the
run log all survive untouched, and no motion-path code changes on the arm.
The URDF, DH generation and reference frame are already canonical here.
*Risk:* the hardware core is single-arm-per-process and base-frame-centric.
Dual-arm simulation in one process, the world frame and the torso all have to
be added, which is real design work, not just plumbing. The human-safety
filter would have to be ported the other way.
*This is the one I recommend*, for one reason: the hardware core's
constraints are the ones you cannot negotiate with, and its extra structure
(winding, trajectory channel, logging, planner wire format) is expensive to
re-derive whereas the simulation's extra structure (dual arm, world frame) is
additive and testable without hardware.

**C. Extract a third, neutral core from both, and fit both backends to it.**
*What it does:* takes the union — dual-arm, world-frame, both reference
channels, both null-space objectives selectable, one log — and rebuilds
`basic_control`'s and `msc_project/cpp`'s controllers as one.
*Difficulty:* high. This is the largest of the four.
*Advantage:* the only route that leaves no duplication anywhere and no
"which one won" ambiguity in the thesis.
*Risk:* a long window where neither stack is trustworthy, and every hardware
run during it is against code that has changed underneath. Given the j6 fault
and the limited remaining project time, I would not start here.

**D. Share only the model, not the controller.**
*What it does:* does the URDF generation of §4 so both stacks build from one
kinematic description, adds the `capabilities()` notion, and stops. Two
controllers remain.
*Difficulty:* low — a day or two.
*Advantage:* removes the mounting-geometry and tool-frame divergences
immediately, which are the two that produce silently wrong numbers, and buys
time to decide between A and B.
*Risk:* it does not deliver the requested architecture; the two control laws
keep drifting apart, and every simulation result still needs a caveat about
which law produced it.

D is also a clean **first stage of B**, and that is how I would sequence it:
do D now to kill the frame divergences, then B.

## 8. Evidence and provenance

Probes were run read-only, in `$CLAUDE_JOB_DIR/tmp`; no repository file was
modified. MuJoCo 3.10.0, `msc_project/.venv`.

1. `MjModel.from_xml_path(GEN3_dual_mounted.urdf)` → `ValueError: Error
   opening file 'package://…/spherical_wrist_1_link.STL'`.
2. Same file, mesh prefix stripped, `<mujoco><compiler meshdir=…/></mujoco>`
   injected → loads, `nq=14 nv=14 nu=0 njnt=14 nsite=0`; joints
   `Actuator1..7`, `leftActuator1..7`; `jnt_type` all hinge;
   `jnt_limited = [F,T,F,T,F,T,F]` per arm.
3. Adding `fusestatic="false"` → `nbody=21`, retaining `mount`, `base_link`,
   `EndEffector_Link`, `ConfiguredTool_Link`, `leftbase_link`,
   `leftEndEffector_Link`. `balanceinertia` not required.
4. `<asset><model file="…urdf"/></asset>` + `<attach body="mount"
   prefix="srl_">` inside a mocap `torso` → compiles, `nbody=22`, fourteen
   `srl_`-prefixed joints.
5. FK cross-check at `q = [0.11, −0.22, 0.33, −0.44, 0.55, −0.66, 0.77]`,
   both expressed in `base_link`: URDF `EndEffector_Link` at
   `(−0.392936648, 0.158120945, 1.000547735)`, MJCF `pinch_site` at
   `(−0.392936382, 0.158128276, 1.000546704)`. Δposition **7.408 µm**,
   Δrotation **10.42 µrad**. URDF `ConfiguredTool_Link` is a further
   **0.120000 m** from the flange.

Source references, all read this session: `Config.h` (frames, 500 Hz,
gains, limits); `dual_arm_mounting.yaml` (separation, tilt, provenance
warning); `GEN3_dual_mounted.urdf` (joint names, `EndEffector` and
`ConfiguredTool` offsets, mount joints); `Kinematics.h/.cpp` and
`test_dual_arm_model.cpp` (`nq=22`, `nv=14`, name-resolved indices);
`Runner.h/.cpp` (takeover sequence, deg→rad at 148-149);
`Hardware.h` (sessions, gates, log format 9);
`tools/set_joint_limits.cpp:52` (1-based device id);
`msc_project` `sim/world.py`, `controller/backend.py`, `controller/runner.py`,
`config/control.toml`, `sim/scene.xml`, `sim/assets/kinova_gen3/gen3.xml`,
and `cpp/src/{sim/Backend.h, sim/MujocoBackend.h/.cpp, core/Types.h,
kinematics/PinModel.h/.cpp, kinematics/Frames.h, control/Runner.h,
control/Servo.h, control/ReactiveController.h}`.

## 9. Open questions for Christian

1. Which controller core is "the existing controller core" — §7 A or B? My
   reading of your prompt is the hardware one, since it is what commands the
   real SRL, but the simulation one is the better-shaped starting point and
   the prompt does not settle it.
2. The mounting geometry disagreement (§3.5) has to be resolved in the URDF's
   favour, which moves the simulated arms ~43 mm and ~21°. Is the current
   simulation tuning something you need to preserve, or is a shift acceptable?
3. Should the simulation control `ConfiguredTool_Link` (matching hardware) or
   the flange? Matching hardware is correct but changes every simulated
   Cartesian result by 120 mm.
4. `dual_arm_mounting.yaml` says its numbers are inherited rather than
   surveyed. Is a rig survey plausible before this consolidation, so it is
   done once against measured geometry?
