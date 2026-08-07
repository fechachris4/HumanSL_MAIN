# Simulation / hardware unification — scoping

Status: scoping only. No code changed. Two decisions are taken (below);
the controller-core decision is deliberately left open pending this
comparison.

Date: 2026-08-07.

## What was asked

Determine the smallest interface that lets one controller core run against
either the real SRL or MuJoCo, with the same URDF and canonical joint
configuration authoritative for both planning and simulation, and without
creating a second manually maintained kinematic model.

## The finding that reframes the question

The work spans two repositories, not one. `HumanSL_MAIN` contains the
hardware controller and the GPMP2 planner; `msc_project` contains the
MuJoCo port and a second controller. There is no MuJoCo code in
`HumanSL_MAIN` at all.

More importantly, **the abstraction being asked for already exists.**
`msc_project/cpp/src/sim/Backend.h` defines:

```cpp
class PlantBackend {
 public:
  virtual PlantState Takeover() = 0;
  virtual PlantState Exchange(const JointPositionCommand& command) = 0;
  virtual void Release() = 0;
};
```

Its own comment describes it as the "minimal plant boundary shared by MuJoCo
and any future hardware backend". `MujocoBackend` implements it. No hardware
sibling exists yet.

So the task is not to design a boundary. It is to write the second
implementation, and to decide which controller lives above it.

### Why this boundary is the right one

`Exchange` means one command out, one state back, per control cycle, with
the backend owning whether that means advancing simulation time or waiting
for a cyclic UDP reply.

The hardware side already has the same shape. In
`basic_control/src/Hardware.h`:

```cpp
k_api::BaseCyclic::Feedback CyclicSession::Send(const JointVector& setpoints_deg);
```

One command in, one feedback out, once per cycle. That is `Exchange` with
different spelling and different units. The correspondence is close enough
that a `HardwareBackend` is mostly unit conversion and index mapping, not
new control logic.

## The two controllers are complementary, not rival

This is the most important result of the comparison, and it changes what
"pick a core" means.

**`HumanSL_MAIN` safety (`basic_control/src/Safety.h`, 215 + 380 lines) is
hardware supervision.** It classifies why the loop stopped across ten
`LoopStop` reasons, decodes Kortex actuator and base fault banks, runs the
pre-takeover readiness gate, watches following error and stale cyclic
acknowledgements, and carries an RAII guard that restores `SINGLE_LEVEL`
servoing on every exit path. It includes `BaseClientRpc.h` and
`BaseCyclicClientRpc.h`. **None of this can exist in simulation** — there
are no fault banks, no servoing mode, and no cyclic acknowledgement to go
stale in MuJoCo.

**`msc_project` safety (`control/SafetyFilter.h`, 73 + 445 lines) is
motion-level safety.** It is a control-barrier-function velocity filter:
for every arm sphere it imposes

```
distance_jacobian · qdot >= -recovery_gain · signed_clearance
```

and projects the requested joint velocity into that half-space set with a
QP (OSQP), re-checking every constraint independently before use. It
includes only `config/RuntimeConfig.h` and `core/Types.h` — **no MuJoCo, no
hardware.**

That second one is worth pausing on. The project instructions in `CLAUDE.md`
name a CBF-style safety filter — "yields the closest command to the
requested one that provably keeps the arm inside the safe set" — as the most
considered alternative to an abrupt stop. It is already built, in
`msc_project`, and it is hardware-independent.

So the two safety layers are not competing implementations of the same
thing. One sits **below** the boundary (backend-specific plant supervision);
the other sits **above** it (backend-independent motion policy). Choosing
between them was a false choice.

### Coupling audit

`msc_project` has **zero** Kortex references in `cpp/src` (the five matches
are documentation, a comment in `LinkSpheres.h`, and a Python generator
tool). All ten `control/*.h` files have zero MuJoCo includes. The controller
core there is already backend-clean.

`HumanSL_MAIN` confines Kortex to four modules — `Hardware.{h,cpp}`,
`Runner.{h,cpp}`, `Safety.{h,cpp}`, `Main.cpp`. Nineteen other files are
already hardware-free, including `Controller`, `ReactiveLaw`, `Kinematics`,
`Targets`, `Actuation`, `JointTrajectory`, `State` and `Arrival`.
(`Kinematics.h` and `Config.h` matched a Kortex grep only on the word
"Kinova" in comments.)

This is a better starting position than expected on both sides.

## Mapping

### Canonical joint names — three schemes

| | right | left |
|---|---|---|
| URDF (`GEN3_dual_mounted.urdf`) | `Actuator1..7` | `leftActuator1..7` |
| MuJoCo (`scene.xml`) | `right_joint_1..7` | `left_joint_1..7` |
| Kortex hardware | index `0..6` | index `0..6` |

MuJoCo's names come from `<attach model="gen3_arm" prefix="right_"/>`, which
composes a single-arm model twice.

### Indices

*Hardware:* `feedback.actuators(i)`, `i = 0..6`, **degrees**, per arm, via a
separate `CyclicSession` per arm (each arm is its own Kortex device at its
own IP).

*MuJoCo:* resolved once at `MujocoBackend` construction by name through
`mj_name2id`, then cached as `qpos_adrs_`, `dof_adrs_`, `ctrl_adrs_`,
`joint_id_`, throwing if a name is absent. State is read as
`data_->qpos[qpos_adrs_[i]]` and `data_->qvel[dof_adrs_[i]]`. **This is
already the mapping requested; it does not need building.**

*Pinocchio in `HumanSL_MAIN`:* `right_q_indices_` / `left_q_indices_` and
the matching `v_indices_`, resolved by joint name in `Kinematics.cpp`. The
dual model has **nq = 22, nv = 14**, because Kinova's continuous joints
1/3/5/7 use Pinocchio's two-value (cos, sin) representation. Configuration
and velocity indices therefore differ and must never be interchanged.

*Pinocchio in `msc_project`:* none needed. `PinModel` is single-arm with a
plain 7-vector.

### Frames

| | `HumanSL_MAIN` | `msc_project` |
|---|---|---|
| root | `mount` (rigid) | `world` → `torso` (**mocap, moving**) |
| arm base | `base_link`, `leftbase_link` | `right_base_link`, `left_base_link` |
| tool | `ConfiguredTool_Link` (right), `leftEndEffector_Link` (left) | `right_pinch_site`, `left_pinch_site` |

Two real discrepancies. First, the right arm's tool frame is a *configured
tool* while the left is a bare flange — the 120 mm difference already found
and fixed on 2026-08-06. Second, `mount` is rigid whereas `torso` is a mocap
body that moves; the simulation models wearer motion that the hardware
planner currently does not. Any shared controller must treat the torso pose
as an input, not a constant, or the hardware path silently pins it to
identity.

### Controller inputs and outputs

Across the boundary, in `msc_project/cpp/src/core/Types.h`:

- **In:** `PlantState { sample_time_s, nominal_dt_s, torso_pose_world,
  torso_twist_world, DualArm<ArmJointState> arms }`
- **Out:** `JointPositionCommand { DualArm<Vector7> position_rad }`

Both are plain Eigen with `enum class Side { Right, Left }` and
`kJoints = 7`. No MuJoCo type and no Kortex type crosses the boundary. That
is precisely why the boundary works, and it is the property to preserve.

Note the unit and shape gap to close in a `HardwareBackend`: radians vs
Kortex degrees, and one dual-arm command struct vs two independent
per-arm cyclic sessions.

### Frequencies

Both projects run **500 Hz**: `kControlDtS = 0.002` in `Config.h`, and
`nominal_dt_s = 0.002` in `config/control.toml`. `MujocoBackend` sets
`model_->opt.timestep = config.run.nominal_dt_s`, **overriding whatever the
MJCF declares**, so there is exactly one integration step per control cycle
and no substepping. The rates already agree; nothing needs reconciling.

## Model unification — decided

**The URDF is authoritative. The MJCF becomes generated plus a small
hand-written overlay.**

### Current state

`msc_project` does not consume a URDF anywhere. `PinModel` loads the
**MJCF** (`sim/assets/kinova_gen3/gen3.xml`) through Pinocchio, so MuJoCo
and Pinocchio there already share one model. Each project is internally
consistent; they simply disagree with each other.

### Why this direction

URDF → MJCF is the supported conversion direction (MuJoCo's compiler ingests
URDF). The reverse loses information: MJCF carries actuator gains, control
ranges, contacts and mocap bodies that URDF cannot express.

A decisive practical point: **`GEN3_dual_mounted.urdf` is already dual-arm**
— 20 links, 14 actuated joints (8 continuous + 6 revolute, matching Gen3's
1/3/5/7-continuous pattern twice), with `mount` as root and fixed
`right_base_mount` joints. Converting it yields **both arms directly**, so
`<attach prefix="right_"/>` is no longer needed and joint names unify on the
URDF's own `Actuator1..7` / `leftActuator1..7`. `MujocoBackend`'s name
construction then changes in exactly one place.

### What the overlay must supply

`gen3.xml` is only 110 lines, and most of it is kinematics that the URDF
already owns. What it adds is small and genuinely simulation-specific, which
is what makes this viable rather than a second model in disguise:

- four `<default>` classes: `visual`, `collision`, `large_actuator`
  (kp 2000, kv 100, forcerange ±105), `small_actuator` (kp 500, kv 50,
  forcerange ±52)
- seven `<position>` actuators per arm with `ctrlrange` on joints 2/4/6
- `<option integrator="implicitfast">` and a `<keyframe>`
- from `scene.xml`: the `torso` mocap body, the `right_target` /
  `left_target` mocap bodies, the named sites, and the `<contact>`
  exclusions

None of that is kinematic. The overlay never restates a link, a joint axis
or an origin, so there is no second kinematic model to keep in sync — which
is the constraint that mattered.

### Two obstacles, both surmountable, neither yet verified

1. **`package://` mesh URIs.** The URDF references
   `package://kortex_description/arms/gen3/7dof/meshes/*.STL`. MuJoCo's
   compiler cannot resolve `package://`; the paths must be rewritten
   relative to a `meshdir`. The 36 STL files are vendored under
   `third_party/ros_kortex/`, including the `7dof` variants the URDF names,
   so the assets exist.

2. **The massless root.** 20 links carry 19 `<inertial>` blocks. The
   exception is `mount`, which is a geometry-free, mass-free root link. As
   the fixed root it should fold into `worldbody`, but MuJoCo's treatment of
   a massless root has not been tested here.

Both are claims about a conversion **not yet run**. The smallest experiment
that settles them is to run MuJoCo's compiler on the URDF once, with the
mesh paths rewritten, and read the errors. That is the recommended next
step, and it is cheap.

## Proposed interface boundary

Unchanged from what exists — `PlantBackend` with `Takeover` / `Exchange` /
`Release`, carrying `PlantState` in and `JointPositionCommand` out.

What moves **behind** it (backend-specific, one implementation each):

| behind the boundary | from |
|---|---|
| `MujocoBackend` | `msc_project/cpp/src/sim/` (already there) |
| **`HardwareBackend`** (to be written) | wraps `Hardware.{h,cpp}` — `Connect`, `CyclicSession::Send`, degrees↔radians, per-arm sessions |
| Kortex fault decoding, readiness gate, servoing guard, following-error and stale-feedback watches | `basic_control/src/Safety.{h,cpp}`, `Runner.{h,cpp}` |

What stays **above** it (shared, backend-independent):

- the CBF `SafetyFilter` and `HumanSafety` envelope from `msc_project`
- `Servo`, `CylinderRouter`, `TargetSource`, `PlannedJointRunner`
- from `HumanSL_MAIN`, the already-hardware-free `Controller`,
  `ReactiveLaw`, `Targets`, `Actuation`, `JointTrajectory`, `Arrival`,
  `State`
- the GPMP2 planner and the whole `planner_bridge`, which are offline and
  already backend-agnostic

The asymmetry to keep in view: hardware supervision has no simulation
counterpart, so `MujocoBackend` will always implement a subset. The
boundary must let a backend report "not applicable" for fault state without
the controller above it treating that as nominal.

## What is not yet decided

Which controller core becomes the shared one. This document exists to inform
that choice rather than pre-empt it. The comparison above suggests the
question should be re-framed: `msc_project`'s core is already
backend-clean and carries the CBF filter, while `HumanSL_MAIN`'s value is
concentrated in the hardware supervision that belongs *below* the boundary
anyway. Those are largely additive rather than exclusive.

The real cost is not in choosing — it is that `HumanSL_MAIN`'s controller
and `msc_project`'s controller implement different control laws, and
whichever is retired takes its hardware-verified tuning with it. That is the
part deserving a deliberate decision, and it is unresolved.

## Honest limitations

- No conversion has been run. The URDF→MJCF claims are reasoned from the
  file contents, not demonstrated.
- No code was built or executed in this investigation.
- The two control laws were compared by interface and by safety
  architecture, not by behaviour. Whether `msc_project`'s reactive law
  reproduces the hardware-tuned behaviour of `basic_control` is untested and
  cannot be settled by reading.
- Nothing here has been validated against hardware, and nothing in it
  should be read as making any configuration safe to run.
