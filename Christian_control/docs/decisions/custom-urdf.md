# Decision: one mounted dual-arm runtime model, right-only actuation

The controller's single runtime model is
`model/GEN3_dual_mounted.urdf`. Its world root, fixed mounting
transforms, dual branches, and arm/link names come from the downloaded
`GEN3_URDF_V802.urdf`. Each arm's Gen3 actuated chain has been cross-checked
against Kinova's official `ros_kortex` v2.5.2
`GEN3-7DOF-VISION_ARM_URDF_V12.urdf` and uses its exact URDF values:

- joints 1, 3, 5, and 7 are continuous; joints 2, 4, and 6 are revolute;
- bounded ranges are ±2.24, ±2.57, and ±2.09 rad for joints 2, 4, and 6;
- velocity fields are 1.3963 rad/s for joints 1–4 and 1.2218 rad/s for
  joints 5–7;
- effort fields are 39 N·m for joints 1–4 and 9 N·m for joints 5–7;
- transforms, axes, link inertials, and mesh geometry match the official V12
  arm, while the mesh prefix uses the checked-in Kortex assets;
- the interface/end-effector links use Kinova's zero inertial. No tool or
  payload mass is represented.

Project-specific differences remain explicit: the common `world` root and both
mounted branches are retained, joint/link names are not renamed, and the
official vision camera frames are not included:

- right: `world -> base_link`, translation `(0, -0.0567075, 0)` m, roll
  `+1.2085` rad;
- left: `world -> leftbase_link`, translation `(0, +0.0567075, 0)` m, roll
  `-1.2085` rad.

A URDF `<origin>` composes as `Trans(xyz)` then `Rot(rpy)`, so one fixed joint
per arm expresses each mount directly.

## The world origin sits at the base midpoint (2026-08-05)

The mount geometry above is *the same rig* it always described, re-expressed
about a different origin. It previously reached each `base_link` through an
intermediate `base_linktras` / `leftbase_linktras` link: roll first, then
translate `0.16` m along the rolled y axis. That put `world` `0.149614` m
*above* the two base origins — a torso-apex point, equidistant (`0.16` m) from
both bases but not between them.

`world` now sits at the midpoint of the two `base_link` origins, so world y
reads directly as "between the arms" and world z as "at mount height". The
separation (`0.113415` m) and tilt (`±1.2085` rad) are unchanged, and the
intermediate links carried no geometry and are gone. Frame count drops from 44
to 40; `nq = 22` and `nv = 14` are unaffected, because every mounting joint is
fixed.

The numbers now live in `model/dual_arm_mounting.yaml`, which is
their declared source of truth, and `tests/test_dual_arm_mounting.cpp` (ctest
`dual_arm_mounting`) fails the test step if the URDF drifts from it.

**These values are inherited, not measured.** They arrived fully formed in
commit `af116a5c` with no measurement note, and the original `0.16` m
base-to-world radius was a round number. They describe the mount's intended
geometry. Replace them with surveyed values when the rig is measured; the YAML
and the URDF mounting block are the only two places that change.

## The controller's internal Cartesian frame is still `base_link`

The mounted branches remain part of the model, but they do not define the
controller's internal Cartesian frame. `DualArmKinematics` transforms the
right tool pose and all six Jacobian rows from the model-root axes into the
right arm's `base_link` frame, and the Jacobian rows the control law consumes
are in those axes. The controller no longer has Cartesian *targets* at all
(see the retired section below); what remains base_link-framed is the
diagnostic surface.

World-frame access is **additive** and does not re-frame the control law.
`world` is the URDF root, so Pinocchio's `oMf` placements already are
world-frame poses; the base-frame methods are the ones doing extra work.
`DualArmKinematics` adds `WorldFromBase`, `ToolPoseInWorld`, and the
`PointBaseToWorld` / `PointWorldToBase` pair (`p_world = T_world_base * p_base`
and its inverse) for either arm. `ReactiveLaw`, `Safety`, and the arrival
monitors continue to reason in `base_link` axes, unchanged.

## One reference frame for the pipeline (2026-08-05)

`config::kReferenceFrame` (`world | right_base | left_base`, default `world`)
is the single switch deciding which frame Cartesian **targets are read in**
and Cartesian **quantities are reported in**, across both projects. A goal
file may override it per file with a `frame:` key governing that whole file;
a bare `--goal X Y Z` always follows the constant.

The transforms themselves stay in the URDF. Nothing in C++ hardcodes
`T_world_base`: the bridge obtains it through
`pinocchio_kinematics_adapter::WorldFromBase`, which reads the loaded model.
Changing `dual_arm_mounting.yaml` and the URDF therefore needs no code change
anywhere — verified by moving the mount 0.30 m and watching the same world
goal resolve to a different `base_link` point with the same binary and no
rebuild.

**The planner stays base_link internally, deliberately.** `gpmp2`'s
`ObstacleSDFFactorArm` pairs the arm model and the SDF in one factor, so they
must share a frame; putting the arm model in world while the SDF grid stayed
`base_link` would leave every collision check silently wrong. World input is
therefore converted **once, at the bridge boundary**, and everything below
that line — grid-bounds check, IK seed, solve — is `base_link`.

Two things the constant deliberately does not govern, because it cannot:

- `ToolPoseAndJacobianInBaseLink` is permanently `base_link`; `utils.cpp`
  composes it with `DhRootInBaseLink().inverse()`, which is only correct in
  that frame.
- `tools/print_dual_arm_fk` prints world and base side by side, because that
  comparison is what validates the mounting transform in the first place.

An obstacle `box:` may only be given in `right_base`. An axis-aligned box in
another frame is not axis-aligned after the mount rotation, and the SDF grid
can only represent axis-aligned boxes; the bridge refuses rather than
shipping a box that is not the one asked for.

## Retired: world-frame target lines on the controller pipe

The controller briefly accepted `WORLD x y z` / `BASE x y z` target lines,
converted to `base_link` in `ParsePoseTarget`. That grammar is GONE: stage 2
deleted the whole Cartesian pose path, so the controller now takes only
`TRAJ_BEGIN … TRAJ_END` joint-trajectory blocks and has no Cartesian target
input at all (`stage2-joint-trajectory-following.md`).

Frame selection therefore lives entirely on the planner side now — see the
section above. The controller's remaining Cartesian surface is diagnostic:
the startup FK print, and the `pd_x..z` / `p_x..z` CSV columns, which the
joint path leaves as NaN.

`tools/print_dual_arm_fk` prints both arms' tool frames in world and in their
own base frames at a chosen configuration. It links the URDF only — not Kortex
— so it cannot connect or command, and is safe to run with the robot off. The
two arms' tool frames are NOT the same point: the right chain ends at
`ConfiguredTool_Link` (the mounted tool) and the left at its bare flange, so
their positions are not a symmetry check. The left arm has no connection and
no feedback, so every left figure is open-loop against the URDF.

Kinova's User Guide Tables 39 and 40 publish 128.9/147.8/120.3 deg position
ranges and 79.64/69.91 deg/s general speed limits. Those values are close to,
but not numerically identical to, Kinova's official URDF fields above. The
runtime model follows the official URDF exactly; controller/trajectory
validation in `Config.h` follows the user-guide degree values. Neither source
proves the installed robot's configured soft limits or actuator firmware
thresholds.

## Explicit 14-DoF-to-7 boundary

The Pinocchio model has 14 velocity variables (`nv = 14`) but 22 configuration
variables (`nq = 22`): each of the eight continuous joints across both arms is
stored as `(cos(q), sin(q))`. Neither the control policy nor the Kortex command
path is 14-wide.
`src/Kinematics.{h,cpp}` (`DualArmKinematics`) is the required boundary:

1. resolve the seven right joints (`Actuator1` through `Actuator7`) and seven
   left joints (`leftActuator1` through `leftActuator7`) by exact URDF name;
2. require the official continuous/revolute pattern and 14 one-variable
   velocity DoFs whose configuration and velocity ranges cover the full model;
3. encode continuous angles as `(cos(q), sin(q))` while assembling full `q`
   from measured right-arm radians plus the explicit `config::kLeftNominalRad`
   left configuration;
4. compute full-model FK and the full 6-by-14 Jacobian;
5. transform the right tool pose and both Jacobian row blocks from model-root
   axes into the right `base_link` frame;
6. select only the seven right-joint columns, in Kortex actuator order, for
   the existing 7-DoF controllers.

The left branch is model-only. There is one compiled robot IP, one `Connect`,
one `CyclicSession`, and one seven-entry actuator command frame, all for the
right arm. No left feedback is synthesized, and no left hardware session or
command can be constructed by this executable.

The official URDF's 1.3963/1.2218 rad/s fields remain model metadata. They do
not prove or configure the limits that bind low-level position streaming. The
right-arm actuator command clip follows the User Guide's general limits
(`config::kQdotLimitDegS`: 79.64 deg/s for joints 1–4 and 69.91 deg/s for
joints 5–7). Gains, loop timing, stop policy, initialization, actuator
integration, and output columns are otherwise unchanged.

Hardware-free tests establish model loading, official joint representations
and limit values, continuous-angle encoding, fixed mount transforms, exact
name/index mapping, left nominal-state persistence, right-base pose/Jacobian
transformation, full-to-right Jacobian column selection, and the seven-wide
command interface. They do not prove
robot identity, physical mounting calibration, collision safety, or safe
hardware behavior. A model change is not hardware safety evidence.
