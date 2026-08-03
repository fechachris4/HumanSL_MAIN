# Decision: one mounted dual-arm runtime model, right-only actuation

The controller's single runtime model is
`basic_control/config/GEN3_dual_mounted.urdf`. Its world root, fixed mounting
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

- right: `world -> base_linktras`, roll `+1.2085` rad; then
  `base_linktras -> base_link`, translation `(0, -0.16, 0)` m;
- left: `world -> leftbase_linktras`, roll `-1.2085` rad; then
  `leftbase_linktras -> leftbase_link`, translation `(0, +0.16, 0)` m.

The mounted branches remain part of the model, but they do not define the
controller's Cartesian interface. `DualArmKinematics` transforms the right
tool pose and all six Jacobian rows from the model-root axes into the right
arm's `base_link` frame. Operator targets, printed positions, orientations,
and CSV Cartesian fields use that same right-base frame. Reach telemetry is
therefore measured from the zero origin of `base_link`.

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
