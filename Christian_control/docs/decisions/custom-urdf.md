# Decision: one mounted dual-arm runtime model, right-only actuation

The controller's single runtime model is
`basic_control/config/GEN3_dual_mounted.urdf`. It is a tracked copy of the
downloaded `GEN3_URDF_V802.urdf`; the only source transformation is:

- Windows-local mesh prefix `D:/urdf4/Simulation/meshes/` becomes
  `package://kortex_description/arms/gen3/7dof/meshes/`;
- CRLF line endings become LF.

All links, movable joints, limits, inertials, and fixed transforms are
otherwise unchanged. In particular, the common `world` root and both mounted
branches are retained:

- right: `world -> base_linktras`, roll `+1.2085` rad; then
  `base_linktras -> base_link`, translation `(0, -0.16, 0)` m;
- left: `world -> leftbase_linktras`, roll `-1.2085` rad; then
  `leftbase_linktras -> leftbase_link`, translation `(0, +0.16, 0)` m.

This deliberately changes Cartesian semantics from the former per-arm local
base frame. Operator targets, FK, Jacobian rows, printed positions,
orientations, and CSV Cartesian fields are now expressed in the dual model's
mounted world/common frame. The reach telemetry flag still measures from the
right `base_link` origin, transformed into that common frame.

## Explicit 14-to-7 boundary

The Pinocchio model has 14 configuration and velocity variables, but neither
the control policy nor the Kortex command path is 14-wide.
`src/math/DualArmKinematics.{h,cpp}` is the required boundary:

1. resolve the seven right joints (`Actuator1` through `Actuator7`) and seven
   left joints (`leftActuator1` through `leftActuator7`) by exact URDF name;
2. require 14 one-variable joints whose indices are disjoint and cover the
   full model;
3. assemble full `q` from measured right-arm radians plus the explicit
   `config::kLeftNominalRad` left configuration;
4. compute full-model FK and the full 6-by-14 Jacobian;
5. select only the seven right-joint columns, in Kortex actuator order, for
   the existing 7-DoF controllers.

The left branch is model-only. There is one compiled robot IP, one `Connect`,
one `CyclicSession`, and one seven-entry actuator command frame, all for the
right arm. No left feedback is synthesized, and no left hardware session or
command can be constructed by this executable.

The downloaded URDF's 0.8727 rad/s joint-limit fields remain unchanged as
model metadata. They do not replace the established right-arm actuator command
clip (`config::kQdotLimitDegS`: 79.6 deg/s for joints 1-4 and 69.9 deg/s for
joints 5-7). Gains, loop timing, stop policy, initialization, actuator
integration, and output columns are unchanged.

Hardware-free tests establish model loading, fixed mount transforms, exact
name/index mapping, left nominal-state persistence, full-to-right Jacobian
column selection, and the seven-wide command interface. They do not prove
robot identity, physical mounting calibration, collision safety, or safe
hardware behavior. A model change is not hardware safety evidence.
