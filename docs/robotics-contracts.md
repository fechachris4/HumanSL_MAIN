# Robotics contracts

These conventions make mathematics, hardware data, and experiments interpretable
across the repository. Subsystems may define additional frames or fields, but must
state them explicitly and remain consistent with these contracts.

## Dimensions and mathematical types

- A Kinova Gen3 has seven physical joints. Application-level joint position,
  velocity, torque, and command quantities use fixed-size seven-element types.
- Cartesian position is 3D. Twist, spatial velocity, wrench, and Cartesian error
  are six-dimensional and must state component ordering and reference frame.
- Rotations use an explicit representation (`Eigen::Matrix3d`, quaternion, or
  angle-axis) and transforms use `Eigen::Isometry3d` or an equivalently clear type.
- Pinocchio configuration and velocity dimensions come from `model.nq` and
  `model.nv`. They may differ from seven, particularly for continuous-joint
  representations, so keep them dynamic at the Pinocchio boundary.
- Optimization horizon and variable observation dimensions may remain dynamic.
  Dynamic size is not a substitute for an invariant physical dimension.

## Units

Use SI units internally:

| Quantity | Internal unit |
|---|---|
| position | metre (`m`) |
| angle | radian (`rad`) |
| linear velocity | metre per second (`m/s`) |
| angular velocity | radian per second (`rad/s`) |
| force | newton (`N`) |
| torque | newton metre (`N m`) |
| time and duration | second (`s`) |

Kortex or operator-facing degrees are converted at a named boundary. Names carrying
non-SI boundary values include a suffix such as `_deg` or `_deg_s`. Never mix or
compare degrees and radians implicitly.

## Coordinate frames and transforms

- Use `T_A_B` to mean the transform that maps coordinates expressed in frame `B`
  into frame `A`: `p_A = T_A_B * p_B`.
- Suffix vectors when the frame is not obvious from a narrow mathematical scope,
  for example `position_world`, `twist_base`, or `wrench_tool`.
- Document the expression frame, reference point, and component ordering for every
  twist, wrench, Jacobian, and Cartesian error.
- Do not compose transforms whose direction is encoded only in comments or local
  variable history. Name intermediate transforms when that exposes the frame chain.
- Record the calibration source and timestamp for transforms between Vicon world,
  robot base, flange, tool, gripper, object, and human-marker frames.

## Time and state snapshots

- Use a monotonic steady clock for scheduling, latency, age, and duration. Use wall
  time only for human-readable experiment identity and record its timezone.
- A control tick consumes one immutable robot-state snapshot obtained from one
  feedback or command/feedback exchange.
- A snapshot should contain a monotonic timestamp or sequence, joint position and
  velocity, available effort/current data, robot/actuator state and faults, and an
  explicit validity/status value.
- Derived kinematics, safety decisions, controller output, and telemetry for a tick
  all refer to the same snapshot. A second state read in the same tick is forbidden
  unless the algorithm explicitly models the time difference.

## Configuration and reproducibility

- Load experiment configuration once before hardware initialization, validate it,
  and expose it as a typed immutable object. Compile-time constants remain suitable
  for dimensions and hard safety ceilings.
- The current `Christian_control/basic_control/src/Config.h` is a compile-time
  configuration source; until deliberately migrated, treat it as the single source
  for that executable rather than introducing a hidden second source.
- Each experiment record should identify:
  - Git commit and whether the working tree was dirty;
  - resolved configuration and controller parameters;
  - URDF and relevant configuration hashes;
  - controller/mode and requested sampling period;
  - random seeds and input-data identifiers;
  - robot identity and relevant firmware/SDK information when available;
  - wall-clock start time and monotonic elapsed time.
- Randomized experiments use an explicit recorded seed. Do not seed scientific
  results implicitly from wall time without recording the resulting seed.

## Logging and failure records

- Structured events record lifecycle, mode transitions, configuration, warnings,
  faults, safe stops, and shutdown with stable field names.
- High-frequency telemetry records commanded and measured quantities, snapshot
  sequence/time, status, units, and frames through a buffered CSV or binary path.
- Preserve the first causal failure and record cleanup failures separately.
- Logs must contain enough context to distinguish controller failure, stale data,
  SDK/communication failure, robot-side safety action, and operator stop.

## Safe-stop contract

Every hardware-moving subsystem documents its exact safe-stop sequence, including
which failures trigger it, what command is sent when communication remains valid,
what happens when it does not, how servoing mode is restored, and which action is
safe to retry. A generic instruction such as “stop safely” is insufficient.

Safe stop must not introduce unplanned retreat or gripper motion. Hardware-specific
actions require explicit analysis and an operator-approved test procedure.
