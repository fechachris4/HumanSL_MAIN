# Repository architecture

HumanSL is a research robotics repository containing a legacy dual-arm pipeline,
an actively documented single-arm controller, and a hardware-free joint-control
experiment. This document records technical responsibilities and dependency
direction; detailed file maps remain in subsystem documentation.

## Current components

| Area | Location | Responsibility | Current status |
|---|---|---|---|
| Application orchestration | root `main.cpp` | Dual-arm state machine, wiring, threads, experiment lifecycle | Legacy hardware-facing path |
| Trajectory planning | `TrajectoryGeneration/` | GPMP2/GTSAM planning, optimization, obstacle factors, trajectory initialization | Legacy planning library |
| Dynamics and execution | `TrajectoryExecution/` | Pinocchio dynamics, controller code, Kortex trajectory execution | Legacy mixed math/hardware layer |
| Motion capture | `ViconDataStream/` | Vicon connection and conversion of tracked data | External-system boundary |
| Joint MPC seam | `TrajectoryRealTime/joint_mpc/` | Eigen-only controller, simulator, switching, and offline tests | Hardware-free experimental path |
| Single-arm controller | `Christian_control/basic_control/` | Kortex connection, feedback, joint motion, kinematics, recording, and timing | Active documented controller |
| Models and experiment data | `config/` and subsystem `config/` | URDFs, limits, task parameters, and recorded inputs | Project data/configuration |
| Bundled dependencies | `third_party/` | Kortex, Vicon, Eigen, Pinocchio, GPMP2/GTSAM, and related libraries | Vendored; do not edit casually |

The current checkout contains `TrajectoryRealTime/joint_mpc/` but no
`TrajectoryRealTime/CMakeLists.txt`, while the root CMake build expects that
missing directory. Treat the top-level build description as incomplete until an
explicit task resolves the legacy/runtime relationship.

## Technical layers

Organize new work by robotics responsibility rather than enterprise pattern:

1. **Mathematics and models:** deterministic kinematics, dynamics, transforms,
   planners, and controller equations. No Kortex, Vicon, filesystem, or terminal
   dependencies.
2. **Control and safety:** controller state, mode transitions, command validation,
   limits, fault decisions, and safe-stop selection. Consumes snapshots and emits
   commands/status.
3. **External boundaries:** Kortex/Vicon adapters, clock, configuration loading,
   and recorder sinks. These translate external formats and failures into project
   types.
4. **Application orchestration:** constructs dependencies, initializes the system,
   runs the state machine/control driver, and coordinates shutdown.
5. **Offline tooling and tests:** simulation, replay, plotting, diagnostics, and
   deterministic verification.

The intended dependency direction is application toward control/mathematics and
external adapters; control depends on project data contracts, not on concrete SDK
clients. External adapters do not contain controller mathematics. Offline tests
must be able to exercise controller mathematics without robot hardware.

## Boundary guidance

- `main.cpp` should read like high-level experiment pseudocode: configuration,
  construction, initialization, run, shutdown, and exit status.
- Use interfaces at external boundaries where simulation/replay or multiple real
  implementations justify them: for example `RobotInterface`, `Recorder`, and
  `Clock`. Do not create an interface for every mathematical class.
- Keep Pinocchio model-sized representations at the model boundary. Convert to
  fixed Gen3 application types explicitly when dimensions and semantics permit.
- Hardware feedback is converted once into an immutable state snapshot per tick.
- High-rate recording crosses into a buffered non-critical writer; it is not a
  responsibility of controller mathematics.

## Documentation ownership

- Subsystem module maps and exact hardware behavior live beside the subsystem,
  such as `Christian_control/docs/`.
- Decisions record why a durable choice was made, together with the empirical
  faults, limitations, and unresolved questions that motivated it.
- Plans and milestone notes are task artifacts, not permanent agent instructions.
