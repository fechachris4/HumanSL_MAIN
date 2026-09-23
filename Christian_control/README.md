# Christian_control

The control system for a wearable Supernumerary Robotic Limb built from Kinova
Gen3 arms. The objective: hold or track the end-effector pose **in the world
frame** while the wearer, and the backpack the arms are mounted on, move.

> **Status, September 2026.** This is my C++ controller for the dual-arm rig in
> [msc_project](https://github.com/fechachris4/msc_project), which has the
> hardware results. It links prebuilt x86-64 Linux binaries checked into
> `third_party/` (Kortex API, Vicon DataStream SDK, Pinocchio, GTSAM/GPMP2,
> Boost). The planning, control and panel tests run without the robot.

<img src="https://github.com/fechachris4/msc_project/raw/main/media/rig.jpg" width="420" alt="The dual-arm rig on the treadmill">

**One control cycle** (500 Hz, `kControlDtS` in `control/Config.h`): take the
latest 100 Hz Vicon mount pose; estimate the mount twist by finite difference
and a first-order low-pass (τ = 50 ms, `kViconMountTwistFilterTauS`); compose
the end-effector pose and Jacobian in the world frame through the mount
(Pinocchio); apply PD on the world-frame pose and twist error with damped least
squares and a damped null-space projector (`control/ReactiveLaw.h`); clamp joint
velocity and integrate to a position command (`control/Actuation.cpp`); send it
over the Kortex cyclic exchange (`runtime/Hardware.cpp`).

Every directory here is named for the engineering job it owns. None is named
after a vendor, an interface, or the history of how it came about.

| Directory | Owns | Builds |
|---|---|---|
| `model/` | The one geometric description of the machine: the URDF, its mounting table, and the physical joint-limit table. Data only. | — |
| `contracts/` | The typed values two subsystems must agree on: the planning request, the world-Cartesian trajectory, its text wire format. | `humansl_contracts` |
| `control/` | What decides a joint command: the Pinocchio model and kinematics (`humansl_robot_model`, shared with the planner), the compiled settings, and the whole per-cycle pipeline. Knows nothing about how a command is delivered. | `humansl_robot_model`, `humansl_execution_core` |
| `runtime/` | The program that runs `control/` against the real arm: the 500 Hz loop, Kortex I/O (owned by `humansl_runtime_hardware`, the only target set that links Kortex), safety decoding, telemetry, the worker threads. | `controller`, `humansl_runtime_hardware` |
| `tracking/` | External body tracking: markers in, a validated world pose and twist out. Split on the SDK boundary: only `humansl_vicon` links the DataStream SDK. | `humansl_tracking_core`, `humansl_vicon` |
| `planning/` | A planning request in, a validated world trajectory out. `optimisation/` is the GPMP2 layer. Depends on `humansl_robot_model`, never on controller machinery. | `humansl_planning`, `planner_bridge` |
| `simulation/` | The MuJoCo execution twin: the same `control/` core, driven by physics instead of hardware. | `humansl_sim` |
| `panel/` | The operator's browser surface: configure, plan, run, watch. | — |
| `docs/` | Decisions, code reads, runbooks, thesis notes. | — |

## The two flows

Everything this system does is one of these two paths. Each is readable from
about six files, and the directory names say which stage you are in.

**Tracking → world state → control → actuation → hardware**

```
tracking/src/ViconInterface.cpp      the DataStream SDK
  → tracking/src/SnapshotBuilder.cpp     mm→m, quaternion validation
  → tracking/src/MountTwistEstimator.cpp filtered mount twist
  → runtime/ViconSource.cpp              acquisition thread  (or ViconSourceStub.cpp)
  → runtime/BasePose.h                   sample contract + wait-free slot
  → runtime/Runner.cpp                   assembles the core's input each cycle
  → control/ExecutionCore.cpp            ── the per-cycle contract ──
        Frames.h                world assembly, world_T_base = world_T_mountseg · mount_T_base
        Kinematics.cpp          Pinocchio FK and Jacobian (RobotModel.cpp owns model/data)
        CartesianReference.cpp  hold → track → hold
        Controller.cpp          the law, via ReactiveLaw.h (DLS + null space)
        Actuation.cpp           velocity clamp, position integration
  → runtime/Hardware.cpp                 Kortex cyclic exchange, run-log CSV
  → control/ExecutionCore.cpp            ResolveStop, ranking the facts from the reply
        ← runtime/Safety.cpp, control/StopPriority.h
```

**Planning request → planner / IK / GPMP2 / collision → trajectory → runtime**

```
control/ExecutionCore.cpp            raises request_replan
  → runtime/PlanningRequestSlot.h        carrying contracts/PlanningRequest.h
  → runtime/InProcessPlanner.cpp         the non-real-time worker thread
  → planning/src/PlannerRuntime.cpp      the typed entry point
        PlannerConfig.cpp   planner.yaml, loaded strictly
        PlannerModel.cpp    where the DH root sits in Vicon world
        StartState.cpp      q_start, read from the run log by column NAME
        CartesianPath / PathFrames / PathIk    geometry, frame crossing, seeded IK
  → planning/src/PlanSolver.cpp
        PathAssembly.cpp                  approach + task phases on one time grid
        planning/optimisation/*           GPMP2: arm model, initialisation, optimisation
        StaticScene.h / MountSdf.cpp       authored primitives and analytic clearance
        ValidatePlan.cpp                  the one dense executable-plan validator
        PlanValidationReport.cpp          its evidence record
  → planning/src/WorldTrajectoryProjection.cpp   joints → world Cartesian
  → contracts/WorldCartesianTrajectory.h
  → control/CartesianTrajectoryMailbox.cpp       typed ownership handoff
  → control/CartesianReference.cpp               and back into the first flow
```

## The boundaries

These are the design rules the layout follows. The tests that used to enforce
the linkage, model-parity and characterization rules were removed with the
pre-migration suite (commit `0c10e17`, 20 Aug 2026) and have not been rebuilt,
so today these rules hold by build structure, not by a test.

- **`control/` never touches the robot.** No Kortex or Vicon SDK header is on
  its include path; only `humansl_runtime_hardware` links Kortex.
- **`simulation/` cannot reach the arm.** It never adds `runtime/`.
- **GPMP2 stays outside the 500 Hz loop.** Planning runs on the worker thread
  in `runtime/InProcessPlanner.cpp`, never in `Runner.cpp`'s cycle.
- **The planner has one executable boundary.** `ValidatePlan` checks exact
  start state, finite state, componentwise joint velocity/acceleration, joint
  position, authored-scene clearance, self clearance, and terminal equality.
  `REACHED` owns a trajectory to the requested terminal; `GOAL_BLOCKED` owns a
  trajectory to an explicitly shortened terminal; `FAILED` owns no trajectory.
  Only the first two cross the runtime mailbox boundary.
- **One URDF, one FK implementation.** `model/GEN3_dual_mounted.urdf` is parsed
  through `control/RobotModel.cpp` by every project.
- **Only `runtime/tools/` can reach the arm.** Those eight binaries link
  Kortex; the offline model tools are in `control/tools/` and cannot connect.

## Tests

22 test files: C++ suites registered with CTest in each project, and pytest for
the panel.

| Area | What is tested |
|---|---|
| `control/` | goal preemption, TCP telemetry |
| `runtime/` | run-log schema, cyclic-exchange retry, goal socket, argument parsing |
| `planning/` | bridge arguments, dynamics peak, joint-type contract, mount SDF, obstacle-aware planning, path frames, start state, projection exit, scene config, terminal posture, traced circle |
| `panel/` | plan, scene config, static files, telemetry, YAML text |

Not tested at the moment: the reactive law and the per-cycle execution core.
Rebuilding a test that holds `ReactiveLaw.h` to the Python law in msc_project
on identical inputs is the first gap to close.

## Building

There is deliberately no top-level `CMakeLists.txt`: each project configures on
its own, and the shared pieces (`control/`, `contracts/`) are added as
subdirectories by whoever needs them.

```
cmake -S runtime    -B runtime/build    && cmake --build runtime/build -j8
cmake -S planning   -B planning/build   && cmake --build planning/build -j8
cmake -S tracking   -B tracking/build   && cmake --build tracking/build -j8
cmake -S simulation -B simulation/build && cmake --build simulation/build -j8
```

`ctest --test-dir runtime/build` runs the whole controller side (control's
tests are registered there too). `cmake -S control -B control/build` on its own
gives the hardware-free subset for quick iteration.

The panel and its tests:

```
python3 Christian_control/panel/control_panel.py          # from the repository root
python3 -m pytest Christian_control/panel/tests           # from the repository root
```

## Before running anything

`runtime/build/controller` commands a physical arm. Building it is not a test
step. Running it requires Christian present, the workspace clear, the emergency
stop to hand, and explicit authorization for that specific run. The same
applies to every binary in `runtime/tools/`.
