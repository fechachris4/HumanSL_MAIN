# Christian_control

The control system for a wearable Supernumerary Robotic Limb built from Kinova
Gen3 arms. The objective: hold or track the end-effector pose **in the world
frame** while the wearer, and the backpack the arms are mounted on, move.

Every directory here is named for the engineering job it owns. None is named
after a vendor, an interface, or the history of how it came about.

| Directory | Owns | Builds |
|---|---|---|
| `model/` | The one geometric description of the machine: the URDF and the mounting table that is its source of truth. Data only. | — |
| `contracts/` | The typed values two subsystems must agree on: the planning request, the world-Cartesian trajectory, its text wire format. | `humansl_contracts` |
| `control/` | What decides a joint command: the Pinocchio model, the compiled settings, and the whole per-cycle pipeline. Knows nothing about how a command is delivered. | `humansl_execution_core` |
| `runtime/` | The program that runs `control/` against the real arm: the 500 Hz loop, Kortex I/O, safety decoding, telemetry, the worker threads. | `controller` |
| `tracking/` | External body tracking: markers in, a validated world pose and twist out. | `vicon_interface`, `vicon_snapshot` |
| `planning/` | A planning request in, a validated world trajectory out. `optimisation/` is the GPMP2 layer. | `bridge_core`, `planner_bridge` |
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
        WorldSdf.cpp                      signed-distance field for collision
        ValidatePlan.cpp / PlanValidationReport.cpp
  → planning/src/WorldTrajectoryProjection.cpp   joints → world Cartesian
  → contracts/WorldCartesianTrajectory.h
  → control/CartesianTrajectoryMailbox.cpp       typed ownership handoff
  → control/CartesianReference.cpp               and back into the first flow
```

## The boundaries, and what enforces them

The layout is not a filing convention; each line below is checked by a test.

- **`control/` never touches the robot.** No Kortex or Vicon SDK header is on
  its include path, and `runtime/tests/check_execution_core_linkage.cmake`
  fails the build if a Kortex or SDK symbol appears in
  `libhumansl_execution_core.a`, or if the hardware `controller` and the
  hardware-free probe stop linking the same archive.
- **`simulation/` cannot reach the arm.** `simulation/tests/test_sim_no_kortex.cmake`
  fails on a Kortex symbol, a robot IP address in the binary, or a hardware
  object in the link command. `simulation/` never adds `runtime/`.
- **GPMP2 stays outside the 500 Hz loop.** `runtime/tests/check_controller_planner_linkage.cmake`
  checks the controller's planner linkage; planning runs on the worker thread
  in `runtime/InProcessPlanner.cpp`, never in `Runner.cpp`'s cycle.
- **One URDF, one FK implementation.** `model/GEN3_dual_mounted.urdf` is parsed
  through `control/RobotModel.cpp` by every project. `control/tests/test_dual_arm_mounting.cpp`
  holds the URDF's mounting block to `model/dual_arm_mounting.yaml`, and
  `simulation/tests/test_model_parity.cpp` holds MuJoCo's kinematics to
  Pinocchio's at 1e-8 m.
- **The control mathematics is frozen against a recorded fixture.**
  `control/tests/test_execution_characterization.cpp` replays
  `tests/fixtures/execution_preextract_v1.csv` through the real units in the
  real per-cycle order and requires the result byte for byte.
- **Only `runtime/tools/` can reach the arm.** Those eight binaries link
  Kortex; the offline model tools are in `control/tools/` and cannot connect.

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
