# Christian_control

C++ control software for the wearable robot arms in my MSc project, [World Stable End Effector under Human Locomotion](https://github.com/fechachris4/msc_project). Two Kinova Gen3 arms are mounted on a backpack. As the wearer moves, the software makes each arm hold a pose, or follow a path, fixed in the room rather than on the body.

Running a controller on real arms takes more than the control law. This software reads the backpack's position from Vicon motion capture, works out joint commands, and sends them to both arms 500 times a second. Planning runs separately, so it never holds up that loop. Each job has its own directory, and the tests run without the robot, though the control law itself is not yet covered (see [Tests](#tests)).

> **Status, September 2026.** I wrote this for the dual-arm rig in [msc_project](https://github.com/fechachris4/msc_project). The controller that ran the walking trials there is on the MUVE Lab machine and is not in this repository. This one links prebuilt x86-64 Linux binaries checked into `third_party/` (Kortex API, Vicon DataStream SDK, Pinocchio, GTSAM/GPMP2, Boost).

<img src="https://github.com/fechachris4/msc_project/raw/main/media/rig.jpg" width="420" alt="The dual-arm rig on the treadmill">

## One control cycle

Every 2 ms (500 Hz, set by `kControlDtS` in `control/Config.h`), for each arm:

1. Take the latest backpack pose from Vicon, which updates at 100 Hz.
2. Estimate how fast the backpack is moving, by differencing successive poses and smoothing the result with a first-order low-pass filter (τ = 50 ms, `kViconMountTwistFilterTauS`).
3. Work out where the arm's hand is in the room, and how each joint moves it, by chaining transforms through the backpack (Pinocchio).
4. Compare that with the target and compute joint speeds that close the gap: feedback on the position and speed error, turned into joint speeds with damped least squares, plus a damped null-space term that uses the arm's spare joint freedom without moving the hand (`control/ReactiveLaw.h`).
5. Clamp the joint speeds and integrate them into a position command (`control/Actuation.cpp`).
6. Send the command to the arm over Kortex's cyclic exchange (`runtime/Hardware.cpp`).

## Layout

Each directory owns one job, and the build targets follow the same split, so only the parts that talk to hardware can reach the arm.

| Directory | Owns | Builds |
|---|---|---|
| `model/` | The single description of the machine: the URDF, how it is mounted, and the physical joint limits. Data only. | (none) |
| `contracts/` | The typed values two subsystems must agree on: the planning request, the trajectory in room coordinates, and its text format. | `humansl_contracts` |
| `control/` | Everything that decides a joint command: the Pinocchio model and kinematics (`humansl_robot_model`, shared with the planner), the compiled settings, and the whole per-cycle pipeline. It knows nothing about how a command is delivered. | `humansl_robot_model`, `humansl_execution_core` |
| `runtime/` | The program that runs `control/` against the real arm: the 500 Hz loop, Kortex I/O (in `humansl_runtime_hardware`, the only target set that links Kortex), safety decoding, telemetry and worker threads. | `controller`, `humansl_runtime_hardware` |
| `tracking/` | Body tracking: markers in, a checked pose and speed in room coordinates out. Only `humansl_vicon` links the Vicon SDK. | `humansl_tracking_core`, `humansl_vicon` |
| `planning/` | A planning request in, a checked trajectory in room coordinates out. `optimisation/` is the GPMP2 layer. Depends on `humansl_robot_model`, never on the controller. | `humansl_planning`, `planner_bridge` |
| `simulation/` | A MuJoCo twin that runs the same `control/` core, driven by physics instead of hardware. | `humansl_sim` |
| `panel/` | The operator's browser page: configure, plan, run, watch. | (none) |
| `docs/` | Decisions, code walkthroughs, runbooks, thesis notes. | (none) |

## The two flows

Everything the system does follows one of two paths. Each can be read from about six files, and the directory names tell you which stage you are in.

**Control: tracking → room coordinates → control → actuation → hardware.** This is the real-time path that runs every cycle.

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

**Planning: request → planner, IK, GPMP2 and collision checks → trajectory → runtime.** This path runs on a separate worker thread when the controller asks for a new plan, and hands the result back to the control path.

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

These are the design rules the layout follows. The tests that used to enforce them were removed with the old test suite (commit `0c10e17`, 20 Aug 2026) and have not been rebuilt, so today the rules hold because of how the build is structured, not because a test checks them.

- **`control/` never touches the robot.** No Kortex or Vicon SDK header is on its include path; only `humansl_runtime_hardware` links Kortex.
- **`simulation/` cannot reach the arm.** It never adds `runtime/`.
- **Planning never slows the control loop.** GPMP2 runs on the worker thread in `runtime/InProcessPlanner.cpp`, never inside `Runner.cpp`'s 500 Hz cycle.
- **Every plan passes one check before it can run.** `ValidatePlan` checks the start state, that every value is finite, joint speed and acceleration, joint position, clearance from the modelled scene and from the arm itself, and that the plan ends where it should. It returns one of three outcomes: `REACHED` carries a trajectory to the requested end point, `GOAL_BLOCKED` carries a trajectory to an explicitly shortened end point, and `FAILED` carries none. Only the first two are passed to the runtime.
- **One robot model, one kinematics implementation.** Every project parses `model/GEN3_dual_mounted.urdf` through `control/RobotModel.cpp`.
- **Only `runtime/tools/` can reach the arm.** Those eight programs link Kortex. The offline model tools are in `control/tools/` and cannot connect.

## Tests

The tests cover the plumbing around the controller, but not yet the control law. There are 22 test files: C++ suites registered with CTest in each project, and pytest for the panel.

| Area | What is tested |
|---|---|
| `control/` | goal preemption, TCP telemetry |
| `runtime/` | run-log schema, cyclic-exchange retry, goal socket, argument parsing |
| `planning/` | bridge arguments, dynamics peak, joint-type contract, mount SDF, obstacle-aware planning, path frames, start state, projection exit, scene config, terminal posture, traced circle |
| `panel/` | plan, scene config, static files, telemetry, YAML text |

Not tested at the moment: the reactive law and the per-cycle execution core. The first gap to close is a test that runs `ReactiveLaw.h` and the Python law in msc_project on identical inputs and checks they agree.

## Building

There is deliberately no top-level `CMakeLists.txt`. Each project configures on its own, and the shared pieces (`control/`, `contracts/`) are added as subdirectories by whichever project needs them.

```
cmake -S runtime    -B runtime/build    && cmake --build runtime/build -j8
cmake -S planning   -B planning/build   && cmake --build planning/build -j8
cmake -S tracking   -B tracking/build   && cmake --build tracking/build -j8
cmake -S simulation -B simulation/build && cmake --build simulation/build -j8
```

`ctest --test-dir runtime/build` runs the whole controller side (the `control/` tests are registered there too). `cmake -S control -B control/build` on its own gives the hardware-free subset for quick iteration.

The panel and its tests, run from the repository root:

```
python3 Christian_control/panel/control_panel.py
python3 -m pytest Christian_control/panel/tests
```

## Before running anything

`runtime/build/controller` and every program in `runtime/tools/` command a physical arm. Building them is not a test step. Running one needs me present, the workspace clear and the emergency stop in hand.
