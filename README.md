# Human - Supernumerary Limb Collaboration Project

Dual Kinova Gen3 7-DoF robotic arm system for human-robot collaborative pipe/tube manipulation, driven by real-time Vicon motion capture feedback.

## What It Does

The system controls two Kinova Gen3 arms (left and right) that cooperatively pick up a tube from a table, transport it over a human subject's head, and place it at a designated target location. The entire pipeline runs in real time: a Vicon motion-capture system continuously tracks the human, the tube, and both robot bases, and the planner re-generates collision-free trajectories on the fly as the scene changes.

The task is executed as a multi-phase state machine:

1. **Right arm engages tube** — GPMP2-based joint-space trajectory to approach the tube, avoiding the human and the other arm.
2. **Right arm grasps tube** — Cartesian X then Z descent to grab the tube; gripper closes.
3. **Right arm transports tube overhead** — Task-space trajectory lifts the tube over the human's head (tracked via Vicon head markers).
4. **Left arm approaches tube** — Joint-space plan to reach the tube while the right arm holds it overhead.
5. **Left arm grasps tube** — Cartesian approach + grip; a handoff from right to left.
6. **Right arm disengages** — Gripper opens, right arm retreats.
7. **Left arm moves to target** — Task-space trajectory to bring the tube to the target position.

States 0→1 and 1→2 are triggered by a `state_monitor` thread that watches the spatial relationship between the human's hands, head, and the tube (all from Vicon). Each phase supports both a "normal" and a "mirror" mode (swapping left/right roles), and all timing, offsets, and tolerances are runtime-configurable via `task_parameters.conf`.

## Repository Structure

```
HumanSL_MAIN-master/
├── main.cpp                      # Entry point — robot connection, threading, state machine loop
├── third_party/
|   ├── include                           # Third party libraries (gtsam, gpmp2, c3d etc)
|   ├── kortex_api                        # Bundled kinova-kortex api
|   ├── lib                               # Bundled third party dependencies
|   └── vicon_api                         # Bundled vicon api
|
├── config/
│   ├── GEN3_With_GRIPPER_DYNAMICS.urdf   # Kinova Gen3 + gripper URDF for Pinocchio dynamics
│   ├── dh_params.yaml                    # DH parameters for FK/IK
│   ├── joint_limits.yaml                 # Position & velocity limits per joint
│   ├── parameters.yaml                   # High-level trajectory planning parameters
|   └── task_parameters.conf  # All tunable task parameters
│
├── TrajectoryRealTime/           # Real-time orchestration layer (the "glue")
│   ├── include/
│   │   ├── core.h                # plan_action(), state_transition(), mirror_trajectory()
│   │   ├── move.h                # Low-level robot control: position/impedance/chicken-head modes
│   │   ├── plan.h                # Gen3Arm class — combines SDF, planner, optimizer, FK/IK
│   │   ├── sdf.h                 # ViconSDF — real-time signed distance fields from Vicon data
│   │   └── task_parameters.h     # Runtime parameter loader from .conf files
│   └── src/
│       ├── core.cpp              # State machine cases 1–11, plan_action(), plan_action_mirror()
│       ├── move.cpp              # joint_position_control_execution(), gripper control, admittance
│       ├── plan.cpp              # Gen3Arm methods: plan_joint(), plan_task(), replan_joint(), etc.
│       ├── sdf.cpp               # Occupancy grid + EDT → SDF from human/tube/arm obstacles
│       └── task_parameters.cpp   # Key=Value config parser
│       
│
├── TrajectoryGeneration/         # Offline / per-query trajectory planning
│   ├── include/
│   │   ├── GenerateArmModel.h    # GPMP2 ArmModel + body sphere creation
│   │   ├── TrajectoryOptimization.h  # LM optimizer with GPMP2 factors (obstacle, GP, joint/vel limits)
│   │   ├── TrajectoryInitiation.h    # IK-based trajectory seeding (spline interp, analytical + QUIK IK)
│   │   ├── GenerateLogs.h        # YAML/visualization logging for trajectories and SDFs
│   │   ├── Obstacles.h           # Obstacle representation utilities
│   │   ├── utils.h               # Core data structures (JointTrajectory, TubeInfo, HumanInfo, DHParameters, etc.)
│   │   └── ...
│   └── src/
│       └── ...
│
├── TrajectoryExecution/          # Low-level motor control & dynamics
│   ├── include/
│   │   ├── KinovaTrajectory.h    # Kinova API wrappers for trajectory execution
│   │   ├── Dynamics.h            # Pinocchio-based M(q), C(q,dq), g(q) computation
│   │   ├── Controller.h          # PD / impedance controller
│   │   ├── Jacobian.h            # Analytical Jacobian computation
│   │   ├── Fwd_kinematics.h      # Forward kinematics
│   │   └── Filter.h              # Signal filtering
│   └── src/
│       └── ...
│
├── ViconDataStream/              # Vicon motion capture interface
│   ├── include/
│   │   ├── ViconInterface.h      # Connection, frame retrieval, marker/force-plate data
│   │   └── ViconInfo.h           # updateViconInfo(), base frame calibration, state_monitor()
│   │   
│   └── src/
│       └── ...
│
└── CMakeLists.txt                # Top-level build — links all four libraries + bundled third-party deps
```

## Threading Architecture

`main.cpp` spawns several detached threads that run concurrently:

- **`info_thread`** — Polls Vicon at frame rate (~100 Hz), updates shared state observations such as `tube_info`, `human_info`, `left/right_base_frame`, joint states. Protected by `vicon_data_mutex` (shared/exclusive).
- **`state_monitor_thread`** — Watches human hand/head/tube geometry to trigger state transitions (e.g. human extends hands → state 0→1).
- **`record_thread`** — Logs every Vicon frame + joint states + targets to a timestamped CSV in `logs/experimental_results/`.
- **`left/right_robot_execution_thread`** — Each runs `joint_position_control_execution()` in a 500 Hz control loop, consuming waypoints from `JointTrajectory` deques. Supports mid-execution trajectory replacement via atomic flags + mutex.
- **Main thread** — Runs the `state_transition()` loop, calling `plan_action()` for each phase, blocking until execution completes, then advancing to the next phase.

## Obstacle Map Pipeline

At the start of planning, `Gen3Arm::make_sdf()` snapshots the current Vicon/joint state and builds a 3D signed distance field (SDF) that GPMP2 uses to penalize trajectories that come too close to obstacles.

### Workspace Grid
A 2.0 × 3.5 × 2.5 m voxel grid at **6 cm resolution** (~34 × 58 × 42 cells), origin at (−1.0, −1.5, 0.0) in the world frame. Each cell is binary: free or occupied.

### Obstacle Layers

The grid is built by compositing four layers in sequence, each adding obstacles to the previous grid:

**1. Human** (`HumanObstacleFromVicon`) — Vicon body markers (RFIN, LFIN, RHIP, LHIP, CLAV, STRN, head, etc.) are each inflated into **10 cm radius** spheres. Only cells within the human's bounding box are checked. This is coarse but fast and conservative.

**2. Other arm** (`ArmObstacleFromVicon`) — The partner arm (the one *not* being planned for) is represented as ~30 GPMP2 body spheres (2.5–6.5 cm radius) distributed along the Kinova Gen3 links. Given its current joint angles, FK places each sphere in world coords and marks overlapping cells as occupied.

**3. Bisecting plane** (`PlaneObstacleFromVicon`) — A half-space wall behind both robot bases, preventing the arm from swinging behind the platform (into the human area). The plane normal is the cross product of the base-to-base vector and the planning arm's Z-axis.

**4. Tube** (`TubeObstacleFromVicon`) — The tube is modeled as a finite cylinder (2 cm radius + 3 cm safety margin). Vicon tracks three marker triplets (tip, mid, end); each triplet's circumcenter gives the tube axis center at that cross-section. Direction, centroid, and length are derived from whichever pair of circumcenters is visible. Cells within the cylinder's bounding box are checked for perpendicular distance to the axis.

The **`include_tube` flag** controls whether layer 4 is included in the final SDF. It's `true` when the arm is approaching the tube (avoid it) and `false` when the arm is carrying or delivering it (don't avoid it).

### Occupancy Grid → SDF

The binary grid is converted to a continuous signed distance field via dual 3D Euclidean Distance Transforms (Dijkstra, 26-connected):

1. `map_dist` — distance from each free cell to the nearest obstacle.
2. `inv_map_dist` — distance from each occupied cell to the nearest free cell.
3. `SDF = (map_dist − inv_map_dist) × cell_size` — positive in free space, negative inside obstacles.

The result is packed into a `gpmp2::SignedDistanceField` (supports trilinear interpolation) and fed to the GPMP2 factor graph as `ObstacleSDFFactorArm` cost terms.

### File Map

| File | Role |
|------|------|
| `TrajectoryRealTime/src/plan.cpp` | `Gen3Arm::make_sdf()` — orchestrates the four layers |
| `TrajectoryRealTime/src/sdf.cpp` | `ViconSDF` class — grid construction, EDT, SDF conversion |
| `TrajectoryGeneration/src/GenerateArmModel.cpp` | Body sphere definitions for the Kinova Gen3 |
| `ViconDataStream/src/ViconInfo.cpp` | `updateTubeInfo()`, `updateHumanInfo()` — Vicon marker → geometry |


## Key Dependencies

All third-party libraries are expected under a `third_party/` directory:

- **Kinova Kortex API** — TCP/UDP communication with Gen3 arms (pre-built static lib)
- **GTSAM** — Factor graph optimization framework
- **GPMP2** — Gaussian Process Motion Planning 2 (obstacle factors, GP priors, arm kinematics)
- **Pinocchio** — Rigid-body dynamics (mass matrix, Coriolis, gravity)
- **Eigen3** — Linear algebra
- **Vicon DataStream SDK** — Real-time marker tracking
- **ezc3d** — C3D file reading (for offline motion capture data)
- **Open3D** — 3D visualization (used in logging/visualization)
- **yaml-cpp** — YAML config parsing

## System Requirement & Setup
This repo is tested on Ubuntu 22.04, and requires CMake installed & configured to build. No additional dedpendencies required as the repo contains bundled kortex and vicon API, along with any other 3rd party dependencies. 

## Building

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

The build produces several executables as detailed in 'CMake.txt'; the primary one is `main`.

## Configuration

Runtime task parameters live in `config/task_parameters.conf`. This controls per-phase timing, spatial offsets, gripper positions, and optimizer tolerances. Robot config (DH params, joint limits, URDF) is in `config/`.

The two arms are addressed by IP: left at `192.168.1.9`, right at `192.168.1.10`. The Vicon system is at `192.168.128.206`. These are hardcoded in `main.cpp`.

## Running

```bash
cd build
./main
```

The system will connect to both arms and the Vicon system, move the arms to their initial configurations, enter low-level control mode, and begin the state machine. State transitions are triggered by the human's movements as detected by Vicon. Emergency shutdown is handled via Ctrl+C (SIGINT), which opens grippers and resets to single-level servoing.

## Notes & Tips

- The main code path flows from `main.cpp` → `core.cpp` (`state_transition` / `plan_action`) → `plan.cpp` (`Gen3Arm` methods) → `TrajectoryOptimization` / `TrajectoryInitiation` for planning, and `move.cpp` (`joint_position_control_execution`) for execution.
- Trajectory execution is achieved via populating 'right/left_joint_trajectory' vector as highlighted in 'main.cpp', once the 'left/right_robot_execution_thread' is active, it will actively pop the 0th index of this vector and feed it to the onboard robot controller at 500Hz (so if 'left_joint_trajectory' contains a trajectory discretized into 500 waypoints, the left arm will execute this trajectory within 1 second). If there is only 1 element left within the trajectory vector, it wont be popped, and the arm is instead actively maintained at that position.
- The trajectory vector should only be populated by position vectors, whether the robot is controlled via position/torque in the end is determined by which control method we choose, detailed in 'move.cpp', the position will be read as is if in position control, and is converted to torque via PI controller if in torque mode. 
- The 'right/left_joint_trajectory' is populated by functions detailed in 'plan.cpp', this computes a static trajectory in its entirety and replaces the current active trajectory vector. 
- If real-time predictive control is required rather than segments of pre-determined trajectory, one could append to the active trajectory vector instead.
- Obstacle avoidance is baked into the planning framework (GPMP2), for obstacle avoidance wthout GPMP2 involved, either try isolating relevant obstacle creation functions from GPMP2, or start from scratch implementing your own framework as the current obstacle avoidance isnt written with real-time predictive control in mind.
- You may notices alot of functions that goes along the line of 'impedance control','impedance execution' etc., those are actually torque control in the joint space and not in the task space. Proper impedance control was never implemented due to a bug I can't for the life of mine figure out, although you will find traces of attempts at it.
- Regarding human as an obstacle, using markers proved to be unreliable most of the times, so the planarObstacle function was introduced, such that eveyrthing infront of the backpack is considered an obstacle



