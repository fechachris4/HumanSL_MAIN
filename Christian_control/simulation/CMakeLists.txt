# Dual-arm MuJoCo execution twin — standalone simulation project.
#
# Own CMake project by design, matching the repo's per-component pattern
# (control, runtime, planning, tracking); there is deliberately no
# top-level CMakeLists. Later plan tasks add the MuJoCo model contract,
# backend, and the humansl_sim executable here, obtaining
# humansl_execution_core via add_subdirectory of ../control with an
# explicit binary dir. Nothing in this project may link Kortex or contain
# a robot address (enforced by a link test in a later task).
#
#   cd Christian_control/simulation
#   cmake -S . -B build
cmake_minimum_required(VERSION 3.16)
project(humansl_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(TP_LIB "${HUMANSL_ROOT}/third_party/lib")
set(CMAKE_BUILD_RPATH "${TP_LIB}")

# --- Vendored MuJoCo 3.10.0 (third_party/; provenance in
#     third_party/MUJOCO_PROVENANCE.md and model/model_provenance.yaml).
#     No system MuJoCo is installed or wanted. ---
add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES
    IMPORTED_LOCATION "${TP_LIB}/libmujoco.so.3.10.0"
    INTERFACE_INCLUDE_DIRECTORIES "${HUMANSL_ROOT}/third_party/include")

# --- humansl_execution_core (and the production Pinocchio kinematics)
#     from ../control, with an explicit binary dir (plan global
#     constraint). EXCLUDE_FROM_ALL keeps control's tools and test
#     executables out of this build; only targets something here depends on
#     are built. The hardware targets (controller, Kortex tools) are not
#     reachable from here at all — they live in ../runtime, which this
#     project never adds. control's ctest entries stay out of this project's
#     test list via its CONTROL_TESTS guard (OFF when it is not the
#     top-level project) — they remain registered and runnable from
#     control/build and runtime/build exactly as before. ---
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../control"
                 "${CMAKE_CURRENT_BINARY_DIR}/control"
                 EXCLUDE_FROM_ALL)

# The generated model (tools/generate_dual_mjcf.py writes it; hand edits are
# forbidden and fail the provenance test via the recorded SHA-256).
set(SIM_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/model")
set(SIM_MODEL_XML "${SIM_MODEL_DIR}/humansl_dual_gen3.xml")

# --- ModelContract: the one owner of name→index resolution and structural
#     validation. Startup-only; after it returns, all control-loop access
#     is by stored integer ids, never name lookup. ---
add_library(sim_model STATIC src/ModelContract.cpp)
add_humansl_warnings_to(sim_model)
target_include_directories(sim_model PUBLIC src)
target_link_libraries(sim_model PUBLIC mujoco)



# --- sim_mount_motion (Plan 03 Task 1): the analytic scripted Mount
#     trajectory. The one owner of "how the Mount moves during a run",
#     returning pose AND its exact derivative from one call so the plant
#     and the ideal sensor can never be given different Mounts. Pure
#     arithmetic on Eigen and State.h records: no MuJoCo, no model, no
#     clock — which is why it is its own tiny library and its test links
#     no simulator. ---
add_library(sim_mount_motion STATIC src/MountMotion.cpp)
add_humansl_warnings_to(sim_mount_motion)
target_include_directories(sim_mount_motion PUBLIC src)
target_link_libraries(sim_mount_motion PUBLIC humansl_execution_core)


# --- sim_backend (Task 4): MujocoBackend, the physics-stepping and
#     Kortex-wire-format adapter. One owner each for: physics stepping and
#     timing (opt.timestep = control_dt_s / physics_substeps, N mj_step
#     calls per Exchange); the degree boundary (continuous command degrees
#     -> radian ctrl once per tick; radian qpos -> wrapped [0,360) +
#     continuous measured degrees, the hardware wire shapes); and the
#     Mount pose prescription (the mocap row written once per tick; the
#     Mount is welded to world, so the arms hang off it gravity-loaded and
#     nothing they do moves it). Links humansl_execution_core for the shared
#     State.h/Config.h records (JointVector, CartesianPose, Twist) — the
#     core itself is consumed by Task 5's coordinator, not here. ---
add_library(sim_backend STATIC src/MujocoBackend.cpp)
add_humansl_warnings_to(sim_backend)
target_link_libraries(sim_backend PUBLIC sim_model humansl_execution_core)


# --- sim_runner (Task 5): the one deterministic coordinator. Owns two
#     ArmExecutionCore instances (right/left), the production Pinocchio
#     kinematics, two empty trajectory mailboxes (the reference source's
#     kAwaitingWorld -> kHolding transition on the first fresh WorldSample
#     IS the hold), the MujocoBackend and the shared stop latch. Builds
#     the raw ideal WorldSample each tick (sequence = tick + 1, age 0);
#     the core owns freshness classification. ---
add_library(sim_runner STATIC src/DualSimulationRunner.cpp)
add_humansl_warnings_to(sim_runner)
target_link_libraries(sim_runner PUBLIC sim_backend sim_mount_motion
                                        humansl_execution_core)

# --- humansl_sim: the headless ideal-mode executable (CLI + telemetry +
#     run provenance live here, outside the coordinator). The passive
#     viewer is OPTIONAL and OFF by default: headless is the gate. ---
option(HUMANSL_SIM_VIEWER
       "Build the optional passive GLFW viewer into humansl_sim" OFF)
add_executable(humansl_sim src/SimMain.cpp)
add_humansl_warnings_to(humansl_sim)
target_link_libraries(humansl_sim sim_runner)
# DT_RUNPATH (the linker's new-dtags default) is non-transitive; DT_RPATH is
# transitive and lets the binary run bare, outside ctest's LD_LIBRARY_PATH.
# Same workaround as planner_bridge for the vendored third_party libraries,
# which carry no RUNPATH of their own.
target_link_options(humansl_sim PRIVATE -Wl,--disable-new-dtags)
target_compile_definitions(humansl_sim PRIVATE
    SIM_MODEL_XML_PATH="${SIM_MODEL_XML}"
    SIM_URDF_PATH="${HUMANSL_ROOT}/Christian_control/model/GEN3_dual_mounted.urdf")
if (HUMANSL_SIM_VIEWER)
    target_sources(humansl_sim PRIVATE src/SimViewer.cpp)
    target_compile_definitions(humansl_sim PRIVATE HUMANSL_SIM_VIEWER)
    # Vendored static GLFW (zlib licence, third_party/include/GLFW);
    # linking needs the system X11/GL stack.
    target_link_libraries(humansl_sim
        "${TP_LIB}/libglfw3.a" X11 GL m pthread dl)
endif ()
