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
#   ctest --test-dir build --output-on-failure
cmake_minimum_required(VERSION 3.12)
project(humansl_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

enable_testing()

add_compile_options(-Wall -Wextra)

# DT_RUNPATH (the linker's new-dtags default) is non-transitive; DT_RPATH is
# transitive and lets binaries run bare, outside ctest's LD_LIBRARY_PATH.
# Same workaround as planner_bridge for the vendored third_party libraries,
# which carry no RUNPATH of their own.
add_link_options(-Wl,--disable-new-dtags)

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
target_include_directories(sim_model PUBLIC src)
target_link_libraries(sim_model PUBLIC mujoco)

# --- sim_model_contract: loads the generated MJCF, validates it against
#     the URDF-derived structural contract, and proves the mutation cases
#     (renamed joint, wrong range, duplicate name, dropped site, Mount
#     stripped of mocap) each fail with a specific message, plus the
#     contact evidence: no contact at the home keyframe, and 8 penetrating
#     contacts once the generated base/shoulder excludes are removed. ---
add_executable(test_sim_model_contract tests/test_model_contract.cpp)
target_link_libraries(test_sim_model_contract sim_model)
target_compile_definitions(test_sim_model_contract PRIVATE
    SIM_MODEL_XML_PATH="${SIM_MODEL_XML}"
    SIM_MODEL_DIR="${SIM_MODEL_DIR}")
add_test(NAME sim_model_contract COMMAND test_sim_model_contract)
set_tests_properties(sim_model_contract PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")

# --- sim_model_parity (Task 3): the numeric gate Task 2's structural
#     checks deliberately defer to. MuJoCo world FK (base bodies, TCP
#     sites) and finite-difference Jacobians of the generated model
#     against the PRODUCTION Pinocchio kinematics (DualArmKinematics on
#     the URDF) — two independent pipelines; tolerances 1e-8 m /
#     1e-8 rad / 1e-6 per Jacobian component, plus the pre-registered
#     mutation checks (swapped joints, inverted axis, mount swap). ---
add_executable(test_model_parity tests/test_model_parity.cpp)
target_link_libraries(test_model_parity sim_model humansl_execution_core)
target_compile_definitions(test_model_parity PRIVATE
    SIM_MODEL_XML_PATH="${SIM_MODEL_XML}"
    SIM_MODEL_DIR="${SIM_MODEL_DIR}"
    SIM_URDF_PATH="${HUMANSL_ROOT}/Christian_control/model/GEN3_dual_mounted.urdf")
add_test(NAME sim_model_parity COMMAND test_model_parity)
set_tests_properties(sim_model_parity PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")

# --- sim_mount_motion (Plan 03 Task 1): the analytic scripted Mount
#     trajectory. The one owner of "how the Mount moves during a run",
#     returning pose AND its exact derivative from one call so the plant
#     and the ideal sensor can never be given different Mounts. Pure
#     arithmetic on Eigen and State.h records: no MuJoCo, no model, no
#     clock — which is why it is its own tiny library and its test links
#     no simulator. ---
add_library(sim_mount_motion STATIC src/MountMotion.cpp)
target_include_directories(sim_mount_motion PUBLIC src)
target_link_libraries(sim_mount_motion PUBLIC humansl_execution_core)

# --- mount_motion: closed-form landmarks (t=0, T/4, T/2), phase, axis
#     normalization, zero amplitude, rejections, channel independence,
#     and the central-finite-difference proof that the reported twist is
#     the derivative of the reported pose — against independent Rodrigues
#     and rotation-log oracles, with the spatial-vs-body,
#     pre-vs-post-multiply and origin-vs-orbit mutations quantified. ---
add_executable(test_mount_motion tests/test_mount_motion.cpp)
target_link_libraries(test_mount_motion sim_mount_motion)
add_test(NAME mount_motion COMMAND test_mount_motion)
set_tests_properties(mount_motion PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")

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
target_link_libraries(sim_backend PUBLIC sim_model humansl_execution_core)

# --- mujoco_backend: seed/exchange/substep semantics, deg boundary,
#     Mount prescription (held still AND moving between ticks), gravity
#     loading, non-finite rejection, diverged-state stop, and
#     byte-identical trace determinism for 1 and 4 substeps. The
#     moving-Mount checks compose the plant's stepped state against the
#     production Pinocchio kinematics, so the URDF path is needed here
#     too. ---
add_executable(test_mujoco_backend tests/test_mujoco_backend.cpp)
target_link_libraries(test_mujoco_backend sim_backend)
target_compile_definitions(test_mujoco_backend PRIVATE
    SIM_MODEL_XML_PATH="${SIM_MODEL_XML}"
    SIM_URDF_PATH="${HUMANSL_ROOT}/Christian_control/model/GEN3_dual_mounted.urdf")
add_test(NAME mujoco_backend COMMAND test_mujoco_backend)
set_tests_properties(mujoco_backend PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")

# --- sim_runner (Task 5): the one deterministic coordinator. Owns two
#     ArmExecutionCore instances (right/left), the production Pinocchio
#     kinematics, two empty trajectory mailboxes (the reference source's
#     kAwaitingWorld -> kHolding transition on the first fresh WorldSample
#     IS the hold), the MujocoBackend and the shared stop latch. Builds
#     the raw ideal WorldSample each tick (sequence = tick + 1, age 0);
#     the core owns freshness classification. ---
add_library(sim_runner STATIC src/DualSimulationRunner.cpp)
target_link_libraries(sim_runner PUBLIC sim_backend sim_mount_motion
                                        humansl_execution_core)

# --- humansl_sim: the headless ideal-mode executable (CLI + telemetry +
#     run provenance live here, outside the coordinator). The passive
#     viewer is OPTIONAL and OFF by default: headless is the gate. ---
option(HUMANSL_SIM_VIEWER
       "Build the optional passive GLFW viewer into humansl_sim" OFF)
add_executable(humansl_sim src/SimMain.cpp)
target_link_libraries(humansl_sim sim_runner)
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

# --- dual_simulation_runner: cycle order, shared stop latch, continuous
#     seeding, reset semantics, determinism, ideal dual hold. ---
add_executable(test_dual_simulation_runner
               tests/test_dual_simulation_runner.cpp)
target_link_libraries(test_dual_simulation_runner sim_runner)
target_compile_definitions(test_dual_simulation_runner PRIVATE
    SIM_MODEL_XML_PATH="${SIM_MODEL_XML}"
    SIM_URDF_PATH="${HUMANSL_ROOT}/Christian_control/model/GEN3_dual_mounted.urdf")
add_test(NAME dual_simulation_runner COMMAND test_dual_simulation_runner)
set_tests_properties(dual_simulation_runner PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")

# --- moving_mount_control (Plan 03 Task 3 Step 1): does the world hold
#     actually hold? Everything above gates the WIRING (one coherent
#     sample per tick, this tick's Mount pose in the plant); this asks
#     whether the resulting motion is right, in simulator truth only —
#     the MuJoCo TCP site and the plant's own mocap row, never the core's
#     estimate of itself. Each scenario asserts BOTH halves: the world
#     TCP error stays inside a bound derived from the accepted analysis
#     packet, AND the TCP travels through the MOUNT frame by the
#     excursion the geometry demands, in the opposite direction. An arm
#     that ignored the base motion satisfies neither, and an arm that was
#     merely carried satisfies the second only in reverse. ---
add_executable(test_moving_mount_control tests/test_moving_mount_control.cpp)
target_link_libraries(test_moving_mount_control sim_runner)
target_compile_definitions(test_moving_mount_control PRIVATE
    SIM_MODEL_XML_PATH="${SIM_MODEL_XML}"
    SIM_URDF_PATH="${HUMANSL_ROOT}/Christian_control/model/GEN3_dual_mounted.urdf")
add_test(NAME moving_mount_control COMMAND test_moving_mount_control)
set_tests_properties(moving_mount_control PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")

# --- sim_cli: the command line, checked by running humansl_sim itself
#     (hardware-free — see sim_no_kortex). Identical arguments must
#     produce a byte-identical summary; different --motion / --phase-rad
#     must not; the named motion must reach the plant's Mount; a headless
#     run must state a positive duration; and a bad number or motion name
#     exits 2 instead of being rounded to zero. Requires humansl_sim to
#     be built first (the documented flow builds targets before ctest). ---
add_test(NAME sim_cli
         COMMAND ${CMAKE_COMMAND}
                 -DSIM_BINARY=$<TARGET_FILE:humansl_sim>
                 -DTP_LIB=${TP_LIB}
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_sim_cli.cmake)

# --- sim_no_kortex: link evidence that humansl_sim cannot reach the
#     robot — fails on Kortex symbols/libraries, a robot IP address in
#     the binary, or hardware Runner.cpp/Hardware.cpp objects in the link
#     command. Requires humansl_sim to be built first (the documented
#     flow builds targets before ctest). ---
add_test(NAME sim_no_kortex
         COMMAND ${CMAKE_COMMAND}
                 -DSIM_BINARY=$<TARGET_FILE:humansl_sim>
                 -DLINK_TXT=${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/humansl_sim.dir/link.txt
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_sim_no_kortex.cmake)

# --- sim_model_provenance: verifies model/model_provenance.yaml against
#     the actual source files (hashes, URDF mount transforms, TCP names,
#     vendored MuJoCo version). Pure Python file hashing and XML parsing;
#     no simulator, no robot. Follows the runtime pattern of
#     checking the interpreter and required module before registering. ---
find_package(Python3 COMPONENTS Interpreter QUIET)
if (Python3_Interpreter_FOUND)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import yaml"
        RESULT_VARIABLE PYTHON_YAML_RESULT
        OUTPUT_QUIET
        ERROR_QUIET
    )
endif ()
if (Python3_Interpreter_FOUND AND PYTHON_YAML_RESULT EQUAL 0)
    add_test(NAME sim_model_provenance
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_model_provenance.py)
else ()
    message(STATUS "Skipping sim_model_provenance: Python3 with yaml is unavailable")
endif ()
