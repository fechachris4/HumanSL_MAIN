# control — what decides a joint command.
#
# This project owns the arm's geometric model (the URDF in ../model, loaded by
# RobotModel through Pinocchio), the kinematics built on it, the compiled
# settings, and the whole per-cycle command pipeline: reference -> control law
# -> velocity limits -> position integration -> stop verdict.
#
# It owns nothing about HOW a command is delivered. There is no Kortex header
# on any include path here, no Vicon SDK, and no GTSAM/GPMP2 — which is why
# both the hardware program (../runtime) and the simulator (../simulation) can
# link the same humansl_execution_core and share one set of control
# mathematics. tests/check_execution_core_linkage.cmake, registered by
# ../runtime, turns that from a convention into a build failure.
#
# Buildable on its own, for the hardware-free tests:
#   cmake -S . -B build && cmake --build build
#   ctest --test-dir build --output-on-failure
cmake_minimum_required(VERSION 3.12)
project(control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
# Warnings on for first-party code; vendor headers (Pinocchio, Eigen) are
# SYSTEM includes below, so their noise is suppressed without -w.
add_compile_options(-Wall -Wextra)

# compile_commands.json, so clang-tidy (or an editor) knows the include paths.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Sanitizer build: cmake .. -DSANITIZER=address  (or: undefined, thread,
# address+undefined). Off by default — a normal `cmake ..` is unaffected.
# address and thread cannot be combined with each other.
set(SANITIZER "" CACHE STRING "Sanitizer(s) to build with, e.g. address, undefined, address+undefined, thread")
if (SANITIZER)
    string(REPLACE "+" "," SANITIZER_FLAG "${SANITIZER}")
    add_compile_options(-fsanitize=${SANITIZER_FLAG} -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=${SANITIZER_FLAG})
endif ()

get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# The planner/controller exchange types. Guarded: whichever project is
# top-level may already have added this directory.
if (NOT TARGET humansl_contracts)
    add_subdirectory("${HUMANSL_ROOT}/Christian_control/contracts"
                     "${CMAKE_CURRENT_BINARY_DIR}/contracts")
endif ()

# The one geometric model, in ../model. It retains the downloaded dual-arm
# world/mount topology, with both Gen3 chains aligned to Kinova's official
# ros_kortex v2.5.2 URDF. DualArmKinematics handles Pinocchio's
# continuous-joint representation and adapts that model to the right-only
# seven-wide controller. Published as a PUBLIC compile definition on the core
# below, so every consumer — the hardware program, the simulator, the planner,
# the tools and the tests — parses the same file without naming it again.
set(GEN3_DUAL_URDF "${HUMANSL_ROOT}/Christian_control/model/GEN3_dual_mounted.urdf")

# --- Helpers. Defined here rather than in each consumer: ../runtime and
#     ../simulation add this directory before they use them, and a CMake
#     function is visible everywhere once defined. ---

# The bundled Eigen, as a SYSTEM include so -Wall stays quiet inside it.
function(add_bundled_eigen_to target)
    target_include_directories(${target} SYSTEM PRIVATE
            ${HUMANSL_ROOT}/third_party/include/eigen3
    )
endfunction()

# One call wires a target to humansl_execution_core: the archive's PUBLIC
# usage requirements carry this directory and the contracts' include dirs, the
# bundled Eigen/Pinocchio SYSTEM includes, the URDF path definition, the
# Pinocchio .so files and the lazy-binding link options; only the runtime
# RPATH is not a propagated usage requirement, so it is set here. Tools and
# tests that need the Pinocchio model link this instead of recompiling the
# library's sources — each source then compiles once, into the archive (a
# static library links only the objects a consumer actually references).
function(add_execution_core_to target)
    target_link_libraries(${target} humansl_execution_core)
    set_target_properties(${target} PROPERTIES
            BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
    )
endfunction()

# --- humansl_execution_core: the ONE hardware-independent arm execution
#     library (Plan 01 Task 4). It owns the explicit per-cycle contract
#     (ExecutionCore), the immutable configuration snapshot
#     (ExecutionConfig), the controller/reference/actuation mathematics, and
#     the Pinocchio model and kinematics. Dependencies are ONLY the bundled
#     Eigen/Pinocchio, humansl_contracts and pthread — a Kinova::Api or Vicon
#     SDK symbol in this archive is a test failure
#     (tests/check_execution_core_linkage.cmake), not a review hope. ---
add_library(humansl_execution_core STATIC
        ExecutionCore.cpp
        ExecutionConfig.cpp
        Controller.cpp
        CartesianReference.cpp
        CartesianTrajectoryMailbox.cpp
        Actuation.cpp
        Kinematics.cpp
        RobotModel.cpp
)
# PUBLIC: every consumer sees this directory plus the bundled Eigen/Pinocchio
# SYSTEM includes, and links the Pinocchio .so files transitively. The
# exchange types arrive the same way, through humansl_contracts' own PUBLIC
# include directory. INTERFACE link options: libpinocchio_parsers.so
# references collision symbols (coal) that are not bundled and that we never
# call (we only build the model, no collision geometry);
# --allow-shlib-undefined leaves them unresolved and -z lazy keeps the loader
# from looking them up at startup.
target_include_directories(humansl_execution_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(humansl_execution_core SYSTEM PUBLIC
        ${HUMANSL_ROOT}/third_party/include
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_compile_definitions(humansl_execution_core PUBLIC
        GEN3_DUAL_URDF_PATH="${GEN3_DUAL_URDF}")
target_link_libraries(humansl_execution_core PUBLIC
        humansl_contracts
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_default.so
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_parsers.so
        pthread
)
target_link_options(humansl_execution_core INTERFACE
        -Wl,--allow-shlib-undefined
        -Wl,-z,lazy
)

# --- Offline tools. Every binary here reads the model or replays a log; none
#     links Kortex, so none can connect to or command an arm. The tools that
#     can reach the robot live in ../runtime/tools and only there. ---

# --- probe_direction: OFFLINE joint-velocity-sign probe for a Cartesian
#     direction at a given pose (the controller's own model + DLS). No robot. ---
add_executable(probe_direction tools/probe_direction.cpp)
add_execution_core_to(probe_direction)

# --- expand_run_poses: turns a run log into a Cartesian pose trace by running
#     the SAME Pinocchio kinematics on the logged joint columns. The log's own
#     Cartesian columns are NaN (the joint motion path computes no pose), so
#     this recovers them offline rather than putting FK in the control loop.
#     URDF + one CSV only — deliberately NOT linked against Kortex. ---
add_executable(expand_run_poses tools/expand_run_poses.cpp)
add_execution_core_to(expand_run_poses)

# --- print_dual_arm_fk: both arms' FK in world and base_link, for checking
#     the mounted model against the physical rig. URDF only — deliberately
#     NOT linked against Kortex, so it cannot connect or command. Safe to run
#     with the robot off. ---
add_executable(print_dual_arm_fk tools/print_dual_arm_fk.cpp)
add_execution_core_to(print_dual_arm_fk)

# --- Tests register only when control is the top-level project (or when a
#     parent sets CONTROL_TESTS=ON). A consuming project (../runtime,
#     ../simulation) adds this directory only for humansl_execution_core with
#     EXCLUDE_FROM_ALL; without this guard its ctest list would inherit every
#     add_test below while the excluded test executables are never built,
#     failing each with "Unable to find executable". ---
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    option(CONTROL_TESTS "Register control's test targets and ctest entries" ON)
else ()
    option(CONTROL_TESTS "Register control's test targets and ctest entries" OFF)
endif ()

if (CONTROL_TESTS)
enable_testing()

# --- control_logic: hardware-free tests for the pure per-cycle logic (DLS
#     resolution, cycle-dt clamp, PositionIntegration, the arrival and
#     acknowledgement monitors). No Kortex, no Pinocchio — safe anywhere. ---
add_executable(test_control_logic
        tests/test_control_logic.cpp
        Actuation.cpp
        ExecutionConfig.cpp
)
target_include_directories(test_control_logic PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_bundled_eigen_to(test_control_logic)
target_link_libraries(test_control_logic pthread)
add_test(NAME control_logic COMMAND test_control_logic)

# --- frames: cross-validation of the world-frame assembly port (Frames.h)
#     against fixtures generated by RUNNING the simulation's frames.py —
#     tests/gen_frames_fixtures.py, same technique as the reactive-law
#     fixtures. Pure Eigen; no model, no SDK, no Kortex. ---
add_executable(test_frames tests/test_frames.cpp)
target_include_directories(test_frames PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_bundled_eigen_to(test_frames)
add_test(NAME frames COMMAND test_frames)

# --- reactive_law: hardware-free tests for the reactive pose law
#     (ReactiveLaw.h — pure Eigen) + pose-target parsing, including
#     cross-validation fixtures generated from the Python simulation law
#     (tests/gen_reactive_fixtures.py). No Kortex, no Pinocchio. ---
add_executable(test_reactive_law tests/test_reactive_law.cpp)
target_include_directories(test_reactive_law PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_bundled_eigen_to(test_reactive_law)
target_link_libraries(test_reactive_law pthread)
add_test(NAME reactive_law COMMAND test_reactive_law)

# --- cartesian_trajectory_input: hardware-free typed ownership handoff for
#     world-Cartesian trajectories. No model, Kortex, Vicon SDK, or planning
#     libraries. ---
add_executable(test_cartesian_trajectory_input
        tests/test_cartesian_trajectory_input.cpp
        CartesianTrajectoryMailbox.cpp
)
target_include_directories(test_cartesian_trajectory_input PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR})
add_bundled_eigen_to(test_cartesian_trajectory_input)
target_link_libraries(test_cartesian_trajectory_input humansl_contracts pthread)
add_test(NAME cartesian_trajectory_input COMMAND test_cartesian_trajectory_input)

add_executable(test_cartesian_reference
        tests/test_cartesian_reference.cpp
        CartesianReference.cpp
        ExecutionConfig.cpp
        CartesianTrajectoryMailbox.cpp
)
target_include_directories(test_cartesian_reference PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_bundled_eigen_to(test_cartesian_reference)
target_link_libraries(test_cartesian_reference humansl_contracts pthread)
add_test(NAME cartesian_reference COMMAND test_cartesian_reference)

# --- execution_config: the immutable execution-configuration snapshot
#     (Plan 01 Task 2). ProductionExecutionConfig() must be the identity
#     mapping onto the compiled config:: constants, ValidateExecutionConfig
#     must reject physically meaningless values, and the injected snapshot —
#     not a fresh config:: read — must be what the per-cycle code consumes.
#     Pure Eigen; no Kortex, no Pinocchio, no robot. ---
add_executable(test_execution_config
        tests/test_execution_config.cpp
        ExecutionConfig.cpp
        Actuation.cpp
        CartesianReference.cpp
        CartesianTrajectoryMailbox.cpp
)
target_include_directories(test_execution_config PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_bundled_eigen_to(test_execution_config)
target_link_libraries(test_execution_config humansl_contracts pthread)
add_test(NAME execution_config COMMAND test_execution_config)

# --- The model-dependent battery: the bundled Pinocchio binaries are Linux
#     x86-64 ELF, so these link only there. ---
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")

    # The full model/mount geometry and explicit right-7-of-14 adapter.
    add_executable(test_dual_arm_model tests/test_dual_arm_model.cpp)
    add_execution_core_to(test_dual_arm_model)
    add_test(NAME dual_arm_model COMMAND test_dual_arm_model)

    # The URDF's mounting block against ../model/dual_arm_mounting.yaml, which
    # is its declared source of truth. Nothing generates one from the other,
    # so this test is what keeps them honest.
    add_executable(test_dual_arm_mounting tests/test_dual_arm_mounting.cpp)
    add_execution_core_to(test_dual_arm_mounting)
    add_test(NAME dual_arm_mounting COMMAND test_dual_arm_mounting
             "${HUMANSL_ROOT}/Christian_control/model/dual_arm_mounting.yaml")

    # TrackingController's channel handling — needs the real model for the
    # hold pose's forward kinematics, but no robot.
    add_executable(test_controller tests/test_controller.cpp)
    add_execution_core_to(test_controller)
    add_test(NAME controller COMMAND test_controller)

    # Pre-extraction characterization (Plan 01 Task 1): drives the REAL
    # controller/reference/actuation units in the exact Runner.cpp per-cycle
    # order and freezes the results in
    # tests/fixtures/execution_preextract_v1.csv. First run generates the
    # fixture with hashed provenance; every later run is the replay gate that
    # must match it byte for byte. Needs the Pinocchio model, no Kortex, no
    # Vicon SDK, no robot.
    add_executable(test_execution_characterization
            tests/test_execution_characterization.cpp)
    target_include_directories(test_execution_characterization PRIVATE tests)
    target_compile_definitions(test_execution_characterization PRIVATE
            EXECUTION_FIXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/execution_preextract_v1.csv"
            CONTROL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_execution_core_to(test_execution_characterization)
    add_test(NAME execution_characterization
             COMMAND test_execution_characterization)

    # The explicit arm execution contract (Plan 01 Task 3): ArmExecutionCore
    # composes the controller/reference/actuation units in the frozen Runner
    # per-cycle order and owns the relocated world-freshness classification
    # and stale-twist decay. The test replays the frozen Task 1 fixture
    # through the core byte-for-byte and hosts the counting operator
    # new/delete hook proving Step/ResolveStop never allocate after Seed.
    # Since Plan 01 Task 4 this target is ALSO the hardware-free linkage
    # probe: it consumes the same humansl_execution_core archive the hardware
    # controller links, instead of recompiling the sources.
    add_executable(test_execution_core tests/test_execution_core.cpp)
    target_include_directories(test_execution_core PRIVATE tests)
    target_compile_definitions(test_execution_core PRIVATE
            EXECUTION_FIXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/execution_preextract_v1.csv")
    add_execution_core_to(test_execution_core)
    add_test(NAME execution_core COMMAND test_execution_core)

endif ()

endif () # CONTROL_TESTS
