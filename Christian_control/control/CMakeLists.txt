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
# mathematics. That isolation is a convention now; nothing enforces it.
#
# Buildable on its own:
#   cmake -S . -B build && cmake --build build
cmake_minimum_required(VERSION 3.16)
project(control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

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

# Warnings on for first-party code only; vendor headers (Pinocchio, Eigen)
# are SYSTEM includes, so their noise is suppressed without -w. Target-scoped
# rather than add_compile_options so nested add_subdirectory consumers and
# vendored code are never affected.
function(add_humansl_warnings_to target)
    target_compile_options(${target} PRIVATE -Wall -Wextra)
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
    target_link_libraries(${target} PRIVATE humansl_execution_core)
    add_humansl_warnings_to(${target})
    set_target_properties(${target} PROPERTIES
            BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
    )
endfunction()

# --- Joint limits, generated from the ONE authoritative table ---------------
# The physical Kinova position and velocity limits live in exactly one file,
# ../model/joint_limits.yaml (with the URDF: they are facts about the machine,
# owned by neither planner nor controller), together with the margins the
# planner and controller each apply to them. This step turns that yaml into a
# constexpr header in the build tree, the same way the planner derives its DH
# tables from the URDF: the constants stay compile-time, the controller gains
# no runtime file dependency, and a yaml edit cannot silently diverge from an
# already-built binary. Listing the yaml in DEPENDS regenerates the header
# whenever it changes. The generator refuses to write a header whose margins
# are not correctly ordered, so a bad edit fails the build.
set(JOINT_LIMITS_YAML
    "${HUMANSL_ROOT}/Christian_control/model/joint_limits.yaml")
set(JOINT_LIMITS_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/JointLimits.h")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# Generated once at configure time as well, so the header exists before ANY
# target compiles. Sibling projects include Config.h from several targets that
# do not all link humansl_execution_core, and a build-time-only rule would
# race them. The custom command below still regenerates it when the yaml
# changes.
execute_process(
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_joint_limits.py"
            "${JOINT_LIMITS_YAML}" "${JOINT_LIMITS_HEADER}"
    RESULT_VARIABLE JOINT_LIMITS_RESULT)
if (NOT JOINT_LIMITS_RESULT EQUAL 0)
    message(FATAL_ERROR "generate_joint_limits.py failed on ${JOINT_LIMITS_YAML}")
endif ()

add_custom_command(OUTPUT "${JOINT_LIMITS_HEADER}"
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_joint_limits.py"
            "${JOINT_LIMITS_YAML}" "${JOINT_LIMITS_HEADER}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_joint_limits.py"
            "${JOINT_LIMITS_YAML}"
    COMMENT "Generating JointLimits.h from joint_limits.yaml"
    VERBATIM)
add_custom_target(joint_limits_header ALL DEPENDS "${JOINT_LIMITS_HEADER}")
# Exported so the sibling projects that include Config.h directly (../runtime's
# hardware tools, ../simulation) can put the generated header on their include
# path without duplicating the location.
set(CONTROL_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated"
    CACHE INTERNAL "control's generated-header directory")

# --- humansl_robot_model: the arm's geometric model and kinematics, and
#     nothing that decides a command. Split from the execution core so the
#     planner can link FK/Jacobians without depending on controller,
#     actuation or mailbox machinery: planning and control both stand on
#     this target independently. It carries every usage requirement a
#     Pinocchio consumer needs — this directory (Config.h/State.h are
#     headers here), the generated JointLimits.h, the bundled Eigen and
#     Pinocchio SYSTEM includes, the URDF path definition and the Pinocchio
#     .so files. INTERFACE link options: libpinocchio_parsers.so references
#     collision symbols (coal) that are not bundled and that we never call
#     (we only build the model, no collision geometry);
#     --allow-shlib-undefined leaves them unresolved and -z lazy keeps the
#     loader from looking them up at startup. ---
add_library(humansl_robot_model STATIC
        Kinematics.cpp
        RobotModel.cpp
)
add_humansl_warnings_to(humansl_robot_model)
target_include_directories(humansl_robot_model PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/generated)
add_dependencies(humansl_robot_model joint_limits_header)
target_include_directories(humansl_robot_model SYSTEM PUBLIC
        ${HUMANSL_ROOT}/third_party/include
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_compile_definitions(humansl_robot_model PUBLIC
        GEN3_DUAL_URDF_PATH="${GEN3_DUAL_URDF}")
target_link_libraries(humansl_robot_model PUBLIC
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_default.so
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_parsers.so
)
target_link_options(humansl_robot_model INTERFACE
        -Wl,--allow-shlib-undefined
        -Wl,-z,lazy
)

# --- humansl_execution_core: the ONE hardware-independent arm execution
#     library (Plan 01 Task 4). It owns the explicit per-cycle contract
#     (ExecutionCore), the immutable configuration snapshot
#     (ExecutionConfig), and the controller/reference/actuation mathematics;
#     the model and kinematics arrive from humansl_robot_model. Dependencies
#     are ONLY that model target, humansl_contracts and pthread — a
#     Kinova::Api or Vicon SDK symbol in this archive breaks that isolation,
#     and nothing checks for one automatically. ---
add_library(humansl_execution_core STATIC
        ExecutionCore.cpp
        ExecutionConfig.cpp
        Controller.cpp
        CartesianReference.cpp
        CartesianTrajectoryMailbox.cpp
        Actuation.cpp
)
add_humansl_warnings_to(humansl_execution_core)
# PUBLIC: consumers see the model target's includes/definitions/libraries and
# the exchange types transitively. pthread is an implementation detail (the
# mailbox); PRIVATE on a static library still reaches the final link line.
target_link_libraries(humansl_execution_core
        PUBLIC humansl_robot_model humansl_contracts
        PRIVATE pthread
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

include(CTest)
if (BUILD_TESTING)
    add_executable(test_runtime_tcp_telemetry
            tests/test_runtime_tcp_telemetry.cpp)
    add_execution_core_to(test_runtime_tcp_telemetry)
    add_test(NAME runtime_tcp_telemetry COMMAND test_runtime_tcp_telemetry)

    add_executable(test_goal_preempt tests/test_goal_preempt.cpp)
    add_execution_core_to(test_goal_preempt)
    add_test(NAME goal_preempt COMMAND test_goal_preempt)
endif ()
