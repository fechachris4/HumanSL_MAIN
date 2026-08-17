# Standalone build for controller — links the bundled Kortex, Pinocchio, and
# planner runtime at the non-real-time process boundary. GTSAM/GPMP2 are never
# linked into humansl_execution_core or called from the 500 Hz loop. Vicon is
# optional at build time: when the vendored SDK exists it supplies the world
# pose/twist used by control; -DNO_VICON=ON (or an absent SDK) builds an honest
# no-world-data stub.
#
#   cd Christian_control/basic_control && mkdir build && cd build
#   cmake .. && make
#   ./controller
cmake_minimum_required(VERSION 3.12)
project(basic_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
add_definitions(-D_OS_UNIX)
# Warnings on for first-party code; vendor headers (Kortex, Pinocchio, Eigen)
# are SYSTEM includes below, so their noise is suppressed without -w.
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

# make cppcheck — static analysis, separate from the compiler build.
find_program(CPPCHECK cppcheck)
if (CPPCHECK)
    add_custom_target(cppcheck
        COMMAND ${CPPCHECK} --enable=warning,performance,portability --std=c++17
                --suppress=missingInclude --inline-suppr --quiet
                -DGEN3_DUAL_URDF_PATH=\\\"${CMAKE_CURRENT_SOURCE_DIR}/config/GEN3_dual_mounted.urdf\\\"
                ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tools
        COMMENT "Running cppcheck on src/ and tools/"
    )
endif ()

# Bundled Kortex API lives two levels up, in HumanSL_MAIN/third_party/.
set(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(KORTEX_DIR "${HUMANSL_ROOT}/third_party/kortex_api")
set(CARTESIAN_CONTRACT_DIR
        "${HUMANSL_ROOT}/Christian_control/cartesian_contract")

# The single runtime model retains the downloaded dual-arm world/mount
# topology, with both Gen3 chains aligned to Kinova's official ros_kortex
# v2.5.2 URDF. DualArmKinematics handles Pinocchio's continuous-joint
# representation and adapts that model to the right-only seven-wide controller.
set(GEN3_DUAL_URDF
        "${CMAKE_CURRENT_SOURCE_DIR}/config/GEN3_dual_mounted.urdf")
add_compile_definitions(GEN3_DUAL_URDF_PATH="${GEN3_DUAL_URDF}")

include_directories(SYSTEM
        ${KORTEX_DIR}/include
        ${KORTEX_DIR}/include/client
        ${KORTEX_DIR}/include/common
        ${KORTEX_DIR}/include/messages
        ${KORTEX_DIR}/include/client_stubs
)

# The bundled Eigen, as a SYSTEM include so -Wall stays quiet inside it.
function(add_bundled_eigen_to target)
    target_include_directories(${target} SYSTEM PRIVATE
            ${HUMANSL_ROOT}/third_party/include/eigen3
    )
endfunction()

# One call wires a target to the bundled Pinocchio: SYSTEM include paths,
# the RobotModel wrapper's include dir, the two .so files, the runtime RPATH,
# and the lazy-binding link options. libpinocchio_parsers.so references
# collision symbols (coal) that are not bundled and that we never call (we
# only build the model, no collision geometry); --allow-shlib-undefined
# leaves them unresolved and -z lazy keeps the loader from looking them up
# at startup.
function(add_pinocchio_to target)
    target_include_directories(${target} SYSTEM PRIVATE
            ${HUMANSL_ROOT}/third_party/include
            ${HUMANSL_ROOT}/third_party/include/eigen3
    )
    target_link_libraries(${target}
            ${HUMANSL_ROOT}/third_party/lib/libpinocchio_default.so
            ${HUMANSL_ROOT}/third_party/lib/libpinocchio_parsers.so
            pthread
    )
    set_target_properties(${target} PROPERTIES
            BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
    )
    target_link_options(${target} PRIVATE
            -Wl,--allow-shlib-undefined
            -Wl,-z,lazy
    )
endfunction()

# One call wires a target to humansl_execution_core: the archive's PUBLIC
# usage requirements carry the src/ + Cartesian-contract include dirs, the
# bundled Eigen/Pinocchio SYSTEM includes, the Pinocchio .so files and the
# lazy-binding link options; only the runtime RPATH is not a propagated
# usage requirement, so it is set here. Tools and tests that need the
# Pinocchio model link this instead of recompiling the library's sources —
# each source then compiles once, into the archive (a static library links
# only the objects a consumer actually references).
function(add_execution_core_to target)
    target_link_libraries(${target} humansl_execution_core)
    set_target_properties(${target} PROPERTIES
            BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
    )
endfunction()

# --- Vicon world-pose/twist source. The controller
#     stays standalone: when the vendored DataStream SDK is absent (or
#     -DNO_VICON=ON), the stub is compiled instead and StartViconSource
#     returns nullptr — no SDK header or library is touched anywhere in
#     the build, and the vicon_* log columns record honest absence. ---
option(NO_VICON
        "Build the controller without the Vicon world-pose source" OFF)
set(VICON_SDK "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/vicon_api")
set(VICON_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../vicon/src")
if (NOT NO_VICON AND EXISTS "${VICON_SDK}/lib/libViconDataStreamSDK_CPP.so")
    set(VICON_SOURCE_FILES
            src/ViconSource.cpp
            ${VICON_SRC}/ViconInterface.cpp
            ${VICON_SRC}/MountTwistEstimator.cpp
            ${VICON_SRC}/SnapshotBuilder.cpp
    )
    set(VICON_SOURCE_MODE "sdk")
else ()
    set(VICON_SOURCE_FILES src/ViconSourceStub.cpp)
    set(VICON_SOURCE_MODE "absent")
endif ()
message(STATUS "Vicon world-pose source: ${VICON_SOURCE_MODE}")

# --- humansl_execution_core: the ONE hardware-independent arm execution
#     library (Plan 01 Task 4). It owns the explicit per-cycle contract
#     (ExecutionCore), the immutable configuration snapshot
#     (ExecutionConfig), the controller/reference/actuation mathematics,
#     the Pinocchio model and kinematics, and the Cartesian contracts.
#     Dependencies are ONLY the bundled Eigen/Pinocchio and pthread — a
#     Kinova::Api or Vicon SDK symbol in this archive is a test failure
#     (tests/check_execution_core_linkage.cmake), not a review hope. The
#     hardware `controller` and the hardware-free test probe link this
#     same target, so hardware and offline builds share one set of control
#     mathematics. ---
add_library(humansl_execution_core STATIC
        src/ExecutionCore.cpp
        src/ExecutionConfig.cpp
        src/Controller.cpp
        src/CartesianReference.cpp
        src/CartesianTrajectoryMailbox.cpp
        src/Actuation.cpp
        src/Kinematics.cpp
        src/RobotModel.cpp
        ${CARTESIAN_CONTRACT_DIR}/WorldCartesianTrajectory.cpp
        ${CARTESIAN_CONTRACT_DIR}/PlanningRequest.cpp
)
# PUBLIC: every consumer sees src/ and the Cartesian contract headers plus
# the bundled Eigen/Pinocchio SYSTEM includes, and links the Pinocchio .so
# files transitively. INTERFACE link options: libpinocchio_parsers.so
# references collision symbols (coal) we never call — see add_pinocchio_to.
target_include_directories(humansl_execution_core PUBLIC
        src ${CARTESIAN_CONTRACT_DIR})
target_include_directories(humansl_execution_core SYSTEM PUBLIC
        ${HUMANSL_ROOT}/third_party/include
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(humansl_execution_core PUBLIC
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_default.so
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_parsers.so
        pthread
)
target_link_options(humansl_execution_core INTERFACE
        -Wl,--allow-shlib-undefined
        -Wl,-z,lazy
)

# The planner implementation is consumed as a library by the controller's
# non-real-time worker. Its standalone executable and tests remain available
# from planner_bridge's own top-level build, but are not added to this build.
set(PLANNER_BRIDGE_SOURCE_DIR
        "${HUMANSL_ROOT}/Christian_control/planner_bridge")
set(PLANNER_BRIDGE_BINARY_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/planner_bridge")
set(PLANNER_BRIDGE_TESTS OFF CACHE BOOL
        "Disable standalone planner tests in the controller's nested build"
        FORCE)
add_subdirectory("${PLANNER_BRIDGE_SOURCE_DIR}"
                 "${PLANNER_BRIDGE_BINARY_DIR}" EXCLUDE_FROM_ALL)

# --- controller: the hardware executable — Kortex adapter sources (loop
#     runner, sessions, safety reports, Vicon acquisition, in-process planner
#     worker, program setup) around the shared execution core above. Kortex
#     links HERE only, never into the core. ---
add_executable(controller
        src/Main.cpp
        src/MainArgs.cpp
        src/ProcessLock.cpp
        src/Runner.cpp
        src/InProcessPlanner.cpp
        src/Hardware.cpp
        src/Safety.cpp
        ${VICON_SOURCE_FILES}
)

# src/ is the include root. One file per job, named for that job:
# State/ReactiveLaw/Controller/CartesianReference/Actuation are pure Eigen,
# Kinematics adds Pinocchio, Hardware adds Kortex, Runner is the loop,
# Main the program, Config the settings. BasePose/ViconSource provide the
# world state (BasePose.h needs the SDK-free ViconSnapshot.h
# from ../vicon/src in both build modes).
target_include_directories(controller PRIVATE src ${VICON_SRC}
        ${CARTESIAN_CONTRACT_DIR} ${PLANNER_BRIDGE_SOURCE_DIR}/src)
target_link_libraries(controller
        humansl_execution_core
        bridge_core
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
)
# bridge_core depends on bundled GTSAM/GPMP2 libraries whose sibling
# dependencies do not carry their own RPATH. Emit DT_RPATH rather than the
# linker's default DT_RUNPATH so this controller's third_party/lib path is
# searched transitively, matching planner_bridge's standalone build.
target_link_options(controller PRIVATE
        -Wl,--disable-new-dtags
)
add_dependencies(controller dh_params_tool dh_params_flange)
target_compile_definitions(controller PRIVATE
        PLANNER_GOAL_FILE="${PLANNER_BRIDGE_SOURCE_DIR}/config/goal.yaml"
        PLANNER_CONFIG_FILE="${PLANNER_BRIDGE_SOURCE_DIR}/config/planner.yaml"
        PLANNER_JOINT_LIMITS_FILE="${PLANNER_BRIDGE_SOURCE_DIR}/config/joint_limits.yaml"
        PLANNER_RIGHT_DH_FILE="${PLANNER_BRIDGE_BINARY_DIR}/config/dh_params_tool.yaml"
        PLANNER_LEFT_DH_FILE="${PLANNER_BRIDGE_BINARY_DIR}/config/dh_params_flange.yaml")
set_target_properties(controller PROPERTIES
        BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
)
if (VICON_SOURCE_MODE STREQUAL "sdk")
    # SYSTEM keeps -Wall quiet inside the vendor header; the RPATH lets the
    # loader find the boost .so files next to the SDK (mirrors ../vicon)
    # alongside the bundled Pinocchio .so directory.
    target_include_directories(controller SYSTEM PRIVATE ${VICON_SDK}/include)
    target_link_libraries(controller
            ${VICON_SDK}/lib/libViconDataStreamSDK_CPP.so
    )
    set_target_properties(controller PROPERTIES
            BUILD_RPATH "${VICON_SDK}/lib;${HUMANSL_ROOT}/third_party/lib"
    )
endif ()

# --- clear_faults: supported whole-arm fault recovery (Base::ClearFaults),
#     read-verify only, no motion. Kortex, no Pinocchio. ---
add_executable(clear_faults
        tools/clear_faults.cpp
        src/Hardware.cpp
        src/Safety.cpp
)
target_include_directories(clear_faults PRIVATE src)
add_bundled_eigen_to(clear_faults)
target_link_libraries(clear_faults
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# --- probe_direction: OFFLINE joint-velocity-sign probe for a Cartesian
#     direction at a given pose (controller's own model + DLS). No robot. ---
add_executable(probe_direction
        tools/probe_direction.cpp
)
add_execution_core_to(probe_direction)

# --- set_joint_limits: WRITES the joint 2/4/6 firmware JOINT_LIMIT
#     thresholds (joint-position enforcement). No motion. ---
add_executable(set_joint_limits
        tools/set_joint_limits.cpp
        src/Hardware.cpp
)
target_include_directories(set_joint_limits PRIVATE src)
add_bundled_eigen_to(set_joint_limits)
target_link_libraries(set_joint_limits
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# --- read_safety_limits: READ-ONLY dump of every actuator's configured
#     safety thresholds (DeviceConfig), next to live positions. No motion. ---
add_executable(read_safety_limits
        tools/read_safety_limits.cpp
        src/Hardware.cpp
        src/Safety.cpp   # DecodeBaseBank / DecodeActuatorBank
)

target_include_directories(read_safety_limits PRIVATE src)
add_bundled_eigen_to(read_safety_limits)
target_link_libraries(read_safety_limits
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# --- print_joint_positions: READ-ONLY single-snapshot dump of live joint
#     positions from cyclic feedback. No motion. ---
add_executable(print_joint_positions
        tools/print_joint_positions.cpp
        src/Hardware.cpp
)

target_include_directories(print_joint_positions PRIVATE src)
add_bundled_eigen_to(print_joint_positions)
target_link_libraries(print_joint_positions
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# --- print_end_effector_pose: READ-ONLY single-snapshot dump of the right
#     end-effector's Cartesian pose, via the controller's own forward
#     kinematics on one live feedback reading. Kortex + Pinocchio, no motion. ---
add_executable(print_end_effector_pose
        tools/print_end_effector_pose.cpp
        src/Hardware.cpp
)

target_link_libraries(print_end_effector_pose
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
)
add_execution_core_to(print_end_effector_pose)

# --- compare_end_effector_pose: READ-ONLY diagnostic — firmware tool_pose
#     (what the web app shows) vs our Pinocchio FK, from one shared
#     feedback snapshot. Kortex + Pinocchio, no motion. ---
add_executable(compare_end_effector_pose
        tools/compare_end_effector_pose.cpp
        src/Hardware.cpp
)

target_link_libraries(compare_end_effector_pose
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
)
add_execution_core_to(compare_end_effector_pose)

# --- print_tool_configuration: READ-ONLY dump of the robot's configured
#     Tool Frame (ControlConfig::GetToolConfiguration) — the offset firmware
#     adds to compute tool_pose_*, invisible to any URDF. No motion. ---
add_executable(print_tool_configuration
        tools/print_tool_configuration.cpp
        src/Hardware.cpp
)

target_include_directories(print_tool_configuration PRIVATE src)
add_bundled_eigen_to(print_tool_configuration)
target_link_libraries(print_tool_configuration
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# --- make_synthetic_log: writes a SYNTHETIC log_format 6 CSV through the
#     real run-log writer, so the telemetry schema and scripts/plot_run.py
#     can be exercised with no robot. The writer now lives in Hardware.cpp,
#     which links Kortex — link-time only; this tool never connects. ---
add_executable(make_synthetic_log
        tools/make_synthetic_log.cpp
        src/Hardware.cpp
)
target_include_directories(make_synthetic_log PRIVATE src)
add_bundled_eigen_to(make_synthetic_log)
target_link_libraries(make_synthetic_log
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# --- expand_run_poses: turns a run log into a Cartesian pose trace by running
#     the SAME Pinocchio kinematics on the logged joint columns. The log's own
#     Cartesian columns are NaN (the joint motion path computes no pose), so
#     this recovers them offline rather than putting FK in the control loop.
#     URDF + one CSV only — deliberately NOT linked against Kortex. ---
add_executable(expand_run_poses
        tools/expand_run_poses.cpp
)
add_execution_core_to(expand_run_poses)

# --- print_dual_arm_fk: both arms' FK in world and base_link, for checking
#     the mounted model against the physical rig. URDF only — deliberately
#     NOT linked against Kortex, so it cannot connect or command. Safe to run
#     with the robot off. ---
add_executable(print_dual_arm_fk
        tools/print_dual_arm_fk.cpp
)
add_execution_core_to(print_dual_arm_fk)

# --- Tests register only when basic_control is the top-level project (or
#     when a parent sets BASIC_CONTROL_TESTS=ON). A consuming project
#     (Christian_control/simulation) adds this directory only for
#     humansl_execution_core with EXCLUDE_FROM_ALL; without this guard its
#     ctest list would inherit every add_test below while the excluded test
#     executables are never built, failing each with "Unable to find
#     executable". basic_control's own build directory is unaffected:
#     the option defaults ON when this file is the top-level project. ---
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    option(BASIC_CONTROL_TESTS
            "Register basic_control test targets and ctest entries" ON)
else ()
    option(BASIC_CONTROL_TESTS
            "Register basic_control test targets and ctest entries" OFF)
endif ()
if (BASIC_CONTROL_TESTS)

# --- test_control_logic: hardware-free tests for the controller's pure
#     logic (DLS resolution, cycle-dt clamp, PositionIntegration). No
#     Kortex, no Pinocchio — safe to run anywhere. ---
enable_testing()
add_executable(test_control_logic
        tests/test_control_logic.cpp
        src/Actuation.cpp
        src/ExecutionConfig.cpp
)
target_include_directories(test_control_logic PRIVATE src)
add_bundled_eigen_to(test_control_logic)
target_link_libraries(test_control_logic pthread)
add_test(NAME control_logic COMMAND test_control_logic)

# --- test_frames: cross-validation of the world-frame assembly port
#     (src/Frames.h) against fixtures generated by RUNNING the simulation's
#     frames.py — scripts/gen_frames_fixtures.py, same technique as the
#     reactive-law fixtures. Pure Eigen; no model, no SDK, no Kortex. ---
add_executable(test_frames
        tests/test_frames.cpp
)
target_include_directories(test_frames PRIVATE src)
add_bundled_eigen_to(test_frames)
add_test(NAME frames COMMAND test_frames)

# --- test_base_pose: hardware-free tests for the world-pose sample
#     contract, snapshot→sample mapping, and the SPSC triple-buffer slot
#     (src/BasePose.h). Uses the SDK-free ViconSnapshot value type from
#     ../vicon/src; no SDK, no Kortex, no Pinocchio — safe to run anywhere. ---
add_executable(test_base_pose
        tests/test_base_pose.cpp
)
target_include_directories(test_base_pose PRIVATE src
        ${CMAKE_CURRENT_SOURCE_DIR}/../vicon/src)
add_bundled_eigen_to(test_base_pose)
target_link_libraries(test_base_pose pthread)
add_test(NAME base_pose COMMAND test_base_pose)

# --- test_reactive_law: hardware-free tests for the reactive pose law
#     (control/ReactiveLaw.h — pure Eigen) + pose-target parsing, including
#     cross-validation fixtures generated from the Python simulation law
#     (scripts/gen_reactive_fixtures.py). No Kortex, no Pinocchio — safe to
#     run anywhere. ---
add_executable(test_reactive_law
        tests/test_reactive_law.cpp
)
target_include_directories(test_reactive_law PRIVATE src)
add_bundled_eigen_to(test_reactive_law)
target_link_libraries(test_reactive_law pthread)
add_test(NAME reactive_law COMMAND test_reactive_law)

# --- test_cartesian_trajectory_input: hardware-free typed ownership handoff
#     for world-Cartesian trajectories. No model, Kortex,
#     Vicon SDK, or planning libraries. ---
add_executable(test_cartesian_trajectory_input
        tests/test_cartesian_trajectory_input.cpp
        src/CartesianTrajectoryMailbox.cpp
        ${CARTESIAN_CONTRACT_DIR}/WorldCartesianTrajectory.cpp
)
target_include_directories(test_cartesian_trajectory_input PRIVATE
        src ${CARTESIAN_CONTRACT_DIR})
add_bundled_eigen_to(test_cartesian_trajectory_input)
target_link_libraries(test_cartesian_trajectory_input pthread)
add_test(NAME cartesian_trajectory_input COMMAND test_cartesian_trajectory_input)

add_executable(test_planning_request
        tests/test_planning_request.cpp
        ${CARTESIAN_CONTRACT_DIR}/PlanningRequest.cpp)
target_include_directories(test_planning_request PRIVATE
        src ${CARTESIAN_CONTRACT_DIR})
add_bundled_eigen_to(test_planning_request)
target_link_libraries(test_planning_request pthread)
add_test(NAME planning_request COMMAND test_planning_request)

# In-process planner-worker ownership characterization. The first red build
# is intentional while the worker API is introduced.
add_executable(test_in_process_planner
        tests/test_in_process_planner.cpp
        src/InProcessPlanner.cpp
        src/CartesianTrajectoryMailbox.cpp
        ${CARTESIAN_CONTRACT_DIR}/PlanningRequest.cpp
        ${CARTESIAN_CONTRACT_DIR}/WorldCartesianTrajectory.cpp
)
target_include_directories(test_in_process_planner PRIVATE
        src ${CARTESIAN_CONTRACT_DIR}
        ${HUMANSL_ROOT}/Christian_control/planner_bridge/src)
add_bundled_eigen_to(test_in_process_planner)
target_link_libraries(test_in_process_planner pthread)
add_test(NAME in_process_planner COMMAND test_in_process_planner)

add_executable(test_cartesian_reference
        tests/test_cartesian_reference.cpp
        src/CartesianReference.cpp
        src/ExecutionConfig.cpp
        src/CartesianTrajectoryMailbox.cpp
        ${CARTESIAN_CONTRACT_DIR}/WorldCartesianTrajectory.cpp
)
target_include_directories(test_cartesian_reference PRIVATE
        src ${CARTESIAN_CONTRACT_DIR})
add_bundled_eigen_to(test_cartesian_reference)
target_link_libraries(test_cartesian_reference pthread)
add_test(NAME cartesian_reference COMMAND test_cartesian_reference)

# --- test_execution_config: the immutable execution-configuration snapshot
#     (Plan 01 Task 2). ProductionExecutionConfig() must be the identity
#     mapping onto the compiled config:: constants, ValidateExecutionConfig
#     must reject physically meaningless values, and the injected snapshot —
#     not a fresh config:: read — must be what the per-cycle code consumes.
#     Pure Eigen; no Kortex, no Pinocchio, no robot. ---
add_executable(test_execution_config
        tests/test_execution_config.cpp
        src/ExecutionConfig.cpp
        src/Actuation.cpp
        src/CartesianReference.cpp
        src/CartesianTrajectoryMailbox.cpp
        ${CARTESIAN_CONTRACT_DIR}/WorldCartesianTrajectory.cpp
)
target_include_directories(test_execution_config PRIVATE
        src ${CARTESIAN_CONTRACT_DIR})
add_bundled_eigen_to(test_execution_config)
target_link_libraries(test_execution_config pthread)
add_test(NAME execution_config COMMAND test_execution_config)

# --- test_main_args: hardware-free tests for the controller's --arm/--log
#     command-line contract (src/MainArgs.cpp — pure string parsing). No
#     Kortex, no Pinocchio, no robot — safe to run anywhere. ---
add_executable(test_main_args
        tests/test_main_args.cpp
        src/MainArgs.cpp
)
target_include_directories(test_main_args PRIVATE src)
add_test(NAME main_args COMMAND test_main_args)

# Python-only regression coverage for offline log-format compatibility. The
# interpreter is optional so portable C++ test configurations remain usable.
find_package(Python3 COMPONENTS Interpreter QUIET)
if (Python3_Interpreter_FOUND)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import numpy"
        RESULT_VARIABLE PYTHON_NUMPY_RESULT
        OUTPUT_QUIET
        ERROR_QUIET
    )
endif ()
if (Python3_Interpreter_FOUND AND PYTHON_NUMPY_RESULT EQUAL 0)
    add_test(NAME runlog_compat
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_runlog_compat.py)
elseif (Python3_Interpreter_FOUND)
    message(STATUS "Skipping runlog_compat: Python numpy is unavailable")
endif ()

# Strict historical-log extractor (Plan 01 Task 1): format 9-11 run logs
# are PARTIAL execution evidence; the extractor copies available fields
# verbatim and never fabricates absent ones. Pure csv module — no numpy.
if (Python3_Interpreter_FOUND)
    add_test(NAME extract_execution_history
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_extract_execution_history.py)
endif ()

# --- hardware-machine test battery: the bundled Kortex/Pinocchio binaries
#     are Linux x86-64 ELF, so these link only there. ctest elsewhere runs
#     the portable suite above. ---
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Linux advisory-lock regression: pure filesystem/process behavior, no
    # Kortex or Pinocchio and no robot connection.
    add_executable(test_process_lock
            tests/test_process_lock.cpp
            src/ProcessLock.cpp
    )
    target_include_directories(test_process_lock PRIVATE src)
    add_test(NAME process_lock COMMAND test_process_lock)

    add_executable(test_supervisor
            tests/test_supervisor.cpp
            src/Safety.cpp
    )
    target_include_directories(test_supervisor PRIVATE src)
    add_bundled_eigen_to(test_supervisor)
    target_link_libraries(test_supervisor
            ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
            pthread
    )
    add_test(NAME supervisor COMMAND test_supervisor)

    # The full model/mount geometry and explicit right-7-of-14 adapter.
    add_executable(test_dual_arm_model
            tests/test_dual_arm_model.cpp
    )
    add_execution_core_to(test_dual_arm_model)
    add_test(NAME dual_arm_model COMMAND test_dual_arm_model)

    # The URDF's mounting block against config/dual_arm_mounting.yaml, which
    # is its declared source of truth. Nothing generates one from the other,
    # so this test is what keeps them honest.
    add_executable(test_dual_arm_mounting
            tests/test_dual_arm_mounting.cpp
    )
    add_execution_core_to(test_dual_arm_mounting)
    add_test(NAME dual_arm_mounting COMMAND test_dual_arm_mounting
             "${CMAKE_CURRENT_SOURCE_DIR}/config/dual_arm_mounting.yaml")

    # The run-log CSV schema: header and row pinned to each other. Kortex is
    # a link-time dependency of Hardware.cpp only; no robot is contacted.
    add_executable(test_log_schema
            tests/test_log_schema.cpp
            src/Hardware.cpp
    )
    target_include_directories(test_log_schema PRIVATE src)
    add_bundled_eigen_to(test_log_schema)
    target_link_libraries(test_log_schema
            ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
            pthread
    )
    add_test(NAME log_schema COMMAND test_log_schema)

    # TrackingController's channel handling — needs the real model for the
    # hold pose's forward kinematics, but no robot.
    add_executable(test_controller
            tests/test_controller.cpp
    )
    add_execution_core_to(test_controller)
    add_test(NAME controller COMMAND test_controller)

    # Pre-extraction characterization (Plan 01 Task 1): drives the REAL
    # controller/reference/actuation units in the exact Runner.cpp
    # per-cycle order and freezes the results in
    # tests/fixtures/execution_preextract_v1.csv. First run generates the
    # fixture with hashed provenance; every later run is the replay gate
    # that must match it byte for byte. Needs the Pinocchio model, no
    # Kortex, no Vicon SDK, no robot.
    add_executable(test_execution_characterization
            tests/test_execution_characterization.cpp
    )
    target_include_directories(test_execution_characterization PRIVATE tests)
    target_compile_definitions(test_execution_characterization PRIVATE
            EXECUTION_FIXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/execution_preextract_v1.csv"
            BASIC_CONTROL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_execution_core_to(test_execution_characterization)
    add_test(NAME execution_characterization
             COMMAND test_execution_characterization)

    # The explicit arm execution contract (Plan 01 Task 3): ArmExecutionCore
    # composes the existing controller/reference/actuation units in the
    # frozen Runner per-cycle order and owns the relocated world-freshness
    # classification and stale-twist decay. The test replays the frozen
    # Task 1 fixture through the core byte-for-byte and hosts the counting
    # operator new/delete hook proving Step/ResolveStop never allocate
    # after Seed. Needs the Pinocchio model, no Kortex, no Vicon SDK, no
    # robot. Since Plan 01 Task 4 this target is ALSO the hardware-free
    # linkage probe: it consumes the same humansl_execution_core archive
    # the hardware controller links, instead of recompiling the sources.
    add_executable(test_execution_core
            tests/test_execution_core.cpp
    )
    target_include_directories(test_execution_core PRIVATE tests)
    target_compile_definitions(test_execution_core PRIVATE
            EXECUTION_FIXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/execution_preextract_v1.csv")
    target_link_libraries(test_execution_core humansl_execution_core)
    set_target_properties(test_execution_core PROPERTIES
            BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib")
    add_test(NAME execution_core COMMAND test_execution_core)

    # Link-structure gate (Plan 01 Task 4 Step 1): the core archive must
    # contain no Kortex/Vicon-SDK symbol needs, and BOTH the hardware
    # controller and the hardware-free probe must link (and actually pull
    # in) the same humansl_execution_core target. Reads build artifacts
    # only; runs nothing.
    find_program(NM_EXECUTABLE nm)
    if (NM_EXECUTABLE)
        get_target_property(CONTROLLER_LINK_LIBRARIES controller
                LINK_LIBRARIES)
        get_target_property(PROBE_LINK_LIBRARIES test_execution_core
                LINK_LIBRARIES)
        add_test(NAME execution_core_linkage
                 COMMAND ${CMAKE_COMMAND}
                         -DCORE_ARCHIVE=$<TARGET_FILE:humansl_execution_core>
                         -DCONTROLLER_BINARY=$<TARGET_FILE:controller>
                         -DPROBE_BINARY=$<TARGET_FILE:test_execution_core>
                         "-DCONTROLLER_LINKS=${CONTROLLER_LINK_LIBRARIES}"
                         "-DPROBE_LINKS=${PROBE_LINK_LIBRARIES}"
                         -DNM_EXECUTABLE=${NM_EXECUTABLE}
                         -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_execution_core_linkage.cmake)
        add_test(NAME controller_planner_linkage
                 COMMAND ${CMAKE_COMMAND}
                         -DCONTROLLER_BINARY=$<TARGET_FILE:controller>
                         -DCORE_ARCHIVE=$<TARGET_FILE:humansl_execution_core>
                         -DNM_EXECUTABLE=${NM_EXECUTABLE}
                         -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_controller_planner_linkage.cmake)
    else ()
        message(STATUS "Skipping execution_core_linkage: nm is unavailable")
    endif ()

endif ()

endif () # BASIC_CONTROL_TESTS

# Where run CSVs go: runs/YYYY-MM-DD/ under the repository root (the layout
# the plot scripts' no-argument mode searches). Baked in absolute, like the
# dual-model path, so the controller writes there from any working directory.
get_filename_component(HUMANSL_ROOT_ABS "${HUMANSL_ROOT}" ABSOLUTE)
target_compile_definitions(controller PRIVATE RUNS_ROOT_DIR="${HUMANSL_ROOT_ABS}/runs")
