# runtime — the program that runs ../control against the real arm.
#
# Everything here exists because the robot is physical: the cyclic loop and
# its timing, the Kortex sessions and takeover sequence, fault decoding and
# the stop policy, the run-log writer, the process lock, the command line, and
# the two worker threads that bridge the non-real-time subsystems into the
# 500 Hz loop (../tracking through ViconSource, ../planning through
# InProcessPlanner). No control mathematics lives here — that is ../control,
# and the simulator in ../simulation drives exactly the same core.
#
# This is also the ONLY project whose binaries link Kortex. Everything in
# tools/ can therefore reach the arm; the offline model tools live in
# ../control/tools instead.
#
#   cd Christian_control/runtime && cmake -S . -B build && cmake --build build
#   ./build/controller --arm right
#
# Building is never a test step. Running ./build/controller commands a
# physical arm and requires Christian's explicit authorization for that run.
cmake_minimum_required(VERSION 3.12)
project(runtime)

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

get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(KORTEX_DIR "${HUMANSL_ROOT}/third_party/kortex_api")
set(CONTROL_SOURCE_DIR "${HUMANSL_ROOT}/Christian_control/control")
set(TRACKING_SRC "${HUMANSL_ROOT}/Christian_control/tracking/src")

# make cppcheck — static analysis, separate from the compiler build.
find_program(CPPCHECK cppcheck)
if (CPPCHECK)
    add_custom_target(cppcheck
        COMMAND ${CPPCHECK} --enable=warning,performance,portability --std=c++17
                --suppress=missingInclude --inline-suppr --quiet
                -DGEN3_DUAL_URDF_PATH=\\\"${HUMANSL_ROOT}/Christian_control/model/GEN3_dual_mounted.urdf\\\"
                ${CMAKE_CURRENT_SOURCE_DIR} ${CONTROL_SOURCE_DIR}
                ${CONTROL_GENERATED_DIR}
        COMMENT "Running cppcheck on runtime/ and control/"
    )
endif ()

include_directories(SYSTEM
        ${KORTEX_DIR}/include
        ${KORTEX_DIR}/include/client
        ${KORTEX_DIR}/include/common
        ${KORTEX_DIR}/include/messages
        ${KORTEX_DIR}/include/client_stubs
)

# --- ../control supplies humansl_execution_core (and, through it,
#     humansl_contracts, the bundled Pinocchio/Eigen and the URDF path). ---
add_subdirectory("${CONTROL_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/control")

# The planner implementation is consumed as a library by the controller's
# non-real-time worker. Its standalone executable remains available from
# planning's own top-level build, but is not added to this build.
set(PLANNING_SOURCE_DIR "${HUMANSL_ROOT}/Christian_control/planning")
set(PLANNING_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/planning")
add_subdirectory("${PLANNING_SOURCE_DIR}"
                 "${PLANNING_BINARY_DIR}" EXCLUDE_FROM_ALL)

# --- The world pose/twist source. The controller stays standalone: when the
#     vendored DataStream SDK is absent (or -DNO_VICON=ON), the stub is
#     compiled instead and StartViconSource returns nullptr — no SDK header or
#     library is touched anywhere in the build, and the vicon_* log columns
#     record honest absence. The sensing implementation itself lives in
#     ../tracking; only the acquisition thread that feeds the loop is here. ---
option(NO_VICON "Build the controller without the Vicon world-pose source" OFF)
set(VICON_SDK "${HUMANSL_ROOT}/third_party/vicon_api")
if (NOT NO_VICON AND EXISTS "${VICON_SDK}/lib/libViconDataStreamSDK_CPP.so")
    set(VICON_SOURCE_FILES
            ViconSource.cpp
            ${TRACKING_SRC}/ViconInterface.cpp
            ${TRACKING_SRC}/MountTwistEstimator.cpp
            ${TRACKING_SRC}/SnapshotBuilder.cpp
    )
    set(VICON_SOURCE_MODE "sdk")
else ()
    set(VICON_SOURCE_FILES ViconSourceStub.cpp)
    set(VICON_SOURCE_MODE "absent")
endif ()
message(STATUS "Vicon world-pose source: ${VICON_SOURCE_MODE}")

# --- controller: the hardware executable — Kortex adapter sources (loop
#     runner, sessions, safety reports, Vicon acquisition, in-process planner
#     worker, program setup) around the shared execution core. Kortex links
#     HERE only, never into the core. ---
add_executable(controller
        Main.cpp
        MainArgs.cpp
        FixedMountSource.cpp
        GoalSocket.cpp
        ProcessLock.cpp
        Runner.cpp
        InProcessPlanner.cpp
        Hardware.cpp
        Safety.cpp
        ${VICON_SOURCE_FILES}
)
# This directory is the include root for the adapter's own headers; the
# control pipeline's headers and the exchange types arrive as usage
# requirements of humansl_execution_core. BasePose.h needs the SDK-free
# ViconSnapshot.h from ../tracking/src in both build modes.
target_include_directories(controller PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} ${TRACKING_SRC}
        ${PLANNING_SOURCE_DIR}/src)
target_link_libraries(controller
        humansl_execution_core
        bridge_core
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
)
# bridge_core depends on bundled GTSAM/GPMP2 libraries whose sibling
# dependencies do not carry their own RPATH. Emit DT_RPATH rather than the
# linker's default DT_RUNPATH so this controller's third_party/lib path is
# searched transitively, matching planner_bridge's standalone build.
target_link_options(controller PRIVATE -Wl,--disable-new-dtags)
add_dependencies(controller dh_params_tool dh_params_flange)
target_compile_definitions(controller PRIVATE
        PLANNER_GOAL_FILE="${PLANNING_SOURCE_DIR}/config/goal.yaml"
        PLANNER_CONFIG_FILE="${PLANNING_SOURCE_DIR}/config/planner.yaml"
        PLANNER_JOINT_LIMITS_FILE="${PLANNING_SOURCE_DIR}/config/joint_limits.yaml"
        PLANNER_RIGHT_DH_FILE="${PLANNING_BINARY_DIR}/config/dh_params_tool.yaml"
        PLANNER_LEFT_DH_FILE="${PLANNING_BINARY_DIR}/config/dh_params_flange.yaml")
set_target_properties(controller PROPERTIES
        BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib")
if (VICON_SOURCE_MODE STREQUAL "sdk")
    # SYSTEM keeps -Wall quiet inside the vendor header; the RPATH lets the
    # loader find the boost .so files next to the SDK (mirrors ../tracking)
    # alongside the bundled Pinocchio .so directory.
    target_include_directories(controller SYSTEM PRIVATE ${VICON_SDK}/include)
    target_link_libraries(controller ${VICON_SDK}/lib/libViconDataStreamSDK_CPP.so)
    set_target_properties(controller PROPERTIES
            BUILD_RPATH "${VICON_SDK}/lib;${HUMANSL_ROOT}/third_party/lib")
endif ()

# Where run CSVs go: runs/YYYY-MM-DD/ under the repository root (the layout
# the plot scripts' no-argument mode searches). Baked in absolute, like the
# model path, so the controller writes there from any working directory.
target_compile_definitions(controller PRIVATE RUNS_ROOT_DIR="${HUMANSL_ROOT}/runs")

# --- Tools. Every binary below links Kortex and can therefore reach the arm.
#     None commands motion, but none is safe to run without authorization for
#     that specific run. ---

# --- clear_faults: supported whole-arm fault recovery (Base::ClearFaults),
#     read-verify only, no motion. Kortex, no Pinocchio. ---
add_executable(clear_faults tools/clear_faults.cpp Hardware.cpp Safety.cpp)
target_include_directories(clear_faults PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} ${CONTROL_SOURCE_DIR}
        ${CONTROL_GENERATED_DIR})
add_bundled_eigen_to(clear_faults)
target_link_libraries(clear_faults
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a pthread)

# --- set_joint_limits: WRITES the joint 2/4/6 firmware JOINT_LIMIT
#     thresholds (joint-position enforcement). No motion. ---
add_executable(set_joint_limits tools/set_joint_limits.cpp Hardware.cpp)
target_include_directories(set_joint_limits PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} ${CONTROL_SOURCE_DIR}
        ${CONTROL_GENERATED_DIR})
add_bundled_eigen_to(set_joint_limits)
target_link_libraries(set_joint_limits
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a pthread)

# --- read_safety_limits: READ-ONLY dump of every actuator's configured
#     safety thresholds (DeviceConfig), next to live positions. No motion. ---
add_executable(read_safety_limits
        tools/read_safety_limits.cpp Hardware.cpp
        Safety.cpp   # DecodeBaseBank / DecodeActuatorBank
)
target_include_directories(read_safety_limits PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} ${CONTROL_SOURCE_DIR}
        ${CONTROL_GENERATED_DIR})
add_bundled_eigen_to(read_safety_limits)
target_link_libraries(read_safety_limits
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a pthread)

# --- print_joint_positions: READ-ONLY single-snapshot dump of live joint
#     positions from cyclic feedback. No motion. ---
add_executable(print_joint_positions tools/print_joint_positions.cpp Hardware.cpp)
target_include_directories(print_joint_positions PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} ${CONTROL_SOURCE_DIR}
        ${CONTROL_GENERATED_DIR})
add_bundled_eigen_to(print_joint_positions)
target_link_libraries(print_joint_positions
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a pthread)

# --- print_end_effector_pose: READ-ONLY single-snapshot dump of the right
#     end-effector's Cartesian pose, via the controller's own forward
#     kinematics on one live feedback reading. Kortex + Pinocchio, no motion. ---
add_executable(print_end_effector_pose tools/print_end_effector_pose.cpp Hardware.cpp)
target_include_directories(print_end_effector_pose PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(print_end_effector_pose ${KORTEX_DIR}/lib/release/libKortexApiCpp.a)
add_execution_core_to(print_end_effector_pose)

# --- compare_end_effector_pose: READ-ONLY diagnostic — firmware tool_pose
#     (what the web app shows) vs our Pinocchio FK, from one shared feedback
#     snapshot. Kortex + Pinocchio, no motion. ---
add_executable(compare_end_effector_pose tools/compare_end_effector_pose.cpp Hardware.cpp)
target_include_directories(compare_end_effector_pose PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(compare_end_effector_pose ${KORTEX_DIR}/lib/release/libKortexApiCpp.a)
add_execution_core_to(compare_end_effector_pose)

# --- print_tool_configuration: READ-ONLY dump of the robot's configured Tool
#     Frame (ControlConfig::GetToolConfiguration) — the offset firmware adds to
#     compute tool_pose_*, invisible to any URDF. No motion. ---
add_executable(print_tool_configuration tools/print_tool_configuration.cpp Hardware.cpp)
target_include_directories(print_tool_configuration PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} ${CONTROL_SOURCE_DIR}
        ${CONTROL_GENERATED_DIR})
add_bundled_eigen_to(print_tool_configuration)
target_link_libraries(print_tool_configuration
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a pthread)

# --- make_synthetic_log: writes a SYNTHETIC log_format 6 CSV through the real
#     run-log writer, so the telemetry schema and scripts/plot_run.py can be
#     exercised with no robot. The writer lives in Hardware.cpp, which links
#     Kortex — link-time only; this tool never connects. ---
add_executable(make_synthetic_log tools/make_synthetic_log.cpp Hardware.cpp)
target_include_directories(make_synthetic_log PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} ${CONTROL_SOURCE_DIR}
        ${CONTROL_GENERATED_DIR})
add_bundled_eigen_to(make_synthetic_log)
target_link_libraries(make_synthetic_log
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a pthread)

include(CTest)
if (BUILD_TESTING)
    add_executable(test_log_schema tests/test_log_schema.cpp Hardware.cpp)
    target_include_directories(test_log_schema PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR} ${CONTROL_SOURCE_DIR}
            ${CONTROL_GENERATED_DIR})
    add_bundled_eigen_to(test_log_schema)
    target_link_libraries(test_log_schema
            ${KORTEX_DIR}/lib/release/libKortexApiCpp.a pthread)
    add_test(NAME log_schema COMMAND test_log_schema)

    add_executable(test_goal_socket tests/test_goal_socket.cpp GoalSocket.cpp)
    target_include_directories(test_goal_socket PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR})
    add_bundled_eigen_to(test_goal_socket)
    target_link_libraries(test_goal_socket humansl_contracts pthread)
    add_test(NAME goal_socket COMMAND test_goal_socket)
endif ()
