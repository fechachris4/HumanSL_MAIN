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
# This is also the ONLY project whose targets link Kortex: the dependency
# enters through humansl_runtime_hardware and nowhere else. Everything in
# tools/ can therefore reach the arm; the offline model tools live in
# ../control/tools instead.
#
#   cd Christian_control/runtime && cmake -S . -B build && cmake --build build
#   ./build/controller --arm right
#
# Building is never a test step. Running ./build/controller commands a
# physical arm and requires Christian's explicit authorization for that run.
cmake_minimum_required(VERSION 3.16)
project(runtime)

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
set(KORTEX_DIR "${HUMANSL_ROOT}/third_party/kortex_api")
set(CONTROL_SOURCE_DIR "${HUMANSL_ROOT}/Christian_control/control")

# --- ../control supplies humansl_execution_core and humansl_robot_model
#     (and, through them, humansl_contracts, the bundled Pinocchio/Eigen and
#     the URDF path). ---
add_subdirectory("${CONTROL_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/control")

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

# The planner implementation is consumed as a library by the controller's
# non-real-time worker. Its standalone executable remains available from
# planning's own top-level build, but is not added to this build.
set(PLANNING_SOURCE_DIR "${HUMANSL_ROOT}/Christian_control/planning")
set(PLANNING_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/planning")
add_subdirectory("${PLANNING_SOURCE_DIR}"
                 "${PLANNING_BINARY_DIR}" EXCLUDE_FROM_ALL)

# --- ../tracking supplies the sensing implementation as targets:
#     humansl_tracking_core (SDK-free snapshot maths) always, humansl_vicon
#     (the live DataStream adapter) when the vendored SDK is present. Only
#     the acquisition thread that feeds the loop lives in this directory.
#     EXCLUDE_FROM_ALL keeps tracking's own tools out of this build. ---
add_subdirectory("${HUMANSL_ROOT}/Christian_control/tracking"
                 "${CMAKE_CURRENT_BINARY_DIR}/tracking" EXCLUDE_FROM_ALL)

# --- humansl_runtime_hardware: the ONE owner of the Kortex dependency —
#     connection, cyclic session, feedback, telemetry and the run-log writer
#     (Hardware), and readiness/stop policy and fault decoding (Safety).
#     Every binary that talks to the arm links this instead of recompiling
#     the sources, so there is one compilation, one set of flags and one
#     place where libKortexApiCpp.a is named. PUBLIC usage requirements
#     carry the Kortex SYSTEM includes, _OS_UNIX, this directory, control's
#     headers (Config.h) and the generated JointLimits.h to every consumer. ---
add_library(humansl_runtime_hardware STATIC
        Hardware.cpp
        Safety.cpp
)
add_humansl_warnings_to(humansl_runtime_hardware)
target_include_directories(humansl_runtime_hardware PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CONTROL_SOURCE_DIR}
        ${CONTROL_GENERATED_DIR})
target_include_directories(humansl_runtime_hardware SYSTEM PUBLIC
        ${KORTEX_DIR}/include
        ${KORTEX_DIR}/include/client
        ${KORTEX_DIR}/include/common
        ${KORTEX_DIR}/include/messages
        ${KORTEX_DIR}/include/client_stubs
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_compile_definitions(humansl_runtime_hardware PUBLIC _OS_UNIX)
target_link_libraries(humansl_runtime_hardware PUBLIC
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# --- The world pose/twist source. The controller stays standalone: when the
#     vendored DataStream SDK is absent (or -DNO_VICON=ON), the stub is
#     compiled instead and StartViconSource returns nullptr — no SDK header or
#     library is touched anywhere in the build, and the vicon_* log columns
#     record honest absence. ---
option(NO_VICON "Build the controller without the Vicon world-pose source" OFF)
set(VICON_SDK "${HUMANSL_ROOT}/third_party/vicon_api")
if (NOT NO_VICON AND TARGET humansl_vicon)
    set(VICON_SOURCE_FILES ViconSource.cpp)
    set(VICON_SOURCE_MODE "sdk")
else ()
    set(VICON_SOURCE_FILES ViconSourceStub.cpp)
    set(VICON_SOURCE_MODE "absent")
endif ()
message(STATUS "Vicon world-pose source: ${VICON_SOURCE_MODE}")

# --- controller: the hardware executable — Kortex adapter sources (loop
#     runner, Vicon acquisition, in-process planner worker, program setup)
#     around the shared execution core, the planner library and the hardware
#     library. ---
add_executable(controller
        Main.cpp
        MainArgs.cpp
        FixedMountSource.cpp
        GoalSocket.cpp
        ProcessLock.cpp
        Runner.cpp
        InProcessPlanner.cpp
        ${VICON_SOURCE_FILES}
)
add_humansl_warnings_to(controller)
# This directory is the include root for the adapter's own headers; the
# control pipeline's headers, the planner's headers, the tracking snapshot
# types (BasePose.h needs the SDK-free ViconSnapshot.h in both build modes)
# and the exchange types all arrive as usage requirements of the linked
# libraries.
target_include_directories(controller PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(controller
        humansl_execution_core
        humansl_planning
        humansl_runtime_hardware
        humansl_tracking_core
)
add_dependencies(controller dh_params_tool dh_params_flange)
target_compile_definitions(controller PRIVATE
        PLANNER_CONFIG_FILE="${PLANNING_SOURCE_DIR}/config/planner.yaml"
        PLANNER_JOINT_LIMITS_FILE="${HUMANSL_ROOT}/Christian_control/model/joint_limits.yaml"
        PLANNER_RIGHT_DH_FILE="${PLANNING_BINARY_DIR}/config/dh_params_tool.yaml"
        PLANNER_LEFT_DH_FILE="${PLANNING_BINARY_DIR}/config/dh_params_flange.yaml")
set_target_properties(controller PROPERTIES
        BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib")
if (VICON_SOURCE_MODE STREQUAL "sdk")
    # The RPATH lets the loader find the boost .so files next to the SDK
    # (mirrors ../tracking) alongside the bundled Pinocchio .so directory.
    target_link_libraries(controller humansl_vicon)
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
#     read-verify only, no motion. ---
add_executable(clear_faults tools/clear_faults.cpp)
add_humansl_warnings_to(clear_faults)
target_link_libraries(clear_faults PRIVATE humansl_runtime_hardware)

# --- set_joint_limits: WRITES the joint 2/4/6 firmware JOINT_LIMIT
#     thresholds (joint-position enforcement). No motion. ---
add_executable(set_joint_limits tools/set_joint_limits.cpp)
add_humansl_warnings_to(set_joint_limits)
target_link_libraries(set_joint_limits PRIVATE humansl_runtime_hardware)

# --- read_safety_limits: READ-ONLY dump of every actuator's configured
#     safety thresholds (DeviceConfig), next to live positions. No motion. ---
add_executable(read_safety_limits tools/read_safety_limits.cpp)
add_humansl_warnings_to(read_safety_limits)
target_link_libraries(read_safety_limits PRIVATE humansl_runtime_hardware)

# --- print_joint_positions: READ-ONLY single-snapshot dump of live joint
#     positions from cyclic feedback. No motion. ---
add_executable(print_joint_positions tools/print_joint_positions.cpp)
add_humansl_warnings_to(print_joint_positions)
target_link_libraries(print_joint_positions PRIVATE humansl_runtime_hardware)

# --- print_end_effector_pose: READ-ONLY single-snapshot dump of the right
#     end-effector's Cartesian pose, via the controller's own forward
#     kinematics on one live feedback reading. Kortex + Pinocchio, no motion. ---
add_executable(print_end_effector_pose tools/print_end_effector_pose.cpp)
target_link_libraries(print_end_effector_pose PRIVATE humansl_runtime_hardware)
add_execution_core_to(print_end_effector_pose)

# --- compare_end_effector_pose: READ-ONLY diagnostic — firmware tool_pose
#     (what the web app shows) vs our Pinocchio FK, from one shared feedback
#     snapshot. Kortex + Pinocchio, no motion. ---
add_executable(compare_end_effector_pose tools/compare_end_effector_pose.cpp)
target_link_libraries(compare_end_effector_pose PRIVATE humansl_runtime_hardware)
add_execution_core_to(compare_end_effector_pose)

# --- print_tool_configuration: READ-ONLY dump of the robot's configured Tool
#     Frame (ControlConfig::GetToolConfiguration) — the offset firmware adds to
#     compute tool_pose_*, invisible to any URDF. No motion. ---
add_executable(print_tool_configuration tools/print_tool_configuration.cpp)
add_humansl_warnings_to(print_tool_configuration)
target_link_libraries(print_tool_configuration PRIVATE humansl_runtime_hardware)

# --- make_synthetic_log: writes a SYNTHETIC log_format 6 CSV through the real
#     run-log writer, so the telemetry schema and scripts/plot_run.py can be
#     exercised with no robot. The writer lives in Hardware.cpp, which links
#     Kortex — link-time only; this tool never connects. ---
add_executable(make_synthetic_log tools/make_synthetic_log.cpp)
add_humansl_warnings_to(make_synthetic_log)
target_link_libraries(make_synthetic_log PRIVATE humansl_runtime_hardware)

include(CTest)
if (BUILD_TESTING)
    add_executable(test_log_schema tests/test_log_schema.cpp)
    add_humansl_warnings_to(test_log_schema)
    target_link_libraries(test_log_schema PRIVATE humansl_runtime_hardware)
    add_test(NAME log_schema COMMAND test_log_schema)

    add_executable(test_cyclic_retry tests/test_cyclic_retry.cpp)
    add_humansl_warnings_to(test_cyclic_retry)
    target_link_libraries(test_cyclic_retry PRIVATE humansl_runtime_hardware)
    add_test(NAME cyclic_retry COMMAND test_cyclic_retry)

    add_executable(test_goal_socket tests/test_goal_socket.cpp GoalSocket.cpp)
    add_humansl_warnings_to(test_goal_socket)
    target_include_directories(test_goal_socket PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR})
    add_bundled_eigen_to(test_goal_socket)
    target_link_libraries(test_goal_socket humansl_contracts pthread)
    add_test(NAME goal_socket COMMAND test_goal_socket)

    add_executable(test_main_args tests/test_main_args.cpp MainArgs.cpp)
    add_humansl_warnings_to(test_main_args)
    target_include_directories(test_main_args PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR})
    add_test(NAME main_args COMMAND test_main_args)
endif ()
