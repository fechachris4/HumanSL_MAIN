# Standalone build for controller — links the bundled Kortex API and Pinocchio.
# No GTSAM / GPMP2 / Vicon required.
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
add_definitions(-D_OS_UNIX -w)

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
                ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tools
        COMMENT "Running cppcheck on src/ and tools/"
    )
endif ()

# Bundled Kortex API lives two levels up, in HumanSL_MAIN/third_party/.
set(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(KORTEX_DIR "${HUMANSL_ROOT}/third_party/kortex_api")

# The arm's URDF model, used by Pinocchio. Our own copy (duplicated from
# ../../config/GEN3_With_GRIPPER_DYNAMICS.urdf) so joint limits are ours to
# edit: velocity limits set to Kinova's general limits (1.3893 rad/s joints
# 1-4, 1.2200 rad/s joints 5-7 = 79.6 / 69.9 deg/s, see
# config::kModelVelocityLimitsDegS) instead of the original 0.8727 (admittance).
set(GEN3_URDF "${CMAKE_CURRENT_SOURCE_DIR}/config/GEN3_custom.urdf")

include_directories(
        ${KORTEX_DIR}/include
        ${KORTEX_DIR}/include/client
        ${KORTEX_DIR}/include/common
        ${KORTEX_DIR}/include/messages
        ${KORTEX_DIR}/include/client_stubs
)

# --- controller: Kortex + bundled Pinocchio (dynamics/kinematics via the
#     Dynamics class reused from TrajectoryExecution) ---
add_executable(controller
        src/app/main.cpp
        src/app/Options.cpp
        src/hardware/Measure.cpp
        src/hardware/Cyclic.cpp
        src/math/Kinematics.cpp
        src/control/ResolvedRate.cpp
        src/control/ReactivePose.cpp
        src/control/Target.cpp
        src/loop/Runner.cpp
        src/hardware/Record.cpp
        src/hardware/Connect.cpp
        src/safety/Supervisor.cpp
        src/safety/FaultReport.cpp
        src/safety/ServoingGuard.cpp
        src/actuation/PositionIntegration.cpp
        ${HUMANSL_ROOT}/TrajectoryExecution/src/Dynamics.cpp
)

# src/ is the include root: local includes are layer-qualified
# ("hardware/Connect.h"), so the technical layer is visible at every include.
target_include_directories(controller PRIVATE
        src
        third_party
        ${HUMANSL_ROOT}/third_party/include
        ${HUMANSL_ROOT}/third_party/include/eigen3
        ${HUMANSL_ROOT}/TrajectoryExecution/include
)

target_link_libraries(controller
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_default.so
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_parsers.so
        pthread
)

# Let the executable find the bundled .so files at runtime.
set_target_properties(controller PROPERTIES
        BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
)

# libpinocchio_parsers.so references collision symbols (coal) that are not
# bundled and that we never call (we only build the model, no collision
# geometry). Allow them to stay unresolved, and bind lazily so the loader
# doesn't look them up at startup either.
target_link_options(controller PRIVATE
        -Wl,--allow-shlib-undefined
        -Wl,-z,lazy
)

# --- query_limits: READ-ONLY helper that prints the robot's kinematic hard
#     limits (no Pinocchio, no motion code) ---
add_executable(query_limits
        tools/query_limits.cpp
        src/hardware/Connect.cpp
)
# query_limits.cpp lives in tools/ but includes headers from src/.
target_include_directories(query_limits PRIVATE src)
target_link_libraries(query_limits
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# --- test_control_logic: hardware-free tests for the controller's pure
#     logic (DLS resolution, target parsing, TargetStore). No Kortex, no
#     Pinocchio — safe to run anywhere. ---
enable_testing()
add_executable(test_control_logic
        tests/test_control_logic.cpp
        src/control/Target.cpp
        src/actuation/PositionIntegration.cpp
)
target_include_directories(test_control_logic PRIVATE
        src
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(test_control_logic pthread)
add_test(NAME control_logic COMMAND test_control_logic)

# --- test_reactive_law: hardware-free tests for the reactive pose law
#     (control/ReactiveLaw.h — pure Eigen) + pose-target parsing, including
#     cross-validation fixtures generated from the Python simulation law
#     (scripts/gen_reactive_fixtures.py). No Kortex, no Pinocchio — safe to
#     run anywhere. ---
add_executable(test_reactive_law
        tests/test_reactive_law.cpp
        src/control/Target.cpp
)
target_include_directories(test_reactive_law PRIVATE
        src
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(test_reactive_law pthread)
add_test(NAME reactive_law COMMAND test_reactive_law)

# --- hardware-machine test battery + the decision-14 replay harness: the
#     bundled Kortex/Pinocchio binaries are Linux x86-64 ELF, so these link
#     only there. ctest elsewhere runs the portable suite above. ---
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(test_supervisor
            tests/test_supervisor.cpp
            src/safety/Supervisor.cpp
            src/safety/FaultReport.cpp
    )
    target_include_directories(test_supervisor PRIVATE
            src
            ${HUMANSL_ROOT}/third_party/include/eigen3
    )
    target_link_libraries(test_supervisor
            ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
            pthread
    )
    add_test(NAME supervisor COMMAND test_supervisor)

    add_executable(test_resolved_rate
            tests/test_resolved_rate.cpp
            src/control/ResolvedRate.cpp
            src/control/Target.cpp
            src/math/Kinematics.cpp
            ${HUMANSL_ROOT}/TrajectoryExecution/src/Dynamics.cpp
    )
    target_include_directories(test_resolved_rate PRIVATE
            src
            ${HUMANSL_ROOT}/third_party/include
            ${HUMANSL_ROOT}/third_party/include/eigen3
            ${HUMANSL_ROOT}/TrajectoryExecution/include
    )
    target_link_libraries(test_resolved_rate
            ${HUMANSL_ROOT}/third_party/lib/libpinocchio_default.so
            ${HUMANSL_ROOT}/third_party/lib/libpinocchio_parsers.so
            pthread
    )
    set_target_properties(test_resolved_rate PROPERTIES
            BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
    )
    target_link_options(test_resolved_rate PRIVATE
            -Wl,--allow-shlib-undefined
            -Wl,-z,lazy
    )
    target_compile_definitions(test_resolved_rate PRIVATE GEN3_URDF_PATH="${GEN3_URDF}")
    add_test(NAME resolved_rate COMMAND test_resolved_rate)

    add_executable(replay_controller
            tests/replay_controller.cpp
            src/actuation/PositionIntegration.cpp
            src/control/ResolvedRate.cpp
            src/control/Target.cpp
            src/math/Kinematics.cpp
            ${HUMANSL_ROOT}/TrajectoryExecution/src/Dynamics.cpp
    )
    target_include_directories(replay_controller PRIVATE
            src
            ${HUMANSL_ROOT}/third_party/include
            ${HUMANSL_ROOT}/third_party/include/eigen3
            ${HUMANSL_ROOT}/TrajectoryExecution/include
    )
    target_link_libraries(replay_controller
            ${HUMANSL_ROOT}/third_party/lib/libpinocchio_default.so
            ${HUMANSL_ROOT}/third_party/lib/libpinocchio_parsers.so
            pthread
    )
    set_target_properties(replay_controller PROPERTIES
            BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
    )
    target_link_options(replay_controller PRIVATE
            -Wl,--allow-shlib-undefined
            -Wl,-z,lazy
    )
    target_compile_definitions(replay_controller PRIVATE GEN3_URDF_PATH="${GEN3_URDF}")
endif ()

target_compile_definitions(controller PRIVATE GEN3_URDF_PATH="${GEN3_URDF}")

# Where run CSVs go: runs/YYYY-MM-DD/ under the repository root (the layout
# the plot scripts' no-argument mode searches). Baked in absolute, like
# GEN3_URDF_PATH, so the controller writes there from any working directory.
get_filename_component(HUMANSL_ROOT_ABS "${HUMANSL_ROOT}" ABSOLUTE)
target_compile_definitions(controller PRIVATE RUNS_ROOT_DIR="${HUMANSL_ROOT_ABS}/runs")

# Default runtime-config file: a COMPILED absolute path (like GEN3_URDF_PATH),
# loaded when it exists and no --config is given. Never cwd-relative — the
# no-auto-discovery rule's hazard was files lying in the working directory
# (docs/decisions/runtime-config.md, "Default config file" amendment).
target_compile_definitions(controller PRIVATE
        DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/control.toml")
