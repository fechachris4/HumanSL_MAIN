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
                -DGEN3_DUAL_URDF_PATH=\\\"${CMAKE_CURRENT_SOURCE_DIR}/config/GEN3_dual_mounted.urdf\\\"
                ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tools
        COMMENT "Running cppcheck on src/ and tools/"
    )
endif ()

# Bundled Kortex API lives two levels up, in HumanSL_MAIN/third_party/.
set(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(KORTEX_DIR "${HUMANSL_ROOT}/third_party/kortex_api")

# The single runtime model retains the downloaded dual-arm world/mount
# topology, with both Gen3 chains aligned to Kinova's official ros_kortex
# v2.5.2 URDF. DualArmKinematics handles Pinocchio's continuous-joint
# representation and adapts that model to the right-only seven-wide controller.
set(GEN3_DUAL_URDF
        "${CMAKE_CURRENT_SOURCE_DIR}/config/GEN3_dual_mounted.urdf")
add_compile_definitions(GEN3_DUAL_URDF_PATH="${GEN3_DUAL_URDF}")

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
        src/Main.cpp
        src/Runner.cpp
        src/Controller.cpp
        src/Targets.cpp
        src/Actuation.cpp
        src/Kinematics.cpp
        src/Hardware.cpp
        src/Safety.cpp
        ${HUMANSL_ROOT}/TrajectoryExecution/src/Dynamics.cpp
)

# src/ is the include root. One file per job, named for that job:
# State/ReactiveLaw/Controller/Targets/Actuation are pure Eigen,
# Kinematics adds Pinocchio, Hardware adds Kortex, Runner is the loop,
# Main the program, Config the settings.
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

# --- clear_faults: supported whole-arm fault recovery (Base::ClearFaults),
#     read-verify only, no motion. Kortex, no Pinocchio. ---
add_executable(clear_faults
        tools/clear_faults.cpp
        src/Hardware.cpp
        src/Safety.cpp
)
target_include_directories(clear_faults PRIVATE
        src
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(clear_faults
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# --- probe_direction: OFFLINE joint-velocity-sign probe for a Cartesian
#     direction at a given pose (controller's own model + DLS). No robot. ---
add_executable(probe_direction
        tools/probe_direction.cpp
        src/Kinematics.cpp
        ${HUMANSL_ROOT}/TrajectoryExecution/src/Dynamics.cpp
)
target_include_directories(probe_direction PRIVATE
        src
        ${HUMANSL_ROOT}/third_party/include
        ${HUMANSL_ROOT}/third_party/include/eigen3
        ${HUMANSL_ROOT}/TrajectoryExecution/include
)
target_link_libraries(probe_direction
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_default.so
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_parsers.so
        pthread
)
set_target_properties(probe_direction PROPERTIES
        BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
)
target_link_options(probe_direction PRIVATE
        -Wl,--allow-shlib-undefined
        -Wl,-z,lazy
)

# --- set_joint_limits: WRITES the joint 4/6 JOINT_LIMIT safety thresholds
#     (degenerate 0/0 -> rated range backstop). No motion. ---
add_executable(set_joint_limits
        tools/set_joint_limits.cpp
        src/Hardware.cpp
)
target_include_directories(set_joint_limits PRIVATE
        src
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
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
target_include_directories(read_safety_limits PRIVATE
        src
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(read_safety_limits
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
target_include_directories(make_synthetic_log PRIVATE
        src
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(make_synthetic_log
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# --- test_control_logic: hardware-free tests for the controller's pure
#     logic (DLS resolution, cycle-dt clamp, PositionIntegration). No
#     Kortex, no Pinocchio — safe to run anywhere. ---
enable_testing()
add_executable(test_control_logic
        tests/test_control_logic.cpp
        src/Actuation.cpp
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
        src/Targets.cpp
)
target_include_directories(test_reactive_law PRIVATE
        src
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(test_reactive_law pthread)
add_test(NAME reactive_law COMMAND test_reactive_law)

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

# --- hardware-machine test battery: the bundled Kortex/Pinocchio binaries
#     are Linux x86-64 ELF, so these link only there. ctest elsewhere runs
#     the portable suite above. ---
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(test_supervisor
            tests/test_supervisor.cpp
            src/Safety.cpp
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

    # The full model/mount geometry and explicit right-7-of-14 adapter.
    add_executable(test_dual_arm_model
            tests/test_dual_arm_model.cpp
            src/Kinematics.cpp
            ${HUMANSL_ROOT}/TrajectoryExecution/src/Dynamics.cpp
    )
    target_include_directories(test_dual_arm_model PRIVATE
            src
            ${HUMANSL_ROOT}/third_party/include
            ${HUMANSL_ROOT}/third_party/include/eigen3
            ${HUMANSL_ROOT}/TrajectoryExecution/include
    )
    target_link_libraries(test_dual_arm_model
            ${HUMANSL_ROOT}/third_party/lib/libpinocchio_default.so
            ${HUMANSL_ROOT}/third_party/lib/libpinocchio_parsers.so
            pthread
    )
    set_target_properties(test_dual_arm_model PROPERTIES
            BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
    )
    target_link_options(test_dual_arm_model PRIVATE
            -Wl,--allow-shlib-undefined
            -Wl,-z,lazy
    )
    add_test(NAME dual_arm_model COMMAND test_dual_arm_model)
endif ()

# Where run CSVs go: runs/YYYY-MM-DD/ under the repository root (the layout
# the plot scripts' no-argument mode searches). Baked in absolute, like the
# dual-model path, so the controller writes there from any working directory.
get_filename_component(HUMANSL_ROOT_ABS "${HUMANSL_ROOT}" ABSOLUTE)
target_compile_definitions(controller PRIVATE RUNS_ROOT_DIR="${HUMANSL_ROOT_ABS}/runs")
