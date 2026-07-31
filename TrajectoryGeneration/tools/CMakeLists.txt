# Standalone build for plan_move — the offline planner CLI. Compiles the
# TrajectoryGeneration sources directly (plus the Jacobian they use from
# TrajectoryExecution and basic_control's TrajectoryFile for the shared
# validation gate); links the bundled GTSAM/GPMP2 stack. Independent of
# the broken root build and of basic_control's build.
#
#   cd TrajectoryGeneration/tools && mkdir -p build && cd build
#   cmake .. && make
#   ./plan_move --help
#
# No Kortex, no robot: this tool only plans and writes files.
cmake_minimum_required(VERSION 3.12)
project(plan_move_tool)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
add_definitions(-w)

set(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
get_filename_component(HUMANSL_ROOT_ABS "${HUMANSL_ROOT}" ABSOLUTE)

add_executable(plan_move
        plan_move.cpp
        ${HUMANSL_ROOT}/TrajectoryGeneration/src/utils.cpp
        ${HUMANSL_ROOT}/TrajectoryGeneration/src/Obstacles.cpp
        ${HUMANSL_ROOT}/TrajectoryGeneration/src/GenerateArmModel.cpp
        ${HUMANSL_ROOT}/TrajectoryGeneration/src/GenerateLogs.cpp
        ${HUMANSL_ROOT}/TrajectoryGeneration/src/TrajectoryInitiation.cpp
        ${HUMANSL_ROOT}/TrajectoryGeneration/src/TrajectoryOptimization.cpp
        ${HUMANSL_ROOT}/TrajectoryExecution/src/Jacobian.cpp
        ${HUMANSL_ROOT}/Christian_control/basic_control/src/control/TrajectoryFile.cpp
)

target_include_directories(plan_move PRIVATE
        ${HUMANSL_ROOT}/TrajectoryGeneration/include
        ${HUMANSL_ROOT}/TrajectoryExecution/include
        ${HUMANSL_ROOT}/Christian_control/basic_control/src
        ${HUMANSL_ROOT}/third_party/include
        ${HUMANSL_ROOT}/third_party/include/eigen3
)

# The planner reads the REPOSITORY-ROOT config (the copy the legacy
# binaries loaded, consistent with the IK solvers' hardcoded tool length).
target_compile_definitions(plan_move PRIVATE
        PLAN_CONFIG_DIR="${HUMANSL_ROOT_ABS}/config")

target_link_libraries(plan_move
        ${HUMANSL_ROOT}/third_party/lib/libgtsam.so
        ${HUMANSL_ROOT}/third_party/lib/libgpmp2.so
        ${HUMANSL_ROOT}/third_party/lib/libezc3d.so
        yaml-cpp
        pthread
)

# Old-style DT_RPATH (not RUNPATH): it applies TRANSITIVELY, so the
# bundled libgtsam.so can find its own bundled dependencies
# (libmetis-gtsam.so etc.) without an LD_LIBRARY_PATH.
set_target_properties(plan_move PROPERTIES
        BUILD_RPATH "${HUMANSL_ROOT_ABS}/third_party/lib")
target_link_options(plan_move PRIVATE -Wl,--disable-new-dtags)
