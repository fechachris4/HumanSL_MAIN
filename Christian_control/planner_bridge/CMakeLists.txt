cmake_minimum_required(VERSION 3.16)
project(planner_bridge CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# DT_RUNPATH (the linker's new-dtags default) is non-transitive: it lets
# planner_bridge find libgtsam.so, but libgtsam.so has no RPATH of its own,
# so libgtsam.so's dependency libmetis-gtsam.so never resolves when the
# binary is run bare (outside ctest, which papers over this via
# LD_LIBRARY_PATH). DT_RPATH is transitive and fixes this for every
# executable below.
add_link_options(-Wl,--disable-new-dtags)

get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(TP_LIB "${HUMANSL_ROOT}/third_party/lib")

# The one canonical URDF (basic_control/config/GEN3_dual_mounted.urdf) — same
# macro name basic_control's CMakeLists.txt defines, so both projects parse
# the same file through the same Dynamics/Kinematics code.
set(GEN3_DUAL_URDF
    "${HUMANSL_ROOT}/Christian_control/basic_control/config/GEN3_dual_mounted.urdf")
add_compile_definitions(GEN3_DUAL_URDF_PATH="${GEN3_DUAL_URDF}")

# TrajectoryGeneration sources compiled directly — this project must not
# depend on the (currently broken) root build. Dynamics.cpp/Kinematics.cpp
# are basic_control's already-Pinocchio-based URDF model and FK/Jacobian
# adapter, reused by source path (PinocchioKinematicsAdapter.cpp wraps them
# for utils.cpp/analytical_ik.h — see TrajectoryGeneration/include).
set(TG_SOURCES
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/utils.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/GenerateArmModel.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/TrajectoryInitiation.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/TrajectoryOptimization.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/PinocchioKinematicsAdapter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/GenerateTrajectory.cpp
    ${HUMANSL_ROOT}/TrajectoryExecution/src/Dynamics.cpp
    ${HUMANSL_ROOT}/Christian_control/basic_control/src/Kinematics.cpp
)

add_library(bridge_core STATIC ${TG_SOURCES})
target_include_directories(bridge_core PUBLIC
    ${HUMANSL_ROOT}/TrajectoryGeneration/include
    ${HUMANSL_ROOT}/TrajectoryExecution/include
    ${HUMANSL_ROOT}/Christian_control/basic_control/src
    ${HUMANSL_ROOT}/third_party/include
    ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(bridge_core PUBLIC
    ${TP_LIB}/libgpmp2.so
    ${TP_LIB}/libgtsam.so
    ${TP_LIB}/libmetis-gtsam.so
    ${TP_LIB}/libpinocchio_default.so
    ${TP_LIB}/libpinocchio_parsers.so
    yaml-cpp
    pthread
)
set_target_properties(bridge_core PROPERTIES BUILD_RPATH "${TP_LIB}")

# libpinocchio_parsers.so references collision symbols (coal) that are not
# bundled and that we never call (URDF parsing only) — same gap
# basic_control's CMakeLists.txt already works around for every Pinocchio
# target. Applied here once, at the INTERFACE level, so every executable
# that links bridge_core (below) inherits it automatically.
target_link_options(bridge_core INTERFACE
    -Wl,--allow-shlib-undefined
    -Wl,-z,lazy
)

add_executable(planner_bridge src/main.cpp)
target_link_libraries(planner_bridge PRIVATE bridge_core)
set_target_properties(planner_bridge PROPERTIES BUILD_RPATH "${TP_LIB}")

# The DH parameter YAML gpmp2 consumes is GENERATED from the URDF at build
# time — never committed, never hand-edited. generate_dh_params derives the
# link lengths from Pinocchio FK and self-verifies before writing; a failed
# derivation fails the build. Listing the URDF in DEPENDS means editing it
# regenerates the YAML on the next build. The LD_LIBRARY_PATH wrapper
# mirrors the ctest workaround below (third_party .so files lack RUNPATHs
# to their own siblings).
add_executable(generate_dh_params tools/generate_dh_params.cpp)
target_include_directories(generate_dh_params PRIVATE src)
target_link_libraries(generate_dh_params PRIVATE bridge_core)
set_target_properties(generate_dh_params PROPERTIES BUILD_RPATH "${TP_LIB}")

set(DH_PARAMS_TOOL_YAML "${CMAKE_CURRENT_BINARY_DIR}/config/dh_params_tool.yaml")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/config")
add_custom_command(OUTPUT "${DH_PARAMS_TOOL_YAML}"
    COMMAND ${CMAKE_COMMAND} -E env "LD_LIBRARY_PATH=${TP_LIB}"
            "$<TARGET_FILE:generate_dh_params>" "${DH_PARAMS_TOOL_YAML}"
    DEPENDS generate_dh_params "${GEN3_DUAL_URDF}"
    COMMENT "Deriving DH parameters from GEN3_dual_mounted.urdf"
    VERBATIM)
add_custom_target(dh_params_tool ALL DEPENDS "${DH_PARAMS_TOOL_YAML}")

enable_testing()

target_sources(bridge_core PRIVATE src/PlannerModel.cpp)
add_executable(test_planner_model tests/test_planner_model.cpp)
target_include_directories(test_planner_model PRIVATE src)
target_link_libraries(test_planner_model PRIVATE bridge_core)
set_target_properties(test_planner_model PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME planner_model COMMAND test_planner_model
         ${DH_PARAMS_TOOL_YAML})

add_executable(test_gpmp2_urdf_validation tests/test_gpmp2_urdf_validation.cpp)
target_include_directories(test_gpmp2_urdf_validation PRIVATE src)
target_link_libraries(test_gpmp2_urdf_validation PRIVATE bridge_core)
set_target_properties(test_gpmp2_urdf_validation PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME gpmp2_urdf_validation COMMAND test_gpmp2_urdf_validation
         ${DH_PARAMS_TOOL_YAML})

add_executable(test_sphere_scaling tests/test_sphere_scaling.cpp)
target_include_directories(test_sphere_scaling PRIVATE src)
target_link_libraries(test_sphere_scaling PRIVATE bridge_core)
set_target_properties(test_sphere_scaling PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME sphere_scaling COMMAND test_sphere_scaling
         ${DH_PARAMS_TOOL_YAML})

target_sources(bridge_core PRIVATE src/WorldSdf.cpp)
add_executable(test_world_sdf tests/test_world_sdf.cpp)
target_include_directories(test_world_sdf PRIVATE src)
target_link_libraries(test_world_sdf PRIVATE bridge_core)
set_target_properties(test_world_sdf PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME world_sdf COMMAND test_world_sdf)

target_sources(bridge_core PRIVATE src/PlanSolver.cpp)
add_executable(test_plan_solver tests/test_plan_solver.cpp)
target_include_directories(test_plan_solver PRIVATE src)
target_link_libraries(test_plan_solver PRIVATE bridge_core)
set_target_properties(test_plan_solver PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME plan_solver COMMAND test_plan_solver
         ${DH_PARAMS_TOOL_YAML}
         ${HUMANSL_ROOT}/TrajectoryGeneration/config/joint_limits.yaml)

target_sources(bridge_core PRIVATE src/Waypoints.cpp)
add_executable(test_waypoints
    tests/test_waypoints.cpp
    ${HUMANSL_ROOT}/Christian_control/basic_control/src/Targets.cpp
    ${HUMANSL_ROOT}/Christian_control/basic_control/src/JointTrajectory.cpp
)
target_include_directories(test_waypoints PRIVATE
    src ${HUMANSL_ROOT}/Christian_control/basic_control/src)
target_link_libraries(test_waypoints PRIVATE bridge_core)
set_target_properties(test_waypoints PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME waypoints COMMAND test_waypoints
         ${DH_PARAMS_TOOL_YAML})

target_sources(bridge_core PRIVATE src/StartState.cpp)
add_executable(test_start_state tests/test_start_state.cpp)
target_include_directories(test_start_state PRIVATE src)
target_link_libraries(test_start_state PRIVATE bridge_core)
set_target_properties(test_start_state PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME start_state COMMAND test_start_state)

target_sources(bridge_core PRIVATE src/TrajectoryEmit.cpp src/BridgeMain.cpp)
add_executable(test_bridge_main
    tests/test_bridge_main.cpp
    ${HUMANSL_ROOT}/Christian_control/basic_control/src/Targets.cpp
    ${HUMANSL_ROOT}/Christian_control/basic_control/src/JointTrajectory.cpp
)
target_include_directories(test_bridge_main PRIVATE
    src ${HUMANSL_ROOT}/Christian_control/basic_control/src)
target_link_libraries(test_bridge_main PRIVATE bridge_core)
set_target_properties(test_bridge_main PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME bridge_main COMMAND test_bridge_main
         ${DH_PARAMS_TOOL_YAML}
         ${HUMANSL_ROOT}/TrajectoryGeneration/config/joint_limits.yaml)

# Every consumer of the generated YAML depends on the generating target, so
# single-target builds (`make test_plan_solver`) produce the file before
# anything runs against it. (`ALL` on dh_params_tool covers bare `ctest`
# after a full build — ctest itself never builds.)
foreach(dh_consumer planner_bridge test_planner_model test_gpmp2_urdf_validation test_sphere_scaling
        test_plan_solver test_waypoints test_bridge_main)
    add_dependencies(${dh_consumer} dh_params_tool)
endforeach()

# libgtsam.so/libgpmp2.so carry no RUNPATH of their own to their sibling
# libs (libmetis-gtsam.so, libcephes-gtsam.so.1, ...), so RUNPATH on our
# binaries alone doesn't resolve transitive deps under ctest's fork/exec —
# a pre-existing gap in the third_party build, worked around once here for
# every bridge_core-linked test rather than per add_test call.
foreach(bridge_test_name planner_model world_sdf plan_solver waypoints start_state bridge_main gpmp2_urdf_validation sphere_scaling)
    set_tests_properties(${bridge_test_name} PROPERTIES
        ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")
endforeach()
