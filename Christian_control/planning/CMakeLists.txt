cmake_minimum_required(VERSION 3.16)
project(planning CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)


# DT_RUNPATH (the linker's new-dtags default) is non-transitive: it lets
# planning find libgtsam.so, but libgtsam.so has no RPATH of its own,
# so libgtsam.so's dependency libmetis-gtsam.so never resolves when the
# binary is run bare. DT_RPATH is transitive and fixes this for every
# executable below.
add_link_options(-Wl,--disable-new-dtags)

get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(TP_LIB "${HUMANSL_ROOT}/third_party/lib")

# The planner/controller exchange types, as a linkable target. Guarded because
# the controller's nested build adds this project as a subdirectory and has
# already defined the target by the time it does.
if (NOT TARGET humansl_contracts)
    add_subdirectory("${HUMANSL_ROOT}/Christian_control/contracts"
                     "${CMAKE_CURRENT_BINARY_DIR}/contracts")
endif ()

# The arm's model and kinematics. GPMP2 optimises; it is not given a robot
# model of its own, so RobotModel/Kinematics are linked from ../control rather
# than compiled a second time here. Guarded for the same reason as above.
if (NOT TARGET humansl_execution_core)
    add_subdirectory("${HUMANSL_ROOT}/Christian_control/control"
                     "${CMAKE_CURRENT_BINARY_DIR}/control")
endif ()

# The one canonical URDF (model/GEN3_dual_mounted.urdf) — same
# macro name control's CMakeLists.txt defines, so both projects parse
# the same file through the same RobotModel/Kinematics code.
set(GEN3_DUAL_URDF
    "${HUMANSL_ROOT}/Christian_control/model/GEN3_dual_mounted.urdf")
add_compile_definitions(GEN3_DUAL_URDF_PATH="${GEN3_DUAL_URDF}")

# optimisation/ is this project's own copy of the GPMP2 pipeline, taken from
# the inherited TrajectoryGeneration tree on 2026-08-10. The upstream tree is
# frozen and no longer compiled from; this copy is the live one. GPMP2 owns
# the factor-graph optimisation; it does NOT own a robot model — the URDF
# model and the FK/Jacobian adapter come from ../control, linked below rather
# than recompiled here (PinocchioKinematicsAdapter.cpp wraps them for
# utils.cpp and analytical_ik.h).
set(OPTIMISATION_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/optimisation/utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/optimisation/GenerateArmModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/optimisation/TrajectoryInitiation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/optimisation/TrajectoryOptimization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/optimisation/OptimisationWaypoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/optimisation/PinocchioKinematicsAdapter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/optimisation/GenerateTrajectory.cpp
)

# The planner's joint-limits table, also this project's own copy (the root
# and TrajectoryGeneration copies are frozen with the inherited tree).
set(JOINT_LIMITS_YAML "${CMAKE_CURRENT_SOURCE_DIR}/config/joint_limits.yaml")

add_library(bridge_core STATIC ${OPTIMISATION_SOURCES})
target_include_directories(bridge_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/optimisation
    ${HUMANSL_ROOT}/third_party/include
    ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(bridge_core PUBLIC
    humansl_execution_core
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
# control's CMakeLists.txt already works around for every Pinocchio
# target. Applied here once, at the INTERFACE level, so every executable
# that links bridge_core (below) inherits it automatically.
target_link_options(bridge_core INTERFACE
    -Wl,--allow-shlib-undefined
    -Wl,-z,lazy
)

# All planner implementation sources belong to the reusable runtime library,
# so the controller's nested build receives the complete library when the
# planner is consumed in process.
target_sources(bridge_core PRIVATE
    src/PlannerModel.cpp
    src/MountSdf.cpp
    src/CartesianPath.cpp
    src/PathFrames.cpp
    src/PathIk.cpp
    src/InterArmDistance.cpp
    src/PathAssembly.cpp
    src/PathValidationReport.cpp
    src/ValidatePath.cpp
    src/WorldTrajectoryProjection.cpp
    src/PlannerConfig.cpp
    src/PlanSolver.cpp
    src/PathValidation.cpp
    src/PlanDebugDump.cpp
    src/StartState.cpp
    src/BridgeMain.cpp
    src/PlannerRuntime.cpp
    src/BaselineBridge.cpp
)
target_link_libraries(bridge_core PUBLIC humansl_contracts)

add_executable(planner_bridge src/main.cpp
    src/PlannerPreview.cpp)
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
    COMMENT "Deriving right-arm DH parameters from GEN3_dual_mounted.urdf"
    VERBATIM)
add_custom_target(dh_params_tool ALL DEPENDS "${DH_PARAMS_TOOL_YAML}")

# Left arm's chain, ending at the bare flange (left_end_effector_link) rather
# than a mounted tool — see generate_dh_params.cpp's header comment. Same
# generator binary, --arm left, a second output file so neither chain's
# YAML is ever overwritten by the other's build step.
set(DH_PARAMS_FLANGE_YAML "${CMAKE_CURRENT_BINARY_DIR}/config/dh_params_flange.yaml")
add_custom_command(OUTPUT "${DH_PARAMS_FLANGE_YAML}"
    COMMAND ${CMAKE_COMMAND} -E env "LD_LIBRARY_PATH=${TP_LIB}"
            "$<TARGET_FILE:generate_dh_params>" "${DH_PARAMS_FLANGE_YAML}" --arm left
    DEPENDS generate_dh_params "${GEN3_DUAL_URDF}"
    COMMENT "Deriving left-arm DH parameters from GEN3_dual_mounted.urdf"
    VERBATIM)
add_custom_target(dh_params_flange ALL DEPENDS "${DH_PARAMS_FLANGE_YAML}")


# ---------------------------------------------------------------------------
# Minimum architectural tests for the mount-frame migration (2026-08-20):
# the one input conversion (ToMount), the static mount SDF grid, and the one
# mount->world output projection. Registered only when planning is the
# top-level project, so the controller's nested build stays test-free.
# ---------------------------------------------------------------------------
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
enable_testing()

add_executable(test_path_frames_to_mount tests/test_path_frames_to_mount.cpp)
target_include_directories(test_path_frames_to_mount PRIVATE src)
target_link_libraries(test_path_frames_to_mount PRIVATE bridge_core)
set_target_properties(test_path_frames_to_mount PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME path_frames_to_mount COMMAND test_path_frames_to_mount)

add_executable(test_mount_sdf tests/test_mount_sdf.cpp)
target_include_directories(test_mount_sdf PRIVATE src)
target_link_libraries(test_mount_sdf PRIVATE bridge_core)
set_target_properties(test_mount_sdf PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME mount_sdf COMMAND test_mount_sdf)

add_executable(test_scene_config tests/test_scene_config.cpp)
target_include_directories(test_scene_config PRIVATE src)
target_link_libraries(test_scene_config PRIVATE bridge_core)
set_target_properties(test_scene_config PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME scene_config COMMAND test_scene_config)

add_executable(test_bridge_args tests/test_bridge_args.cpp src/PlannerPreview.cpp)
target_include_directories(test_bridge_args PRIVATE src)
target_link_libraries(test_bridge_args PRIVATE bridge_core)
set_target_properties(test_bridge_args PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME bridge_args COMMAND test_bridge_args)

add_test(NAME solver_scene_ownership
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}/src/PlanSolver.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_solver_scene_ownership.cmake)

add_executable(test_projection_exit tests/test_projection_exit.cpp)
target_include_directories(test_projection_exit PRIVATE src)
target_link_libraries(test_projection_exit PRIVATE bridge_core)
set_target_properties(test_projection_exit PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME projection_exit COMMAND test_projection_exit
         ${DH_PARAMS_TOOL_YAML} ${DH_PARAMS_FLANGE_YAML})

add_executable(test_plan_verdict tests/test_plan_verdict.cpp)
target_include_directories(test_plan_verdict PRIVATE src)
target_link_libraries(test_plan_verdict PRIVATE bridge_core)
set_target_properties(test_plan_verdict PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME plan_verdict COMMAND test_plan_verdict)

# The joint-type contract: continuous joints 1/3/5/7 carry no position
# limit anywhere in the planner, bounded joints 2/4/6 do. Checked against
# the generated kBoundedMask, so the validator cannot drift from
# joint_limits.yaml and the URDF.
add_executable(test_joint_type_contract tests/test_joint_type_contract.cpp)
target_include_directories(test_joint_type_contract PRIVATE src)
target_link_libraries(test_joint_type_contract PRIVATE bridge_core)
set_target_properties(test_joint_type_contract PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME joint_type_contract COMMAND test_joint_type_contract)

endif ()

# Offline reachability probe for traced paths. Its target registration was
# lost with the retired test infrastructure (2026-08-20); reinstated here so
# the tool keeps compiling with the library it probes. Hardware-free.
add_executable(probe_path_reachability tools/probe_path_reachability.cpp)
target_include_directories(probe_path_reachability PRIVATE src)
target_link_libraries(probe_path_reachability PRIVATE bridge_core)
set_target_properties(probe_path_reachability PROPERTIES BUILD_RPATH "${TP_LIB}")
