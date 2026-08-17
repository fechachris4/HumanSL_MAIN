cmake_minimum_required(VERSION 3.16)
project(planner_bridge CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    option(PLANNER_BRIDGE_TESTS
           "Register planner_bridge's hardware-free test targets" ON)
else ()
    option(PLANNER_BRIDGE_TESTS
           "Register planner_bridge's hardware-free test targets" OFF)
endif ()

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
# the same file through the same RobotModel/Kinematics code.
set(GEN3_DUAL_URDF
    "${HUMANSL_ROOT}/Christian_control/basic_control/config/GEN3_dual_mounted.urdf")
add_compile_definitions(GEN3_DUAL_URDF_PATH="${GEN3_DUAL_URDF}")

# trajectory_generation/ is this project's own copy of the GPMP2 pipeline,
# taken from the inherited TrajectoryGeneration tree on 2026-08-10. The
# upstream tree is frozen and no longer compiled from; this copy is the live
# one. RobotModel.cpp/Kinematics.cpp are basic_control's Pinocchio-based URDF
# model and FK/Jacobian adapter, reused by source path
# (PinocchioKinematicsAdapter.cpp wraps them for utils.cpp/analytical_ik.h —
# see trajectory_generation/include).
set(TG_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_generation/src/utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_generation/src/GenerateArmModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_generation/src/TrajectoryInitiation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_generation/src/TrajectoryOptimization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_generation/src/OptimisationWaypoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_generation/src/PinocchioKinematicsAdapter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/GenerateTrajectory.cpp
    ${HUMANSL_ROOT}/Christian_control/basic_control/src/RobotModel.cpp
    ${HUMANSL_ROOT}/Christian_control/basic_control/src/Kinematics.cpp
)

# The planner's joint-limits table, also this project's own copy (the root
# and TrajectoryGeneration copies are frozen with the inherited tree).
set(JOINT_LIMITS_YAML "${CMAKE_CURRENT_SOURCE_DIR}/config/joint_limits.yaml")

add_library(bridge_core STATIC ${TG_SOURCES})
target_include_directories(bridge_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_generation/include
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

# All planner implementation sources belong to the reusable runtime library.
# Test registration below is optional, but the controller's nested build must
# still receive this complete library when planner_bridge is consumed in
# process.
target_sources(bridge_core PRIVATE
    src/PlannerModel.cpp
    src/WorldSdf.cpp
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
    src/StartState.cpp
    src/BridgeMain.cpp
    src/PlannerRuntime.cpp
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract/PlanningRequest.cpp
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract/WorldCartesianTrajectory.cpp
)
target_include_directories(bridge_core PUBLIC
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract)

add_executable(planner_bridge src/main.cpp
    src/PlannerPreview.cpp
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract/WorldCartesianTrajectoryWire.cpp)
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

# Left arm's chain, ending at the bare flange (leftEndEffector_Link) rather
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

if (PLANNER_BRIDGE_TESTS)
enable_testing()

add_test(NAME run_session
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_run_session.sh)

# Shared planner/controller world-Cartesian wire contract. Eigen-only: its
# parser and validator run entirely off the real-time path and link no robot
# or planning library.
add_executable(test_cartesian_contract
    tests/test_cartesian_contract.cpp
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract/WorldCartesianTrajectory.cpp
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract/WorldCartesianTrajectoryWire.cpp
)
target_include_directories(test_cartesian_contract PRIVATE
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract
    ${HUMANSL_ROOT}/third_party/include/eigen3
)
add_test(NAME cartesian_contract COMMAND test_cartesian_contract)

add_executable(test_planning_request_contract
    ${HUMANSL_ROOT}/Christian_control/basic_control/tests/test_planning_request.cpp
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract/PlanningRequest.cpp)
target_include_directories(test_planning_request_contract PRIVATE
    ${HUMANSL_ROOT}/Christian_control/basic_control/src
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract
    ${HUMANSL_ROOT}/third_party/include/eigen3)
add_test(NAME planning_request_contract COMMAND test_planning_request_contract)

add_executable(test_world_trajectory_projection
    tests/test_world_trajectory_projection.cpp
)
target_include_directories(test_world_trajectory_projection PRIVATE
    src
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract
)
target_link_libraries(test_world_trajectory_projection PRIVATE bridge_core)
set_target_properties(test_world_trajectory_projection PROPERTIES
    BUILD_RPATH "${TP_LIB}")
add_test(NAME world_trajectory_projection COMMAND test_world_trajectory_projection
         ${DH_PARAMS_TOOL_YAML} ${DH_PARAMS_FLANGE_YAML})

add_executable(test_planner_model tests/test_planner_model.cpp)
target_include_directories(test_planner_model PRIVATE src)
target_link_libraries(test_planner_model PRIVATE bridge_core)
set_target_properties(test_planner_model PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME planner_model COMMAND test_planner_model
         ${DH_PARAMS_TOOL_YAML} ${DH_PARAMS_FLANGE_YAML})

add_executable(test_gpmp2_urdf_validation tests/test_gpmp2_urdf_validation.cpp)
target_include_directories(test_gpmp2_urdf_validation PRIVATE src)
target_link_libraries(test_gpmp2_urdf_validation PRIVATE bridge_core)
set_target_properties(test_gpmp2_urdf_validation PROPERTIES BUILD_RPATH "${TP_LIB}")
add_dependencies(test_gpmp2_urdf_validation dh_params_flange)
add_test(NAME gpmp2_urdf_validation COMMAND test_gpmp2_urdf_validation
         ${DH_PARAMS_TOOL_YAML} ${DH_PARAMS_FLANGE_YAML})

add_executable(test_sphere_scaling tests/test_sphere_scaling.cpp)
target_include_directories(test_sphere_scaling PRIVATE src)
target_link_libraries(test_sphere_scaling PRIVATE bridge_core)
set_target_properties(test_sphere_scaling PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME sphere_scaling COMMAND test_sphere_scaling
         ${DH_PARAMS_TOOL_YAML})

# The Cartesian path types and circle generator: pure Eigen geometry, no
# gtsam, no Pinocchio, no robot. Kept out of bridge_core's link line for
# that reason — it compiles standalone and runs anywhere.
add_executable(test_cartesian_path
    tests/test_cartesian_path.cpp
    src/CartesianPath.cpp
)
target_include_directories(test_cartesian_path PRIVATE
    src
    ${HUMANSL_ROOT}/Christian_control/basic_control/src
    ${HUMANSL_ROOT}/third_party/include/eigen3
)
add_test(NAME cartesian_path COMMAND test_cartesian_path)

# Trajectory assembly: the free approach and the constrained task phase on
# ONE uniform time grid. Pure Eigen — no gtsam, no Pinocchio, no robot.
add_executable(test_path_assembly
    tests/test_path_assembly.cpp
    src/CartesianPath.cpp
    src/PathAssembly.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_generation/src/OptimisationWaypoint.cpp
)
target_include_directories(test_path_assembly PRIVATE
    src
    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_generation/include
    ${HUMANSL_ROOT}/Christian_control/basic_control/src
    ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(test_path_assembly PRIVATE bridge_core)
set_target_properties(test_path_assembly PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME path_assembly COMMAND test_path_assembly)

# Determinism of the IK's random restarts: same request, same plan.
add_executable(test_ik_determinism tests/test_ik_determinism.cpp)
target_include_directories(test_ik_determinism PRIVATE src)
target_link_libraries(test_ik_determinism PRIVATE bridge_core)
set_target_properties(test_ik_determinism PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME ik_determinism COMMAND test_ik_determinism)

# Inter-arm clearance: the two arms compared at CORRESPONDING times over the
# start-skew window, never as a static swept volume.
add_executable(test_inter_arm_distance tests/test_inter_arm_distance.cpp)
target_include_directories(test_inter_arm_distance PRIVATE src)
target_link_libraries(test_inter_arm_distance PRIVATE bridge_core)
set_target_properties(test_inter_arm_distance PROPERTIES BUILD_RPATH "${TP_LIB}")
add_dependencies(test_inter_arm_distance dh_params_flange)
add_test(NAME inter_arm_distance COMMAND test_inter_arm_distance
         ${DH_PARAMS_TOOL_YAML} ${DH_PARAMS_FLANGE_YAML})


# The Cartesian path types and the frame crossing that consumes them, in
# bridge_core so both the probe tool and (next) PlanSolver share one copy.
# The one place gpmp2's [rotation; translation] pose-error ordering is
# written down, held to gtsam's own logmap rather than to a comment.
add_executable(test_pose_noise_ordering tests/test_pose_noise_ordering.cpp)
target_include_directories(test_pose_noise_ordering PRIVATE src)
target_link_libraries(test_pose_noise_ordering PRIVATE bridge_core)
set_target_properties(test_pose_noise_ordering PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME pose_noise_ordering COMMAND test_pose_noise_ordering)

# probe_path_reachability: OFFLINE reachability report for a Cartesian path.
# Solves IK at every sample; NEVER connects to a robot, so it is safe to run
# with the arms powered off.
add_executable(probe_path_reachability tools/probe_path_reachability.cpp)
target_include_directories(probe_path_reachability PRIVATE src)
target_link_libraries(probe_path_reachability PRIVATE bridge_core)
set_target_properties(probe_path_reachability PROPERTIES BUILD_RPATH "${TP_LIB}")

add_executable(test_world_sdf tests/test_world_sdf.cpp)
target_include_directories(test_world_sdf PRIVATE src)
target_link_libraries(test_world_sdf PRIVATE bridge_core)
set_target_properties(test_world_sdf PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME world_sdf COMMAND test_world_sdf)

add_executable(test_grid_coverage tests/test_grid_coverage.cpp)
target_include_directories(test_grid_coverage PRIVATE src)
target_link_libraries(test_grid_coverage PRIVATE bridge_core)
set_target_properties(test_grid_coverage PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME grid_coverage COMMAND test_grid_coverage
         ${DH_PARAMS_TOOL_YAML}
         ${DH_PARAMS_FLANGE_YAML}
         ${JOINT_LIMITS_YAML})

# The planner's tuning file, read by planner_bridge on every run. Its
# default path is resolved from the executable (config/ beside the build
# tree's parent), never from the working directory, so tests and the real
# binary agree on which file configures a run.
set(PLANNER_CONFIG_YAML "${CMAKE_CURRENT_SOURCE_DIR}/config/planner.yaml")

# End-to-end: plan a circle, emit it, reconstruct with the CONTROLLER's own
# interpolator, and validate the reconstruction. The test that answers
# "can the planner draw a circle and prove it".
add_executable(test_circle_plan tests/test_circle_plan.cpp)
target_include_directories(test_circle_plan PRIVATE src)
target_link_libraries(test_circle_plan PRIVATE bridge_core)
set_target_properties(test_circle_plan PROPERTIES BUILD_RPATH "${TP_LIB}")
add_dependencies(test_circle_plan dh_params_flange)
add_test(NAME circle_plan COMMAND test_circle_plan
         ${DH_PARAMS_FLANGE_YAML}
         ${JOINT_LIMITS_YAML}
         ${PLANNER_CONFIG_YAML})

add_executable(test_planner_config tests/test_planner_config.cpp)
target_include_directories(test_planner_config PRIVATE src)
target_link_libraries(test_planner_config PRIVATE bridge_core)
set_target_properties(test_planner_config PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME planner_config COMMAND test_planner_config
         ${PLANNER_CONFIG_YAML} ${CMAKE_CURRENT_BINARY_DIR})

add_executable(test_plan_solver tests/test_plan_solver.cpp)
target_include_directories(test_plan_solver PRIVATE src)
target_link_libraries(test_plan_solver PRIVATE bridge_core)
set_target_properties(test_plan_solver PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME plan_solver COMMAND test_plan_solver
         ${DH_PARAMS_TOOL_YAML}
         ${JOINT_LIMITS_YAML}
         ${PLANNER_CONFIG_YAML})

add_executable(test_path_validation tests/test_path_validation.cpp)
target_include_directories(test_path_validation PRIVATE
    src ${HUMANSL_ROOT}/Christian_control/basic_control/src)
target_link_libraries(test_path_validation PRIVATE bridge_core)
set_target_properties(test_path_validation PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME path_validation COMMAND test_path_validation
         ${DH_PARAMS_TOOL_YAML})

add_executable(test_start_state tests/test_start_state.cpp)
target_include_directories(test_start_state PRIVATE src)
target_link_libraries(test_start_state PRIVATE bridge_core)
set_target_properties(test_start_state PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME start_state COMMAND test_start_state)

add_executable(test_bridge_main tests/test_bridge_main.cpp)
target_sources(test_bridge_main PRIVATE
    src/PlannerPreview.cpp
    ${HUMANSL_ROOT}/Christian_control/cartesian_contract/WorldCartesianTrajectoryWire.cpp)
target_include_directories(test_bridge_main PRIVATE
    src ${HUMANSL_ROOT}/Christian_control/basic_control/src)
target_link_libraries(test_bridge_main PRIVATE bridge_core)
set_target_properties(test_bridge_main PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME bridge_main COMMAND test_bridge_main
         ${DH_PARAMS_TOOL_YAML}
         ${JOINT_LIMITS_YAML}
         ${DH_PARAMS_FLANGE_YAML})

# Typed planner result characterization. The test is the first consumer of
# the in-process planner API; its initial red build is intentional during the
# migration.
add_executable(test_planner_runtime tests/test_planner_runtime.cpp)
target_include_directories(test_planner_runtime PRIVATE
    src ${HUMANSL_ROOT}/Christian_control/cartesian_contract)
target_link_libraries(test_planner_runtime PRIVATE bridge_core)
set_target_properties(test_planner_runtime PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME planner_runtime COMMAND test_planner_runtime
         ${PLANNER_CONFIG_YAML}
         ${JOINT_LIMITS_YAML}
         ${DH_PARAMS_TOOL_YAML}
         ${DH_PARAMS_FLANGE_YAML})

# Every consumer of the generated YAML depends on the generating target, so
# single-target builds (`make test_plan_solver`) produce the file before
# anything runs against it. (`ALL` on dh_params_tool covers bare `ctest`
# after a full build — ctest itself never builds.)
foreach(dh_consumer planner_bridge probe_path_reachability test_planner_model test_gpmp2_urdf_validation test_sphere_scaling test_grid_coverage
        test_plan_solver test_path_validation test_bridge_main test_planner_runtime
        test_world_trajectory_projection)
    add_dependencies(${dh_consumer} dh_params_tool)
endforeach()

# planner_bridge and its CLI-argument test also need the left/flange YAML —
# BridgeMain.cpp picks between the two files by --arm, so both must exist
# before either binary runs.
foreach(dh_flange_consumer planner_bridge test_bridge_main test_grid_coverage test_planner_model test_world_trajectory_projection)
    add_dependencies(${dh_flange_consumer} dh_params_flange)
endforeach()

# libgtsam.so/libgpmp2.so carry no RUNPATH of their own to their sibling
# libs (libmetis-gtsam.so, libcephes-gtsam.so.1, ...), so RUNPATH on our
# binaries alone doesn't resolve transitive deps under ctest's fork/exec —
# a pre-existing gap in the third_party build, worked around once here for
# every bridge_core-linked test rather than per add_test call.
foreach(bridge_test_name pose_noise_ordering ik_determinism inter_arm_distance circle_plan planner_model world_sdf plan_solver path_validation start_state bridge_main gpmp2_urdf_validation sphere_scaling grid_coverage planner_config path_assembly world_trajectory_projection)
    set_tests_properties(${bridge_test_name} PROPERTIES
        ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")
endforeach()

endif () # PLANNER_BRIDGE_TESTS
