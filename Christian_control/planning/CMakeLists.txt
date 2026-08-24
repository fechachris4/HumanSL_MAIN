# planning — a planning request in, a validated world trajectory out.
# optimisation/ is the GPMP2 layer (this project's own copy of the inherited
# pipeline; the upstream TrajectoryGeneration tree is frozen).
#
# The library is humansl_planning. It depends on the arm's model and
# kinematics (humansl_robot_model, from ../control) and the exchange types
# (humansl_contracts) — never on controller, actuation or mailbox machinery.
# GTSAM/GPMP2/yaml-cpp are implementation details and stay PRIVATE.
cmake_minimum_required(VERSION 3.16)
project(planning CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(TP_LIB "${HUMANSL_ROOT}/third_party/lib")

# Every target here runs against the vendored third_party .so files; the
# variable seeds each target's BUILD_RPATH at creation.
set(CMAKE_BUILD_RPATH "${TP_LIB}")

# The planner/controller exchange types, as a linkable target. Guarded because
# a consuming project (../runtime, or ../control below) may have already
# defined the target.
if (NOT TARGET humansl_contracts)
    add_subdirectory("${HUMANSL_ROOT}/Christian_control/contracts"
                     "${CMAKE_CURRENT_BINARY_DIR}/contracts")
endif ()

# The arm's model and kinematics. GPMP2 optimises; it is not given a robot
# model of its own, so RobotModel/Kinematics are linked from ../control's
# humansl_robot_model rather than compiled a second time here. EXCLUDE_FROM_ALL
# keeps control's tools, tests and execution core out of this build; only the
# model target and its generated header are built. Guarded: the controller's
# nested build adds ../control before this project.
if (NOT TARGET humansl_robot_model)
    add_subdirectory("${HUMANSL_ROOT}/Christian_control/control"
                     "${CMAKE_CURRENT_BINARY_DIR}/control" EXCLUDE_FROM_ALL)
endif ()

# The one canonical URDF (model/GEN3_dual_mounted.urdf). Targets receive the
# GEN3_DUAL_URDF_PATH definition from humansl_robot_model; this variable only
# feeds the DH-generation dependencies below.
set(GEN3_DUAL_URDF
    "${HUMANSL_ROOT}/Christian_control/model/GEN3_dual_mounted.urdf")

# --- humansl_planning: the planner. GPMP2 factor-graph optimisation
#     (optimisation/), and the planner implementation (src/): request
#     framing, path assembly and IK, the mount SDF, solving, validation and
#     the world projection, plus the bridge entry logic and preview used by
#     the standalone executable and the controller's in-process worker. ---
add_library(humansl_planning STATIC
    optimisation/utils.cpp
    optimisation/GenerateArmModel.cpp
    optimisation/TrajectoryInitiation.cpp
    optimisation/TrajectoryOptimization.cpp
    optimisation/OptimisationWaypoint.cpp
    optimisation/PinocchioKinematicsAdapter.cpp
    src/PlannerModel.cpp
    src/MountSdf.cpp
    src/CartesianPath.cpp
    src/PathFrames.cpp
    src/PathIk.cpp
    src/InterArmDistance.cpp
    src/PathAssembly.cpp
    src/PlanValidationReport.cpp
    src/ValidatePlan.cpp
    src/WorldTrajectoryProjection.cpp
    src/PlannerConfig.cpp
    src/PlanSolver.cpp
    src/SceneBlockerSummary.cpp
    src/PlanDebugDump.cpp
    src/StartState.cpp
    src/BridgeMain.cpp
    src/PlannerPreview.cpp
    src/PlannerRuntime.cpp
)
add_humansl_warnings_to(humansl_planning)
# Consumers include planner headers directly; the GTSAM/GPMP2 headers those
# expose come through humansl_robot_model's PUBLIC third_party include.
target_include_directories(humansl_planning PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/optimisation
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
# PUBLIC: the model/kinematics and the exchange types appear in the planner's
# own interface. PRIVATE: the solver stack is an implementation detail — a
# consumer links humansl_planning, not GTSAM (CMake still carries these onto
# the final link line for the static archive).
target_link_libraries(humansl_planning
    PUBLIC
        humansl_robot_model
        humansl_contracts
    PRIVATE
        ${TP_LIB}/libgpmp2.so
        ${TP_LIB}/libgtsam.so
        ${TP_LIB}/libmetis-gtsam.so
        yaml-cpp
        pthread
)
# DT_RUNPATH (the linker's new-dtags default) is non-transitive: it lets an
# executable find libgtsam.so, but libgtsam.so has no RPATH of its own, so
# its dependency libmetis-gtsam.so never resolves when the binary is run
# bare. DT_RPATH is transitive; INTERFACE, so every executable that links the
# planner inherits the fix. (--allow-shlib-undefined / -z lazy arrive the
# same way from humansl_robot_model.)
target_link_options(humansl_planning INTERFACE -Wl,--disable-new-dtags)

add_executable(planner_bridge src/main.cpp)
add_humansl_warnings_to(planner_bridge)
target_link_libraries(planner_bridge PRIVATE humansl_planning)

# The DH parameter YAML gpmp2 consumes is GENERATED from the URDF at build
# time — never committed, never hand-edited. generate_dh_params derives the
# link lengths from Pinocchio FK and self-verifies before writing; a failed
# derivation fails the build. Listing the URDF in DEPENDS means editing it
# regenerates the YAML on the next build. The LD_LIBRARY_PATH wrapper
# mirrors the ctest workaround below (third_party .so files lack RUNPATHs
# to their own siblings).
add_executable(generate_dh_params tools/generate_dh_params.cpp)
add_humansl_warnings_to(generate_dh_params)
target_link_libraries(generate_dh_params PRIVATE humansl_planning)

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
# add_planning_test defines one hardware-free test executable against the
# production planner library; extra arguments become the test command's.
# ---------------------------------------------------------------------------
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
enable_testing()

function(add_planning_test name)
    add_executable(test_${name} tests/test_${name}.cpp)
    add_humansl_warnings_to(test_${name})
    target_link_libraries(test_${name} PRIVATE humansl_planning)
    add_test(NAME ${name} COMMAND test_${name} ${ARGN})
endfunction()

add_planning_test(path_frames_to_mount)
add_planning_test(mount_sdf ${DH_PARAMS_TOOL_YAML} ${DH_PARAMS_FLANGE_YAML})
add_planning_test(scene_config)
add_planning_test(bridge_args)
add_planning_test(terminal_posture)
add_planning_test(traced_circle ${DH_PARAMS_FLANGE_YAML})
add_planning_test(projection_exit ${DH_PARAMS_TOOL_YAML} ${DH_PARAMS_FLANGE_YAML})
add_planning_test(planner_start_state ${DH_PARAMS_TOOL_YAML} ${DH_PARAMS_FLANGE_YAML})
add_planning_test(obstacle_aware_planner ${DH_PARAMS_TOOL_YAML})

# Dynamic-limit evidence: a rejected-for-dynamics report names the peak's
# joint and time, so a diverging duration repair is diagnosable from the
# plan summary alone (2026-08-24 traced-circle runaway).
add_planning_test(dynamics_peak ${DH_PARAMS_FLANGE_YAML})

# The joint-type contract: continuous joints 1/3/5/7 carry no position
# limit anywhere in the planner, bounded joints 2/4/6 do. Checked against
# the generated kBoundedMask, so the validator cannot drift from
# joint_limits.yaml and the URDF.
add_planning_test(joint_type_contract)

endif ()

# Offline reachability probe for traced paths. Hardware-free.
add_executable(probe_path_reachability tools/probe_path_reachability.cpp)
add_humansl_warnings_to(probe_path_reachability)
target_link_libraries(probe_path_reachability PRIVATE humansl_planning)
