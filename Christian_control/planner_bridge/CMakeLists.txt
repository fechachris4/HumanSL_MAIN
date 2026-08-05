cmake_minimum_required(VERSION 3.16)
project(planner_bridge CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(TP_LIB "${HUMANSL_ROOT}/third_party/lib")

# TrajectoryGeneration sources compiled directly — this project must not
# depend on the (currently broken) root build. utils.cpp pulls
# TrajectoryExecution's Jacobian, hence Jacobian.cpp and Pinocchio below.
set(TG_SOURCES
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/utils.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/GenerateArmModel.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/TrajectoryInitiation.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/TrajectoryOptimization.cpp
    ${HUMANSL_ROOT}/TrajectoryGeneration/src/GenerateTrajectory.cpp
    ${HUMANSL_ROOT}/TrajectoryExecution/src/Jacobian.cpp
)

add_library(bridge_core STATIC ${TG_SOURCES})
target_include_directories(bridge_core PUBLIC
    ${HUMANSL_ROOT}/TrajectoryGeneration/include
    ${HUMANSL_ROOT}/TrajectoryExecution/include
    ${HUMANSL_ROOT}/third_party/include
    ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(bridge_core PUBLIC
    ${TP_LIB}/libgpmp2.so
    ${TP_LIB}/libgtsam.so
    ${TP_LIB}/libmetis-gtsam.so
    ${TP_LIB}/libpinocchio_default.so
    yaml-cpp
    pthread
)
set_target_properties(bridge_core PROPERTIES BUILD_RPATH "${TP_LIB}")

add_executable(planner_bridge src/main.cpp)
target_link_libraries(planner_bridge PRIVATE bridge_core)
set_target_properties(planner_bridge PROPERTIES BUILD_RPATH "${TP_LIB}")

enable_testing()

target_sources(bridge_core PRIVATE src/PlannerModel.cpp)
add_executable(test_planner_model
    tests/test_planner_model.cpp
    ${HUMANSL_ROOT}/Christian_control/basic_control/tools/AnalyticalKinematics.cpp
)
target_include_directories(test_planner_model PRIVATE
    src ${HUMANSL_ROOT}/Christian_control/basic_control/tools)
target_link_libraries(test_planner_model PRIVATE bridge_core)
set_target_properties(test_planner_model PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME planner_model COMMAND test_planner_model
         ${CMAKE_CURRENT_SOURCE_DIR}/config/dh_params_tool.yaml)
# libgtsam.so/libgpmp2.so carry no RUNPATH of their own to their sibling
# libs (libmetis-gtsam.so, libcephes-gtsam.so.1, ...), so RUNPATH on our
# binary alone doesn't resolve transitive deps under ctest's fork/exec.
# Not related to this task's DH/flip logic — a pre-existing gap in the
# third_party build.
set_tests_properties(planner_model PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")

target_sources(bridge_core PRIVATE src/WorldSdf.cpp)
add_executable(test_world_sdf tests/test_world_sdf.cpp)
target_include_directories(test_world_sdf PRIVATE src)
target_link_libraries(test_world_sdf PRIVATE bridge_core)
set_target_properties(test_world_sdf PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME world_sdf COMMAND test_world_sdf)
set_tests_properties(world_sdf PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")

target_sources(bridge_core PRIVATE src/PlanSolver.cpp)
add_executable(test_plan_solver tests/test_plan_solver.cpp)
target_include_directories(test_plan_solver PRIVATE src)
target_link_libraries(test_plan_solver PRIVATE bridge_core)
set_target_properties(test_plan_solver PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME plan_solver COMMAND test_plan_solver
         ${CMAKE_CURRENT_SOURCE_DIR}/config/dh_params_tool.yaml
         ${HUMANSL_ROOT}/TrajectoryGeneration/config/joint_limits.yaml)
set_tests_properties(plan_solver PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")

target_sources(bridge_core PRIVATE src/Waypoints.cpp)
add_executable(test_waypoints
    tests/test_waypoints.cpp
    ${HUMANSL_ROOT}/Christian_control/basic_control/src/Targets.cpp
)
target_include_directories(test_waypoints PRIVATE
    src ${HUMANSL_ROOT}/Christian_control/basic_control/src)
target_link_libraries(test_waypoints PRIVATE bridge_core)
set_target_properties(test_waypoints PROPERTIES BUILD_RPATH "${TP_LIB}")
add_test(NAME waypoints COMMAND test_waypoints
         ${CMAKE_CURRENT_SOURCE_DIR}/config/dh_params_tool.yaml)
set_tests_properties(waypoints PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=${TP_LIB}")
