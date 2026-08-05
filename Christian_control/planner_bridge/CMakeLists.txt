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
