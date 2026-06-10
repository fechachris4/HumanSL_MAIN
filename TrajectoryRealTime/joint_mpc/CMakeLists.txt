# Standalone build for the joint-control MPC seam (Milestone 1).
# Eigen-only + pthread — NO GTSAM / GPMP2 / Kortex / yaml-cpp. Builds in seconds,
# runs on a laptop with no robot.
#
#   cd TrajectoryRealTime/joint_mpc
#   cmake -S . -B build && cmake --build build
#   ./build/test_joint_control
cmake_minimum_required(VERSION 3.12)
project(joint_mpc CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Bundled Eigen lives at repo_root/third_party/include/eigen3 (two levels up).
set(EIGEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/include/eigen3")

include_directories(
  ${EIGEN_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_package(Threads REQUIRED)

add_executable(test_joint_control test/test_joint_control.cpp)
target_link_libraries(test_joint_control Threads::Threads)
