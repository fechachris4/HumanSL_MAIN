# Standalone build for basic_control — links ONLY the bundled Kortex API.
# No GTSAM / GPMP2 / Pinocchio / Vicon required.
#
#   cd Christian_control/basic_control && mkdir build && cd build
#   cmake .. && make
#   ./basic_control state
cmake_minimum_required(VERSION 3.12)
project(basic_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_definitions(-D_OS_UNIX -w)

# Bundled Kortex API lives two levels up, in HumanSL_MAIN/third_party/.
set(KORTEX_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/kortex_api")

include_directories(
  ${KORTEX_DIR}/include
  ${KORTEX_DIR}/include/client
  ${KORTEX_DIR}/include/common
  ${KORTEX_DIR}/include/messages
  ${KORTEX_DIR}/include/client_stubs
)

add_executable(basic_control basic_control.cpp)

target_link_libraries(basic_control
  ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
  pthread
)
