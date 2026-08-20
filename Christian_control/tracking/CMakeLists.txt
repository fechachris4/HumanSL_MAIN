# tracking — external body tracking: markers in, a validated world pose and
# twist out. The ViconInterface wrapper was ported from the
# legacy ViconDataStream/ tree, built against the vendored DataStream SDK
# in third_party/vicon_api. No Kortex, no robot — sensing only.
#
#   cmake -S . -B build && cmake --build build
#   ./build/connect_vicon [host:port]

cmake_minimum_required(VERSION 3.12)
project(tracking CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
add_compile_options(-Wall -Wextra)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Repository root, two levels up: Christian_control/tracking -> HumanSL_MAIN.
get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(VICON_SDK "${HUMANSL_ROOT}/third_party/vicon_api")

add_library(vicon_interface STATIC
        src/ViconInterface.cpp
)
target_include_directories(vicon_interface PUBLIC src)
# SYSTEM keeps -Wall quiet inside the vendor header.
target_include_directories(vicon_interface SYSTEM PUBLIC ${VICON_SDK}/include)
target_link_libraries(vicon_interface PUBLIC
        ${VICON_SDK}/lib/libViconDataStreamSDK_CPP.so
)
# The SDK .so pulls in the boost .so files sitting next to it; the RPATH
# lets the loader find them without LD_LIBRARY_PATH.
set_target_properties(vicon_interface PROPERTIES
        BUILD_RPATH "${VICON_SDK}/lib"
)

add_library(vicon_snapshot STATIC
        src/MountTwistEstimator.cpp
        src/SnapshotBuilder.cpp
        src/ViconRecorder.cpp
        src/ViconReplaySource.cpp
)
target_include_directories(vicon_snapshot PUBLIC src)
target_include_directories(vicon_snapshot SYSTEM PUBLIC
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(vicon_snapshot PUBLIC vicon_interface)

add_executable(connect_vicon tools/connect_vicon.cpp)
target_link_libraries(connect_vicon PRIVATE vicon_interface)
set_target_properties(connect_vicon PROPERTIES
        BUILD_RPATH "${VICON_SDK}/lib"
)

add_executable(record_vicon tools/record_vicon.cpp)
target_link_libraries(record_vicon PRIVATE vicon_snapshot vicon_interface)
set_target_properties(record_vicon PROPERTIES
        BUILD_RPATH "${VICON_SDK}/lib"
)

add_executable(track_segment tools/track_segment.cpp)
target_link_libraries(track_segment PRIVATE vicon_snapshot vicon_interface)
set_target_properties(track_segment PROPERTIES
        BUILD_RPATH "${VICON_SDK}/lib"
)
