# tracking — external body tracking: markers in, a validated world pose and
# twist out. The ViconInterface wrapper was ported from the
# legacy ViconDataStream/ tree, built against the vendored DataStream SDK
# in third_party/vicon_api. No Kortex, no robot — sensing only.
#
# Two libraries, split on the SDK boundary:
#   humansl_tracking_core — snapshot maths and file replay. SDK-free: builds
#       and links on a machine with no DataStream SDK at all.
#   humansl_vicon — the live adapter. The ONLY tracking target that touches
#       the SDK: ViconInterface wraps the client, and SnapshotBuilder sits
#       here too because its header consumes ViconInterface's SDK frame
#       types. Defined only when the vendored SDK is present.
# Consumers (../runtime) link these targets; nothing outside this directory
# compiles a tracking source file.
#
#   cmake -S . -B build && cmake --build build
#   ./build/connect_vicon [host:port]

cmake_minimum_required(VERSION 3.16)
project(tracking CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Repository root, two levels up: Christian_control/tracking -> HumanSL_MAIN.
get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(VICON_SDK "${HUMANSL_ROOT}/third_party/vicon_api")

# --- humansl_tracking_core: everything that works on a ViconSnapshot after
#     acquisition — twist estimation, CSV recording, file replay. Eigen only. ---
add_library(humansl_tracking_core STATIC
        src/MountTwistEstimator.cpp
        src/ViconRecorder.cpp
        src/ViconReplaySource.cpp
)
target_compile_options(humansl_tracking_core PRIVATE -Wall -Wextra)
target_include_directories(humansl_tracking_core PUBLIC src)
target_include_directories(humansl_tracking_core SYSTEM PUBLIC
        ${HUMANSL_ROOT}/third_party/include/eigen3
)

# --- humansl_vicon: the live SDK adapter. ---
if (EXISTS "${VICON_SDK}/lib/libViconDataStreamSDK_CPP.so")
    add_library(humansl_vicon STATIC
            src/ViconInterface.cpp
            src/SnapshotBuilder.cpp
    )
    target_compile_options(humansl_vicon PRIVATE -Wall -Wextra)
    # SYSTEM keeps -Wall quiet inside the vendor header.
    target_include_directories(humansl_vicon SYSTEM PUBLIC ${VICON_SDK}/include)
    target_link_libraries(humansl_vicon PUBLIC
            humansl_tracking_core
            ${VICON_SDK}/lib/libViconDataStreamSDK_CPP.so
    )
    # The SDK .so pulls in the boost .so files sitting next to it; the RPATH
    # lets the loader find them without LD_LIBRARY_PATH. BUILD_RPATH is not a
    # usage requirement, so executables set their own below.
    set_target_properties(humansl_vicon PROPERTIES
            BUILD_RPATH "${VICON_SDK}/lib"
    )

    add_executable(connect_vicon tools/connect_vicon.cpp)
    target_compile_options(connect_vicon PRIVATE -Wall -Wextra)
    target_link_libraries(connect_vicon PRIVATE humansl_vicon)
    set_target_properties(connect_vicon PROPERTIES
            BUILD_RPATH "${VICON_SDK}/lib"
    )

    add_executable(record_vicon tools/record_vicon.cpp)
    target_compile_options(record_vicon PRIVATE -Wall -Wextra)
    target_link_libraries(record_vicon PRIVATE humansl_vicon)
    set_target_properties(record_vicon PROPERTIES
            BUILD_RPATH "${VICON_SDK}/lib"
    )

    add_executable(track_segment tools/track_segment.cpp)
    target_compile_options(track_segment PRIVATE -Wall -Wextra)
    target_link_libraries(track_segment PRIVATE humansl_vicon)
    set_target_properties(track_segment PROPERTIES
            BUILD_RPATH "${VICON_SDK}/lib"
    )
else ()
    message(STATUS "tracking: DataStream SDK absent — humansl_vicon and the live tools are not defined")
endif ()
