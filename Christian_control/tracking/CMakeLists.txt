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

# The bundled Eigen, as a SYSTEM include so -Wall stays quiet inside it.
# Mirrors control/CMakeLists.txt's add_bundled_eigen_to.
function(add_bundled_eigen_to target)
    target_include_directories(${target} SYSTEM PRIVATE
            ${HUMANSL_ROOT}/third_party/include/eigen3
    )
endfunction()

# Test binaries must catch real assertion failures regardless of
# CMAKE_BUILD_TYPE (this project defaults to Release, whose -DNDEBUG
# strips assert() to a no-op). -UNDEBUG re-enables it for tests only;
# production/tool targets (vicon_interface, connect_vicon, etc.) keep
# their release optimisation untouched.
function(enable_assertions_for target)
    target_compile_options(${target} PRIVATE -UNDEBUG)
endfunction()

# One helper for every test target so enable_assertions_for can never be
# forgotten on a new one (it was, once, earlier in this project, and had
# to be fixed retroactively). test_name is the ctest test name (kept
# distinct from the executable target name, matching this project's
# existing convention); extra arguments after `source` are libraries to
# link PRIVATE, omitted for a test with no library dependency.
function(add_vicon_test test_name target source)
    add_executable(${target} ${source})
    if (ARGN)
        target_link_libraries(${target} PRIVATE ${ARGN})
    endif ()
    enable_assertions_for(${target})
    add_test(NAME ${test_name} COMMAND ${target})
endfunction()

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

include(CTest)
if (BUILD_TESTING)
    add_vicon_test(vicon_segment_data test_segment_data
            tests/test_segment_data.cpp vicon_interface)

    # No library to link -- it only needs the bundled Eigen headers and src/
    # for the snapshot struct definitions, so the shared helper is called
    # with no trailing libraries and the include lines stay alongside it.
    add_vicon_test(vicon_snapshot test_vicon_snapshot
            tests/test_vicon_snapshot.cpp)
    target_include_directories(test_vicon_snapshot PRIVATE src)
    add_bundled_eigen_to(test_vicon_snapshot)

    add_vicon_test(vicon_snapshot_builder test_snapshot_builder
            tests/test_snapshot_builder.cpp vicon_snapshot)

    add_vicon_test(vicon_recorder test_vicon_recorder
            tests/test_vicon_recorder.cpp vicon_snapshot)

    add_vicon_test(vicon_replay test_vicon_replay
            tests/test_vicon_replay.cpp vicon_snapshot)

    add_vicon_test(vicon_mount_twist test_mount_twist_estimator
            tests/test_mount_twist_estimator.cpp vicon_snapshot)
endif ()
