# Standalone Vicon project: the ViconInterface wrapper ported from the
# legacy ViconDataStream/ tree, built against the vendored DataStream SDK
# in third_party/vicon_api. No Kortex, no robot — sensing only.
#
#   cmake -S . -B build && cmake --build build
#   ./build/connect_vicon [host:port]

cmake_minimum_required(VERSION 3.12)
project(vicon CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
add_compile_options(-Wall -Wextra)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The bundled Eigen, as a SYSTEM include so -Wall stays quiet inside it.
# Mirrors basic_control/CMakeLists.txt's add_bundled_eigen_to.
function(add_bundled_eigen_to target)
    target_include_directories(${target} SYSTEM PRIVATE
            ${HUMANSL_ROOT}/third_party/include/eigen3
    )
endfunction()

# Repository root, two levels up: Christian_control/vicon -> HumanSL_MAIN.
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

add_executable(connect_vicon tools/connect_vicon.cpp)
target_link_libraries(connect_vicon PRIVATE vicon_interface)
set_target_properties(connect_vicon PROPERTIES
        BUILD_RPATH "${VICON_SDK}/lib"
)

include(CTest)
if (BUILD_TESTING)
    add_executable(test_segment_data tests/test_segment_data.cpp)
    target_link_libraries(test_segment_data PRIVATE vicon_interface)
    add_test(NAME vicon_segment_data COMMAND test_segment_data)

    add_executable(test_vicon_snapshot tests/test_vicon_snapshot.cpp)
    target_include_directories(test_vicon_snapshot PRIVATE src)
    add_bundled_eigen_to(test_vicon_snapshot)
    add_test(NAME vicon_snapshot COMMAND test_vicon_snapshot)
endif ()
