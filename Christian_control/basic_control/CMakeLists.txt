# Standalone build for controller — links the bundled Kortex API and Pinocchio.
# No GTSAM / GPMP2 / Vicon required.
#
#   cd Christian_control/basic_control && mkdir build && cd build
#   cmake .. && make
#   ./controller
cmake_minimum_required(VERSION 3.12)
project(basic_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
add_definitions(-D_OS_UNIX -w)

# Bundled Kortex API lives two levels up, in HumanSL_MAIN/third_party/.
set(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(KORTEX_DIR "${HUMANSL_ROOT}/third_party/kortex_api")

include_directories(
        ${KORTEX_DIR}/include
        ${KORTEX_DIR}/include/client
        ${KORTEX_DIR}/include/common
        ${KORTEX_DIR}/include/messages
        ${KORTEX_DIR}/include/client_stubs
)

# --- controller: Kortex + bundled Pinocchio (dynamics/kinematics via the
#     Dynamics class reused from TrajectoryExecution) ---
add_executable(controller
        src/main.cpp
        src/Measure.cpp
        src/Kinematics.cpp
        src/Controller.cpp
        src/Record.cpp
        src/Motion.cpp
        src/Timing.cpp
        src/Connect.cpp
        ${HUMANSL_ROOT}/TrajectoryExecution/src/Dynamics.cpp
)

target_include_directories(controller PRIVATE
        ${HUMANSL_ROOT}/third_party/include
        ${HUMANSL_ROOT}/third_party/include/eigen3
        ${HUMANSL_ROOT}/TrajectoryExecution/include
)

target_link_libraries(controller
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_default.so
        ${HUMANSL_ROOT}/third_party/lib/libpinocchio_parsers.so
        pthread
)

# Let the executable find the bundled .so files at runtime.
set_target_properties(controller PROPERTIES
        BUILD_RPATH "${HUMANSL_ROOT}/third_party/lib"
)

# libpinocchio_parsers.so references collision symbols (coal) that are not
# bundled and that we never call (we only build the model, no collision
# geometry). Allow them to stay unresolved, and bind lazily so the loader
# doesn't look them up at startup either.
target_link_options(controller PRIVATE
        -Wl,--allow-shlib-undefined
        -Wl,-z,lazy
)

# --- query_limits: READ-ONLY helper that prints the robot's kinematic
#     hard/soft limits (no Pinocchio, no motion code) ---
add_executable(query_limits
        tools/query_limits.cpp
        src/Connect.cpp
)
# query_limits.cpp lives in tools/ but includes headers from src/.
target_include_directories(query_limits PRIVATE src)
target_link_libraries(query_limits
        ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
        pthread
)

# The arm's URDF model, used by Pinocchio. Our own copy (duplicated from
# ../../config/GEN3_With_GRIPPER_DYNAMICS.urdf) so joint limits are ours to
# edit: velocity limits set to Kinova's general limits (1.39 rad/s joints
# 1-4, 1.22 rad/s joints 5-7) instead of the original 0.8727 (admittance).
set(GEN3_URDF "${CMAKE_CURRENT_SOURCE_DIR}/config/GEN3_custom.urdf")
target_compile_definitions(controller PRIVATE GEN3_URDF_PATH="${GEN3_URDF}")
