# TrajectoryExecution Library — Dynamics (the Pinocchio/URDF model loader,
# also compiled directly by basic_control and planner_bridge) plus the
# Filter/Connect support the legacy root build links. The hand-rolled DH
# kinematics that used to live here was deleted when all kinematics moved
# to the Pinocchio/URDF path.
add_library(TrajectoryExecution
    src/Dynamics.cpp
    src/Filter.cpp
        ../Christian_control/basic_control/src/Connect.cpp
        ../Christian_control/basic_control/src/Connect.h
)

# Make headers available to other targets
target_include_directories(TrajectoryExecution PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../ViconDataStream/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../TrajectoryGeneration/include
)

target_link_libraries(TrajectoryExecution 
    ViconDataStream
    TrajectoryGeneration
)