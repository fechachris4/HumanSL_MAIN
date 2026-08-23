# humansl_contracts — the typed exchange between subsystems.
#
# Three value types and nothing else: the request the controller sends when it
# wants a new plan, the world-Cartesian trajectory the planner returns, and its
# offline text formatter. Eigen only — no Kortex, no Pinocchio, no GTSAM/GPMP2,
# no Vicon SDK and no I/O.
#
# A file belongs here only if it names data that two subsystems must agree on.
# Nothing that merely happens to be shared qualifies: mechanisms (the slots and
# mailboxes that carry these types across threads) stay with the subsystem that
# owns the thread.
#
# This directory has no executable and is never built on its own. Consumers add
# it with add_subdirectory and link humansl_contracts, which is what stopped
# these three sources being compiled separately into both the controller's
# execution core and the planner's library.
cmake_minimum_required(VERSION 3.12)

get_filename_component(HUMANSL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_library(humansl_contracts STATIC
        GoalCommand.cpp
        PlanningRequest.cpp
        WorldCartesianTrajectory.cpp
        WorldCartesianTrajectoryWire.cpp
)

target_include_directories(humansl_contracts PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# The bundled Eigen, as a SYSTEM include so -Wall stays quiet inside it.
target_include_directories(humansl_contracts SYSTEM PUBLIC
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
