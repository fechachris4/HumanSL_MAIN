# Read-only link-structure gate for the in-process planner handoff.
cmake_policy(SET CMP0057 NEW)

foreach (required CONTROLLER_BINARY CORE_ARCHIVE NM_EXECUTABLE)
    if (NOT DEFINED ${required} OR "${${required}}" STREQUAL "")
        message(FATAL_ERROR
            "check_controller_planner_linkage: ${required} not set")
    endif ()
endforeach ()

foreach (artifact "${CONTROLLER_BINARY}" "${CORE_ARCHIVE}")
    if (NOT EXISTS "${artifact}")
        message(FATAL_ERROR
            "planner handoff linkage artifact not found: ${artifact}")
    endif ()
endforeach ()

execute_process(
    COMMAND "${NM_EXECUTABLE}" --defined-only -C "${CONTROLLER_BINARY}"
    OUTPUT_VARIABLE controller_defined
    RESULT_VARIABLE nm_result
    ERROR_VARIABLE nm_error)
if (NOT nm_result EQUAL 0)
    message(FATAL_ERROR "nm failed on controller: ${nm_error}")
endif ()
if (NOT controller_defined MATCHES "RunInProcessPlanner")
    message(FATAL_ERROR
        "controller does not define RunInProcessPlanner; the in-process "
        "planner worker is not linked")
endif ()
if (NOT controller_defined MATCHES "SolveWorldTrajectoryForRequest")
    message(FATAL_ERROR
        "controller does not define SolveWorldTrajectoryForRequest; the "
        "typed planner runtime is not linked")
endif ()

# The shared cyclic execution archive must remain independent of the planner
# stack. GPMP2/GTSAM symbols in its undefined set would put planning libraries
# into the hardware-independent 500 Hz core.
execute_process(
    COMMAND "${NM_EXECUTABLE}" -u -C "${CORE_ARCHIVE}"
    OUTPUT_VARIABLE core_undefined
    RESULT_VARIABLE nm_result
    ERROR_VARIABLE nm_error)
if (NOT nm_result EQUAL 0)
    message(FATAL_ERROR "nm failed on execution core: ${nm_error}")
endif ()
if (core_undefined MATCHES "gtsam::|gpmp2::")
    message(FATAL_ERROR
        "humansl_execution_core needs GPMP2/GTSAM symbols; planner work "
        "must remain outside the execution core:\n${core_undefined}")
endif ()

message(STATUS "controller/planner typed link structure OK")
