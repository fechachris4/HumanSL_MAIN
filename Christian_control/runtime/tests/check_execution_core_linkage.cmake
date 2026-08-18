# Link-structure gate for the execution-core library split (Plan 01 Task 4).
#
# What this enforces, mechanically rather than by memory
# (docs/engineering/robotics-analysis-workflow.md, "Mechanical enforcement"):
#
#   1. The core archive libhumansl_execution_core.a exists.
#   2. Its undefined symbols contain NO Kinova::Api and NO Vicon DataStream
#      SDK symbols — the hardware-independent core must be linkable with
#      only Eigen/Pinocchio/pthread, so a Kortex or Vicon dependency that
#      sneaks into a core source is a test failure, not a code review hope.
#   3. Both the hardware `controller` executable and the hardware-free
#      probe (`test_execution_core`) link the SAME library target: their
#      LINK_LIBRARIES lists, captured at configure time, name
#      humansl_execution_core, and each built binary DEFINES an
#      ArmExecutionCore symbol — proof the archive member was actually
#      pulled in, not merely mentioned.
#
# Run by CTest as:
#   cmake -DCORE_ARCHIVE=... -DCONTROLLER_BINARY=... -DPROBE_BINARY=...
#         -DCONTROLLER_LINKS=... -DPROBE_LINKS=... -DNM_EXECUTABLE=...
#         -P check_execution_core_linkage.cmake
#
# This script only READS build artifacts. It never runs any binary — the
# controller commands a physical arm and must not be executed by tests.

# Script mode (cmake -P) starts with every policy unset; IN_LIST needs
# CMP0057.
cmake_policy(SET CMP0057 NEW)

foreach (required CORE_ARCHIVE CONTROLLER_BINARY PROBE_BINARY NM_EXECUTABLE)
    if (NOT DEFINED ${required} OR "${${required}}" STREQUAL "")
        message(FATAL_ERROR "check_execution_core_linkage: ${required} not set")
    endif ()
endforeach ()

# --- 1. The archive must exist. -------------------------------------------
if (NOT EXISTS "${CORE_ARCHIVE}")
    message(FATAL_ERROR
        "humansl_execution_core archive not found at ${CORE_ARCHIVE} — the "
        "hardware-independent execution-core library was not built")
endif ()

# --- 2. No hardware SDK symbols may be needed by the core. -----------------
# nm -u lists undefined (needed-from-outside) symbols; -C demangles so the
# namespaces are matched by name. Kinova::Api is the Kortex API namespace;
# ViconDataStreamSDK is the Vicon SDK namespace (the SDK-free ViconSnapshot
# value type carries no such symbols).
execute_process(
    COMMAND "${NM_EXECUTABLE}" -u -C "${CORE_ARCHIVE}"
    OUTPUT_VARIABLE core_undefined
    RESULT_VARIABLE nm_result
    ERROR_VARIABLE nm_error)
if (NOT nm_result EQUAL 0)
    message(FATAL_ERROR
        "nm failed on ${CORE_ARCHIVE}: ${nm_error}")
endif ()
if (core_undefined MATCHES "Kinova::")
    message(FATAL_ERROR
        "humansl_execution_core needs Kinova (Kortex) symbols — the core "
        "must stay hardware-independent. Offending references:\n"
        "${core_undefined}")
endif ()
if (core_undefined MATCHES "ViconDataStreamSDK")
    message(FATAL_ERROR
        "humansl_execution_core needs Vicon DataStream SDK symbols — the "
        "core must stay hardware-independent. Offending references:\n"
        "${core_undefined}")
endif ()

# --- 3. Both executables must link the shared core target. -----------------
if (NOT "humansl_execution_core" IN_LIST CONTROLLER_LINKS)
    message(FATAL_ERROR
        "the hardware `controller` target does not link "
        "humansl_execution_core (LINK_LIBRARIES: '${CONTROLLER_LINKS}') — "
        "hardware and hardware-free builds must share one core library")
endif ()
if (NOT "humansl_execution_core" IN_LIST PROBE_LINKS)
    message(FATAL_ERROR
        "the hardware-free probe target does not link "
        "humansl_execution_core (LINK_LIBRARIES: '${PROBE_LINKS}') — "
        "hardware and hardware-free builds must share one core library")
endif ()

# The binaries must DEFINE an ArmExecutionCore symbol (mangled name contains
# 16ArmExecutionCore), proving the archive member was pulled into each link,
# not merely listed. `nm --defined-only` reads the symbol table; both
# binaries are unstripped build-tree artifacts.
foreach (binary "${CONTROLLER_BINARY}" "${PROBE_BINARY}")
    if (NOT EXISTS "${binary}")
        message(FATAL_ERROR
            "linked binary not found at ${binary} — build it first "
            "(building is allowed; running the controller is not)")
    endif ()
    execute_process(
        COMMAND "${NM_EXECUTABLE}" --defined-only -C "${binary}"
        OUTPUT_VARIABLE binary_defined
        RESULT_VARIABLE nm_result
        ERROR_VARIABLE nm_error)
    if (NOT nm_result EQUAL 0)
        message(FATAL_ERROR "nm failed on ${binary}: ${nm_error}")
    endif ()
    if (NOT binary_defined MATCHES "ArmExecutionCore::Step")
        message(FATAL_ERROR
            "${binary} does not define ArmExecutionCore::Step — the "
            "execution core was not linked into it")
    endif ()
endforeach ()

message(STATUS "execution-core link structure OK: shared archive, no "
    "Kortex/Vicon SDK symbols, both binaries define ArmExecutionCore")
