# sim_no_kortex — link evidence that humansl_sim cannot reach the robot
# (Plan 02 Task 5 Step 4; mechanical enforcement rule of
# docs/engineering/robotics-analysis-workflow.md).
#
# Run as: cmake -DSIM_BINARY=<path> -DLINK_TXT=<path> -P this_file
#
# Fails if any of the following appears:
#   - a Kortex library in the binary's dynamic dependencies (ldd);
#   - a Kinova::Api symbol among the binary's undefined symbols (nm -uC);
#   - a robot IP address embedded in the binary (the production robot
#     addresses live in Config.h; none may be baked into the sim);
#   - hardware Runner.cpp/Hardware.cpp objects or a Kortex library in the
#     actual link command CMake generated for humansl_sim.

if (NOT DEFINED SIM_BINARY OR NOT EXISTS "${SIM_BINARY}")
    message(FATAL_ERROR
            "sim_no_kortex: humansl_sim binary not found at '${SIM_BINARY}' "
            "— build the humansl_sim target first")
endif ()

# --- 1. Dynamic dependencies -------------------------------------------
execute_process(COMMAND ldd "${SIM_BINARY}"
                OUTPUT_VARIABLE LDD_OUTPUT
                RESULT_VARIABLE LDD_RESULT)
if (NOT LDD_RESULT EQUAL 0)
    message(FATAL_ERROR "sim_no_kortex: ldd failed on '${SIM_BINARY}'")
endif ()
string(TOLOWER "${LDD_OUTPUT}" LDD_LOWER)
if (LDD_LOWER MATCHES "kortex")
    message(FATAL_ERROR
            "sim_no_kortex: humansl_sim dynamically links a Kortex "
            "library:\n${LDD_OUTPUT}")
endif ()

# --- 2. Undefined symbols ----------------------------------------------
execute_process(COMMAND nm -uC "${SIM_BINARY}"
                OUTPUT_VARIABLE NM_OUTPUT
                RESULT_VARIABLE NM_RESULT)
if (NOT NM_RESULT EQUAL 0)
    message(FATAL_ERROR "sim_no_kortex: nm failed on '${SIM_BINARY}'")
endif ()
if (NM_OUTPUT MATCHES "Kinova::Api" OR NM_OUTPUT MATCHES "KortexApi")
    message(FATAL_ERROR
            "sim_no_kortex: humansl_sim references Kortex symbols")
endif ()

# --- 3. Robot addresses in the binary ----------------------------------
# The production arms live at 192.168.1.x (Config.h kRightRobotIp /
# kLeftRobotIp). Any dotted 192.168 address embedded in the sim binary is
# treated as robot reach and fails the gate.
execute_process(COMMAND grep -c -a -E "192\\.168\\.[0-9]+\\.[0-9]+"
                        "${SIM_BINARY}"
                OUTPUT_VARIABLE IP_COUNT
                RESULT_VARIABLE IP_RESULT)
# grep exits 1 when nothing matches — that is the passing case.
if (IP_RESULT EQUAL 0)
    string(STRIP "${IP_COUNT}" IP_COUNT)
    message(FATAL_ERROR
            "sim_no_kortex: a robot IP address (192.168.x.x) appears in "
            "the humansl_sim binary (${IP_COUNT} match(es))")
endif ()

# --- 4. The generated link command --------------------------------------
if (NOT DEFINED LINK_TXT OR NOT EXISTS "${LINK_TXT}")
    message(FATAL_ERROR
            "sim_no_kortex: link command file not found at '${LINK_TXT}'")
endif ()
file(READ "${LINK_TXT}" LINK_COMMAND)
string(TOLOWER "${LINK_COMMAND}" LINK_LOWER)
if (LINK_LOWER MATCHES "kortex")
    message(FATAL_ERROR
            "sim_no_kortex: the humansl_sim link command references "
            "Kortex:\n${LINK_COMMAND}")
endif ()
if (LINK_COMMAND MATCHES "Runner\\.cpp" OR LINK_COMMAND MATCHES "Hardware\\.cpp")
    message(FATAL_ERROR
            "sim_no_kortex: the humansl_sim link command contains hardware "
            "Runner.cpp/Hardware.cpp objects:\n${LINK_COMMAND}")
endif ()

message(STATUS
        "sim_no_kortex: no Kortex library, no Kortex symbol, no robot "
        "address, no hardware runner object — PASS")
