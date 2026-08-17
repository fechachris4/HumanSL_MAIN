# sim_cli — the humansl_sim command line, checked by running the binary
# (Plan 03 Task 3 Step 4). humansl_sim is hardware-free: it links no
# Kortex, contains no robot address, and cannot command anything (ctest
# sim_no_kortex is the standing evidence for that).
#
# Run as: cmake -DSIM_BINARY=<path> -DTP_LIB=<path> -P this_file
#
# What it checks, and why each one is here:
#
#   1. DETERMINISM. The scripted Mount motion is analytic and seed-free,
#      and nothing in the program reads a clock into its output, so the
#      same command line must produce a byte-identical summary. This is
#      the property every reported number depends on: a run whose result
#      moves between invocations cannot be evidence of anything.
#   2. THE FLAGS ARE ACTUALLY READ. Determinism alone is satisfied
#      perfectly by a binary that ignores every flag and always runs the
#      same simulation. So a translation run and a rotation run must
#      DIFFER, and a phase-shifted run must differ from its unshifted
#      twin. Without this the first check is vacuous.
#   3. THE MOTION REACHES THE PHYSICS. A run that named a motion but held
#      the Mount still would still pass 1 and 2 (the summary text
#      differs). The Mount-excursion line, which is read back from the
#      plant's own mocap row, must therefore report the amplitude that
#      was asked for.
#   4. THE RUN-LENGTH RULE. A headless run must state a positive
#      duration; unbounded running belongs to the viewer alone. Rejection
#      is exit 2 (bad command line), not a crash or a silent zero-cycle
#      run.
#   5. BAD INPUT IS REJECTED, NOT ROUNDED. "--amplitude-m banana" and
#      "--motion sideways" exit 2. A silently-zeroed amplitude would
#      produce a stationary run wearing a "translation" label in its own
#      summary — the one CLI mistake that fabricates evidence rather than
#      just failing.

if (NOT DEFINED SIM_BINARY OR NOT EXISTS "${SIM_BINARY}")
    message(FATAL_ERROR
            "sim_cli: humansl_sim binary not found at '${SIM_BINARY}' — "
            "build the humansl_sim target first")
endif ()

set(ENV{LD_LIBRARY_PATH} "${TP_LIB}")

# Runs humansl_sim and returns its exit code and stdout.
function(run_sim OUT_CODE OUT_TEXT)
    execute_process(COMMAND "${SIM_BINARY}" ${ARGN}
                    OUTPUT_VARIABLE STDOUT_TEXT
                    ERROR_VARIABLE STDERR_TEXT
                    RESULT_VARIABLE EXIT_CODE)
    set(${OUT_CODE} "${EXIT_CODE}" PARENT_SCOPE)
    set(${OUT_TEXT} "${STDOUT_TEXT}" PARENT_SCOPE)
endfunction()

# A short run: 0.2 s is 100 control ticks, enough for a summary and far
# too short to be waiting on.
set(SHORT --headless --duration-s 0.2)

# --- 1 and 2: identical arguments repeat; different arguments differ ----
run_sim(CODE_A TEXT_A ${SHORT} --motion translation)
run_sim(CODE_B TEXT_B ${SHORT} --motion translation)
if (NOT CODE_A EQUAL 0 OR NOT CODE_B EQUAL 0)
    message(FATAL_ERROR
            "sim_cli: a translation run failed (exit ${CODE_A}, "
            "${CODE_B}):\n${TEXT_A}")
endif ()
if (NOT TEXT_A STREQUAL TEXT_B)
    message(FATAL_ERROR
            "sim_cli: two identical command lines produced different "
            "output — the run is not deterministic:\n"
            "--- first ---\n${TEXT_A}\n--- second ---\n${TEXT_B}")
endif ()

run_sim(CODE_R TEXT_R ${SHORT} --motion rotation)
if (NOT CODE_R EQUAL 0)
    message(FATAL_ERROR "sim_cli: the rotation run failed:\n${TEXT_R}")
endif ()
if (TEXT_R STREQUAL TEXT_A)
    message(FATAL_ERROR
            "sim_cli: --motion rotation produced exactly the translation "
            "run's output — the flag is not being read")
endif ()

run_sim(CODE_P TEXT_P ${SHORT} --motion translation --phase-rad 1.5707963)
if (NOT CODE_P EQUAL 0)
    message(FATAL_ERROR "sim_cli: the phase-shifted run failed:\n${TEXT_P}")
endif ()
if (TEXT_P STREQUAL TEXT_A)
    message(FATAL_ERROR
            "sim_cli: --phase-rad changed nothing — the flag is not being "
            "read")
endif ()

# --- 3: the motion reached the physics ----------------------------------
# A quarter period of the shipped defaults (0.05 m at 0.1 Hz) is 2.5 s,
# at which the sinusoid stands at its full amplitude, so the Mount pose
# read back from the plant's own mocap row must show ~0.05 m of travel —
# a number no static run could print. The axis flag is exercised at the
# same time (vertical rather than the default +X).
run_sim(CODE_M TEXT_M --headless --duration-s 2.5 --motion translation
        --motion-axis z)
if (NOT CODE_M EQUAL 0)
    message(FATAL_ERROR "sim_cli: the z-axis run failed:\n${TEXT_M}")
endif ()
if (NOT TEXT_M MATCHES "Mount excursion from the anchor: 0\\.0[45][0-9]+ m")
    message(FATAL_ERROR
            "sim_cli: a quarter period of the default 0.05 m translation "
            "should have moved the plant's Mount to its full amplitude; "
            "the summary says otherwise:\n${TEXT_M}")
endif ()

# --- 4: the run-length rule ---------------------------------------------
run_sim(CODE_Z TEXT_Z --headless --duration-s 0)
if (NOT CODE_Z EQUAL 2)
    message(FATAL_ERROR
            "sim_cli: '--headless --duration-s 0' should be a command-line "
            "error (exit 2), got ${CODE_Z}")
endif ()

# --- 5: bad input is rejected -------------------------------------------
run_sim(CODE_BAD_NUM TEXT_BAD_NUM ${SHORT} --amplitude-m banana)
if (NOT CODE_BAD_NUM EQUAL 2)
    message(FATAL_ERROR
            "sim_cli: '--amplitude-m banana' should be a command-line error "
            "(exit 2), got ${CODE_BAD_NUM} — a silently-zeroed amplitude "
            "would label a stationary run as a moving one")
endif ()

run_sim(CODE_BAD_KIND TEXT_BAD_KIND ${SHORT} --motion sideways)
if (NOT CODE_BAD_KIND EQUAL 2)
    message(FATAL_ERROR
            "sim_cli: '--motion sideways' should be a command-line error "
            "(exit 2), got ${CODE_BAD_KIND}")
endif ()

# --- 6: the representation caveat follows the run -----------------------
# The preamble states how large the unmodelled base-motion (fictitious)
# term is for the run. That figure used to be a literal, true only at the
# defaults — and the flags above are exactly what lets a run leave the
# defaults, so a quoted figure becomes a wrong quantity sitting inside the
# evidence. It must therefore be derived from the resolved motion.
#
# The oracle is derived here by hand, independently of the program: for a
# translation of amplitude A at frequency f the peak base acceleration is
# A (2 pi f)^2, so the shipped 0.05 m at 0.1 Hz is 0.05 * 0.394784 =
# 0.019739 m/s^2 (0.20 % of 9.81), and doubling the frequency must
# QUADRUPLE it to 0.078957 m/s^2 (0.80 %). The translation channel uses no
# model geometry, so these two numbers involve nothing the program could
# get wrong and still agree with. A static run has no such term at all.
run_sim(CODE_C1 TEXT_C1 ${SHORT} --motion translation)
run_sim(CODE_C2 TEXT_C2 ${SHORT} --motion translation --frequency-hz 0.2)
run_sim(CODE_C0 TEXT_C0 ${SHORT} --motion static)
if (NOT CODE_C1 EQUAL 0 OR NOT CODE_C2 EQUAL 0 OR NOT CODE_C0 EQUAL 0)
    message(FATAL_ERROR
            "sim_cli: a caveat-check run failed (exit ${CODE_C1}, "
            "${CODE_C2}, ${CODE_C0})")
endif ()
if (NOT TEXT_C1 MATCHES "at most 0\\.0197 m/s\\^2, 0\\.20% of g")
    message(FATAL_ERROR
            "sim_cli: at the shipped 0.05 m / 0.1 Hz translation the "
            "unmodelled base-motion term is A (2 pi f)^2 = 0.0197 m/s^2, "
            "0.20 % of g; the preamble says otherwise:\n${TEXT_C1}")
endif ()
if (NOT TEXT_C2 MATCHES "at most 0\\.0790 m/s\\^2, 0\\.80% of g")
    message(FATAL_ERROR
            "sim_cli: doubling the frequency must QUADRUPLE the unmodelled "
            "base-motion term to 0.0790 m/s^2, 0.80 % of g (it grows with "
            "f^2); the preamble says otherwise — a figure that does not "
            "follow the flags is a wrong quantity in the "
            "evidence:\n${TEXT_C2}")
endif ()
if (NOT TEXT_C0 MATCHES "omission on THIS run: exactly zero")
    message(FATAL_ERROR
            "sim_cli: a static Mount does not accelerate, so the "
            "unmodelled base-motion term is exactly zero; the preamble "
            "says otherwise:\n${TEXT_C0}")
endif ()

message(STATUS
        "sim_cli: deterministic, flags read, motion reaches the plant, "
        "run-length rule, bad input and the derived representation caveat "
        "enforced — PASS")
