// The baseline-bridge adapter, proven against a FAKE bridge script so the
// wiring is testable without the frozen binary: the script records the argv
// it received (proving arm, goal file and the WRAPPED start state cross the
// process boundary exactly once and in degrees), prints a canned report to
// stderr (proving the planner's own verdict lines reach diagnostics for the
// panel), and emits a canned joint wire block on stdout (proving deg->rad,
// timing and point count survive adaptation 1:1). Projection runs through
// the real current model, so frame/arm identity is exercised, not mocked.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "BaselineBridge.h"

namespace {

std::string ReadAll(const std::string& path) {
    std::ifstream in(path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

void WriteFakeBridge(const std::string& script_path,
                     const std::string& argv_path,
                     int exit_code) {
    std::ofstream out(script_path);
    out << "#!/bin/sh\n"
        << "printf '%s\\n' \"$@\" > " << argv_path << "\n"
        << "echo 'planner Vicon sequence: fake' 1>&2\n"
        << "echo '  e_command       (desired vs 500 Hz reconstruction)  "
           "max 4.595 mm, rms 1.179 mm' 1>&2\n"
        << "echo '  hardware_execution_allowed yes' 1>&2\n";
    if (exit_code == 0) {
        // 3 samples, 0.5 s apart: q ramps 10 -> 30 deg on joint 1, others
        // fixed at a recognisable pattern; qdot constant 40 deg/s there.
        out << "echo 'TRAJ_BEGIN 3'\n";
        const char* rows[3] = {
            "0.000000 10 20 30 40 50 60 70 40 0 0 0 0 0 0",
            "0.500000 20 20 30 40 50 60 70 40 0 0 0 0 0 0",
            "1.000000 30 20 30 40 50 60 70 0 0 0 0 0 0 0"};
        for (const char* row : rows) out << "echo '" << row << "'\n";
        out << "echo 'TRAJ_END'\n";
    }
    out << "exit " << exit_code << "\n";
    out.close();
    std::string chmod = "chmod +x " + script_path;
    assert(std::system(chmod.c_str()) == 0);
}

}  // namespace

int main(int argc, char** argv) {
    assert(argc == 3 &&
           "usage: test_baseline_bridge <dh_params_tool.yaml> "
           "<dh_params_flange.yaml>");

    const std::string script = "tbb_fake_bridge.sh";
    const std::string argv_file = "tbb_fake_bridge_argv.txt";

    PlannerRuntimeConfig config;
    config.goal_file = "tbb_goal.yaml";  // existence not checked here — the
                                         // fake records it, the real binary
                                         // would read it
    config.right_dh_file = argv[1];
    config.left_dh_file = argv[2];
    config.baseline_bridge_binary = "./" + script;

    PlanningRequest request;
    request.request_id = 42;
    request.vicon_sequence = 7;
    request.arm = PlanningArm::kLeft;
    // 359.93 deg on joint 1: the exact seam value from the 2026-08-19
    // failure. The adapter must hand the fake bridge its PRINCIPAL value
    // (-0.07 deg), not the raw Kortex angle.
    request.q_rad(0) = 359.93 * M_PI / 180.0;
    for (int j = 1; j < 7; ++j) request.q_rad(j) = 0.1 * j;

    // --- a successful solve adapts end to end -------------------------
    WriteFakeBridge(script, argv_file, 0);
    std::ostringstream diagnostics;
    const PlannerSolveResult result =
        SolveBaselineBridgeForRequest(request, config, diagnostics);
    const std::string log = diagnostics.str();

    assert(result.exit_code == 0 && "a clean fake solve must adapt");
    assert(result.trajectory && "a clean fake solve must yield a trajectory");
    assert(result.trajectory->points.size() == 3 &&
           "every wire sample becomes exactly one Cartesian point");
    assert(result.trajectory->trajectory_id == 42 &&
           result.trajectory->planner_vicon_sequence == 7 &&
           "identity fields pass through untouched");
    assert(std::abs(result.trajectory->points.back().t_from_start_s - 1.0) <
               1e-12 &&
           "wire timing passes through untouched");
    assert(result.trajectory->points.back().arrival_eligible &&
           "the final point is the arrival point");

    // The argv the subprocess actually received: arm identity, the goal
    // file, and the wrapped start in degrees.
    const std::string seen = ReadAll(argv_file);
    assert(seen.find("left") != std::string::npos && "arm identity crossed");
    assert(seen.find("tbb_goal.yaml") != std::string::npos &&
           "the panel-edited goal file crossed");
    {
        // First --start-deg value, parsed: 359.93 deg must cross as its
        // principal value -0.07 deg (the wrap), never the raw angle.
        std::istringstream argv_lines(seen);
        std::string token, first_start_deg;
        while (std::getline(argv_lines, token))
            if (token == "--start-deg") {
                std::getline(argv_lines, first_start_deg);
                break;
            }
        assert(!first_start_deg.empty() && "--start-deg crossed");
        const double crossed = std::stod(first_start_deg);
        assert(std::abs(crossed - (-0.07)) < 1e-9 &&
               "359.93 deg crossed as its principal value -0.07 deg");
    }
    assert(seen.find("359.9") == std::string::npos &&
           "the raw [0,360) angle must never reach the baseline");

    // The planner's own verdict lines reach the diagnostics verbatim, so
    // the panel's plan-verdict display reads them unchanged.
    assert(log.find("hardware_execution_allowed yes") != std::string::npos);
    assert(log.find("e_command") != std::string::npos);
    assert(log.find("BASELINE bridge (5abc1b2c)") != std::string::npos &&
           "the log names which planner implementation ran");

    // --- a rejecting bridge adapts to exit 4, no trajectory ------------
    WriteFakeBridge(script, argv_file, 4);
    std::ostringstream reject_diag;
    const PlannerSolveResult rejected =
        SolveBaselineBridgeForRequest(request, config, reject_diag);
    assert(rejected.exit_code == 4 && !rejected.trajectory &&
           "a planner rejection passes through as exit 4, nothing published");

    // --- an unconfigured binary fails loudly ---------------------------
    PlannerRuntimeConfig unconfigured = config;
    unconfigured.baseline_bridge_binary.clear();
    std::ostringstream missing_diag;
    const PlannerSolveResult missing =
        SolveBaselineBridgeForRequest(request, unconfigured, missing_diag);
    assert(missing.exit_code != 0 && !missing.trajectory);

    std::remove(script.c_str());
    std::remove(argv_file.c_str());
    std::printf("all baseline-bridge adapter tests passed\n");
    return 0;
}
