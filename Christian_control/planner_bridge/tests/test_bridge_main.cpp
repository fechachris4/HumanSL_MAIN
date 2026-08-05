// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include "BridgeMain.h"
#include "Targets.h"

int main(int argc, char** argv) {
    assert(argc == 3 && "usage: test_bridge_main <dh_tool.yaml> <joint_limits.yaml>");
    std::ostringstream targets, diagnostics;
    // Zero-config tool position measured 2026-08-05: (0.0, -0.0246, 1.3073)
    // in base_link. (0.20, 0.35, 0.90) is 0.59 m away — unreachable from a
    // single-solve offset — so the goal below reuses the (0.15, 0.10, -0.10)
    // offset already proven solvable by test_plan_solver.cpp (~20.6 cm).
    const std::vector<std::string> args = {
        "--goal", "0.15", "0.075", "1.207",
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2]};
    const int exit_code = RunBridge(args, targets, diagnostics);
    assert(exit_code == 0);

    std::istringstream lines(targets.str());
    std::string line, error;
    int count = 0;
    while (std::getline(lines, line)) {
        assert(ParsePoseTarget(line, error).has_value());
        ++count;
    }
    assert(count >= 1 && count <= 8);

    // Bad arguments produce exit code 1 and NO target output.
    std::ostringstream empty_targets, ignored;
    assert(RunBridge({"--goal", "not-a-number"}, empty_targets, ignored) == 1);
    assert(empty_targets.str().empty());

    // A --box outside the SDF grid volume (z up to 1.6 m; this box sits at
    // z=5 m) must be rejected before solving, not silently treated as "no
    // obstacle" by gpmp2 — exit 1, no target output.
    std::ostringstream box_targets, box_diagnostics;
    const std::vector<std::string> out_of_grid_box_args = {
        "--goal", "0.15", "0.075", "1.207",
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2],
        "--box", "0", "0", "5.0", "0.05", "0.05", "0.05"};
    assert(RunBridge(out_of_grid_box_args, box_targets, box_diagnostics) == 1);
    assert(box_targets.str().empty());

    // --runs-root auto-discovery: a fixture dated subdir with one valid
    // 13-column CSV (same shape as test_start_state's fixture, all joints
    // at 0 degrees) is found and used as the start state.
    std::filesystem::create_directories("tbm_tmp/2026-08-05");
    {
        std::ofstream csv("tbm_tmp/2026-08-05/loop_log_x.csv");
        csv << "time_s,dt_s,meas_j1,extra,meas_j2,meas_j3,meas_j4,"
               "meas_j5,meas_j6,meas_j7,vel_j1,torque_j1,fault_j1\n";
        csv << "0.001,0.002,0,99,0,0,0,0,0,0,100,200,300\n";
    }
    std::ostringstream auto_targets, auto_diagnostics;
    const std::vector<std::string> auto_args = {
        "--goal", "0.15", "0.075", "1.207",
        "--dh", argv[1], "--joint-limits", argv[2],
        "--runs-root", "tbm_tmp"};
    assert(RunBridge(auto_args, auto_targets, auto_diagnostics) == 0);
    {
        std::istringstream auto_lines(auto_targets.str());
        std::string auto_line, auto_error;
        int auto_count = 0;
        while (std::getline(auto_lines, auto_line)) {
            assert(ParsePoseTarget(auto_line, auto_error).has_value());
            ++auto_count;
        }
        assert(auto_count >= 1);
    }
    std::filesystem::remove_all("tbm_tmp");

    // Empty/missing runs root: exit 2, no target output, diagnostics point
    // at starting the controller.
    std::filesystem::create_directories("tbm_empty_tmp");
    std::ostringstream missing_targets, missing_diagnostics;
    const std::vector<std::string> missing_args = {
        "--goal", "0.15", "0.075", "1.207",
        "--dh", argv[1], "--joint-limits", argv[2],
        "--runs-root", "tbm_empty_tmp"};
    assert(RunBridge(missing_args, missing_targets, missing_diagnostics) == 2);
    assert(missing_targets.str().empty());
    assert(missing_diagnostics.str().find("start the controller") != std::string::npos);
    std::filesystem::remove_all("tbm_empty_tmp");
    return 0;
}
