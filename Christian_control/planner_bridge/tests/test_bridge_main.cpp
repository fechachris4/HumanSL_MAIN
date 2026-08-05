// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
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
    return 0;
}
