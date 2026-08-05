// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include "BridgeMain.h"
#include "JointTrajectory.h"
#include "Targets.h"
#include "TrajectoryEmit.h"

static_assert(kMaxTrajectoryBlockPoints == kMaxJointTrajectoryPoints,
              "the bridge's block cap must match what the controller accepts");

int main(int argc, char** argv) {
    assert(argc == 3 && "usage: test_bridge_main <dh_tool.yaml> <joint_limits.yaml>");
    // Zero-config tool position measured 2026-08-05: (0.0, -0.0246, 1.3073)
    // in base_link. (0.20, 0.35, 0.90) is 0.59 m away — unreachable from a
    // single-solve offset — so the goal below reuses the (0.15, 0.10, -0.10)
    // offset already proven solvable by test_plan_solver.cpp (~20.6 cm).
    const std::vector<std::string> args = {
        "--goal", "0.15", "0.075", "1.207",
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2]};

    // Default output is the timed joint-trajectory block, parsed here by the
    // controller's own accumulator so the wire contract is tested end-to-end.
    {
        std::ostringstream targets, diagnostics;
        assert(RunBridge(args, targets, diagnostics) == 0);

        JointTrajectoryAccumulator accumulator;
        std::istringstream lines(targets.str());
        std::string line, error;
        int complete_blocks = 0;
        JointTrajectory traj;
        while (std::getline(lines, line)) {
            const std::optional<JointTrajectory> finished =
                accumulator.Feed(line, error);
            assert(error.empty());
            if (finished) {
                traj = *finished;
                ++complete_blocks;
            }
        }
        assert(complete_blocks == 1);
        assert(!accumulator.Collecting());
        assert(traj.points.size() >= 2);
        assert(traj.points.front().t_s == 0.0);
        for (std::size_t i = 1; i < traj.points.size(); ++i)
            assert(traj.points[i].t_s > traj.points[i - 1].t_s);

        Eigen::Matrix<double, 7, 1> low_deg, high_deg, vel_limit_deg_s;
        low_deg.setConstant(-360.0);
        high_deg.setConstant(360.0);
        vel_limit_deg_s.setConstant(45.0);
        const std::optional<std::string> traj_error =
            ValidateJointTrajectory(traj, low_deg, high_deg, vel_limit_deg_s);
        assert(!traj_error.has_value());

        // --start-deg above is all zeros, so the block's first state is it.
        for (int joint = 0; joint < 7; ++joint)
            assert(std::abs(traj.points.front().q_rad(joint)) < 1e-6);
    }

    std::ostringstream targets, diagnostics;
    std::vector<std::string> waypoint_args = args;
    waypoint_args.push_back("--emit-waypoints");
    const int exit_code = RunBridge(waypoint_args, targets, diagnostics);
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
        "--runs-root", "tbm_tmp", "--emit-waypoints"};
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

    // --goal-file: the goal (and optional box) come from a YAML file, so a
    // run needs no typed coordinates. Same proven-reachable goal as above.
    {
        std::ofstream goal_yaml("tbm_goal.yaml");
        goal_yaml << "goal: [0.15, 0.075, 1.207]\n";
    }
    std::ostringstream file_targets, file_diagnostics;
    const std::vector<std::string> file_args = {
        "--goal-file", "tbm_goal.yaml",
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2], "--emit-waypoints"};
    assert(RunBridge(file_args, file_targets, file_diagnostics) == 0);
    {
        std::istringstream file_lines(file_targets.str());
        std::string file_line, file_error;
        int file_count = 0;
        while (std::getline(file_lines, file_line)) {
            assert(ParsePoseTarget(file_line, file_error).has_value());
            ++file_count;
        }
        assert(file_count >= 1);
    }
    std::filesystem::remove("tbm_goal.yaml");

    // A goal file with a box outside the SDF grid is rejected exactly like
    // the --box flag: exit 1, no target output.
    {
        std::ofstream goal_yaml("tbm_goal_box.yaml");
        goal_yaml << "goal: [0.15, 0.075, 1.207]\n";
        goal_yaml << "box:\n";
        goal_yaml << "  center: [0, 0, 5.0]\n";
        goal_yaml << "  half_extent: [0.05, 0.05, 0.05]\n";
    }
    std::ostringstream fbox_targets, fbox_diagnostics;
    const std::vector<std::string> fbox_args = {
        "--goal-file", "tbm_goal_box.yaml",
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2]};
    assert(RunBridge(fbox_args, fbox_targets, fbox_diagnostics) == 1);
    assert(fbox_targets.str().empty());
    std::filesystem::remove("tbm_goal_box.yaml");

    // Giving both --goal and --goal-file is ambiguous — hard error, since a
    // silently-ignored file would hide which goal the arm is about to get.
    std::ostringstream both_targets, both_ignored;
    assert(RunBridge({"--goal", "0.15", "0.075", "1.207",
                      "--goal-file", "tbm_goal.yaml"},
                     both_targets, both_ignored) == 1);
    assert(both_targets.str().empty());

    // A missing/unreadable goal file is exit 1 with no target output, and
    // the diagnostics name the file that failed.
    std::ostringstream nofile_targets, nofile_diagnostics;
    const std::vector<std::string> nofile_args = {
        "--goal-file", "tbm_absent.yaml",
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2]};
    assert(RunBridge(nofile_args, nofile_targets, nofile_diagnostics) == 1);
    assert(nofile_targets.str().empty());
    assert(nofile_diagnostics.str().find("tbm_absent.yaml") != std::string::npos);
    return 0;
}
