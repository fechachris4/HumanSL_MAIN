// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include "BridgeMain.h"
#include "Config.h"   // basic_control — config::kReferenceFrame
#include "PinocchioKinematicsAdapter.h"
#include "WorldCartesianTrajectory.h"
#include "WorldCartesianTrajectoryWire.h"

static WorldCartesianTrajectory ParseSingleCartesianBlock(
    const std::string& text) {
    WorldCartesianTrajectoryAccumulator accumulator;
    std::istringstream lines(text);
    std::string line, error;
    int complete_blocks = 0;
    WorldCartesianTrajectory parsed;
    while (std::getline(lines, line)) {
        const std::optional<WorldCartesianTrajectory> finished =
            accumulator.Feed(line, error);
        assert(error.empty());
        if (finished) {
            parsed = *finished;
            ++complete_blocks;
        }
    }
    assert(complete_blocks == 1);
    assert(!accumulator.Collecting());
    return parsed;
}

static int CompleteBlocksIn(const std::string& text) {
    (void)ParseSingleCartesianBlock(text);
    return 1;
}

static std::vector<std::string> WithWorldContext(
    std::vector<std::string> args, const std::string& trajectory_id = "9") {
    args.insert(args.end(), {
        "--world-mount-pose-m-quat", "1", "2", "3", "0", "0",
        "0.7071067811865475", "0.7071067811865476",
        "--vicon-sequence", "42", "--trajectory-id", trajectory_id});
    return args;
}

int main(int argc, char** argv) {
    assert(argc == 4 &&
          "usage: test_bridge_main <dh_tool.yaml> <joint_limits.yaml> <dh_flange.yaml>");
    // Zero-config tool position measured 2026-08-05: (0.0, -0.0246, 1.3073)
    // in base_link. (0.20, 0.35, 0.90) is 0.59 m away — unreachable from a
    // single-solve offset — so the goal below reuses the (0.15, 0.10, -0.10)
    // offset already proven solvable by test_plan_solver.cpp (~20.6 cm).
    //
    // A bare --goal is read in config::kReferenceFrame, so the SAME physical
    // point is spelled differently per frame. Both spellings appear below and
    // must plan identically — that equivalence is the frame test.
    // The one physical point, spelled in all three frames, so the test
    // compiles and runs whatever config::kReferenceFrame is set to. Spelling
    // only two of them would turn a legal config into a build break.
    const char* const kGoalPerFrame[3][3] = {
        {"0.15", "-1.158774", "0.497919"},   // kMount
        {"0.15", "0.075", "1.207"},          // kRightBase
        {"0.15", "-0.896391", "-0.960105"},  // kLeftBase
    };
    const char* const* const kGoalInConfiguredFrame =
        kGoalPerFrame[static_cast<int>(config::kReferenceFrame)];

    std::string configured_frame_targets;
    const std::vector<std::string> base_args = {
        "--arm", "right",
        "--goal", kGoalInConfiguredFrame[0], kGoalInConfiguredFrame[1],
        kGoalInConfiguredFrame[2],
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2]};
    const std::vector<std::string> args = WithWorldContext(base_args);

    // The default and sole output is a dense world-Cartesian block. The
    // shared parser checks the exact planner/controller wire contract.
    {
        std::ostringstream targets, diagnostics;
        assert(RunBridge(args, targets, diagnostics) == 0);
        configured_frame_targets = targets.str();
        assert(targets.str().rfind("CART_TRAJ_BEGIN 1 9 42 WORLD ", 0) == 0);
        assert(targets.str().rfind("TRAJ_BEGIN ", 0) != 0);
        assert(targets.str().find("q1") == std::string::npos);

        const WorldCartesianTrajectory trajectory =
            ParseSingleCartesianBlock(targets.str());
        assert(trajectory.trajectory_id == 9);
        assert(trajectory.planner_vicon_sequence == 42);
        assert(trajectory.points.size() > 1000 &&
               "world output must retain the final dense GPMP2 artefact");

        const Eigen::Isometry3d world_T_mount =
            Eigen::Translation3d(1.0, 2.0, 3.0) *
            Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
        const Eigen::Isometry3d world_T_base =
            world_T_mount *
            pinocchio_kinematics_adapter::MountFromBase(/*left_arm=*/false);
        const auto start_fk =
            pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
                Eigen::Matrix<double, 7, 1>::Zero(),
                config::kRightEndEffectorFrame);
        const Eigen::Vector3d expected_start = world_T_base * start_fk.position;
        const Eigen::Matrix3d expected_start_rotation =
            world_T_base.linear() * start_fk.rotation;
        assert((trajectory.points.front().position_world_m - expected_start).norm() <
               0.002);
        assert((trajectory.points.front().orientation_world.toRotationMatrix() -
                expected_start_rotation).norm() < 0.005);

        const Eigen::Vector3d declared_goal(
            std::stod(kGoalInConfiguredFrame[0]),
            std::stod(kGoalInConfiguredFrame[1]),
            std::stod(kGoalInConfiguredFrame[2]));
        Eigen::Vector3d expected_goal_world = declared_goal;
        if (config::kReferenceFrame == config::ReferenceFrame::kMount) {
            expected_goal_world = world_T_mount * declared_goal;
        } else if (config::kReferenceFrame ==
                   config::ReferenceFrame::kRightBase) {
            expected_goal_world = world_T_base * declared_goal;
        } else if (config::kReferenceFrame ==
                   config::ReferenceFrame::kLeftBase) {
            expected_goal_world =
                world_T_mount *
                pinocchio_kinematics_adapter::MountFromBase(/*left_arm=*/true) *
                declared_goal;
        }
        assert((trajectory.points.back().position_world_m - expected_goal_world).norm() <
               0.010);
        assert(trajectory.points.back().arrival_eligible);
        assert(trajectory.points.back().linear_velocity_world_m_s.isZero(0.0));
        assert(trajectory.points.back().angular_velocity_world_rad_s.isZero(0.0));

        const std::string diag = diagnostics.str();
        assert(diag.find("planner Vicon sequence: 42") != std::string::npos);
        assert(diag.find("trajectory ID: 9") != std::string::npos);
        assert(diag.find("T_W_M position [1, 2, 3] m") != std::string::npos);
        assert(diag.find("output frame: WORLD") != std::string::npos);
    }

    // Cartesian output is meaningless without a coherent snapshot and its
    // provenance. Reject incomplete, non-unit, non-finite, or malformed
    // values during argument parsing and leave stdout completely untouched.
    const auto ExpectBadCartesianArgs = [&](std::vector<std::string> bad_args,
                                             const std::string& diagnostic) {
        std::ostringstream targets, diagnostics;
        assert(RunBridge(bad_args, targets, diagnostics) == 1);
        assert(targets.str().empty());
        assert(diagnostics.str().find(diagnostic) != std::string::npos);
    };
    {
        std::vector<std::string> bad = base_args;
        bad.insert(bad.end(), {"--output", "world-cartesian",
                               "--vicon-sequence", "42",
                               "--trajectory-id", "9"});
        ExpectBadCartesianArgs(bad, "--world-mount-pose-m-quat");
    }
    {
        std::vector<std::string> bad = base_args;
        bad.insert(bad.end(), {
            "--output", "world-cartesian",
            "--world-mount-pose-m-quat", "1", "2", "3", "0", "0", "0", "2",
            "--vicon-sequence", "42", "--trajectory-id", "9"});
        ExpectBadCartesianArgs(bad, "unit quaternion");
    }
    {
        std::vector<std::string> bad = base_args;
        bad.insert(bad.end(), {
            "--output", "world-cartesian",
            "--world-mount-pose-m-quat", "nan", "2", "3", "0", "0", "0", "1",
            "--vicon-sequence", "42", "--trajectory-id", "9"});
        ExpectBadCartesianArgs(bad, "finite number");
    }
    {
        std::vector<std::string> bad = base_args;
        bad.insert(bad.end(), {
            "--output", "world-cartesian",
            "--world-mount-pose-m-quat", "1", "2", "3", "0", "0", "0", "1",
            "--vicon-sequence", "4.2", "--trajectory-id", "9"});
        ExpectBadCartesianArgs(bad, "unsigned integer");
    }


    // Bad arguments produce exit code 1 and NO target output.
    std::ostringstream empty_targets, ignored;
    assert(RunBridge({"--arm", "right", "--goal", "not-a-number"}, empty_targets,
                     ignored) == 1);
    assert(empty_targets.str().empty());

    // --arm is required — refused before anything else runs.
    std::ostringstream noarm_targets, noarm_diagnostics;
    assert(RunBridge({"--goal", kGoalInConfiguredFrame[0], kGoalInConfiguredFrame[1],
                      kGoalInConfiguredFrame[2]},
                     noarm_targets, noarm_diagnostics) == 1);
    assert(noarm_targets.str().empty());
    assert(noarm_diagnostics.str().find("--arm is required") != std::string::npos);

    // A --box outside the SDF grid volume (this one sits at z=5 m) must be
    // rejected before solving, not silently treated as "no obstacle" by
    // gpmp2 — exit 1, no target output.
    //
    // The diagnostic is asserted, not just the exit code. Before the planner
    // moved into mount this case exited 1 for an entirely different reason —
    // a box was only legal in the controlled arm's own base frame, so it was
    // refused on frame grounds and never reached the grid check at all. An
    // exit-code-only assertion cannot tell those apart.
    std::ostringstream box_targets, box_diagnostics;
    const std::vector<std::string> out_of_grid_box_args = WithWorldContext({
        "--arm", "right",
        "--goal", kGoalInConfiguredFrame[0], kGoalInConfiguredFrame[1],
        kGoalInConfiguredFrame[2],
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2],
        "--box", "0", "0", "5.0", "0.05", "0.05", "0.05"});
    assert(RunBridge(out_of_grid_box_args, box_targets, box_diagnostics) == 1);
    assert(box_targets.str().empty());
    assert(box_diagnostics.str().find("outside the SDF grid volume") !=
           std::string::npos);

    // A box declared in an arm's BASE frame is now accepted and converted
    // into mount, with its half-extents inflated to the enclosing
    // axis-aligned box (the bases are rolled ~69 deg, so the rotated box is
    // not grid-aligned). This capability did not exist before the move — the
    // frame rule was the other way round — so nothing else covers it.
    //
    // The box is placed well away from the arm so it cannot make the goal
    // unreachable: this asserts the frame/inflation path, not avoidance.
    {
        std::ostringstream targets, diagnostics;
        std::ofstream goal_yaml("tbm_goal_basebox.yaml");
        goal_yaml << "right:\n"
                  << "  frame: right_base\n"
                  << "  goal: [" << kGoalPerFrame[1][0] << ", " << kGoalPerFrame[1][1]
                  << ", " << kGoalPerFrame[1][2] << "]\n"
                  << "  box:\n"
                  << "    center: [-0.6, -0.6, 0.1]\n"
                  << "    half_extent: [0.05, 0.05, 0.05]\n";
        goal_yaml.close();
        const std::vector<std::string> base_box_args = WithWorldContext({
            "--arm", "right", "--goal-file", "tbm_goal_basebox.yaml",
            "--start-deg", "0", "0", "0", "0", "0", "0", "0",
            "--dh", argv[1], "--joint-limits", argv[2]});
        assert(RunBridge(base_box_args, targets, diagnostics) == 0);
        assert(!targets.str().empty());
        // A cube rotated 69 deg must come out strictly larger on some axis;
        // if the conversion silently kept the requested half-extents the
        // obstacle would be smaller than asked for, the unsafe direction.
        assert(diagnostics.str().find("half-extent inflated") != std::string::npos);
        std::filesystem::remove("tbm_goal_basebox.yaml");
    }

    // --runs-root auto-discovery: a fixture dated subdir with one valid
    // 13-column CSV (same shape as test_start_state's fixture, all joints
    // at 0 degrees) is found and used as the start state.
    std::filesystem::create_directories("tbm_tmp/2026-08-05");
    {
        // Prefix must match the arm-scoped search below (--arm right ->
        // "loop_log_right"): a right-arm run never seeds a left-arm plan or
        // vice versa (config::ArmConfig::log_prefix, basic_control/Config.h).
        std::ofstream csv("tbm_tmp/2026-08-05/loop_log_right_x.csv");
        csv << "time_s,dt_s,meas_j1,extra,meas_j2,meas_j3,meas_j4,"
               "meas_j5,meas_j6,meas_j7,vel_j1,torque_j1,fault_j1\n";
        csv << "0.001,0.002,0,99,0,0,0,0,0,0,100,200,300\n";
    }
    std::ostringstream auto_targets, auto_diagnostics;
    const std::vector<std::string> auto_args = WithWorldContext({
        "--arm", "right",
        "--goal", kGoalInConfiguredFrame[0], kGoalInConfiguredFrame[1],
        kGoalInConfiguredFrame[2],
        "--dh", argv[1], "--joint-limits", argv[2],
        "--runs-root", "tbm_tmp"});
    assert(RunBridge(auto_args, auto_targets, auto_diagnostics) == 0);
    assert(CompleteBlocksIn(auto_targets.str()) == 1);
    std::filesystem::remove_all("tbm_tmp");

    // Frame equivalence: the same physical point, declared right_base in a
    // goal file, must produce the same plan as the configured-frame --goal
    // above. This is what proves the boundary conversion is the identity on
    // the physical target rather than merely running without error.
    {
        {
            std::ofstream frame_yaml("tbm_frame.yaml");
            frame_yaml << "right:\n"
                       << "  frame: right_base\n"
                       << "  goal: [0.15, 0.075, 1.207]\n";
        }
        std::ostringstream frame_targets, frame_diagnostics;
        const std::vector<std::string> frame_args = WithWorldContext({
            "--arm", "right",
            "--goal-file", "tbm_frame.yaml",
            "--start-deg", "0", "0", "0", "0", "0", "0", "0",
            "--dh", argv[1], "--joint-limits", argv[2]});
        assert(RunBridge(frame_args, frame_targets, frame_diagnostics) == 0);
        const WorldCartesianTrajectory via_file =
            ParseSingleCartesianBlock(frame_targets.str());
        const WorldCartesianTrajectory via_cli =
            ParseSingleCartesianBlock(configured_frame_targets);

        // The claim is that both spellings command the same world-frame tool
        // position. No planned joint posture exists at this boundary.
        //
        // Tolerance: the mount spelling carries 6 decimals, so it round-trips
        // a micrometre off, and each solve converges to within its own ~3 mm
        // goal error — two independent solves can differ by roughly twice
        // that, so 10 mm is the honest bound here, not a slack one.
        const Eigen::Vector3d tool_file = via_file.points.back().position_world_m;
        const Eigen::Vector3d tool_cli = via_cli.points.back().position_world_m;
        assert((tool_file - tool_cli).norm() < 0.010);
        std::filesystem::remove("tbm_frame.yaml");

        // The SAME physical point in every frame the constant accepts, so
        // each conversion branch in ToControlledBase is exercised — including
        // left_base, whose two-step composition
        // (T_rightbase_mount * T_mount_leftbase) is the easiest to get
        // backwards and would otherwise ship untested because it is named in
        // config but never the default.
        for (int frame = 0; frame < 3; ++frame) {
            {
                std::ofstream each_yaml("tbm_eachframe.yaml");
                each_yaml << "right:\n"
                          << "  frame: "
                          << config::kReferenceFrameNames[frame] << "\n"
                          << "  goal: [" << kGoalPerFrame[frame][0] << ", "
                          << kGoalPerFrame[frame][1] << ", "
                          << kGoalPerFrame[frame][2] << "]\n";
            }
            std::ostringstream each_targets, each_diagnostics;
            const std::vector<std::string> each_args = WithWorldContext({
                "--arm", "right",
                "--goal-file", "tbm_eachframe.yaml",
                "--start-deg", "0", "0", "0", "0", "0", "0", "0",
                "--dh", argv[1], "--joint-limits", argv[2]});
            assert(RunBridge(each_args, each_targets, each_diagnostics) == 0);
            const WorldCartesianTrajectory each =
                ParseSingleCartesianBlock(each_targets.str());
            const Eigen::Vector3d tool_each =
                each.points.back().position_world_m;
            assert((tool_each - tool_cli).norm() < 0.010);
            std::filesystem::remove("tbm_eachframe.yaml");
        }
    }

    // An unknown frame name is refused, naming the accepted values.
    {
        {
            std::ofstream bad_yaml("tbm_badframe.yaml");
            bad_yaml << "right:\n  frame: torso\n  goal: [0.15, 0.075, 1.207]\n";
        }
        std::ostringstream bad_targets, bad_diagnostics;
        assert(RunBridge({"--arm", "right", "--goal-file", "tbm_badframe.yaml"},
                         bad_targets, bad_diagnostics) == 1);
        assert(bad_targets.str().empty());
        assert(bad_diagnostics.str().find("unknown frame 'torso'") !=
               std::string::npos);
        std::filesystem::remove("tbm_badframe.yaml");
    }

    // A goal file missing the block for the requested --arm is refused,
    // naming which block is missing — not silently read as the other arm's
    // numbers.
    {
        {
            std::ofstream noblock_yaml("tbm_noblock.yaml");
            noblock_yaml << "right:\n  frame: right_base\n  goal: [0.15, 0.075, 1.207]\n";
        }
        std::ostringstream noblock_targets, noblock_diagnostics;
        assert(RunBridge({"--arm", "left", "--goal-file", "tbm_noblock.yaml"},
                         noblock_targets, noblock_diagnostics) == 1);
        assert(noblock_targets.str().empty());
        assert(noblock_diagnostics.str().find("no 'left:' block") != std::string::npos);
        std::filesystem::remove("tbm_noblock.yaml");
    }

    // Empty/missing runs root: exit 2, no target output, diagnostics point
    // at starting the controller.
    std::filesystem::create_directories("tbm_empty_tmp");
    std::ostringstream missing_targets, missing_diagnostics;
    const std::vector<std::string> missing_args = WithWorldContext({
        "--arm", "right",
        "--goal", kGoalInConfiguredFrame[0], kGoalInConfiguredFrame[1],
        kGoalInConfiguredFrame[2],
        "--dh", argv[1], "--joint-limits", argv[2],
        "--runs-root", "tbm_empty_tmp"});
    assert(RunBridge(missing_args, missing_targets, missing_diagnostics) == 2);
    assert(missing_targets.str().empty());
    assert(missing_diagnostics.str().find("start the controller") != std::string::npos);
    std::filesystem::remove_all("tbm_empty_tmp");

    // --goal-file: the goal (and optional box) come from a YAML file, so a
    // run needs no typed coordinates. Same proven-reachable goal as above.
    {
        std::ofstream goal_yaml("tbm_goal.yaml");
        goal_yaml << "right:\n  frame: right_base\n  goal: [0.15, 0.075, 1.207]\n";
    }
    std::ostringstream file_targets, file_diagnostics;
    const std::vector<std::string> file_args = WithWorldContext({
        "--arm", "right",
        "--goal-file", "tbm_goal.yaml",
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2]});
    assert(RunBridge(file_args, file_targets, file_diagnostics) == 0);
    assert(CompleteBlocksIn(file_targets.str()) == 1);
    std::filesystem::remove("tbm_goal.yaml");

    // A goal file with a box outside the SDF grid is rejected exactly like
    // the --box flag: exit 1, no target output.
    {
        std::ofstream goal_yaml("tbm_goal_box.yaml");
        goal_yaml << "right:\n";
        goal_yaml << "  frame: right_base\n  goal: [0.15, 0.075, 1.207]\n";
        goal_yaml << "  box:\n";
        goal_yaml << "    center: [0, 0, 5.0]\n";
        goal_yaml << "    half_extent: [0.05, 0.05, 0.05]\n";
    }
    std::ostringstream fbox_targets, fbox_diagnostics;
    const std::vector<std::string> fbox_args = WithWorldContext({
        "--arm", "right",
        "--goal-file", "tbm_goal_box.yaml",
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2]});
    assert(RunBridge(fbox_args, fbox_targets, fbox_diagnostics) == 1);
    assert(fbox_targets.str().empty());
    std::filesystem::remove("tbm_goal_box.yaml");

    // Giving both --goal and --goal-file is ambiguous — hard error, since a
    // silently-ignored file would hide which goal the arm is about to get.
    std::ostringstream both_targets, both_ignored;
    assert(RunBridge({"--arm", "right",
                      "--goal", kGoalInConfiguredFrame[0], kGoalInConfiguredFrame[1],
        kGoalInConfiguredFrame[2],
                      "--goal-file", "tbm_goal.yaml"},
                     both_targets, both_ignored) == 1);
    assert(both_targets.str().empty());

    // A missing/unreadable goal file is exit 1 with no target output, and
    // the diagnostics name the file that failed.
    std::ostringstream nofile_targets, nofile_diagnostics;
    const std::vector<std::string> nofile_args = WithWorldContext({
        "--arm", "right",
        "--goal-file", "tbm_absent.yaml",
        "--start-deg", "0", "0", "0", "0", "0", "0", "0",
        "--dh", argv[1], "--joint-limits", argv[2]});
    assert(RunBridge(nofile_args, nofile_targets, nofile_diagnostics) == 1);
    assert(nofile_targets.str().empty());
    assert(nofile_diagnostics.str().find("tbm_absent.yaml") != std::string::npos);

    // --arm left, end to end, against the left/flange DH file. left_base is
    // only reachable via a goal FILE (a bare --goal always reads
    // config::kReferenceFrame; only --goal-file's frame: key can name it).
    // The goal is the left arm's own zero-config tool position plus the
    // same offset already proven solvable for the right arm above
    // (~20.6 cm) — computed from the flange chain itself rather than a
    // hand measurement, since no physical left-arm survey exists yet.
    {
        const Eigen::Vector3d left_zero_config_position =
            pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
                Eigen::Matrix<double, 7, 1>::Zero(), config::kLeftEndEffectorFrame,
                /*left_arm=*/true)
                .position;
        const Eigen::Vector3d left_goal =
            left_zero_config_position + Eigen::Vector3d(0.15, 0.10, -0.10);
        {
            std::ofstream left_goal_yaml("tbm_left_goal.yaml");
            left_goal_yaml << "left:\n  frame: left_base\n  goal: ["
                          << left_goal.x() << ", " << left_goal.y() << ", "
                          << left_goal.z() << "]\n";
        }
        std::ostringstream left_targets, left_diagnostics;
        const std::vector<std::string> left_file_args = WithWorldContext({
            "--arm", "left",
            "--goal-file", "tbm_left_goal.yaml",
            "--start-deg", "0", "0", "0", "0", "0", "0", "0",
            "--dh", argv[3], "--joint-limits", argv[2]});
        assert(RunBridge(left_file_args, left_targets, left_diagnostics) == 0);
        assert(CompleteBlocksIn(left_targets.str()) == 1);
        assert(left_diagnostics.str().find("arm: left") != std::string::npos);

        // Tripwire for the 2026-08-06 frame-mismatch bug: when any
        // Pinocchio-backed evaluation in the solve (IK init, start pose,
        // the error metric itself) queries the RIGHT arm's tool chain
        // against the left/flange DH model, the reported final goal error
        // lands at the tool-vs-flange offset — a constant ~120 mm. A
        // healthy consistent-frame solve reports a few mm, so 50 mm cleanly
        // separates the two. Parsed from the same diagnostics line the
        // operator reads.
        const std::string diag = left_diagnostics.str();
        const std::string error_key = "final goal error: ";
        const std::size_t error_at = diag.find(error_key);
        assert(error_at != std::string::npos);
        const double final_error_mm = std::stod(diag.substr(error_at + error_key.size()));
        assert(final_error_mm < 50.0 &&
               "left-arm final goal error must not carry the tool-vs-flange offset");
        std::filesystem::remove("tbm_left_goal.yaml");
    }
    return 0;
}
