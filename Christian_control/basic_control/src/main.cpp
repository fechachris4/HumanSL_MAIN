/*
 * main.cpp — the story of the program, told at a high level.
 * Details live in the modules: Connect (sessions), Measure (sensors),
 * Kinematics (FK), Record (logging loop), Dynamics (model).
 *
 * Usage: ./controller   (no flags — settings live in Config.h)
 */

#include <iostream>
#include <fstream>
#include <string>
#include <atomic>
#include <csignal>

#include <iomanip>

#include "Config.h"
#include "Connect.h"
#include "controllers/legacy_advanced/Controller.h"
#include "controllers/simple_joint_position_hold/SimpleJointPositionHoldLoop.h"
#include "Kinematics.h"
#include "Measure.h"
#include "Record.h"
#include "Motion.h"
#include "Dynamics.h"

// Ctrl+C sets this flag; loops check it and exit cleanly.
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop = true; }

int main()
{
    std::signal(SIGINT, on_sigint);

    try {
        // A motion.txt (searched in cwd, the executable's dir, and its
        // parent) makes this run a move. Load and validate it BEFORE
        // touching the robot, so a bad config exits here (code 2) without
        // ever connecting.
        std::string motion_path = find_motion_config();
        MotionConfig motion;
        if (motion_path.empty()) {
            std::cout << "No motion.txt found -> recording mode.\n";
        } else {
            std::cout << "Using motion config: " << motion_path << "\n";
            try {
                motion = load_motion_config(motion_path);
            } catch (std::exception& e) {
                std::cerr << "motion.txt error: " << e.what() << "\n";
                return 2;
            }
        }
        bool move_requested = !motion_path.empty();
        bool simple_hold = move_requested &&
            motion.mode == "simple_joint_position_hold";
        bool legacy_advanced = move_requested &&
            motion.mode == "legacy_advanced";
        std::string out_file = !move_requested ? config::kRecordFile
            : timestamped_csv_name(simple_hold ? config::kSimpleHoldLogPrefix
                : legacy_advanced ? config::kControlLogPrefix
                                  : config::kMoveLogPrefix);

        // Connect once. Simple hold branches before Dynamics/FK: its complete
        // data path is measured joints -> target -> error -> position command.
        Connect connection(config::kRobotIp);
        connection.base()->ClearFaults();

        if (simple_hold) {
            std::cout << "mode: simple_joint_position_hold — fixed measured "
                         "startup pose, Kp=" << config::kSimpleHoldKp
                      << ", max command lead="
                      << config::kSimpleHoldMaxCommandLeadDeg
                      << " deg (Ctrl+C to stop)\nLogging every cycle -> "
                      << out_file << "\n";
            std::ofstream log(out_file);
            bool ok = run_simple_joint_position_hold(
                connection.base(), connection.base_cyclic(), g_stop, &log);
            return ok ? 0 : 1;
        }

        // The remaining modes retain the existing model/FK startup checks.
        Dynamics dynamics(GEN3_URDF_PATH);

        // Startup check: does our model agree with the robot?
        report_fk_vs_robot(dynamics, connection.base(),
                           connection.base_cyclic(), std::cout);

        // Controller groundwork (read-only): FK position vs the hard-coded
        // target in Config.h — prints target, current, and error once.
        report_position_error(dynamics, connection.base_cyclic(), std::cout);

        // If motion.txt was found, do the move instead of recording.
        // MOVES THE ARM: keep the workspace clear and the e-stop in hand.
        // Prints one labeled line of all 7 joint angles, e.g.
        // "joints at start (deg):  12.34  ...". Lambda: a tiny local
        // function; [&] lets it use `connection` from this scope.
        auto print_joints = [&](const std::string& label) {
            std::cout << label << " (deg):" << std::fixed << std::setprecision(2);
            for (double a : measure_joint_angles(connection.base_cyclic()))
                std::cout << "  " << a;
            std::cout << std::defaultfloat << "\n";
        };

        // Explicit legacy mode: preserved advanced task-space controller.
        // MOVES THE ARM continuously until Ctrl+C: workspace clear, e-stop
        // in hand.
        if (legacy_advanced) {
            std::cout << "mode: legacy_advanced — servoing EE to target ("
                      << config::kTargetPosition[0] << ", "
                      << config::kTargetPosition[1] << ", "
                      << config::kTargetPosition[2]
                      << ") m until Ctrl+C\nLogging every cycle -> "
                      << out_file << "\n";
            std::ofstream log(out_file);
            bool ok = run_reactive_control(connection.base(),
                                           connection.base_cyclic(),
                                           dynamics, g_stop, &log);
            return ok ? 0 : 1;
        }

        if (move_requested) {
            std::cout << "motion.txt found — moving joints (relative deg):";
            for (double d : motion.deltas) std::cout << " " << d;
            std::cout << "  (Ctrl+C to stop)\n";
            std::cout << "Logging every cycle -> " << out_file << "\n";
            print_joints("joints at start");
            std::ofstream log(out_file);
            bool ok = move_joints_relative(connection.base(), connection.base_cyclic(),
                                           motion.deltas, motion.speeds, g_stop, &log);
            print_joints("joints at end  ");
            std::cout << (ok ? "Move finished.\n" : "Move incomplete.\n");
            log.close();               // everything on disk before plotting
            plot_move_log(out_file);   // stats + PNGs from this run's log
            return ok ? 0 : 1;
        }

        // Record joint angles at a fixed rate until Ctrl+C.
        std::ofstream csv(out_file);
        std::cout << "Recording joints at " << config::kRecordRateHz << " Hz -> "
                  << out_file << "  (Ctrl+C to stop)\n";

        long samples = record_joint_angles(connection.base_cyclic(),
                                           csv, config::kRecordRateHz, g_stop);

        // Orderly shutdown (Connect's destructor closes the sessions).
        std::cout << "Stopped. " << samples << " samples written.\n";
    }
    catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
