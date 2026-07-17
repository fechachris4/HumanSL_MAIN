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
#include "Kinematics.h"
#include "Measure.h"
#include "Record.h"
#include "Motion.h"
#include "Dynamics.h"

// Ctrl+C sets this flag; loops check it and exit cleanly.
std::atomic<bool> g_stop{false};
void on_sigint(int)
{
    g_stop = true;
}

int main()
{
    std::signal(SIGINT, on_sigint);

    try {
        // Startup mode is a compiled-in constant (config::kStartupMode,
        // Config.h) — set it there and rebuild; no file or flag needed.
        constexpr bool joints_move = config::kStartupMode == config::StartupMode::kJoints;
        std::cout << "startup mode: " << (joints_move ? "joints" : "record")
                  << " (config::kStartupMode)\n";
        std::string out_file =
            joints_move ? timestamped_csv_name(config::kMoveLogPrefix) : config::kRecordFile;

        // Load the robot model and connect (both channels, once).
        Dynamics dynamics(GEN3_URDF_PATH);
        Connect connection(config::kRobotIp);
        connection.base()->ClearFaults();

        // Startup check: does our model agree with the robot?
        report_fk_vs_robot(dynamics, connection.base(), connection.base_cyclic(), std::cout);

        // If kStartupMode is kJoints, do the move instead of recording.
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

        if (joints_move) {
            std::cout << "joints mode — moving joints (relative deg):";
            for (double d : config::kJointDeltasDeg)
                std::cout << " " << d;
            std::cout << "  (Ctrl+C to stop)\n";
            std::cout << "Logging every cycle -> " << out_file << "\n";
            print_joints("joints at start");
            std::ofstream log(out_file);
            bool ok = move_joints_relative(connection.base(), connection.base_cyclic(),
                                           config::kJointDeltasDeg, config::kJointSpeedsDegS,
                                           g_stop, &log);
            print_joints("joints at end  ");
            std::cout << (ok ? "Move finished.\n" : "Move incomplete.\n");
            log.close();             // everything on disk before plotting
            plot_move_log(out_file); // stats + PNGs from this run's log
            return ok ? 0 : 1;
        }

        // Record joint angles at a fixed rate until Ctrl+C.
        std::ofstream csv(out_file);
        std::cout << "Recording joints at " << config::kRecordRateHz << " Hz -> " << out_file
                  << "  (Ctrl+C to stop)\n";

        long samples =
            record_joint_angles(connection.base_cyclic(), csv, config::kRecordRateHz, g_stop);

        // Orderly shutdown (Connect's destructor closes the sessions).
        std::cout << "Stopped. " << samples << " samples written.\n";
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
