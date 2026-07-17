/*
 * query_limits — READ-ONLY: print the robot's kinematic limits.
 *
 * Connects to the arm, then prints the hard limits and the per-control-mode
 * soft limits the base enforces (joint speeds in deg/s). Never sends motion
 * commands and ignores config::kStartupMode entirely.
 *
 * Usage: ./query_limits   (no flags — robot IP comes from Config.h)
 */

#include <iostream>
#include <string>

#include <ControlConfigClientRpc.h>

#include "Config.h"
#include "Connect.h"

namespace ctl = Kinova::Api::ControlConfig;

static void print_limits(const ctl::KinematicLimits& limits, const std::string& label)
{
    std::cout << label << "\n  joint speeds (deg/s): ";
    for (int i = 0; i < limits.joint_speed_limits_size(); ++i)
        std::cout << " " << limits.joint_speed_limits(i);
    std::cout << "\n  joint accels (deg/s^2):";
    for (int i = 0; i < limits.joint_acceleration_limits_size(); ++i)
        std::cout << " " << limits.joint_acceleration_limits(i);
    std::cout << "\n  cartesian: " << limits.twist_linear() << " m/s, " << limits.twist_angular()
              << " deg/s\n";
}

int main()
{
    try {
        Connect connection(config::kRobotIp);
        ctl::ControlConfigClient config(connection.router());

        print_limits(config.GetKinematicHardLimits(), "HARD limits (robot capability):");

        // Soft limits differ per control mode; print each mode the arm reports.
        ctl::KinematicLimitsList all = config.GetAllKinematicSoftLimits();
        for (int m = 0; m < all.kinematic_limits_list_size(); ++m) {
            const auto& lim = all.kinematic_limits_list(m);
            print_limits(lim, "SOFT limits, control mode " +
                                  ctl::ControlMode_Name(lim.control_mode()) + ":");
        }
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
