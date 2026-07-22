/*
 * query_limits — READ-ONLY: print the robot's kinematic hard limits.
 *
 * Connects to the arm, then prints its reported hard limits. Never sends
 * motion commands.
 *
 * Usage: ./query_limits   (no flags — robot IP comes from Config.h)
 */

#include <iostream>
#include <string>

#include <ControlConfigClientRpc.h>

#include "app/Config.h"
#include "hardware/Connect.h"

namespace ctl = Kinova::Api::ControlConfig;

namespace
{
    void print_limits(const ctl::KinematicLimits& limits, const std::string& label)
    {
        std::cout << label << "\n  joint speeds (deg/s): ";
        for (int i = 0; i < limits.joint_speed_limits_size(); ++i)
            std::cout << " " << limits.joint_speed_limits(i);
        std::cout << "\n  joint accels (deg/s^2):";
        for (int i = 0; i < limits.joint_acceleration_limits_size(); ++i)
            std::cout << " " << limits.joint_acceleration_limits(i);
        std::cout << "\n  cartesian: " << limits.twist_linear() << " m/s, "
                  << limits.twist_angular() << " deg/s\n";
    }
} // namespace

int main()
{
    try {
        Connect connection(config::kRobotIp);
        ctl::ControlConfigClient config(connection.router());

        print_limits(config.GetKinematicHardLimits(), "HARD limits (robot capability):");
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
