//
// print_tool_configuration — READ-ONLY: print the robot's configured Tool
// Frame (Kortex ControlConfig::GetToolConfiguration). This is the offset
// firmware adds on top of the physical flange to compute the tool_pose_*
// fields in cyclic feedback — the same numbers the Kinova web dashboard
// displays. No URDF encodes this: it is a setting stored on the robot
// (e.g. for a mounted gripper or a manually calibrated TCP).
//
// Purpose: find out directly whether a nonzero Tool Frame explains the gap
// between the dashboard's tool_pose and this repo's Pinocchio FK, which
// computes the pose at the URDF's modeled tool offset instead. No motion,
// no writes, no servoing change.
//
//   ./print_tool_configuration [--ip <address>]   (default: config::kRightRobotIp)
//

#include "Config.h"
#include "Hardware.h"
#include "ToolArgs.h"

#include <iomanip>
#include <iostream>

namespace k_api = Kinova::Api;

int main(int argc, char** argv)
{
    const std::string ip = ParseIpArg(argc, argv, "print_tool_configuration");
    try
    {
        Connect connection(ip);

        const k_api::ControlConfig::ToolConfiguration tool =
            connection.control_config()->GetToolConfiguration();

        std::cout << std::fixed << std::setprecision(6);
        if (tool.has_tool_transform())
        {
            const auto& t = tool.tool_transform();
            std::cout << "configured tool_transform (flange -> tool_pose "
                          "origin):\n"
                      << "  xyz:   " << t.x() << " " << t.y() << " " << t.z()
                      << " (m)\n"
                      << "  theta: " << t.theta_x() << " " << t.theta_y()
                      << " " << t.theta_z() << " (deg)\n";
            const bool nonzero =
                std::abs(t.x()) > 1e-6 || std::abs(t.y()) > 1e-6 ||
                std::abs(t.z()) > 1e-6 || std::abs(t.theta_x()) > 1e-6 ||
                std::abs(t.theta_y()) > 1e-6 || std::abs(t.theta_z()) > 1e-6;
            std::cout << (nonzero
                              ? "  NONZERO — the dashboard's tool_pose is "
                                "offset from this repo's FK (EndEffector_Link) "
                                "by roughly this transform.\n"
                              : "  all zero — the dashboard should read "
                                "the same point as this repo's FK "
                                "(EndEffector_Link); the gap has another "
                                "cause.\n");
        }
        else
        {
            std::cout << "no tool_transform reported in the response\n";
        }

        if (tool.has_tool_mass_center())
        {
            const auto& c = tool.tool_mass_center();
            std::cout << "tool_mass_center: " << c.x() << " " << c.y() << " "
                      << c.z() << " (m)\n";
        }
        std::cout << "tool_mass: " << tool.tool_mass() << " (kg)\n";
        return 0;
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex API error: " << ex.what() << "\n";
        return 1;
    }
    catch (std::exception& ex)
    {
        std::cout << "error: " << ex.what() << "\n";
        return 1;
    }
}
