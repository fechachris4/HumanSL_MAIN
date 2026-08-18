//
// print_joint_positions — READ-ONLY: connect to the arm, take one cyclic
// feedback reading, print each joint's live position, and exit. No motion,
// no writes, no servoing change.
//
//   ./print_joint_positions [--ip <address>]   (default: config::kRightRobotIp)
//

#include "Config.h"
#include "Hardware.h"
#include "ToolArgs.h"

#include <iomanip>
#include <iostream>

namespace k_api = Kinova::Api;

int main(int argc, char** argv)
{
    const std::string ip = ParseIpArg(argc, argv, "print_joint_positions");
    try
    {
        Connect connection(ip);

        // The ONLY RefreshFeedback call this tool makes. It pulls one
        // snapshot from the 1 kHz cyclic (UDP) channel; it does not open a
        // continuous stream and sends no command.
        const k_api::BaseCyclic::Feedback fb =
            connection.base_cyclic()->RefreshFeedback();

        std::cout << "arm state "
                  << k_api::Common::ArmState_Name(static_cast<k_api::Common::ArmState>(
                         fb.base().active_state()))
                  << "\n";
        for (int i = 0; i < fb.actuators_size(); ++i)
        {
            std::cout << "  joint " << (i + 1) << "  position "
                      << std::fixed << std::setprecision(2)
                      << fb.actuators(i).position() << " deg\n"
                      << std::defaultfloat;
        }
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
