//
// ToolArgs — the --ip <address> flag shared by every hardware bring-up tool
// in tools/: which physical arm's IP to connect to. Defaults to
// config::kRightRobotIp, so every existing invocation (no --ip) is
// unchanged; pass --ip config::kLeftRobotIp (or any address) to point the
// same read-only/config diagnostic at the other arm.
//

#pragma once

#include <cstdlib>
#include <iostream>
#include <string>

#include "Config.h"

inline std::string ParseIpArg(int argc, char** argv, const char* usage_name)
{
    std::string ip = config::kRightRobotIp;
    for (int i = 1; i < argc; ++i)
    {
        const std::string flag = argv[i];
        if (flag == "--ip")
        {
            if (++i >= argc)
            {
                std::cerr << "error: --ip needs an address\n"
                          << "usage: " << usage_name << " [--ip <address>]\n";
                std::exit(2);
            }
            ip = argv[i];
        }
        else
        {
            std::cerr << "error: unknown option '" << flag << "'\n"
                      << "usage: " << usage_name << " [--ip <address>]\n";
            std::exit(2);
        }
    }
    return ip;
}
