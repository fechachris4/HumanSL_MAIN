#include "MainArgs.h"

#include <stdexcept>

ParsedMainArgs ParseMainArgs(const std::vector<std::string>& args) {
    ParsedMainArgs parsed;
    std::size_t i = 0;
    const auto next = [&]() -> const std::string& {
        if (i >= args.size())
            throw std::invalid_argument("missing value after flag");
        return args[i++];
    };
    while (i < args.size()) {
        const std::string flag = args[i++];
        if (flag == "--arm") {
            parsed.arm = next();
        } else if (flag == "--log") {
            parsed.log_file = next();
        } else {
            throw std::invalid_argument("unknown option '" + flag + "'");
        }
    }
    if (parsed.arm != "right" && parsed.arm != "left" && parsed.arm != "both")
        throw std::invalid_argument(
            "--arm is required and must be one of: right, left, both");
    if (parsed.arm == "both" && !parsed.log_file.empty())
        throw std::invalid_argument(
            "--log is not valid with --arm both — each arm writes its own "
            "timestamped, prefixed default file");
    return parsed;
}
