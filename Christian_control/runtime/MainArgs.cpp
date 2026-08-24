#include "MainArgs.h"

#include <cmath>
#include <stdexcept>

namespace {
bool ParseOnOff(const std::string& flag, const std::string& value) {
    if (value == "on") return true;
    if (value == "off") return false;
    throw std::invalid_argument(flag + " must be 'on' or 'off' (got '" +
                                value + "')");
}
} // namespace

ParsedMainArgs ParseMainArgs(const std::vector<std::string>& args) {
    ParsedMainArgs parsed;
    bool fixed_pose_given = false;
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
        } else if (flag == "--mount") {
            parsed.mount = next();
        } else if (flag == "--plan") {
            parsed.plan = ParseOnOff(flag, next());
        } else if (flag == "--planner-artifacts") {
            parsed.planner_artifacts_dir = next();
            if (parsed.planner_artifacts_dir.empty())
                throw std::invalid_argument(
                    "--planner-artifacts requires a non-empty directory");
        } else if (flag == "--record") {
            parsed.record = ParseOnOff(flag, next());
        } else if (flag == "--fixed-pose") {
            fixed_pose_given = true;
            for (double& value : parsed.fixed_world_t_mount) {
                const std::string& text = next();
                std::size_t consumed = 0;
                try {
                    value = std::stod(text, &consumed);
                } catch (const std::exception&) {
                    consumed = 0;
                }
                if (consumed != text.size() || !std::isfinite(value))
                    throw std::invalid_argument(
                        "--fixed-pose needs 7 finite numbers "
                        "(x y z qx qy qz qw); '" + text + "' is not one");
            }
        } else {
            throw std::invalid_argument("unknown option '" + flag + "'");
        }
    }
    if (parsed.arm != "right" && parsed.arm != "left" && parsed.arm != "both")
        throw std::invalid_argument(
            "--arm is required and must be one of: right, left, both");
    if (parsed.mount != "fixed" && parsed.mount != "vicon")
        throw std::invalid_argument(
            "--mount is required and must be 'fixed' or 'vicon'");
    if (!parsed.planner_artifacts_dir.empty() && !parsed.plan)
        throw std::invalid_argument(
            "--planner-artifacts is only meaningful with --plan on");
    if (parsed.arm == "both" && !parsed.log_file.empty())
        throw std::invalid_argument(
            "--log is not valid with --arm both — each arm writes its own "
            "timestamped, prefixed default file");
    if (fixed_pose_given && parsed.mount != "fixed")
        throw std::invalid_argument(
            "--fixed-pose is only meaningful with --mount fixed");
    if (fixed_pose_given) {
        double norm2 = 0.0;
        for (int q = 3; q < 7; ++q)
            norm2 += parsed.fixed_world_t_mount[q] *
                     parsed.fixed_world_t_mount[q];
        const double norm = std::sqrt(norm2);
        if (std::abs(norm - 1.0) > 1e-3)
            throw std::invalid_argument(
                "--fixed-pose quaternion is not unit norm (|q| = " +
                std::to_string(norm) + ") — a rotation, not a typo, please");
        for (int q = 3; q < 7; ++q)
            parsed.fixed_world_t_mount[q] /= norm;
    }
    return parsed;
}
