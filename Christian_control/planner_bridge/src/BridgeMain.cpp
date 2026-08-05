#include "BridgeMain.h"

#include <array>
#include <climits>
#include <cmath>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <unistd.h>

#include "PlannerModel.h"
#include "PlanSolver.h"
#include "StartState.h"
#include "WorldSdf.h"
#include "Waypoints.h"

namespace {

constexpr char kUsageText[] =
    "usage: planner_bridge --goal X Y Z (--state-csv PATH | --start-deg J1..J7)\n"
    "                       [--dh PATH] [--joint-limits PATH]\n"
    "                       [--box CX CY CZ HX HY HZ]\n"
    "\n"
    "  --goal X Y Z           Target tool position, metres, base_link.\n"
    "  --state-csv PATH       Start state: latest measured joint angles\n"
    "                         (meas_j1..meas_j7) read from a controller\n"
    "                         telemetry CSV.\n"
    "  --start-deg J1..J7     Start state: seven joint angles, degrees,\n"
    "                         Kortex actuator order. Exactly one of\n"
    "                         --state-csv or --start-deg is required.\n"
    "  --dh PATH               DH/tool parameters YAML. Default:\n"
    "                         config/dh_params_tool.yaml resolved\n"
    "                         relative to the executable's directory.\n"
    "  --joint-limits PATH    Joint limits YAML. Default:\n"
    "                         TrajectoryGeneration/config/joint_limits.yaml\n"
    "                         resolved relative to the executable's\n"
    "                         directory (../../.. up to the repo root).\n"
    "  --box CX CY CZ HX HY HZ  Optional axis-aligned obstacle box:\n"
    "                         centre and half-extents, metres, base_link.\n"
    "\n"
    "Exit codes: 0 targets emitted, 1 bad arguments, 2 start-state\n"
    "unavailable, 3 solve failed, 4 validation rejected the plan.\n";

// Directory containing the running executable, via /proc/self/exe. Falls
// back to "." if the link cannot be read (e.g. non-Linux, sandboxed exec).
std::string ExecutableDirectory() {
    std::array<char, PATH_MAX> buffer{};
    const ssize_t length = readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
    if (length <= 0)
        return ".";
    const std::string path(buffer.data(), static_cast<std::size_t>(length));
    const auto slash = path.find_last_of('/');
    return slash == std::string::npos ? "." : path.substr(0, slash);
}

std::string DefaultDhPath() {
    return ExecutableDirectory() + "/config/dh_params_tool.yaml";
}

std::string DefaultJointLimitsPath() {
    return ExecutableDirectory() +
           "/../../../TrajectoryGeneration/config/joint_limits.yaml";
}

// std::stod that rejects trailing garbage and non-finite results, so
// "not-a-number" or "1.2xyz" fail instead of silently truncating.
double ParseDouble(const std::string& token) {
    std::size_t consumed = 0;
    const double value = std::stod(token, &consumed);
    if (consumed != token.size() || !std::isfinite(value))
        throw std::invalid_argument("not a finite number: '" + token + "'");
    return value;
}

struct ParsedArgs {
    std::optional<Eigen::Vector3d> goal;
    std::optional<std::string> state_csv;
    std::optional<std::array<double, 7>> start_deg;
    std::string dh_path = DefaultDhPath();
    std::string joint_limits_path = DefaultJointLimitsPath();
    std::optional<AxisAlignedBox> box;
};

// Throws std::invalid_argument / std::out_of_range on any malformed input;
// RunBridge turns that into exit code 1 with the usage text.
ParsedArgs ParseArgs(const std::vector<std::string>& args) {
    ParsedArgs parsed;
    std::size_t i = 0;
    const auto next = [&]() -> const std::string& {
        if (i >= args.size())
            throw std::invalid_argument("missing value after flag");
        return args[i++];
    };
    while (i < args.size()) {
        const std::string flag = args[i++];
        if (flag == "--goal") {
            const double x = ParseDouble(next());
            const double y = ParseDouble(next());
            const double z = ParseDouble(next());
            parsed.goal = Eigen::Vector3d(x, y, z);
        } else if (flag == "--state-csv") {
            parsed.state_csv = next();
        } else if (flag == "--start-deg") {
            std::array<double, 7> degrees{};
            for (double& d : degrees) d = ParseDouble(next());
            parsed.start_deg = degrees;
        } else if (flag == "--dh") {
            parsed.dh_path = next();
        } else if (flag == "--joint-limits") {
            parsed.joint_limits_path = next();
        } else if (flag == "--box") {
            AxisAlignedBox box;
            box.center = Eigen::Vector3d(ParseDouble(next()), ParseDouble(next()),
                                          ParseDouble(next()));
            box.half_extent = Eigen::Vector3d(ParseDouble(next()), ParseDouble(next()),
                                               ParseDouble(next()));
            parsed.box = box;
        } else {
            throw std::invalid_argument("unrecognized flag: '" + flag + "'");
        }
    }
    if (!parsed.goal)
        throw std::invalid_argument("--goal is required");
    if (parsed.state_csv.has_value() == parsed.start_deg.has_value())
        throw std::invalid_argument(
            "exactly one of --state-csv or --start-deg is required");
    return parsed;
}

constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

int RunBridge(const std::vector<std::string>& args, std::ostream& targets,
              std::ostream& diagnostics) {
    for (const std::string& arg : args) {
        if (arg == "--help" || arg == "-h") {
            diagnostics << kUsageText;
            return 0;
        }
    }

    ParsedArgs parsed;
    try {
        parsed = ParseArgs(args);
    } catch (const std::exception& error) {
        diagnostics << "error: " << error.what() << "\n\n" << kUsageText;
        return 1;
    }

    Eigen::Matrix<double, 7, 1> q_start_rad;
    if (parsed.start_deg) {
        for (int joint = 0; joint < 7; ++joint)
            q_start_rad(joint) = (*parsed.start_deg)[joint] * kDegToRad;
    } else {
        std::string error;
        const std::optional<Eigen::Matrix<double, 7, 1>> q =
            ReadLatestMeasuredQ(*parsed.state_csv, error);
        if (!q) {
            diagnostics << "error: start state unavailable: " << error << "\n";
            return 2;
        }
        q_start_rad = *q;
    }

    PlannerModel model;
    try {
        model = LoadPlannerModel(parsed.dh_path);
    } catch (const std::exception& error) {
        diagnostics << "error: solve failed: could not load planner model from "
                    << parsed.dh_path << ": " << error.what() << "\n";
        return 3;
    }

    PlanRequest request;
    request.q_start_rad = q_start_rad;
    request.goal_position_m = *parsed.goal;
    request.obstacle = parsed.box;

    // The legacy optimizer prints its own progress chatter straight to
    // std::cout. In the real binary `targets` IS std::cout (see main.cpp),
    // piped directly into the controller's stdin over the operator FIFO,
    // so that chatter must never reach it. Redirect std::cout into
    // `diagnostics` for the duration of the solve and restore it
    // afterwards, whatever the outcome.
    std::streambuf* const saved_cout_buf = std::cout.rdbuf(diagnostics.rdbuf());
    const PlanOutcome outcome =
        SolveToPosition(model, request, parsed.joint_limits_path);
    std::cout.rdbuf(saved_cout_buf);

    if (!outcome.ok) {
        diagnostics << "error: solve failed: " << outcome.error << "\n";
        return 3;
    }

    const std::optional<std::string> validation_error =
        ValidateJointPath(outcome.result.trajectory_pos);
    if (validation_error) {
        diagnostics << "error: plan rejected: " << *validation_error << "\n";
        return 4;
    }

    const std::vector<Eigen::Vector3d> waypoints =
        SampleCartesianWaypoints(model, outcome.result.trajectory_pos);

    // Buffer everything and write to `targets` only once validation has
    // fully passed, so no non-zero exit path can leave partial output
    // there.
    std::ostringstream buffered_targets;
    for (const Eigen::Vector3d& waypoint : waypoints)
        buffered_targets << FormatTargetLine(waypoint) << "\n";
    targets << buffered_targets.str();

    diagnostics << "waypoints: " << waypoints.size()
                << ", solve: " << outcome.result.optimization_duration.count()
                << " ms, final goal error: " << (outcome.final_goal_error_m * 1000.0)
                << " mm\n";
    return 0;
}
