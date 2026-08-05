#include "BridgeMain.h"

#include <array>
#include <climits>
#include <cmath>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "PlannerModel.h"
#include "PlanSolver.h"
#include "StartState.h"
#include "WorldSdf.h"
#include "Waypoints.h"

namespace {

constexpr char kUsageText[] =
    "usage: planner_bridge [--goal X Y Z | --goal-file PATH]\n"
    "                       [--state-csv PATH | --start-deg J1..J7]\n"
    "                       [--runs-root PATH] [--dh PATH]\n"
    "                       [--joint-limits PATH]\n"
    "                       [--box CX CY CZ HX HY HZ]\n"
    "\n"
    "  --goal X Y Z           Target tool position, metres, base_link.\n"
    "  --goal-file PATH       YAML goal file: `goal: [x, y, z]` (metres,\n"
    "                         base_link) plus an optional `box:` with\n"
    "                         `center:` and `half_extent:` lists. When\n"
    "                         neither --goal nor --goal-file is given, the\n"
    "                         default config/goal.yaml beside the\n"
    "                         executable's parent directory is read — so\n"
    "                         editing that one file is the normal way to\n"
    "                         choose where the arm goes.\n"
    "  --state-csv PATH       Start state: latest measured joint angles\n"
    "                         (meas_j1..meas_j7) read from a controller\n"
    "                         telemetry CSV.\n"
    "  --start-deg J1..J7     (test-only) Start state: seven joint angles,\n"
    "                         degrees, Kortex actuator order.\n"
    "  --runs-root PATH       Root of dated run-log directories to search\n"
    "                         when neither --state-csv nor --start-deg is\n"
    "                         given. Default: <repo>/runs resolved\n"
    "                         relative to the executable's directory.\n"
    "  --dh PATH               DH/tool parameters YAML. Default:\n"
    "                         config/dh_params_tool.yaml resolved\n"
    "                         relative to the executable's directory.\n"
    "  --joint-limits PATH    Joint limits YAML. Default:\n"
    "                         TrajectoryGeneration/config/joint_limits.yaml\n"
    "                         resolved relative to the executable's\n"
    "                         directory (../../.. up to the repo root).\n"
    "  --box CX CY CZ HX HY HZ  Optional axis-aligned obstacle box:\n"
    "                         centre and half-extents, metres, base_link.\n"
    "                         Must lie fully inside the SDF grid volume\n"
    "                         (WorldSdf.h WorldGridBounds()) or the run is\n"
    "                         rejected — outside that volume gpmp2 silently\n"
    "                         reports no obstacle.\n"
    "  --emit-orientation      Emit each waypoint as 7 fields\n"
    "                         (x y z qx qy qz qw) instead of 3. The\n"
    "                         controller's parser rejects 7-field lines\n"
    "                         while config::kAcceptOrientationTargets is\n"
    "                         false, so this is off by default.\n"
    "\n"
    "Exit codes: 0 targets emitted (also returned by --help), 1 bad\n"
    "arguments, 2 start-state unavailable, 3 solve failed, 4 validation\n"
    "rejected the plan.\n";

// Describes a GridBounds volume once, for both the --box rejection
// diagnostic and anywhere else the checked volume needs stating.
std::string DescribeGridBounds(const GridBounds& bounds) {
    std::ostringstream text;
    text << "x [" << bounds.min_m.x() << ", " << bounds.max_m.x() << "] m, "
         << "y [" << bounds.min_m.y() << ", " << bounds.max_m.y() << "] m, "
         << "z [" << bounds.min_m.z() << ", " << bounds.max_m.z() << "] m";
    return text.str();
}

// True when `box`'s full extent (center +/- half_extent, every axis) fits
// inside `bounds`. Used to reject a --box before it is ever handed to
// gpmp2, which returns zero obstacle cost for out-of-grid queries with no
// warning of its own.
bool BoxWithinGridBounds(const AxisAlignedBox& box, const GridBounds& bounds) {
    const Eigen::Vector3d box_min = box.center - box.half_extent;
    const Eigen::Vector3d box_max = box.center + box.half_extent;
    return (box_min.array() >= bounds.min_m.array()).all() &&
           (box_max.array() <= bounds.max_m.array()).all();
}

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

std::string DefaultGoalPath() {
    return ExecutableDirectory() + "/../config/goal.yaml";
}

std::string DefaultDhPath() {
    // config/ lives beside the CMakeLists.txt in planner_bridge/, one
    // level above the build/ directory the executable runs from — not
    // beside the executable itself.
    return ExecutableDirectory() + "/../config/dh_params_tool.yaml";
}

std::string DefaultJointLimitsPath() {
    return ExecutableDirectory() +
           "/../../../TrajectoryGeneration/config/joint_limits.yaml";
}

std::string DefaultRunsRootPath() {
    return ExecutableDirectory() + "/../../../runs";
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
    std::optional<std::string> goal_file;
    std::optional<std::string> state_csv;
    std::optional<std::array<double, 7>> start_deg;
    std::string dh_path = DefaultDhPath();
    std::string joint_limits_path = DefaultJointLimitsPath();
    std::string runs_root = DefaultRunsRootPath();
    std::optional<AxisAlignedBox> box;
    bool emit_orientation = false;
};

// Reads a YAML sequence of exactly three finite numbers into a vector,
// naming `what` in the error so the operator sees which key was malformed.
Eigen::Vector3d ReadVector3(const YAML::Node& node, const std::string& what) {
    if (!node || !node.IsSequence() || node.size() != 3)
        throw std::invalid_argument(what + " must be a list of three numbers");
    Eigen::Vector3d value;
    for (int axis = 0; axis < 3; ++axis) {
        value(axis) = node[static_cast<std::size_t>(axis)].as<double>();
        if (!std::isfinite(value(axis)))
            throw std::invalid_argument(what + " contains a non-finite number");
    }
    return value;
}

// Fills `parsed.goal` (and `parsed.box`, unless --box already set one) from
// a YAML goal file. Any failure — missing file, wrong shape, non-numeric
// value — throws std::invalid_argument naming the file, so a typo becomes
// a refusal to plan rather than a coordinate the arm accepts.
void LoadGoalFile(const std::string& path, ParsedArgs& parsed) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& error) {
        throw std::invalid_argument("cannot read goal file " + path + ": " +
                                    error.what());
    }
    try {
        parsed.goal = ReadVector3(root["goal"], "goal");
        if (root["box"] && !parsed.box) {
            AxisAlignedBox box;
            box.center = ReadVector3(root["box"]["center"], "box center");
            box.half_extent =
                ReadVector3(root["box"]["half_extent"], "box half_extent");
            parsed.box = box;
        }
    } catch (const std::exception& error) {
        throw std::invalid_argument("goal file " + path + ": " + error.what());
    }
}

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
        } else if (flag == "--goal-file") {
            parsed.goal_file = next();
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
        } else if (flag == "--runs-root") {
            parsed.runs_root = next();
        } else if (flag == "--box") {
            AxisAlignedBox box;
            box.center = Eigen::Vector3d(ParseDouble(next()), ParseDouble(next()),
                                          ParseDouble(next()));
            box.half_extent = Eigen::Vector3d(ParseDouble(next()), ParseDouble(next()),
                                               ParseDouble(next()));
            parsed.box = box;
        } else if (flag == "--emit-orientation") {
            parsed.emit_orientation = true;
        } else {
            throw std::invalid_argument("unrecognized flag: '" + flag + "'");
        }
    }
    if (parsed.goal && parsed.goal_file)
        throw std::invalid_argument(
            "at most one of --goal or --goal-file may be given");
    if (!parsed.goal)
        LoadGoalFile(parsed.goal_file ? *parsed.goal_file : DefaultGoalPath(),
                     parsed);
    if (parsed.state_csv && parsed.start_deg)
        throw std::invalid_argument(
            "at most one of --state-csv or --start-deg may be given");
    return parsed;
}

constexpr double kDegToRad = M_PI / 180.0;

// Redirects std::cout's stream buffer to another stream for the guard's
// lifetime, restoring the original buffer on destruction — including via
// an exception unwinding through the guarded scope. Used to keep the
// legacy optimizer's stdout chatter out of `targets` during the solve
// (see the call site below).
class CoutRedirectGuard {
public:
    explicit CoutRedirectGuard(std::ostream& to) : old_(std::cout.rdbuf(to.rdbuf())) {}
    ~CoutRedirectGuard() { std::cout.rdbuf(old_); }
    CoutRedirectGuard(const CoutRedirectGuard&) = delete;
    CoutRedirectGuard& operator=(const CoutRedirectGuard&) = delete;

private:
    std::streambuf* old_;
};

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

    if (parsed.box) {
        const GridBounds bounds = WorldGridBounds();
        if (!BoxWithinGridBounds(*parsed.box, bounds)) {
            diagnostics << "error: --box extends outside the SDF grid volume ("
                        << DescribeGridBounds(bounds) << "); gpmp2 reports no "
                        << "obstacle for out-of-grid queries, so this box "
                        << "cannot be honoured\n";
            return 1;
        }
    }

    Eigen::Matrix<double, 7, 1> q_start_rad;
    if (parsed.start_deg) {
        for (int joint = 0; joint < 7; ++joint)
            q_start_rad(joint) = (*parsed.start_deg)[joint] * kDegToRad;
    } else {
        std::string state_csv;
        if (parsed.state_csv) {
            state_csv = *parsed.state_csv;
        } else {
            std::string find_error;
            const std::optional<std::string> found =
                FindLatestRunCsv(parsed.runs_root, find_error);
            if (!found) {
                diagnostics << "error: no run log found under " << parsed.runs_root
                            << " — start the controller first (it creates the "
                            << "log), or pass --state-csv/--start-deg\n";
                return 2;
            }
            state_csv = *found;
        }
        std::string error;
        const std::optional<Eigen::Matrix<double, 7, 1>> q =
            ReadLatestMeasuredQ(state_csv, error);
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
    // `diagnostics` for the duration of the solve; the guard restores it
    // on scope exit, including if an exception unwinds through here.
    PlanOutcome outcome;
    {
        const CoutRedirectGuard cout_guard(diagnostics);
        outcome = SolveToPosition(model, request, parsed.joint_limits_path);
    }

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

    std::vector<Eigen::Quaterniond> rotations_xyzw;
    const std::vector<Eigen::Vector3d> waypoints = SampleCartesianWaypoints(
        model, outcome.result.trajectory_pos, /*max_count=*/8, /*min_spacing_m=*/0.05,
        parsed.emit_orientation ? &rotations_xyzw : nullptr);

    // Buffer everything and write to `targets` only once validation has
    // fully passed, so no non-zero exit path can leave partial output
    // there.
    std::ostringstream buffered_targets;
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        if (parsed.emit_orientation)
            buffered_targets << FormatTargetLine(waypoints[i], rotations_xyzw[i]) << "\n";
        else
            buffered_targets << FormatTargetLine(waypoints[i]) << "\n";
    }
    targets << buffered_targets.str();

    diagnostics << "waypoints: " << waypoints.size()
                << ", solve: " << outcome.result.optimization_duration.count()
                << " ms, final goal error: " << (outcome.final_goal_error_m * 1000.0)
                << " mm\n";
    return 0;
}
