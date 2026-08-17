#include "PlannerRuntime.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <climits>
#include <cmath>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "PlannerConfig.h"
#include "PlannerModel.h"
#include "PlanSolver.h"
#include "StartState.h"
#include "WorldTrajectoryProjection.h"
#include "WorldSdf.h"
#include "PathFrames.h"
#include "PathValidation.h"
#include "PinocchioKinematicsAdapter.h"
#include "Config.h"   // basic_control — config::kReferenceFrame

namespace {

// The inherited optimizer writes progress to process-global std::cout. Keep
// concurrent arm workers from redirecting that stream over one another. This
// mutex is touched only by non-real-time planner calls; the 500 Hz path never
// waits on it.
std::mutex g_planner_solve_mutex;

constexpr char kUsageText[] =
    "usage: planner_bridge --arm <right|left>\n"
    "                       [--goal X Y Z | --goal-file PATH]\n"
    "                       [--state-csv PATH | --start-deg J1..J7]\n"
    "                       [--runs-root PATH] [--dh PATH]\n"
    "                       [--joint-limits PATH]\n"
    "                       [--box CX CY CZ HX HY HZ]\n"
    "                       [--output world-cartesian]\n"
    "                       [--world-mount-pose-m-quat PX PY PZ QX QY QZ QW\n"
    "                        --vicon-sequence N --trajectory-id N]\n"
    "\n"
    "Output (stdout): one versioned CART_TRAJ world-frame pose/twist block.\n"
    "GPMP2 remains joint-space internally; planned q/qdot never cross the\n"
    "controller boundary.\n"
    "\n"
    "  --arm <right|left>     Required — which physical arm this plan is\n"
    "                         for. Selects the default DH file (right:\n"
    "                         dh_params_tool.yaml, mounted-tool collision\n"
    "                         model; left: dh_params_flange.yaml, bare-\n"
    "                         flange collision model), which run log this\n"
    "                         process reads the start state from, and which\n"
    "                         arm's base frame --box must be given in. No\n"
    "                         default: every run states its target arm.\n"
    "  --goal X Y Z           Target tool position, metres, in the\n"
    "                         compiled config::kReferenceFrame.\n"
    "  --goal-file PATH       YAML goal file, ARM-KEYED: a top-level `right:`\n"
    "                         and/or `left:` block (only the one matching\n"
    "                         --arm is read), each with its own `goal:\n"
    "                         [x, y, z]` metres, an optional `frame:` (mount,\n"
    "                         right_base or left_base; default\n"
    "                         config::kReferenceFrame) governing that block\n"
    "                         whole, and an optional `box:` with `center:`\n"
    "                         and `half_extent:` lists. One file can hold\n"
    "                         both arms' targets for a --arm both session\n"
    "                         without either reading the other's numbers.\n"
    "                         When neither --goal nor --goal-file is given,\n"
    "                         the default config/goal.yaml beside the\n"
    "                         executable's parent directory is read — so\n"
    "                         editing that one file is the normal way to\n"
    "                         choose where each arm goes.\n"
    "  --state-csv PATH       Start state: latest measured joint angles\n"
    "                         (meas_j1..meas_j7) read from a controller\n"
    "                         telemetry CSV.\n"
    "  --start-deg J1..J7     (test-only) Start state: seven joint angles,\n"
    "                         degrees, Kortex actuator order.\n"
    "  --runs-root PATH       Root of dated run-log directories to search\n"
    "                         when neither --state-csv nor --start-deg is\n"
    "                         given (the newest loop_log_<arm>*.csv under\n"
    "                         it, matching --arm). Default: <repo>/runs\n"
    "                         resolved relative to the executable's\n"
    "                         directory.\n"
    "  --dh PATH               DH parameters YAML. Default: the\n"
    "                         build-generated config/dh_params_tool.yaml\n"
    "                         (--arm right) or config/dh_params_flange.yaml\n"
    "                         (--arm left) beside the executable (derived\n"
    "                         from the URDF at build time — do not\n"
    "                         hand-edit).\n"
    "  --joint-limits PATH    Joint limits YAML. Default:\n"
    "                         config/joint_limits.yaml beside\n"
    "                         config/goal.yaml, resolved relative to the\n"
    "                         executable's directory.\n"
    "  --planner-config PATH  Planner tuning YAML: plan pacing and every\n"
    "                         factor-graph weight. Default: config/\n"
    "                         planner.yaml beside config/goal.yaml. Every\n"
    "                         key is required and unknown keys are refused,\n"
    "                         so a typo fails the run naming the key rather\n"
    "                         than silently planning something else. The\n"
    "                         effective values and the file's digest are\n"
    "                         echoed here on every run.\n"
    "  --box CX CY CZ HX HY HZ  Optional axis-aligned obstacle box:\n"
    "                         centre and half-extents, metres, in the\n"
    "                         --arm-selected arm's own base frame only\n"
    "                         (see --goal-file). Must lie fully inside the\n"
    "                         SDF grid volume (WorldSdf.h WorldGridBounds())\n"
    "                         or the run is rejected — outside that volume\n"
    "                         gpmp2 silently reports no obstacle.\n"
    "  --output MODE          Optional compatibility spelling; the only\n"
    "                         accepted mode is `world-cartesian`, also the\n"
    "                         default and sole output.\n"
    "  --world-mount-pose-m-quat PX PY PZ QX QY QZ QW\n"
    "                         Immutable Vicon T_W_M snapshot: translation in\n"
    "                         metres and unit quaternion x y z w. Required\n"
    "                         for every plan.\n"
    "  --vicon-sequence N     Vicon frame sequence associated with T_W_M.\n"
    "                         Required for every plan.\n"
    "  --trajectory-id N      Caller-assigned trajectory identity. Required\n"
    "                         for every plan.\n"
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
// The upper comparison is STRICT. gpmp2 accepts a query landing exactly on
// origin + (n-1)*cell but cannot interpolate there — it reads one sample past
// the end (see WorldGridBounds in WorldSdf.cpp). The lower face has no such
// problem, so only the upper one is exclusive.
bool BoxWithinGridBounds(const AxisAlignedBox& box, const GridBounds& bounds) {
    const Eigen::Vector3d box_min = box.center - box.half_extent;
    const Eigen::Vector3d box_max = box.center + box.half_extent;
    return (box_min.array() >= bounds.min_m.array()).all() &&
           (box_max.array() < bounds.max_m.array()).all();
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

std::string DefaultDhPath(bool left_arm) {
    // The DH YAML is GENERATED from the URDF at build time into the build
    // tree's config/ directory, beside the executable (see
    // generate_dh_params in CMakeLists.txt). There is no committed copy.
    // Two files, one per arm's own chain — dh_params_tool.yaml (right, ends
    // at the mounted tool) and dh_params_flange.yaml (left, bare flange).
    return ExecutableDirectory() + "/config/" +
           (left_arm ? "dh_params_flange.yaml" : "dh_params_tool.yaml");
}

std::string DefaultJointLimitsPath() {
    return ExecutableDirectory() + "/../config/joint_limits.yaml";
}

// Beside goal.yaml, and resolved the same way — from the executable, never
// from the working directory, so which file configures a run never depends
// on where it was started from (docs/decisions/runtime-config.md).
std::string DefaultPlannerConfigPath() {
    return ExecutableDirectory() + "/../config/planner.yaml";
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

std::uint64_t ParseUint64(const std::string& token) {
    std::uint64_t value = 0;
    const char* const begin = token.data();
    const char* const end = begin + token.size();
    const auto result = std::from_chars(begin, end, value);
    if (token.empty() || result.ec != std::errc() || result.ptr != end)
        throw std::invalid_argument("not an unsigned integer: '" + token + "'");
    return value;
}

struct ParsedArgs {
    std::optional<Eigen::Vector3d> goal;
    // The frame `goal` and `box` were written in, before conversion. Set from
    // a goal file's `frame:` key, otherwise config::kReferenceFrame.
    config::ReferenceFrame frame = config::kReferenceFrame;
    std::optional<std::string> goal_file;
    std::optional<std::string> state_csv;
    std::optional<std::array<double, 7>> start_deg;
    // Required, no default: which physical arm this plan is for. unset ==
    // --arm was never given, refused by RunBridge before anything else runs.
    std::optional<bool> left_arm;
    // Goal orientation as roll/pitch/yaw in the block's declared frame,
    // before conversion. Unset = inherit the start pose's orientation.
    std::optional<Eigen::Vector3d> goal_rpy_rad;
    // A traced path instead of a point goal. Mutually exclusive with
    // `goal` — a block naming both is refused rather than one silently
    // winning, because which one won would not be visible in any output.
    std::optional<CircleSpec> circle;
    // unset == use DefaultDhPath(*left_arm), resolved once --arm is known
    // (its default depends on left_arm, so it cannot be a member initializer).
    std::optional<std::string> dh_path;
    std::string joint_limits_path = DefaultJointLimitsPath();
    std::string planner_config_path = DefaultPlannerConfigPath();
    std::string runs_root = DefaultRunsRootPath();
    std::optional<AxisAlignedBox> box;
    std::optional<Eigen::Isometry3d> world_T_mount;
    std::optional<std::uint64_t> vicon_sequence;
    std::optional<std::uint64_t> trajectory_id;
};


// ---------------------------------------------------------------
// Frame boundary
// ---------------------------------------------------------------
//
// The planner is Vicon `world` internally, everywhere: the gpmp2 arm model and the
// SDF are paired in one ObstacleSDFFactorArm, so they must share a frame or
// every collision check is silently wrong — and since PlannerModel builds the
// arm at DhRootInWorld(), that shared frame is world for both arms. Input
// written in mount or an arm's base frame is converted here once at the edge;
// input already in world passes through untouched.
//
// The transform comes from the URDF through Pinocchio, never from a constant
// in this file, so surveying the rig and regenerating the URDF needs no code
// change. Every production and one-shot solve receives an explicit immutable
// Vicon T_W_M snapshot at this application boundary.

const char* FrameName(config::ReferenceFrame frame) {
    return config::kReferenceFrameNames[static_cast<int>(frame)];
}

config::ReferenceFrame FrameFromName(const std::string& name) {
    for (int i = 0; i < 4; ++i)
        if (name == config::kReferenceFrameNames[i])
            return static_cast<config::ReferenceFrame>(i);
    throw std::invalid_argument(
        "unknown frame '" + name +
        "' (expected mount, right_base, left_base or world)");
}

// Declared frame -> Vicon world, the one frame everything downstream uses.
Eigen::Vector3d ToWorld(const Eigen::Vector3d& point,
                        config::ReferenceFrame frame,
                        const Eigen::Isometry3d& world_T_mount) {
    if (frame == config::ReferenceFrame::kWorld)
        return point;
    if (frame == config::ReferenceFrame::kMount)
        return world_T_mount * point;
    const bool declared_left_arm = frame == config::ReferenceFrame::kLeftBase;
    return world_T_mount *
           pinocchio_kinematics_adapter::MountFromBase(declared_left_arm) * point;
}

// Rotation matrix from roll/pitch/yaw, R = Rz*Ry*Rx — the convention
// FramePrint.h prints and the controller's orientation line uses, so a
// number read off a diagnostic can be pasted straight into a goal file.
Eigen::Matrix3d RotationFromRpy(const Eigen::Vector3d& rpy_rad) {
    return (Eigen::AngleAxisd(rpy_rad.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy_rad.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy_rad.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

// The inverse of RotationFromRpy, so an orientation can be echoed back in
// the frame it was converted INTO rather than the one it was written in.
Eigen::Vector3d RpyFromRotation(const Eigen::Matrix3d& rotation) {
    return rotation.eulerAngles(2, 1, 0).reverse();  // R = Rz*Ry*Rx
}

// Declared frame -> world, for an ORIENTATION. Unlike a point (ToWorld), a
// rotation carries no translation, so only the rotational parts compose.
// Getting this wrong is silent — the goal still looks like a valid rotation.
Eigen::Matrix3d RotationToWorld(const Eigen::Matrix3d& rotation,
                                config::ReferenceFrame frame,
                                const Eigen::Isometry3d& world_T_mount) {
    if (frame == config::ReferenceFrame::kWorld)
        return rotation;
    if (frame == config::ReferenceFrame::kMount)
        return world_T_mount.linear() * rotation;
    const bool declared_left_arm = frame == config::ReferenceFrame::kLeftBase;
    return world_T_mount.linear() *
           pinocchio_kinematics_adapter::MountFromBase(declared_left_arm).linear() *
           rotation;
}

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
// a YAML goal file. Goal files are ARM-KEYED: one top-level block per arm
// ("right:" and/or "left:"), each with its own goal/frame/box, so one file
// can hold both arms' targets for a --arm both session without either
// silently reading the other's numbers. This function reads only the block
// matching `left_arm`; the other block (if present) is untouched. Any
// failure — missing file, missing arm block, wrong shape, non-numeric value
// — throws std::invalid_argument naming the file, so a typo becomes a
// refusal to plan rather than a coordinate the arm accepts.
void LoadGoalFile(const std::string& path, ParsedArgs& parsed, bool left_arm) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& error) {
        throw std::invalid_argument("cannot read goal file " + path + ": " +
                                    error.what());
    }
    const std::string arm_key = left_arm ? "left" : "right";
    try {
        const YAML::Node arm_node = root[arm_key];
        if (!arm_node)
            throw std::invalid_argument(
                "no '" + arm_key + ":' block — goal files are arm-keyed: a "
                "top-level 'right:' and/or 'left:' block, each with its own "
                "goal/frame/box");
        // One `frame:` per arm block governs that block whole — goal and box
        // alike, so a block can never end up half-converted. Omitted means
        // the compiled config::kReferenceFrame, which is also what a bare
        // --goal uses.
        if (arm_node["frame"])
            parsed.frame = FrameFromName(arm_node["frame"].as<std::string>());
        // `goal:` and `path:` are mutually exclusive. Refusing both-present
        // matters more than it looks: silently preferring one would make the
        // arm trace something the file also appears to ask against.
        const bool has_goal = static_cast<bool>(arm_node["goal"]);
        const bool has_path = static_cast<bool>(arm_node["path"]);
        if (has_goal && has_path)
            throw std::invalid_argument(
                "block has BOTH 'goal:' and 'path:' — they are mutually "
                "exclusive; a point goal and a traced path are different "
                "requests");
        if (!has_goal && !has_path)
            throw std::invalid_argument(
                "block has neither 'goal:' nor 'path:' — one is required");

        if (has_path) {
            const YAML::Node path_node = arm_node["path"];
            const std::string type =
                path_node["type"] ? path_node["type"].as<std::string>() : "";
            if (type != "circle")
                throw std::invalid_argument(
                    "path.type must be 'circle' (got '" + type +
                    "'); other shapes use the same CartesianPath pipeline but "
                    "have no generator yet");
            CircleSpec circle;
            circle.centre_m = ReadVector3(path_node["centre"], "path.centre");
            circle.radius_m = path_node["radius_m"]
                                  ? path_node["radius_m"].as<double>()
                                  : throw std::invalid_argument("path.radius_m is required");
            if (!(circle.radius_m > 0.0) || !std::isfinite(circle.radius_m))
                throw std::invalid_argument("path.radius_m must be finite and positive");
            circle.normal = ReadVector3(path_node["normal"], "path.normal");
            if (circle.normal.norm() < 1e-9)
                throw std::invalid_argument(
                    "path.normal is degenerate — it cannot define a plane");
            if (!path_node["duration_s"])
                throw std::invalid_argument("path.duration_s is required");
            circle.duration_s = path_node["duration_s"].as<double>();
            if (!(circle.duration_s > 0.0) || !std::isfinite(circle.duration_s))
                throw std::invalid_argument("path.duration_s must be finite and positive");
            if (path_node["start_angle_deg"])
                circle.start_angle_rad =
                    path_node["start_angle_deg"].as<double>() * M_PI / 180.0;
            const std::string orientation =
                path_node["orientation"] ? path_node["orientation"].as<std::string>()
                                         : "fixed";
            if (orientation == "fixed") {
                circle.orientation = OrientationPolicy::kFixed;
                if (!path_node["orientation_rpy_deg"])
                    throw std::invalid_argument(
                        "path.orientation: fixed requires path.orientation_rpy_deg — "
                        "inheriting the start orientation makes a traced shape's "
                        "feasibility depend on where the arm was parked");
                circle.fixed_rpy_rad =
                    ReadVector3(path_node["orientation_rpy_deg"],
                                "path.orientation_rpy_deg") * (M_PI / 180.0);
            } else if (orientation == "radial") {
                circle.orientation = OrientationPolicy::kRadialInward;
            } else {
                throw std::invalid_argument(
                    "path.orientation must be 'fixed' or 'radial', got '" +
                    orientation + "'");
            }
            circle.frame = parsed.frame;
            parsed.circle = circle;
        } else {
            parsed.goal = ReadVector3(arm_node["goal"], "goal");
        }
        // Optional: the orientation to hold at the goal, degrees, same
        // frame as the position. Omitting it inherits the start pose's
        // orientation, which RunBridge reports rather than leaving silent.
        if (arm_node["orientation_rpy_deg"]) {
            const Eigen::Vector3d rpy_deg =
                ReadVector3(arm_node["orientation_rpy_deg"], "orientation_rpy_deg");
            parsed.goal_rpy_rad = rpy_deg * (M_PI / 180.0);
        }
        if (arm_node["box"] && !parsed.box) {
            AxisAlignedBox box;
            box.center = ReadVector3(arm_node["box"]["center"], "box center");
            box.half_extent =
                ReadVector3(arm_node["box"]["half_extent"], "box half_extent");
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
        if (flag == "--arm") {
            const std::string value = next();
            if (value == "right") parsed.left_arm = false;
            else if (value == "left") parsed.left_arm = true;
            else throw std::invalid_argument(
                "--arm must be 'right' or 'left' (got '" + value + "')");
        } else if (flag == "--goal") {
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
        } else if (flag == "--planner-config") {
            parsed.planner_config_path = next();
        } else if (flag == "--runs-root") {
            parsed.runs_root = next();
        } else if (flag == "--box") {
            AxisAlignedBox box;
            box.center = Eigen::Vector3d(ParseDouble(next()), ParseDouble(next()),
                                          ParseDouble(next()));
            box.half_extent = Eigen::Vector3d(ParseDouble(next()), ParseDouble(next()),
                                               ParseDouble(next()));
            parsed.box = box;
        } else if (flag == "--output") {
            const std::string value = next();
            if (value != "world-cartesian")
                throw std::invalid_argument(
                    "--output must be 'world-cartesian' (got '" +
                    value + "')");
        } else if (flag == "--world-mount-pose-m-quat") {
            std::array<double, 7> pose{};
            for (double& value : pose)
                value = ParseDouble(next());
            Eigen::Quaterniond world_q_mount(pose[6], pose[3], pose[4], pose[5]);
            if (std::abs(world_q_mount.norm() - 1.0) > 1e-3)
                throw std::invalid_argument(
                    "--world-mount-pose-m-quat must contain a unit quaternion");
            world_q_mount.normalize();
            Eigen::Isometry3d world_T_mount = Eigen::Isometry3d::Identity();
            world_T_mount.linear() = world_q_mount.toRotationMatrix();
            world_T_mount.translation() =
                Eigen::Vector3d(pose[0], pose[1], pose[2]);
            parsed.world_T_mount = world_T_mount;
        } else if (flag == "--vicon-sequence") {
            parsed.vicon_sequence = ParseUint64(next());
        } else if (flag == "--trajectory-id") {
            parsed.trajectory_id = ParseUint64(next());
        } else {
            throw std::invalid_argument("unrecognized flag: '" + flag + "'");
        }
    }
    if (!parsed.left_arm)
        throw std::invalid_argument(
            "--arm is required and must be 'right' or 'left'");
    if (parsed.goal && parsed.goal_file)
        throw std::invalid_argument(
            "at most one of --goal or --goal-file may be given");
    if (!parsed.goal)
        LoadGoalFile(parsed.goal_file ? *parsed.goal_file : DefaultGoalPath(),
                     parsed, *parsed.left_arm);
    if (parsed.state_csv && parsed.start_deg)
        throw std::invalid_argument(
            "at most one of --state-csv or --start-deg may be given");
    if (!parsed.world_T_mount)
        throw std::invalid_argument(
            "--world-mount-pose-m-quat is required for world-cartesian output");
    if (!parsed.vicon_sequence || *parsed.vicon_sequence == 0)
        throw std::invalid_argument(
            "--vicon-sequence must be nonzero for world-cartesian output");
    if (!parsed.trajectory_id || *parsed.trajectory_id == 0)
        throw std::invalid_argument(
            "--trajectory-id must be nonzero for world-cartesian output");
    return parsed;
}

constexpr double kDegToRad = M_PI / 180.0;

// Redirects std::cout's stream buffer to another stream for the guard's
// lifetime, restoring the original buffer on destruction — including via
// an exception unwinding through the guarded scope. Used to keep the
// legacy optimizer's stdout chatter out of the preview `targets` stream
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

PlannerSolveResult SolveWorldTrajectory(const std::vector<std::string>& args,
                                        std::ostream& diagnostics) {
    std::lock_guard<std::mutex> solve_lock(g_planner_solve_mutex);
    PlannerSolveResult result;
    for (const std::string& arg : args) {
        if (arg == "--help" || arg == "-h") {
            diagnostics << kUsageText;
            result.exit_code = 0;
            return result;
        }
    }

    ParsedArgs parsed;
    try {
        parsed = ParseArgs(args);
    } catch (const std::exception& error) {
        diagnostics << "error: " << error.what() << "\n\n" << kUsageText;
        result.exit_code = 1;
        return result;
    }

    const bool left_arm = *parsed.left_arm;
    const Eigen::Isometry3d world_T_mount = *parsed.world_T_mount;
    const Eigen::Quaterniond world_q_mount(world_T_mount.linear());
    diagnostics << "planner Vicon sequence: " << *parsed.vicon_sequence << "\n"
                << "trajectory ID: " << *parsed.trajectory_id << "\n"
                << "T_W_M position [" << world_T_mount.translation().x() << ", "
                << world_T_mount.translation().y() << ", "
                << world_T_mount.translation().z() << "] m, quaternion xyzw ["
                << world_q_mount.x() << ", " << world_q_mount.y() << ", "
                << world_q_mount.z() << ", " << world_q_mount.w() << "]\n"
                << "output frame: WORLD\n";

    // THE frame boundary. Everything below this point is `world`: the
    // grid-bounds check, the solve, and the SDF all share that one frame.
    const config::ReferenceFrame declared_frame = parsed.frame;
    try {
        if (parsed.goal)
            parsed.goal = ToWorld(*parsed.goal, declared_frame, world_T_mount);
        if (parsed.box) {
            // A box declared in an arm's base frame is axis-aligned THERE,
            // and the arm bases are rolled ~69 deg from mount, so the rotated
            // box is not axis-aligned in the grid. The SDF can only represent
            // axis-aligned boxes, so the rotated shape is replaced by its
            // enclosing axis-aligned box: bigger than asked for, never
            // smaller, which is the safe direction for obstacle avoidance.
            // The inflation is printed rather than applied quietly.
            const Eigen::Vector3d requested_half = parsed.box->half_extent;
            parsed.box->center =
                ToWorld(parsed.box->center, declared_frame, world_T_mount);
            if (declared_frame != config::ReferenceFrame::kWorld) {
                Eigen::Matrix3d world_R_declared = world_T_mount.linear();
                if (declared_frame == config::ReferenceFrame::kRightBase ||
                    declared_frame == config::ReferenceFrame::kLeftBase) {
                    const bool declared_left =
                        declared_frame == config::ReferenceFrame::kLeftBase;
                    world_R_declared *=
                        pinocchio_kinematics_adapter::MountFromBase(declared_left)
                            .linear();
                }
                // Enclosing AABB of a rotated box: |R| * half_extent, the
                // standard result (each mount axis picks up every declared
                // axis in proportion to |cos| of the angle between them).
                parsed.box->half_extent =
                    world_R_declared.cwiseAbs() * requested_half;
                diagnostics << "obstacle box: " << FrameName(declared_frame) << " -> "
                            << FrameName(config::ReferenceFrame::kWorld)
                            << "; half-extent inflated to the enclosing axis-aligned "
                            << "box, [" << requested_half.x() << ", " << requested_half.y()
                            << ", " << requested_half.z() << "] -> ["
                            << parsed.box->half_extent.x() << ", "
                            << parsed.box->half_extent.y() << ", "
                            << parsed.box->half_extent.z() << "] m\n";
            }
        }
    } catch (const std::exception& error) {
        diagnostics << "error: " << error.what() << "\n";
        result.exit_code = 1;
        return result;
    }
    // Printed on every POINT-GOAL run, not only when a conversion happened:
    // the goal the planner will actually aim at, in the one frame everything
    // below uses. A traced-path run has no point goal — parsed.goal is unset
    // there, and dereferencing it printed uninitialised memory.
    if (parsed.goal) {
        diagnostics << "goal: [" << parsed.goal->x() << ", " << parsed.goal->y() << ", "
                    << parsed.goal->z() << "] m in "
                    << FrameName(config::ReferenceFrame::kWorld);
        if (declared_frame != config::ReferenceFrame::kWorld)
            diagnostics << " (converted from " << FrameName(declared_frame) << ")";
        diagnostics << "\n";
    }

    if (parsed.box) {
        const GridBounds bounds =
            WorldGridBounds(WorldGridGeometry(world_T_mount));
        if (!BoxWithinGridBounds(*parsed.box, bounds)) {
            diagnostics << "error: obstacle box extends outside the SDF grid volume ("
                        << DescribeGridBounds(bounds) << ", world frame); gpmp2 reports "
                        << "no obstacle for out-of-grid queries, so this box "
                        << "cannot be honoured\n";
            result.exit_code = 1;
            return result;
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
            // config::ArmConfig::log_prefix (basic_control/src/Config.h) —
            // a right-arm log is not evidence of the left arm's state, or
            // vice versa, so this searches only the selected arm's own
            // logs, never the generic "loop_log" prefix.
            const std::string log_prefix = left_arm ? "loop_log_left" : "loop_log_right";
            std::string find_error;
            const std::optional<std::string> found =
                FindLatestRunCsv(parsed.runs_root, find_error, log_prefix);
            if (!found) {
                diagnostics << "error: no " << log_prefix << "*.csv run log found "
                            << "under " << parsed.runs_root << " — start the controller "
                            << "(--arm " << (left_arm ? "left" : "right")
                            << ") first (it creates the log), or pass "
                            << "--state-csv/--start-deg\n";
                result.exit_code = 2;
                return result;
            }
            state_csv = *found;
        }
        std::string error;
        const std::optional<Eigen::Matrix<double, 7, 1>> q =
            ReadLatestMeasuredQ(state_csv, error);
        if (!q) {
            diagnostics << "error: start state unavailable: " << error << "\n";
            result.exit_code = 2;
            return result;
        }
        q_start_rad = *q;
    }

    const std::string dh_path = parsed.dh_path.value_or(DefaultDhPath(left_arm));
    PlannerModel model;
    try {
        // Same reason as the solve below: constructing the model builds a
        // Dynamics, and anything the Pinocchio/legacy stack prints to
        // std::cout here would corrupt the standalone preview stream. The
        // final block is written only after the guard restores std::cout.
        const CoutRedirectGuard cout_guard(diagnostics);
        // has_tool = !left_arm: the right chain ends at the mounted tool,
        // the left chain at a bare flange (GenerateArmModel.h has_tool).
        model = LoadPlannerModel(dh_path, /*has_tool=*/!left_arm,
                                 world_T_mount);
    } catch (const std::exception& error) {
        diagnostics << "error: solve failed: could not load planner model from "
                    << dh_path << ": " << error.what() << "\n";
        result.exit_code = 3;
        return result;
    }

    // Loaded before the solve and echoed whole, so a session log records
    // the tuning that produced its trajectory. A bad key fails here, with
    // the arm still holding at start and nothing written to the preview.
    PlannerConfig planner_config;
    try {
        planner_config = LoadPlannerConfig(parsed.planner_config_path);
    } catch (const std::exception& error) {
        diagnostics << "error: " << error.what() << "\n";
        result.exit_code = 1;
        return result;
    }
    diagnostics << EffectiveConfigText(planner_config);

    // ---------------------------------------------------------------
    // Cartesian path following
    // ---------------------------------------------------------------
    // Taken before the point-to-point path below, and returning from
    // inside, so point-to-point behaviour is reached by exactly the same
    // code it always was.
    if (parsed.circle) {
        CircleSpec circle = *parsed.circle;
        // Sample count from the geometry, not from a hand-picked number:
        // a stated tolerance is meaningless if the chords between samples
        // already miss the arc by more than it.
        circle.samples = CircleSamplesForChordError(
            circle.radius_m, planner_config.path_following.max_chord_error_m);

        CartesianPath task_path;
        try {
            task_path = PathToWorld(GenerateCircle(circle), world_T_mount);
        } catch (const std::exception& error) {
            diagnostics << "error: " << error.what() << "\n";
            result.exit_code = 1;
            return result;
        }
        diagnostics << "path: circle, radius " << circle.radius_m << " m, "
                    << circle.samples << " samples (chord error <= "
                    << planner_config.path_following.max_chord_error_m * 1000.0
                    << " mm), lap " << circle.duration_s << " s, declared in "
                    << FrameName(declared_frame) << " -> "
                    << FrameName(config::ReferenceFrame::kWorld) << "\n";

        ValidationInputs validation;
        validation.circle_applicable = true;
        validation.circle_centre = PoseToWorld(
            Eigen::Isometry3d(Eigen::Translation3d(circle.centre_m)),
            declared_frame, world_T_mount).translation();
        validation.circle_normal =
            PoseToWorld(Eigen::Isometry3d(Eigen::Quaterniond::Identity()),
                        declared_frame, world_T_mount).linear() *
            circle.normal.normalized();
        validation.circle_radius_m = circle.radius_m;

        // The legacy optimiser prints progress straight to std::cout, and in
        // the standalone binary's stdout is the preview stream. Guarding the
        // solve is not optional:
        // without it "Creating arm trajectory..." is the first line of the
        // emitted block (observed 2026-08-07).
        PathPlanOutcome plan;
        {
            const CoutRedirectGuard cout_guard(diagnostics);
            plan = SolveAlongPath(model, task_path, q_start_rad, parsed.box,
                                  parsed.joint_limits_path, planner_config,
                                  validation);
        }
        if (!plan.ok) {
            diagnostics << "error: solve failed: " << plan.error << "\n";
            result.exit_code = 3;
            return result;
        }

        diagnostics << "continuation IK: largest joint step "
                    << plan.maximum_joint_step_rad * 180.0 / M_PI
                    << " deg, closure drift "
                    << plan.closure_drift_rad * 180.0 / M_PI << " deg\n";
        if (plan.time_scaling_passes > 1)
            diagnostics << "time scaling: " << plan.time_scaling_passes
                        << " pass(es), final duration " << plan.total_time_sec
                        << " s" << (plan.time_scaling_settled
                                        ? "\n"
                                        : " — DID NOT SETTLE within the pass limit\n");
        diagnostics << plan.report.Summary();

        if (!plan.report.hardware_execution_allowed) {
            // Not emitted. Every check the report lists is a MODELLED one,
            // so a failure here is the strongest statement available that
            // this trajectory should not reach the arm.
            diagnostics << "error: plan rejected — one or more validity checks "
                           "failed (see the report above). Nothing was emitted.\n";
            result.exit_code = 4;
            return result;
        }
        try {
            WorldCartesianTrajectory projected = ProjectWorldTrajectory(
                model, plan.result.trajectory_pos, plan.result.trajectory_vel,
                plan.total_time_sec, *parsed.trajectory_id,
                *parsed.vicon_sequence);
            result.trajectory =
                std::make_unique<WorldCartesianTrajectory>(std::move(projected));
        } catch (const std::exception& error) {
            diagnostics << "error: plan rejected: " << error.what() << "\n";
            result.exit_code = 4;
            return result;
        }
        diagnostics << "arm: " << (left_arm ? "left" : "right")
                    << ", traced circle emitted, trajectory points: "
                    << result.trajectory->points.size() << ", duration "
                    << plan.total_time_sec << " s\n";
        result.exit_code = 0;
        return result;
    }

    PlanRequest request;
    request.q_start_rad = q_start_rad;
    request.goal_position_m = *parsed.goal;
    request.obstacle = parsed.box;
    if (parsed.goal_rpy_rad) {
        request.goal_rotation =
            RotationToWorld(RotationFromRpy(*parsed.goal_rpy_rad), declared_frame,
                            world_T_mount);
        // Echoed in mount, like the goal position above — reporting one in
        // the declared frame and the other post-conversion put two frames in
        // one block and made them impossible to compare.
        const Eigen::Vector3d rpy_world = RpyFromRotation(*request.goal_rotation);
        diagnostics << "goal orientation: explicit, rpy [" << rpy_world.x() * 180.0 / M_PI
                    << ", " << rpy_world.y() * 180.0 / M_PI << ", "
                    << rpy_world.z() * 180.0 / M_PI << "] deg in "
                    << FrameName(config::ReferenceFrame::kWorld);
        if (declared_frame != config::ReferenceFrame::kWorld)
            diagnostics << " (converted from " << FrameName(declared_frame) << ")";
        diagnostics << "\n";
    } else {
        // Never silent. Inheriting means this goal's feasibility depends on
        // where the arm was parked before the run — the same left-arm goal
        // solved to 1.34 mm from one start and was unreachable from another
        // on 2026-08-06, purely for this reason.
        diagnostics << "goal orientation: INHERITED from the start pose (no "
                       "orientation_rpy_deg in the goal block). Feasibility "
                       "therefore depends on where the arm started; set it "
                       "explicitly to make this goal mean the same thing "
                       "every run.\n";
    }

    // The legacy optimizer prints its own progress chatter straight to
    // std::cout. Redirect std::cout into
    // `diagnostics` for the duration of the solve; the guard restores it
    // on scope exit, including if an exception unwinds through here.
    PlanOutcome outcome;
    {
        const CoutRedirectGuard cout_guard(diagnostics);
        outcome = SolveToPosition(model, request, parsed.joint_limits_path,
                                  planner_config);
    }

    if (!outcome.ok) {
        diagnostics << "error: solve failed: " << outcome.error << "\n";
        result.exit_code = 3;
        return result;
    }

    // How the optimiser's starting sketch was built. Reported HERE, before
    // validation and emission, because a degraded initialisation is most
    // worth knowing about on the runs that go on to fail — printing it only
    // on success would hide it exactly when it explains something. A
    // solved pose says so in one word; the degraded cases give how far the
    // IK landed from the requested pose, split into position and
    // orientation, because that split identifies WHICH half was
    // unreachable. This is a description, not a verdict: the plan is not
    // refused for a poor initialisation, and the achieved goal error
    // reported after emission is what the optimiser actually managed.
    diagnostics << "plan initialisation: ";
    switch (outcome.init_source) {
    case InitSource::kSolvedIk:
        diagnostics << "solved IK pose\n";
        break;
    case InitSource::kNearMiss:
        diagnostics << "NEAR MISS — no IK pose passed; seeded from the closest "
                       "one reached, "
                    << (outcome.init_position_error_m * 1000.0) << " mm and "
                    << (outcome.init_orientation_error_rad * 180.0 / M_PI)
                    << " deg from the requested pose. The requested position "
                       "may be reachable only with a different tool "
                       "orientation; check the achieved goal error below.\n";
        break;
    case InitSource::kHeldStart:
        diagnostics << "HELD START — IK produced no usable pose at all; every "
                       "waypoint seeded at the start configuration and the "
                       "optimiser pulled from there alone. Treat the achieved "
                       "goal error below as the only statement of where this "
                       "plan actually goes.\n";
        break;
    }

    const std::optional<std::string> validation_error =
        ValidateJointPath(outcome.result.trajectory_pos);
    if (validation_error) {
        diagnostics << "error: plan rejected: " << *validation_error << "\n";
        result.exit_code = 4;
        return result;
    }

    try {
        WorldCartesianTrajectory projected = ProjectWorldTrajectory(
            model, outcome.result.trajectory_pos,
            outcome.result.trajectory_vel, outcome.total_time_sec,
            *parsed.trajectory_id, *parsed.vicon_sequence);
        result.trajectory =
            std::make_unique<WorldCartesianTrajectory>(std::move(projected));
    } catch (const std::exception& error) {
        diagnostics << "error: plan rejected: " << error.what() << "\n";
        result.exit_code = 4;
        return result;
    }

    diagnostics << "arm: " << (left_arm ? "left" : "right")
                << ", trajectory points: " << result.trajectory->points.size()
                << ", solve: " << outcome.result.optimization_duration.count()
                << " ms, final goal error: " << (outcome.final_goal_error_m * 1000.0)
                << " mm\n";
    result.exit_code = 0;
    return result;
}
