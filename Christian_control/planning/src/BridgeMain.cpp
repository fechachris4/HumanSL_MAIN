#include "PlannerRuntime.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <climits>
#include <iomanip>
#include <limits>
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
#include "MountSdf.h"
#include "PathFrames.h"
#include "PathValidation.h"
#include "PlanDebugDump.h"
#include "PinocchioKinematicsAdapter.h"
#include "Config.h"   // control — config::kReferenceFrame

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
    "                         process reads the start state from. No\n"
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
    "                         goal block's declared frame (default\n"
    "                         config::kReferenceFrame, currently mount);\n"
    "                         converted to mount at the boundary. Must lie\n"
    "                         fully inside the\n"
    "                         SDF grid volume (MountSdf.h MountGridBounds())\n"
    "                         or the run is rejected — outside that volume\n"
    "                         gpmp2 silently reports no obstacle.\n"
    "  --verbose              Echo the full effective planner config and\n"
    "                         other low-priority detail. Without it a run\n"
    "                         prints the config path, digest and IK seed\n"
    "                         only — enough to reproduce, not to drown in.\n"
    "  --debug-dir PATH       Optional diagnostic dump directory. Writes\n"
    "                         joints.csv, joint_limits.csv, meta.csv and,\n"
    "                         for a traced path, path_ik.csv — the\n"
    "                         per-sample continuation walk, written even\n"
    "                         when the walk FAILED, which is the case worth\n"
    "                         looking at. Nothing goes to stdout and the\n"
    "                         controller never reads these files; plot them\n"
    "                         with scripts/plot_plan.py.\n"
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

// One number formatted for the summary, trimmed to what an eye can compare.
std::string Fixed(double value, int decimals = 1) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(decimals) << value;
    return text.str();
}

// The high-priority result block every planning attempt ends its
// diagnostics with. Everything here is a value the planner already
// produced; the block only arranges it so the answer to "what happened and
// where" does not have to be assembled from a scroll of detail.
struct SummaryWriter {
    std::ostream& diagnostics;
    std::vector<std::pair<std::string, std::string>>& extra;

    void Line(const std::string& key, const std::string& value) {
        diagnostics << "  " << key << ": " << value << "\n";
        extra.emplace_back(key, value);
    }
};

// The IK walk's summary lines: solved count, the failed ranges with their
// percent of the way along the path, the worst residual, the smallest
// joint-limit margin, and the solved neighbours around each failed range —
// the samples a diagnosis starts from.
void SummarizeWalk(SummaryWriter& out, const PathIkResult& walk,
                   const PlanJointLimits& limits, double acceptance_m) {
    const std::size_t count = walk.samples.size();
    if (count == 0) return;
    const double denominator = count > 1 ? static_cast<double>(count - 1) : 1.0;
    const auto percent = [denominator](std::size_t index) {
        return Fixed(100.0 * static_cast<double>(index) / denominator, 0) + "%";
    };

    // Anchors are the attempted samples: solved, or carrying a failure
    // reason. Interpolated samples were never attempted and are the walk's
    // normal state, not a shortfall.
    const auto failed = [&](const PathIkSample& sample) {
        return !sample.solved && sample.failure != PathIkFailure::kNone;
    };
    std::size_t solved = 0, failed_anchors = 0, interpolated = 0;
    double worst_residual_m = 0.0;
    std::size_t worst_residual_index = 0;
    double min_margin_rad = std::numeric_limits<double>::infinity();
    std::size_t min_margin_index = 0;
    for (std::size_t index = 0; index < count; ++index) {
        const PathIkSample& sample = walk.samples[index];
        if (sample.solved) ++solved;
        if (failed(sample)) ++failed_anchors;
        if (sample.interpolated) ++interpolated;
        if (sample.position_residual_m > worst_residual_m &&
            std::isfinite(sample.position_residual_m)) {
            worst_residual_m = sample.position_residual_m;
            worst_residual_index = index;
        }
        const double margin = JointLimitMarginRad(sample.configuration, limits);
        if (margin < min_margin_rad) {
            min_margin_rad = margin;
            min_margin_index = index;
        }
    }

    out.Line("IK anchors", std::to_string(solved) + " solved, " +
                               std::to_string(failed_anchors) +
                               " failed (dropped); " +
                               std::to_string(interpolated) + " of " +
                               std::to_string(count) +
                               " samples interpolated");
    const std::string ranges = DescribeFailedRanges(walk);
    if (!ranges.empty()) {
        // First-to-last failed anchor as percent of the way along the path.
        std::size_t first = count, last = 0;
        for (std::size_t index = 0; index < count; ++index)
            if (failed(walk.samples[index])) {
                first = std::min(first, index);
                last = std::max(last, index);
            }
        out.Line("failed anchors", ranges + " (" + percent(first) + "-" +
                                       percent(last) + " along the path)");
        // Solved neighbours around each failed range.
        std::ostringstream neighbours;
        bool first_range = true;
        for (std::size_t index = 0; index < count;) {
            if (!failed(walk.samples[index])) { ++index; continue; }
            std::size_t end = index;
            while (end + 1 < count && failed(walk.samples[end + 1])) ++end;
            if (!first_range) neighbours << "; ";
            first_range = false;
            const auto describe = [&](std::size_t at) {
                neighbours << "sample " << at << " (residual "
                           << Fixed(walk.samples[at].position_residual_m * 1e3)
                           << " mm, margin "
                           << Fixed(JointLimitMarginRad(
                                        walk.samples[at].configuration, limits) *
                                    180.0 / M_PI)
                           << " deg)";
            };
            // The adjacent sample may be interpolated (never attempted), so
            // scan outward to the nearest SOLVED anchor on each side.
            std::size_t before = index;
            while (before > 0 && !walk.samples[before - 1].solved) --before;
            if (before > 0) describe(before - 1);
            else neighbours << "none before";
            neighbours << " / ";
            std::size_t after = end;
            while (after + 1 < count && !walk.samples[after + 1].solved) ++after;
            if (after + 1 < count) describe(after + 1);
            else neighbours << "none after";
            index = end + 1;
        }
        out.Line("solved neighbours", neighbours.str());
        // Which failure the walk actually recorded, per kind.
        std::size_t limits_failures = 0, convergence_failures = 0;
        for (const PathIkSample& sample : walk.samples) {
            if (sample.failure == PathIkFailure::kJointLimits) ++limits_failures;
            if (sample.failure == PathIkFailure::kNoConvergence)
                ++convergence_failures;
        }
        std::ostringstream reasons;
        reasons << convergence_failures << " no-convergence, " << limits_failures
                << " converged-only-outside-joint-limits";
        out.Line("failure reasons", reasons.str());
    }
    out.Line("worst position residual",
             Fixed(worst_residual_m * 1e3) + " mm at sample " +
                 std::to_string(worst_residual_index) + " (acceptance " +
                 Fixed(acceptance_m * 1e3) + " mm)");
    if (std::isfinite(min_margin_rad))
        out.Line("min joint-limit margin (walk)",
                 Fixed(min_margin_rad * 180.0 / M_PI) + " deg at sample " +
                     std::to_string(min_margin_index));
}

// Writes the diagnostic dump, if one was asked for. Deliberately never
// fatal and never able to change an exit code: a plan's success is a
// statement about the plan, not about whether a debug file could be
// written. A failure to write is reported and the run carries on.
void DumpPlanDebug(const std::optional<std::string>& directory,
                   const PlanDebugMeta& meta, const TrajectoryResult& trajectory,
                   const PlanJointLimits& limits,
                   const CartesianPath* path_mount, const PathIkResult* walk,
                   std::ostream& diagnostics)
{
    if (!directory)
        return;
    const auto report = [&diagnostics](const std::optional<std::string>& error) {
        if (error)
            diagnostics << "warning: debug dump: " << *error << "\n";
    };
    report(WritePlanMetaCsv(*directory, meta));
    report(WriteJointLimitsCsv(*directory, limits));
    if (!trajectory.trajectory_pos.empty())
        report(WriteJointTrajectoryCsv(*directory, trajectory));
    if (path_mount != nullptr && walk != nullptr && !walk->samples.empty())
        report(WritePathIkCsv(*directory, *path_mount, *walk, limits));
    diagnostics << "debug dump written to " << *directory << "\n";
}

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
// the end (see MountGridBounds in MountSdf.cpp). The lower face has no such
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

// An obstacle box exactly as DECLARED — centre and half-extents in the goal
// block's declared frame, which may be mount, an arm base, or world. A
// deliberately different type from AxisAlignedBox (MountSdf.h), which is
// mount-frame by contract: keeping the raw declaration out of that type
// means nothing downstream can mistake unconverted numbers for mount ones.
// Only the frame boundary below turns one into the other, via ToMount.
struct DeclaredBox {
    Eigen::Vector3d center;
    Eigen::Vector3d half_extent;
};

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
    // unset == no diagnostic dump. Off by default: a plan run in a session
    // should not silently start writing files beside itself.
    std::optional<std::string> debug_dir;
    // Full config echo and similar low-priority detail. Off by default so
    // the summary is what a normal run's diagnostics end with.
    bool verbose = false;
    std::optional<DeclaredBox> box;
    std::optional<Eigen::Isometry3d> world_T_mount;
    std::optional<std::uint64_t> vicon_sequence;
    std::optional<std::uint64_t> trajectory_id;
};


// ---------------------------------------------------------------
// Frame boundary
// ---------------------------------------------------------------
//
// The planner is `mount` internally, everywhere: the gpmp2 arm model and the
// SDF are paired in one ObstacleSDFFactorArm, so they must share a frame or
// every collision check is silently wrong — and since PlannerModel builds the
// arm at DhRootInMount(), that shared frame is mount for both arms. Input
// declared in an arm's base frame or in Vicon `world` is converted here once
// at the edge, through the ONE conversion module (PathFrames.h); input
// already in mount passes through untouched. A world-declared input
// requires the run's valid world_T_mount snapshot and is rejected without
// one; mount and base inputs never need it.
//
// The base transforms come from the URDF through Pinocchio, never from a
// constant in this file, so surveying the rig and regenerating the URDF
// needs no code change.

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
            DeclaredBox box;
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
        } else if (flag == "--debug-dir") {
            parsed.debug_dir = next();
        } else if (flag == "--verbose") {
            parsed.verbose = true;
        } else if (flag == "--box") {
            DeclaredBox box;
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

PlannerSolveResult SolvePlan(const std::vector<std::string>& args,
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
                << "declared_input_frame=" << FrameName(parsed.frame)
                << " planning_frame=mount output_frame=world\n";

    // THE frame boundary. Everything below this point is `mount`, the
    // planner's only internal frame: the grid-bounds check, the solve, IK
    // and the SDF all share it. The one mount->world conversion is the
    // output projection that builds the controller's world-frame block.
    const config::ReferenceFrame declared_frame = parsed.frame;
    // The mount-frame obstacle. AxisAlignedBox is mount-by-contract
    // (MountSdf.h), so it is CONSTRUCTED here from the declaration and
    // nowhere else — the declared numbers never travel in the mount type.
    std::optional<AxisAlignedBox> box_mount;
    try {
        if (parsed.goal)
            parsed.goal = PointToMount(*parsed.goal, declared_frame, world_T_mount);
        if (parsed.box) {
            // A box declared in an arm's base frame (rolled ~69 deg from
            // mount) or in world (wherever the rig currently sits) is
            // axis-aligned THERE, not in the mount grid. The SDF can only
            // represent axis-aligned boxes, so the rotated shape is replaced
            // by its enclosing axis-aligned box: bigger than asked for,
            // never smaller, which is the safe direction for obstacle
            // avoidance. The inflation is printed rather than applied
            // quietly.
            AxisAlignedBox converted;
            converted.center =
                PointToMount(parsed.box->center, declared_frame, world_T_mount);
            converted.half_extent = parsed.box->half_extent;
            if (declared_frame != config::ReferenceFrame::kMount) {
                const Eigen::Matrix3d mount_R_declared = RotationToMount(
                    Eigen::Matrix3d::Identity(), declared_frame, world_T_mount);
                // Enclosing AABB of a rotated box: |R| * half_extent, the
                // standard result (each mount axis picks up every declared
                // axis in proportion to |cos| of the angle between them).
                converted.half_extent =
                    mount_R_declared.cwiseAbs() * parsed.box->half_extent;
                diagnostics << "obstacle_box: declared_frame="
                            << FrameName(declared_frame)
                            << " -> mount; half-extent inflated to the "
                            << "enclosing axis-aligned box, ["
                            << parsed.box->half_extent.x() << ", "
                            << parsed.box->half_extent.y() << ", "
                            << parsed.box->half_extent.z() << "] -> ["
                            << converted.half_extent.x() << ", "
                            << converted.half_extent.y() << ", "
                            << converted.half_extent.z() << "] m\n";
            }
            box_mount = converted;
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
        diagnostics << "goal_mount_m: [" << parsed.goal->x() << ", "
                    << parsed.goal->y() << ", " << parsed.goal->z()
                    << "] (declared_frame=" << FrameName(declared_frame) << ")\n";
    }

    if (box_mount) {
        const GridBounds bounds = MountGridBounds(MountGridGeometry());
        if (!BoxWithinGridBounds(*box_mount, bounds)) {
            diagnostics << "error: obstacle box extends outside the SDF grid volume ("
                        << DescribeGridBounds(bounds) << ", mount frame); gpmp2 reports "
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
            // config::ArmConfig::log_prefix (control/Config.h) —
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
    // The planner start state: q0_plan = CanonicalizeForPlanner(q_measured).
    // The ONE canonicalisation site — every transport above delivers raw
    // Kortex-convention radians, and GPMP2's flat signed-radian space gets
    // the principal values. Unwrapped, a measured 253 deg lands outside a
    // flat +/-150 deg band (observed 2026-08-19: margin -59.68 deg, splice
    // 45.24 deg, plan rejected).
    for (int joint = 0; joint < 7; ++joint)
        q_start_rad(joint) = WrapToPrincipalRad(q_start_rad(joint));

    const std::string dh_path = parsed.dh_path.value_or(DefaultDhPath(left_arm));
    PlannerModel model;
    try {
        // Same reason as the solve below: constructing the model builds a
        // RobotModel, and anything the Pinocchio/legacy stack prints to
        // std::cout here would corrupt the standalone preview stream. The
        // final block is written only after the guard restores std::cout.
        const CoutRedirectGuard cout_guard(diagnostics);
        // has_tool = !left_arm: the right chain ends at the mounted tool,
        // the left chain at a bare flange (GenerateArmModel.h has_tool).
        model = LoadPlannerModel(dh_path, /*has_tool=*/!left_arm);
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
    if (parsed.verbose) {
        diagnostics << EffectiveConfigText(planner_config);
    } else {
        // The reproduction essentials stay on every run (the digest and the
        // seed are what docs/decisions/runtime-config.md wants a session log
        // to carry); the full value listing moves behind --verbose.
        diagnostics << "planner config: " << planner_config.source_path
                    << " digest(fnv1a64)=" << std::hex << std::showbase
                    << planner_config.source_fnv1a64 << std::dec
                    << std::noshowbase << " ik_seed="
                    << planner_config.effective_ik_seed
                    << " (--verbose for all values)\n";
    }

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
            task_path = PathToMount(GenerateCircle(circle), world_T_mount);
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
                    << FrameName(config::ReferenceFrame::kMount) << "\n";

        ValidationInputs validation;
        validation.circle_applicable = true;
        validation.circle_centre =
            PointToMount(circle.centre_m, declared_frame, world_T_mount);
        validation.circle_normal =
            RotationToMount(Eigen::Matrix3d::Identity(), declared_frame,
                            world_T_mount) *
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
            plan = SolveAlongPath(model, task_path, q_start_rad, box_mount,
                                  parsed.joint_limits_path, planner_config,
                                  validation);
        }
        // ---- the high-priority summary, printed on every attempt ------
        PlanDebugMeta meta;
        meta.arm = *parsed.left_arm ? "left" : "right";
        meta.plan_kind = "path";
        meta.status = plan.ok ? "ok" : plan.error;
        meta.total_time_s = plan.total_time_sec;

        if (plan.ok) {
            diagnostics << "continuation IK: largest joint step "
                        << plan.maximum_joint_step_rad * 180.0 / M_PI
                        << " deg, closure drift "
                        << plan.closure_drift_rad * 180.0 / M_PI << " deg\n";
            if (plan.ik_unresolved_samples > 0)
                diagnostics << "continuation IK gaps: "
                            << plan.ik_unresolved_samples
                            << " unresolved sample(s) seeded ("
                            << plan.ik_interpolated_samples
                            << " interpolated) — GPMP2 keeps the configured "
                               "pose priors and the final "
                               "validation judges the result\n";
            if (plan.time_scaling_passes > 1)
                diagnostics << "time scaling: " << plan.time_scaling_passes
                            << " pass(es), final duration "
                            << plan.total_time_sec << " s"
                            << (plan.time_scaling_settled
                                    ? "\n"
                                    : " — DID NOT SETTLE within the pass limit\n");
            if (parsed.verbose)
                diagnostics << plan.report.Summary();
        }
        diagnostics << "---- PLAN SUMMARY (" << meta.arm
                    << " arm, traced path) ----\n";
        SummaryWriter summary{diagnostics, meta.extra};
        if (!plan.ok) {
            const bool ik_stage = plan.error.rfind("path IK", 0) == 0;
            summary.Line("result", "FAILED at " +
                                       std::string(ik_stage
                                                       ? "IK initialization "
                                                         "(before GPMP2 ran)"
                                                       : "GPMP2 solve/validation"));
            summary.Line("error", plan.error);
        } else {
            const char* verdict =
                plan.report.verdict == PlanVerdict::kAccept ? "ACCEPT"
                : plan.report.verdict == PlanVerdict::kWarning ? "WARNING"
                                                               : "REJECT";
            summary.Line("result", std::string(verdict) + ", duration " +
                                       Fixed(plan.total_time_sec, 2) + " s");
            for (const std::string& warning : plan.report.warnings)
                summary.Line("warning", warning);
            const FidelityError& fidelity = plan.report.command;
            summary.Line("task fidelity (gated)",
                         "max " + Fixed(fidelity.max_position_m * 1e3, 2) +
                             " mm / p95 " +
                             Fixed(fidelity.p95_position_m * 1e3, 2) +
                             " mm position, max " +
                             Fixed(fidelity.max_orientation_rad * 180.0 / M_PI) +
                             " deg orientation");
            summary.Line("worst point",
                         "t=" + Fixed(fidelity.worst_time_s, 2) + " s, " +
                             Fixed(fidelity.worst_path_parameter * 100.0, 0) +
                             "% along the path");
            summary.Line("min modelled clearance",
                         Fixed(plan.report.minimum_clearance_m * 1e3) +
                             " mm at t=" +
                             Fixed(plan.report.minimum_clearance_time_s, 2) +
                             " s (SDF models workspace + declared box only)");
            summary.Line("min joint-limit margin (trajectory)",
                         Fixed(plan.report.minimum_joint_limit_margin_rad *
                               180.0 / M_PI) +
                             " deg");
        }
        SummarizeWalk(summary, plan.ik_walk, plan.joint_limits,
                      planner_config.path_following.maximum_planning_error_m);
        // Machine-oriented: when the constrained task phase begins in
        // trajectory time, so plot_plan.py can shade it and place each path
        // sample on the joint-trajectory time axis. Not printed as a
        // summary line — it is a coordinate, not a finding.
        if (plan.ok)
            meta.extra.emplace_back("task_start_time_s",
                                    Fixed(plan.task_start_time_s, 6));
        DumpPlanDebug(parsed.debug_dir, meta, plan.result, plan.joint_limits,
                      &task_path, &plan.ik_walk, diagnostics);
        diagnostics << "----\n";

        if (!plan.ok) {
            diagnostics << "error: solve failed: " << plan.error << "\n";
            result.exit_code = 3;
            return result;
        }

        if (plan.report.verdict == PlanVerdict::kReject) {
            // Not emitted. REJECT means unsafe or clearly failing the
            // requested task: non-finite states, modelled collision, a real
            // joint-limit violation, a splice jump, velocity still over
            // after time scaling, or task error beyond twice the requested
            // tolerance. Small imperfections warn instead (see the report).
            diagnostics << "error: plan rejected — an unsafe or clearly "
                           "invalid condition (see the verdict in the report "
                           "above). Nothing was emitted.\n";
            result.exit_code = 4;
            return result;
        }
        if (plan.report.verdict == PlanVerdict::kWarning)
            diagnostics << "plan accepted WITH WARNINGS — usable but "
                           "imperfect; the warnings above say how.\n";
        try {
            WorldCartesianTrajectory projected = ProjectWorldTrajectory(
                model, world_T_mount,
                plan.result.trajectory_pos, plan.result.trajectory_vel,
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
    request.obstacle = box_mount;
    if (parsed.goal_rpy_rad) {
        request.goal_rotation =
            RotationToMount(RotationFromRpy(*parsed.goal_rpy_rad), declared_frame,
                            world_T_mount);
        // Echoed in mount, like the goal position above — reporting one in
        // the declared frame and the other post-conversion put two frames in
        // one block and made them impossible to compare.
        const Eigen::Vector3d rpy_mount = RpyFromRotation(*request.goal_rotation);
        diagnostics << "goal_orientation_rpy_mount_deg: ["
                    << rpy_mount.x() * 180.0 / M_PI << ", "
                    << rpy_mount.y() * 180.0 / M_PI << ", "
                    << rpy_mount.z() * 180.0 / M_PI
                    << "] (declared_frame=" << FrameName(declared_frame)
                    << ")\n";
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

    {
        PlanDebugMeta meta;
        meta.arm = *parsed.left_arm ? "left" : "right";
        meta.plan_kind = "point";
        meta.status = outcome.ok ? "ok" : outcome.error;
        meta.final_goal_error_m = outcome.final_goal_error_m;
        meta.total_time_s = outcome.total_time_sec;

        if (outcome.ok) {
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
        }

        diagnostics << "---- PLAN SUMMARY (" << meta.arm
                    << " arm, point goal) ----\n";
        SummaryWriter summary{diagnostics, meta.extra};
        if (!outcome.ok) {
            summary.Line("result", "FAILED at GPMP2 solve");
            summary.Line("error", outcome.error);
        } else {
            summary.Line("result",
                         "ok, duration " + Fixed(outcome.total_time_sec, 2) +
                             " s");
            summary.Line("final goal error",
                         Fixed(outcome.final_goal_error_m * 1e3, 3) + " mm");
            // Smallest distance any dense state comes to a bounded joint
            // limit — arithmetic over the trajectory the solver produced.
            double min_margin_rad = std::numeric_limits<double>::infinity();
            double min_margin_time_s = 0.0;
            for (std::size_t state = 0;
                 state < outcome.result.trajectory_pos.size(); ++state) {
                Eigen::Matrix<double, 7, 1> q;
                for (int joint = 0; joint < 7; ++joint)
                    q(joint) = outcome.result.trajectory_pos[state](joint);
                const double margin =
                    JointLimitMarginRad(q, outcome.joint_limits);
                if (margin < min_margin_rad) {
                    min_margin_rad = margin;
                    min_margin_time_s =
                        static_cast<double>(state) * outcome.result.dt;
                }
            }
            if (std::isfinite(min_margin_rad))
                summary.Line("min joint-limit margin (trajectory)",
                             Fixed(min_margin_rad * 180.0 / M_PI) +
                                 " deg at t=" + Fixed(min_margin_time_s, 2) +
                                 " s");
            summary.Line("collision clearance",
                         "not computed for point-goal plans (path validation "
                         "runs on traced paths only)");
        }
        DumpPlanDebug(parsed.debug_dir, meta, outcome.result,
                      outcome.joint_limits, nullptr, nullptr, diagnostics);
        diagnostics << "----\n";
    }
    if (!outcome.ok) {
        diagnostics << "error: solve failed: " << outcome.error << "\n";
        result.exit_code = 3;
        return result;
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
            model, world_T_mount, outcome.result.trajectory_pos,
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
