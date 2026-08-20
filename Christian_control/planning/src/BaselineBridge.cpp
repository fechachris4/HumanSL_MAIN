#include "BaselineBridge.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

#include "Config.h"
#include "PlannerModel.h"
#include "StartState.h"
#include "WorldTrajectoryProjection.h"

namespace {

// One parsed wire sample: time + 7 joint positions + 7 joint velocities,
// exactly the block row FormatTrajectoryBlock has emitted since 2026-08-05.
struct WireSample {
    double t_s = 0.0;
    Eigen::Matrix<double, 7, 1> q_deg;
    Eigen::Matrix<double, 7, 1> qdot_deg_s;
};

// Shell-quote one argument. The command line is handed to popen (= /bin/sh),
// so every path and number is single-quoted; a single quote inside a path is
// closed-escaped-reopened. Numbers we format ourselves never contain one.
std::string Quoted(const std::string& text) {
    std::string out = "'";
    for (char c : text) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    out += "'";
    return out;
}

// The baseline block, parsed strictly: TRAJ_BEGIN <count>, then exactly
// <count> rows of 15 finite numbers with strictly increasing times starting
// at 0, then TRAJ_END. Anything else is a hard error — a malformed block
// must never become a trajectory. stdout carries ONLY the block by the
// baseline's own design (its solve runs under a cout-redirect guard so the
// wire stays clean), which is what makes strict parsing possible.
bool ParseWireBlock(const std::string& wire, std::vector<WireSample>& samples,
                    std::string& error) {
    std::istringstream in(wire);
    std::string line;
    bool begun = false;
    std::size_t expected = 0;
    while (std::getline(in, line)) {
        if (!begun) {
            if (line.rfind("TRAJ_BEGIN", 0) != 0) {
                if (!line.empty()) {
                    error = "unexpected stdout before TRAJ_BEGIN: '" + line + "'";
                    return false;
                }
                continue;
            }
            std::istringstream header(line);
            std::string tag;
            header >> tag >> expected;
            if (header.fail() || expected < 2) {
                error = "unparseable TRAJ_BEGIN header: '" + line + "'";
                return false;
            }
            begun = true;
            samples.reserve(expected);
            continue;
        }
        if (line == "TRAJ_END") {
            if (samples.size() != expected) {
                error = "TRAJ_END after " + std::to_string(samples.size()) +
                        " rows, header promised " + std::to_string(expected);
                return false;
            }
            return true;
        }
        WireSample sample;
        std::istringstream row(line);
        row >> sample.t_s;
        for (int j = 0; j < 7; ++j) row >> sample.q_deg(j);
        for (int j = 0; j < 7; ++j) row >> sample.qdot_deg_s(j);
        std::string trailing;
        if (row.fail() || (row >> trailing)) {
            error = "wire row is not 15 numbers: '" + line + "'";
            return false;
        }
        if (!std::isfinite(sample.t_s) || !sample.q_deg.allFinite() ||
            !sample.qdot_deg_s.allFinite()) {
            error = "non-finite value in wire row: '" + line + "'";
            return false;
        }
        if (samples.empty()) {
            if (sample.t_s != 0.0) {
                error = "first wire sample must be at t=0, got " +
                        std::to_string(sample.t_s);
                return false;
            }
        } else if (!(sample.t_s > samples.back().t_s)) {
            error = "wire times must strictly increase (row " +
                    std::to_string(samples.size()) + ")";
            return false;
        }
        if (samples.size() == expected) {
            error = "more wire rows than the header promised";
            return false;
        }
        samples.push_back(sample);
    }
    error = begun ? "stdout ended before TRAJ_END"
                  : "no TRAJ_BEGIN block on the baseline planner's stdout";
    return false;
}

}  // namespace

PlannerSolveResult SolveBaselineBridgeForRequest(
    const PlanningRequest& request,
    const PlannerRuntimeConfig& config,
    std::ostream& diagnostics)
{
    PlannerSolveResult result;
    if (config.baseline_bridge_binary.empty()) {
        diagnostics << "baseline planner selected but no binary configured\n";
        return result;
    }

    const bool left = request.arm == PlanningArm::kLeft;

    // The one accommodation: principal-value wrap BEFORE the subprocess,
    // reproducing the baseline's CSV-transport semantics on its (unwrapped)
    // --start-deg transport. See BaselineBridge.h.
    Eigen::Matrix<double, 7, 1> q_wrapped_rad;
    for (int j = 0; j < 7; ++j)
        q_wrapped_rad(j) = WrapToPrincipalRad(request.q_rad(j));

    // stderr (the baseline's full report and diagnostics) goes to a file so
    // the block on stdout stays uninterleaved, then is replayed verbatim
    // into this solve's diagnostics — the panel's plan verdict reads the
    // same verdict / e_command lines it always has.
    char stderr_path[] = "/tmp/baseline_bridge_stderr_XXXXXX";
    const int stderr_fd = mkstemp(stderr_path);
    if (stderr_fd < 0) {
        diagnostics << "baseline planner: cannot create stderr capture file\n";
        return result;
    }
    close(stderr_fd);

    std::ostringstream command;
    command << Quoted(config.baseline_bridge_binary)
            << " --arm " << (left ? "left" : "right")
            << " --goal-file " << Quoted(config.goal_file)
            << " --start-deg";
    for (int j = 0; j < 7; ++j) {
        char value[32];
        std::snprintf(value, sizeof value, " %.17g",
                      q_wrapped_rad(j) * 180.0 / M_PI);
        command << value;
    }
    command << " 2> " << Quoted(stderr_path);

    diagnostics << "planner implementation: BASELINE bridge (5abc1b2c) at "
                << config.baseline_bridge_binary << "\n"
                << "  arm: " << (left ? "left" : "right")
                << ", goal file: " << config.goal_file << "\n"
                << "  tuning/limits/DH: the baseline's own, resolved beside "
                   "its binary (frozen 2026-08-07 values)\n"
                << "  start (wrapped principal deg):";
    for (int j = 0; j < 7; ++j)
        diagnostics << " " << q_wrapped_rad(j) * 180.0 / M_PI;
    diagnostics << "\n";

    FILE* pipe = popen(command.str().c_str(), "r");
    if (pipe == nullptr) {
        diagnostics << "baseline planner: popen failed\n";
        std::remove(stderr_path);
        return result;
    }
    std::string wire;
    char buffer[4096];
    std::size_t got = 0;
    while ((got = std::fread(buffer, 1, sizeof buffer, pipe)) > 0)
        wire.append(buffer, got);
    const int raw_status = pclose(pipe);

    // The baseline's own report, verbatim, before any adapter verdict — so
    // the log always shows what the PLANNER said even when adaptation fails.
    {
        std::ifstream captured(stderr_path);
        diagnostics << captured.rdbuf();
    }
    std::remove(stderr_path);

    const int exit_code =
        WIFEXITED(raw_status) ? WEXITSTATUS(raw_status) : -1;
    result.exit_code = exit_code == 0 ? 1 : exit_code;  // 0 only after adaptation
    if (exit_code != 0) {
        diagnostics << "baseline planner exit code " << exit_code
                    << " (0 emitted, 1 args, 2 start state, 3 solve, "
                       "4 validation rejected) — nothing to adapt\n";
        return result;
    }

    std::vector<WireSample> samples;
    std::string parse_error;
    if (!ParseWireBlock(wire, samples, parse_error)) {
        diagnostics << "baseline planner emitted, but its wire block did not "
                       "adapt: " << parse_error << "\n";
        return result;
    }

    // deg -> rad, the only unit conversion in the adapter. Times are used
    // exactly as emitted; the block is uniformly spaced by construction
    // (FormatTrajectoryBlock), and the projection re-derives the same
    // spacing from total_time/(N-1).
    std::vector<gtsam::Vector> position_rad, velocity_rad_s;
    position_rad.reserve(samples.size());
    velocity_rad_s.reserve(samples.size());
    constexpr double kDegToRad = M_PI / 180.0;
    for (const WireSample& sample : samples) {
        gtsam::Vector q(7), qdot(7);
        for (int j = 0; j < 7; ++j) {
            q(j) = sample.q_deg(j) * kDegToRad;
            qdot(j) = sample.qdot_deg_s(j) * kDegToRad;
        }
        position_rad.push_back(q);
        velocity_rad_s.push_back(qdot);
    }
    const double total_time_s = samples.back().t_s;

    // Projection through the CURRENT model — the same FK the controller
    // itself uses. The model is mount-frame; the request's world_T_mount
    // enters only here at the output projection, so a mount-fixed session
    // (identity) executes in mount semantics end to end.
    PlannerModel model = LoadPlannerModel(
        left ? config.left_dh_file : config.right_dh_file,
        /*has_tool=*/!left);
    result.trajectory =
        std::make_unique<WorldCartesianTrajectory>(ProjectWorldTrajectory(
            model, request.world_T_mount, position_rad, velocity_rad_s,
            total_time_s,
            request.request_id, request.vicon_sequence));
    result.exit_code = 0;

    diagnostics << "baseline block adapted: " << samples.size()
                << " joint samples (deg wire), duration " << total_time_s
                << " s, dt "
                << total_time_s / static_cast<double>(samples.size() - 1)
                << " s -> " << result.trajectory->points.size()
                << " world-Cartesian points via the CURRENT model's FK ("
                << model.end_effector_frame << ", "
                << (left ? "left" : "right")
                << " chain), trajectory id " << request.request_id
                << ", vicon sequence " << request.vicon_sequence << "\n"
                << "  frame note: the baseline resolved mount-frame goals "
                   "against its OWN 2026-08-07 URDF (base spacing 0.113 m); "
                   "projection and execution use the CURRENT model. The "
                   "joint trajectory passes through 1:1.\n";
    return result;
}
