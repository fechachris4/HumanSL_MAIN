//
// plan_move: offline planner CLI — runs the existing TrajectoryGeneration
// stack (GPMP2/GTSAM, unmodified) for ONE free-space right-arm move and
// writes the result as a trajectory_format = 1 CSV that basic_control's
// playback controller can execute (contract:
// Christian_control/basic_control/src/control/TrajectoryFile.h).
//
// This tool NEVER touches the robot. Planning happens here, offline;
// execution happens later, in basic_control, after its own validation
// and start-state gates.
//
// Frames: the planner's arm model is built with an IDENTITY base pose, so
// Cartesian goals are expressed in the RIGHT-ARM BASE frame (the frame of
// the arm's own base link, x forward / z up as in the Gen3 DH convention).
// The trajectory itself is joint-space and frame-free. The end-effector
// point is the DH model's grasp point (dh_params.yaml extends the last
// link past the flange), so it differs from the URDF flange frame that
// basic_control's FK cross-check prints — compare DISPLACEMENT NORMS
// between the two printouts (frame- and tool-offset-invariant), not
// absolute positions.
//
// Scope (first milestone): free space only — the SDF is a uniform
// 10 m clearance; no obstacles, no Vicon.
//
//   ./plan_move --start "q1 q2 q3 q4 q5 q6 q7"   degrees, as printed by
//                                                 the controller at startup
//               --goal-joints "q1 .. q7"          degrees (recommended: the
//                                                 goal pose is the FK of a
//                                                 known-reachable config)
//     OR        --goal "x y z"                    meters, right-arm base
//                                                 frame; orientation held
//                                                 at the start pose's
//                                                 unless --goal-rpy is given
//               [--goal-rpy "roll pitch yaw"]     radians, R = Rz*Ry*Rx
//               [--duration <s>]                  default 8
//               [--timesteps <n>]                 GPMP2 knots, default 10
//               [--out <file>]                    default plan_<timestamp>.csv
//
// The written file is immediately RE-LOADED and validated with the same
// code and gates basic_control applies before a run (velocity <= 90% of
// the command clip, Kinova Table 43 acceleration, consistency, rest at
// both ends). A FAIL keeps the file for inspection but exits nonzero —
// fix by increasing --duration or shrinking the move.
//

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "GenerateArmModel.h"
#include "TrajectoryInitiation.h"
#include "TrajectoryOptimization.h"
#include "utils.h"

#include "control/TrajectoryFile.h" // basic_control's contract + validation

namespace
{
    constexpr int NUM_JOINTS = 7;
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    // The same gates basic_control compiles in (src/app/Config.h). Kept in
    // sync by the shared validation code reading these exact numbers there;
    // duplicated here because this standalone tool does not link the app.
    const JointVector kQdotClipDegS = {79.6, 79.6, 79.6, 79.6, 69.9, 69.9, 69.9};
    constexpr double kVelGateFactor = 0.9;
    const JointVector kAccelLimitDegS2 = {57.3, 57.3, 57.3, 57.3,
                                          573.0, 573.0, 573.0};

    // Legacy acceptance threshold for the optimizer's final graph error
    // (the deleted plan.cpp retried until final_error < 100); the REAL
    // gate is the trajectory validation below.
    constexpr double kFinalErrorAcceptance = 100.0;
    constexpr int kMaxPlanAttempts = 20;

    [[noreturn]] void UsageAndExit(const std::string& error)
    {
        if (!error.empty())
            std::cerr << "error: " << error << "\n";
        std::cerr
            << "usage: plan_move --start \"q1 .. q7\" (deg)\n"
            << "                 --goal-joints \"q1 .. q7\" (deg)\n"
            << "               | --goal \"x y z\" (m, right-arm base frame)\n"
            << "                 [--goal-rpy \"roll pitch yaw\"] (rad)\n"
            << "                 [--duration <s>=8] [--timesteps <n>=10]\n"
            << "                 [--out <file>]\n";
        std::exit(2);
    }

    std::vector<double> ParseNumbers(const std::string& text, size_t expected,
                                     const std::string& what)
    {
        std::vector<double> values;
        std::istringstream in(text);
        double v;
        while (in >> v)
            values.push_back(v);
        if (values.size() != expected)
            UsageAndExit(what + " needs exactly " + std::to_string(expected) +
                         " numbers (got " + std::to_string(values.size()) + ")");
        for (double x : values)
            if (!std::isfinite(x))
                UsageAndExit(what + " contains a non-finite value");
        return values;
    }
} // namespace

int main(int argc, char** argv)
{
    std::string start_text, goal_joints_text, goal_text, goal_rpy_text, out_path;
    double duration_s = 8.0;
    int timesteps = 10;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string
        {
            if (i + 1 >= argc)
                UsageAndExit(arg + " needs a value");
            return argv[++i];
        };
        if (arg == "--start") start_text = value();
        else if (arg == "--goal-joints") goal_joints_text = value();
        else if (arg == "--goal") goal_text = value();
        else if (arg == "--goal-rpy") goal_rpy_text = value();
        else if (arg == "--duration") duration_s = std::atof(value().c_str());
        else if (arg == "--timesteps") timesteps = std::atoi(value().c_str());
        else if (arg == "--out") out_path = value();
        else if (arg == "--help") UsageAndExit("");
        else UsageAndExit("unknown option '" + arg + "'");
    }
    if (start_text.empty())
        UsageAndExit("--start is required");
    if (goal_joints_text.empty() == goal_text.empty())
        UsageAndExit("exactly one of --goal-joints / --goal is required");
    if (!(duration_s > 0.5) || duration_s > 60.0)
        UsageAndExit("--duration must be in (0.5, 60] s");
    if (timesteps < 2 || timesteps > 100)
        UsageAndExit("--timesteps must be in [2, 100]");

    // Config: the repository-root config/ (the copy the legacy binaries
    // loaded and the one consistent with the IK solvers' hardcoded tool
    // length — NOT the stale module-local copy).
    const std::string config_dir = PLAN_CONFIG_DIR;
    const DHParameters dh = createDHParams(config_dir + "/dh_params.yaml");
    const auto [pos_limits, vel_limits] =
        createJointLimits(config_dir + "/joint_limits.yaml");

    // Identity base: goals are in the right-arm base frame (header note).
    const gtsam::Pose3 base_pose;

    // Start configuration: degrees as the arm reports them, wrapped to
    // (-180, 180] and converted — the same normalization the legacy
    // pipeline applied before planning.
    gtsam::Vector start_rad(NUM_JOINTS);
    {
        const std::vector<double> q = ParseNumbers(start_text, NUM_JOINTS, "--start");
        for (int j = 0; j < NUM_JOINTS; ++j)
            start_rad[j] = std::remainder(q[j], 360.0) * kDegToRad;
    }
    const gtsam::Vector start_vel = gtsam::Vector::Zero(NUM_JOINTS);

    // Goal pose.
    gtsam::Pose3 goal_pose;
    std::string goal_description;
    if (!goal_joints_text.empty())
    {
        const std::vector<double> q =
            ParseNumbers(goal_joints_text, NUM_JOINTS, "--goal-joints");
        gtsam::Vector goal_rad(NUM_JOINTS);
        for (int j = 0; j < NUM_JOINTS; ++j)
            goal_rad[j] = std::remainder(q[j], 360.0) * kDegToRad;
        goal_pose = forwardKinematics(dh, goal_rad, base_pose);
        goal_description = "joints(deg) " + goal_joints_text;
    }
    else
    {
        const std::vector<double> p = ParseNumbers(goal_text, 3, "--goal");
        gtsam::Rot3 rotation;
        if (!goal_rpy_text.empty())
        {
            const std::vector<double> rpy =
                ParseNumbers(goal_rpy_text, 3, "--goal-rpy");
            rotation = gtsam::Rot3::Rz(rpy[2]) * gtsam::Rot3::Ry(rpy[1]) *
                       gtsam::Rot3::Rx(rpy[0]);
        }
        else
        {
            // Hold the start pose's tool orientation.
            rotation = forwardKinematics(dh, start_rad, base_pose).rotation();
        }
        goal_pose = gtsam::Pose3(rotation, gtsam::Point3(p[0], p[1], p[2]));
        goal_description = "position(m,base) " + goal_text +
                           (goal_rpy_text.empty() ? " orientation held"
                                                  : " rpy(rad) " + goal_rpy_text);
    }

    ArmModel builder;
    const std::unique_ptr<gpmp2::ArmModel> arm =
        builder.createArmModel(base_pose, dh);

    // Free-space SDF: 4 x 4 x 4 m around the base, 10 cm cells, 10 m
    // clearance everywhere (first-milestone scope: no obstacles).
    const size_t grid = 40;
    gpmp2::SignedDistanceField sdf(gtsam::Point3(-2.0, -2.0, -2.0), 0.1, grid,
                                   grid, grid);
    const gtsam::Matrix layer = gtsam::Matrix::Constant(grid, grid, 10.0);
    for (size_t z = 0; z < grid; ++z)
        sdf.initFieldData(z, layer);

    const gtsam::Pose3 start_ee = forwardKinematics(dh, start_rad, base_pose);
    std::cout << std::fixed << std::setprecision(4)
              << "start EE (DH grasp point, base frame): "
              << start_ee.translation().transpose() << " m\n"
              << "goal  EE (DH grasp point, base frame): "
              << goal_pose.translation().transpose() << " m\n"
              << std::defaultfloat;

    // Plan: init (IK + straight-line seed) -> optimize -> densify to 1 kHz.
    // Retried because the IK initialization is stochastic.
    InitializeTrajectory initializer(dh);
    OptimizeTrajectory optimizer;
    TrajectoryResult result;
    bool accepted = false;
    for (int attempt = 1; attempt <= kMaxPlanAttempts && !accepted; ++attempt)
    {
        try
        {
            const gtsam::Values seed = initializer.initJointTrajectoryFromTarget(
                start_rad, goal_pose, base_pose,
                static_cast<size_t>(timesteps));
            result = optimizer.optimizeJointTrajectory(
                *arm, sdf, seed, goal_pose, start_rad, start_vel, pos_limits,
                vel_limits, static_cast<size_t>(timesteps), duration_s,
                /*target_dt=*/0.001);
            accepted = result.final_error < kFinalErrorAcceptance;
            std::cout << "attempt " << attempt << ": final_error "
                      << result.final_error << (accepted ? " (accepted)" : "")
                      << "\n";
        }
        catch (const std::exception& e)
        {
            std::cout << "attempt " << attempt << ": " << e.what() << "\n";
        }
    }
    if (!accepted)
    {
        std::cerr << "error: no plan reached final_error < "
                  << kFinalErrorAcceptance << " in " << kMaxPlanAttempts
                  << " attempts — is the goal reachable?\n";
        return 1;
    }

    const JointTrajectory trajectory =
        convertTrajectory<JointTrajectory>(result, result.dt);

    const gtsam::Vector final_rad = result.trajectory_pos.back();
    const gtsam::Pose3 final_ee = forwardKinematics(dh, final_rad, base_pose);
    const gtsam::Point3 ee_move = final_ee.translation() - start_ee.translation();
    std::cout << std::fixed << std::setprecision(4)
              << "final EE (DH grasp point, base frame): "
              << final_ee.translation().transpose() << " m\n"
              << "EE displacement: " << ee_move.transpose() << " m (|d| = "
              << ee_move.norm() << " m)\n"
              << "goal-vs-final EE gap: "
              << (goal_pose.translation() - final_ee.translation()).norm()
              << " m\n"
              << std::defaultfloat;

    // Output file, contract format. High precision so the consistency
    // validation sees the planner's numbers, not quantization.
    if (out_path.empty())
    {
        char stamp[32];
        const std::time_t now = std::time(nullptr);
        std::strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S",
                      std::localtime(&now));
        out_path = std::string("plan_") + stamp + ".csv";
    }
    {
        std::ofstream out(out_path);
        if (!out)
        {
            std::cerr << "error: cannot open " << out_path << "\n";
            return 1;
        }
        out << std::setprecision(10);
        out << "# trajectory_format = 1\n";
        out << "# source = plan_move (TrajectoryGeneration GPMP2, free-space SDF)\n";
        out << "# start_deg = " << start_text << "\n";
        out << "# goal = " << goal_description << "\n";
        out << "# duration_s = " << duration_s << "\n";
        out << "# timesteps = " << timesteps << "\n";
        out << "# final_error = " << result.final_error << "\n";
        out << "# dh_config = " << config_dir << "/dh_params.yaml\n";
        out << "# ee_displacement_base_m = " << ee_move.x() << " " << ee_move.y()
            << " " << ee_move.z() << " (norm " << ee_move.norm() << ")\n";
        out << "t_s";
        for (int j = 1; j <= NUM_JOINTS; ++j) out << ",q" << j << "_deg";
        for (int j = 1; j <= NUM_JOINTS; ++j) out << ",qd" << j << "_degs";
        out << "\n";
        for (size_t i = 0; i < trajectory.pos.size(); ++i)
        {
            out << i * result.dt;
            for (int j = 0; j < NUM_JOINTS; ++j)
                out << "," << trajectory.pos[i][j];
            for (int j = 0; j < NUM_JOINTS; ++j)
                out << "," << trajectory.vel[i][j];
            out << "\n";
        }
    }
    std::cout << "wrote " << out_path << " (" << trajectory.pos.size()
              << " samples at " << result.dt << " s)\n";

    // The gate: re-load the file we just wrote and validate it EXACTLY as
    // basic_control will before a run.
    JointVector vel_gate;
    for (size_t j = 0; j < vel_gate.size(); ++j)
        vel_gate[j] = kVelGateFactor * kQdotClipDegS[j];
    TrajectorySummary summary;
    Trajectory reloaded = LoadTrajectoryCsv(out_path);
    const std::vector<std::string> violations =
        ValidateTrajectory(reloaded, vel_gate, kAccelLimitDegS2, summary);

    std::cout << "validation summary:\n  duration " << summary.duration_s
              << " s, " << summary.samples << " samples\n  joint displacement (deg):";
    for (int j = 0; j < NUM_JOINTS; ++j)
        std::cout << " " << summary.displacement_deg[j];
    std::cout << "\n  peak velocity (deg/s):";
    for (int j = 0; j < NUM_JOINTS; ++j)
        std::cout << " " << summary.peak_vel_deg_s[j];
    std::cout << "\n  peak acceleration (deg/s^2):";
    for (int j = 0; j < NUM_JOINTS; ++j)
        std::cout << " " << summary.peak_accel_deg_s2[j];
    std::cout << "\n";

    if (!violations.empty())
    {
        std::cerr << "validation: FAIL (" << violations.size() << " violation"
                  << (violations.size() == 1 ? "" : "s") << "):\n";
        for (const std::string& v : violations)
            std::cerr << "  - " << v << "\n";
        std::cerr << "the file is kept for inspection but MUST NOT be "
                     "executed — increase --duration or shrink the move\n";
        return 1;
    }
    std::cout << "validation: PASS (same gates basic_control applies "
                 "before a run)\n";
    return 0;
}
