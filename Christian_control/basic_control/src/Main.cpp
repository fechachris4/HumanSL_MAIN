//
// Main — the program: parse --log, load the model, connect, check
// readiness, wire the reference source to the controller, run the loop,
// report. The loop itself lives in Runner.h/.cpp.
//

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <tuple>
#include <vector>

#include <KDetailedException.h>

#include "Actuation.h"
#include "Config.h"
#include "Controller.h"
#include "Hardware.h"
#include "Kinematics.h"
#include "Runner.h"
#include "Safety.h"
#include "State.h"
#include "Targets.h"
#include "Trajectory.h"

namespace k_api = Kinova::Api;

// ---------------------------------------------------------------
// run configuration echo (startup print + CSV preamble)
// ---------------------------------------------------------------

// --log is the only runtime-selectable value; every controller setting is
// compiled in Config.h. The same key = value lines serve as the startup
// echo and, '#'-prefixed, as the CSV preamble that makes every data file
// self-describing (parsers skip '#' lines).

namespace {
[[noreturn]] void UsageAndExit(const std::string& error) {
    if (!error.empty()) std::cerr << "error: " << error << "\n";
    std::cerr << "usage: controller [--log <file>]\n"
              << "  --log <file>          CSV filename (default: timestamped run file)\n"
              << "All controller settings are compiled in src/Config.h.\n";
    std::exit(2);
}

std::string FormatDouble(double value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

void WriteConfigLines(const std::string& log_file, std::ostream& out, const char* prefix) {
    const auto line = [&](const char* key, const std::string& value) {
        out << prefix << key << " = " << value << " (Config.h)\n";
    };
    line("reference_source", config::kReferenceSource);
    line("target_file", config::kTargetFile[0] ? config::kTargetFile : "<none>");
    line("trajectory_file", config::kTrajectoryFile[0] ? config::kTrajectoryFile : "<none>");
    line("playback_kp", FormatDouble(config::kPlaybackKp));
    line("start_mismatch_limit_deg", FormatDouble(config::kStartMismatchLimitDeg));
    line("kp", FormatDouble(config::kKpCartesian));
    line("dls_lambda", FormatDouble(config::kDlsLambda));
    line("kp_rot", FormatDouble(config::kKpRotation));
    line("kd_pos", FormatDouble(config::kKdPosition));
    line("kd_rot", FormatDouble(config::kKdRotation));
    line("null_gain", FormatDouble(config::kNullGain));
    line("orientation_enabled", config::kOrientationEnabled ? "true" : "false");
    line("velocity_term_enabled", config::kVelocityTermEnabled ? "true" : "false");
    line("null_space_enabled", config::kNullSpaceEnabled ? "true" : "false");
    line("use_fixed_target", config::kUseFixedTarget ? "true" : "false");
    if (config::kUseFixedTarget) {
        line("fixed_target_m", FormatDouble(config::kFixedTargetM[0]) + " " +
                                   FormatDouble(config::kFixedTargetM[1]) + " " +
                                   FormatDouble(config::kFixedTargetM[2]));
        line("fixed_target_use_rpy", config::kFixedTargetUseRpy ? "true" : "false");
        if (config::kFixedTargetUseRpy)
            line("fixed_target_rpy_rad",
                 FormatDouble(config::kFixedTargetRpyRad[0]) + " " +
                     FormatDouble(config::kFixedTargetRpyRad[1]) + " " +
                     FormatDouble(config::kFixedTargetRpyRad[2]));
    }
    line("following_error_limit_deg", FormatDouble(config::kFollowingErrorLimitDeg));
    // Guard overrides — recorded on every run so a log can be read back
    // knowing which protections were active while it was captured.
    line("stop_on_fault", config::kStopOnFault ? "true" : "false");
    line("allow_unverified_actuators",
         config::kAllowUnverifiedActuators ? "true" : "false");
    line("skip_startup_gates", config::kSkipStartupGates ? "true" : "false");
    line("disable_following_error_stop",
         config::kDisableFollowingErrorStop ? "true" : "false");
    line("arrival_tolerance_m", FormatDouble(config::kArrivalToleranceM));
    line("nonfinite_stop_cycles", std::to_string(config::kNonFiniteStopCycles));
    line("overrun_stop_cycles", std::to_string(config::kOverrunStopCycles));
    line("overrun_factor", FormatDouble(config::kOverrunFactor));
    out << prefix << "log_file = " << (log_file.empty() ? "<timestamped>" : log_file)
        << (log_file.empty() ? " (default)\n" : " (--log)\n");
    line("control_dt_s", FormatDouble(config::kControlDtS));
}

std::string ParseLogFileArg(int argc, char** argv) {
    std::string log_file;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) != "--log") UsageAndExit("unknown option '" + std::string(argv[i]) + "'");
        if (++i >= argc) UsageAndExit("--log needs a filename");
        log_file = argv[i];
    }
    return log_file;
}

void WriteCsvPreamble(const std::string& log_file, std::ostream& csv) {
    csv << "# controller run config — parsers skip '#' lines\n";
    csv << "# log_format = 5 (compiled)\n";
    WriteConfigLines(log_file, csv, "# ");
}
} // namespace

/*
 * The story of the program, told at a high level:
 *
 *   parse --log (the only runtime argument; everything else is Config.h)
 *     -> load the mounted dual-arm URDF (Pinocchio; exactly 14 velocity vars)
 *     -> bind measured right joints + nominal left joints through the explicit
 *        DualArmKinematics 14-DoF-model/7-controller adapter
 *     -> connect only to the right arm (TCP + UDP, Connect)
 *     -> readiness check on one feedback frame (before any takeover)
 *     -> print the current joint state and end-effector position
 *     -> start the desired-target input thread (stdin: x y z meters, plus
 *        roll pitch yaw radians; operator source only)
 *     -> run the control loop (RunControlLoop above — MOVES THE ARM:
 *        takeover sequence T1-T5, the selected reference source feeding
 *        the TrackingController + PositionIntegration actuation,
 *        single-level servoing restored on every exit path)
 *     -> stop the input thread, drain the last of the loop log (the rest
 *        of the CSV was written by the writer thread as the run happened)
 *     -> RAII teardown, exit 0 only on a clean operator stop.
 *
 * Usage: ./controller
 */




namespace
{

    constexpr int kLabelWidth = 16;
    constexpr int kJointColumnWidth = 10;

    void PrintJointHeader()
    {
        std::cout << std::left << std::setw(kLabelWidth) << "joint" << std::right;
        for (size_t i = 0; i < std::tuple_size_v<JointVector>; ++i)
            std::cout << std::setw(kJointColumnWidth) << i + 1;
        std::cout << "\n";
    }

    void PrintRow(const char* label, const JointVector& values)
    {
        std::cout << std::left << std::setw(kLabelWidth) << label << std::right << std::fixed
                  << std::setprecision(2);
        for (double v : values)
            std::cout << std::setw(kJointColumnWidth) << v;
        std::cout << std::defaultfloat << "\n";
    }

    // Current joint state plus the end-effector position (our FK) — the
    // printed p is what a "hold here" desired position looks like, so the
    // operator can start from it and edit one coordinate.
    void PrintRobotState(const k_api::BaseCyclic::Feedback& feedback,
                         DualArmKinematics& model)
    {
        JointVector position_deg;
        JointVector velocity_deg_s;
        Eigen::Matrix<double, 7, 1> q_rad;
        for (size_t i = 0; i < position_deg.size(); ++i) {
            position_deg[i] = feedback.actuators(i).position();
            velocity_deg_s[i] = feedback.actuators(i).velocity();
            q_rad[static_cast<int>(i)] = position_deg[i] * M_PI / 180.0;
        }
        PrintJointHeader();
        PrintRow("position deg", position_deg);
        PrintRow("velocity deg/s", velocity_deg_s);

        KinematicsWorkspace workspace(model.dynamics());
        const PoseJacobian ee = model.RightPoseAndJacobian(q_rad, workspace);
        // Decompose R back into the SAME roll/pitch/yaw convention targets
        // use (R = Rz·Ry·Rx), so the printed triple can be pasted straight
        // into kFixedTargetRpyRad or typed as the last 3 of a 6-number line.
        const Eigen::Vector3d zyx = ee.rotation.eulerAngles(2, 1, 0);
        std::cout << "right end-effector (" << config::kRightEndEffectorFrame
                  << " in " << config::kRightBaseFrame << "): "
                  << std::fixed << std::setprecision(4)
                  << ee.position.x() << " " << ee.position.y() << " "
                  << ee.position.z()
                  << " (m, right-arm base frame)\n"
                  << "  orientation rpy: " << zyx.z() << " " << zyx.y() << " "
                  << zyx.x() << " (rad, R = Rz*Ry*Rx)"
                  << std::defaultfloat << "\n";
    }

} // namespace

// Ctrl+C (SIGINT) and SIGTERM (CLion Stop, kill) set this flag; the loop
// and the input thread both observe it. Both signals must leave through
// the same graceful path — stop report, CSV drain, servoing restore —
// or an IDE stop kills the process mid-takeover with none of them.
std::atomic<bool> g_stop{false};
void on_stop_signal(int)
{
    g_stop = true;
}

// Joins the input thread on scope exit, whatever the exit path: an
// exception between thread start and the explicit join must not reach
// std::thread's destructor on a joinable thread — that calls
// std::terminate, aborting past every cleanup (no stop report, no CSV,
// and before the loop's catch-all existed, no servoing restore either).
struct InputThreadJoiner {
    std::thread& thread;
    ~InputThreadJoiner()
    {
        g_stop = true;
        if (thread.joinable())
            thread.join();
    }
};

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_stop_signal);
    std::signal(SIGTERM, on_stop_signal);

    const std::string log_file_arg = ParseLogFileArg(argc, argv);

    try {
        std::cout << "controller config (Config.h):\n";
        WriteConfigLines(log_file_arg, std::cout, "  ");
        const bool playback = std::string(config::kReferenceSource) == "trajectory";
        const bool operator_targets = std::string(config::kReferenceSource) == "operator";
        if (!playback && !operator_targets)
            throw std::runtime_error("Config.h kReferenceSource must be operator or trajectory");
        if (config::kTargetFile[0] != '\0' && !operator_targets)
            throw std::runtime_error("Config.h kTargetFile requires the operator source");
        if (playback && config::kTrajectoryFile[0] == '\0')
            throw std::runtime_error("Config.h kTrajectoryFile is required for the trajectory source");

        // Playback: load and validate the trajectory BEFORE any hardware
        // session — a file that fails the contract or the motion gates must
        // never reach a takeover. Violations print in full and stop the run.
        std::optional<Trajectory> trajectory;
        TrajectorySummary trajectory_summary;
        if (playback) {
            trajectory = LoadTrajectoryCsv(config::kTrajectoryFile); // throws on
                                                                 // contract violations
            JointVector vel_gate_deg_s;
            for (size_t i = 0; i < vel_gate_deg_s.size(); ++i)
                vel_gate_deg_s[i] =
                    config::kTrajectoryVelGateFactor * config::kQdotLimitDegS[i];
            const std::vector<std::string> violations = ValidateTrajectory(
                *trajectory, vel_gate_deg_s, config::kTrajectoryAccelLimitDegS2,
                config::kTrajectoryPosLimitDeg, trajectory_summary);
            std::cout << "trajectory " << config::kTrajectoryFile << ":\n  "
                      << trajectory_summary.samples << " samples, dt "
                      << trajectory->dt_s << " s, duration "
                      << trajectory_summary.duration_s << " s\n";
            PrintJointHeader();
            PrintRow("displace deg", trajectory_summary.displacement_deg);
            PrintRow("peak vel deg/s", trajectory_summary.peak_vel_deg_s);
            PrintRow("peak acc deg/s2", trajectory_summary.peak_accel_deg_s2);
            for (const auto& [key, value] : trajectory->metadata)
                std::cout << "  # " << key << " = " << value << "\n";
            if (!violations.empty()) {
                std::cerr << "Error: trajectory FAILED validation ("
                          << violations.size() << " violation"
                          << (violations.size() == 1 ? "" : "s")
                          << ") — not starting:\n";
                for (const std::string& v : violations)
                    std::cerr << "  - " << v << "\n";
                return 1;
            }
            std::cout << "trajectory validation: PASS (vel gate "
                      << config::kTrajectoryVelGateFactor
                      << " x qdot clip, accel gate Kinova Table 43)\n";
        }

        // Full mounted model + explicit right-arm adapter before any hardware
        // session. The adapter validates nq=nv=14 and the exact named mapping
        // to the seven-wide controller interface.
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics controlled_model(
            dynamics, config::kLeftNominalRad,
            config::kRightBaseFrame,
            config::kRightEndEffectorFrame);

        // The left branch is model-only: this is the program's sole hardware
        // connection, and it is always the right arm.
        Connect connection(config::kRightRobotIp);

        // Clearing faults (TrajectoryExecution's pattern): unconditional,
        // immediately after connecting, before anything else talks to the
        // arm. Commands no motion; a latched leftover from a previous run
        // clears, and anything LIVE simply re-latches and is caught by the
        // readiness gate below, which runs on a fresh post-clear read.
        try {
            connection.base()->ClearFaults();
        } catch (...) {
            std::cout << "Unable to clear robot faults" << std::endl;
            return 1;
        }
        // The base needs a moment to re-arm the actuators after a clear
        // before the gate reads a frame that reflects it.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Readiness check on a standalone read, BEFORE the takeover: a live
        // fault means we never enter low-level servoing at all.
        const k_api::BaseCyclic::Feedback initial = read_feedback(connection.base_cyclic());

        const bool robot_ready = RobotReadyForTakeover(initial, std::cout); // T1
        if (!robot_ready)
            return 1;
        // Joint limits FIRST. They do not survive a robot power cycle, and a
        // degenerate 0/0 band makes the firmware fault any motion away from
        // zero (Config.h). Restoring them is the more important of the two
        // gates and must not be blocked by the other: until 2026-08-04 the
        // control-mode probe ran first, so a single actuator whose
        // configuration service had stopped answering exited the program
        // before any limit was ever re-applied.
        if (config::kSkipStartupGates)
            std::cout
                << "WARNING: STARTUP GATES SKIPPED (config::kSkipStartupGates)"
                   " — no control mode verified, and the joint 4/6 JOINT_LIMIT\n"
                   "         bands are NOT restored. A band left at 0/0 faults"
                   " outward motion at the firmware level.\n";
        else {
            if (!connection.EnsureJointLimits(std::cout))
                return 1;
            if (!connection.EnsurePositionControlModes(std::cout))
                return 1;
        }
        PrintRobotState(initial, controlled_model);

        if (playback) {
            // Start-state gate on the pre-takeover read: the arm must
            // already BE at the trajectory's first row (wrapped compare —
            // the file's continuous angles vs the arm's [0,360) feedback).
            // TrajectorySource::Reset re-checks at the takeover itself.
            bool start_ok = true;
            std::cout << "start-state gate (per-joint limit "
                      << config::kStartMismatchLimitDeg << " deg):\n";
            JointVector start_error_deg{};
            for (size_t j = 0; j < start_error_deg.size(); ++j) {
                start_error_deg[j] = std::abs(std::remainder(
                    initial.actuators(static_cast<int>(j)).position() -
                        trajectory->pos_deg.front()[static_cast<int>(j)],
                    360.0));
                if (start_error_deg[j] > config::kStartMismatchLimitDeg)
                    start_ok = false;
            }
            PrintJointHeader();
            PrintRow("|meas-start| deg", start_error_deg);
            if (!start_ok) {
                std::cerr << "Error: measured position does not match the "
                             "trajectory start — refusing to start. Re-plan "
                             "from the current position (printed above) or "
                             "move the arm to the trajectory start first.\n";
                return 1;
            }
            std::cout << "start-state gate: PASS\n";

            // FK cross-check, visible before execution: what this
            // trajectory means for the end-effector according to the URDF
            // dual model (the controller's own kinematic authority — an
            // independent check on the planner's DH model).
            KinematicsWorkspace fk_workspace(controlled_model.dynamics());
            Eigen::Matrix<double, 7, 1> q_row_rad;
            for (int j = 0; j < 7; ++j)
                q_row_rad[j] = trajectory->pos_deg.front()[j] * M_PI / 180.0;
            const PoseJacobian ee_start =
                controlled_model.RightPoseAndJacobian(q_row_rad, fk_workspace);
            for (int j = 0; j < 7; ++j)
                q_row_rad[j] = trajectory->pos_deg.back()[j] * M_PI / 180.0;
            const PoseJacobian ee_end =
                controlled_model.RightPoseAndJacobian(q_row_rad, fk_workspace);
            const Eigen::Vector3d ee_move =
                ee_end.position - ee_start.position;
            std::cout << std::fixed << std::setprecision(4)
                      << "URDF FK cross-check (right-arm base frame, "
                      << config::kRightEndEffectorFrame << "):\n"
                      << "  start EE: " << ee_start.position.x() << " "
                      << ee_start.position.y() << " " << ee_start.position.z()
                      << " m\n"
                      << "  final EE: " << ee_end.position.x() << " "
                      << ee_end.position.y() << " " << ee_end.position.z()
                      << " m\n"
                      << "  displacement: " << ee_move.x() << " " << ee_move.y()
                      << " " << ee_move.z() << " m (|d| = " << ee_move.norm()
                      << " m)\n"
                      << std::defaultfloat;
        }

        // All logging memory is allocated here, before the loop starts. This
        // is the handoff queue to the writer thread, not the run's record —
        // the record is the CSV, written as the run happens.
        // cycles per second = 1e6 / period_us (500 at the 500 Hz default)
        LoopLog log(config::kLogBufferSeconds *
                    (1'000'000 / static_cast<std::size_t>(config::kCyclePeriod.count())));
        PoseTargetStore pose_targets; // the operator source's shared store

        if (playback)
            std::cout << "PLAYBACK RUN: the arm will follow the validated "
                         "trajectory as soon as the takeover completes — "
                      << trajectory_summary.duration_s
                      << " s of motion, then a hold at the final point. "
                         "Ctrl+C stops at any time (the position servo holds "
                         "where the command stopped).\n";
        else {
            if (!config::kUseFixedTarget)
                std::cout << "type a desired end-effector target and press Enter; Ctrl+C to "
                             "stop:\n"
                             "  x y z                  (meters, right-arm base frame; "
                             "orientation target "
                             "unchanged)\n"
                             "  x y z roll pitch yaw   (meters + radians, R = Rz·Ry·Rx, "
                             "right-arm base frame)\n";
            std::cout << "reactive-pose position integration at " << config::kControlFrequencyHz
                      << " Hz (full settings echoed above and in the CSV preamble)\n";
        }
        // Open the run's CSV BEFORE the takeover: a hardware run must never
        // end with zero evidence because the file could not be created. The
        // '#' config preamble makes every data file self-describing (F3).
        // The rows follow during the run, from the writer thread below.
        // Default: <repo>/runs/YYYY-MM-DD/ (RUNS_ROOT_DIR, baked in by
        // CMake) — the layout the plot scripts search; an explicit
        // An explicit --log filename is used verbatim.
        std::string log_file = log_file_arg;
        if (log_file.empty()) {
            const std::string run_dir = dated_run_dir(RUNS_ROOT_DIR);
            std::error_code dir_error;
            std::filesystem::create_directories(run_dir, dir_error);
            if (dir_error) {
                std::cerr << "Error: cannot create " << run_dir << " ("
                          << dir_error.message() << ") — not starting\n";
                return 1;
            }
            log_file = run_dir + "/" + timestamped_csv_name(config::kLoopLogPrefix);
        }
        std::ofstream csv(log_file);
        if (!csv) {
            std::cerr << "Error: cannot open " << log_file << " — not starting\n";
            return 1;
        }
        WriteCsvPreamble(log_file_arg, csv);

        // From here the CSV writes itself: the writer thread drains the log
        // to disk every kLogDrainInterval for as long as it is alive. It is
        // declared after `csv` and before the loop, so it is torn down
        // first on every exit path — including an exception — and its
        // destructor performs the final drain.
        LoopLogWriter log_writer(log, csv, config::kLogDrainInterval);

        // Controller + reference source + actuation, constructed before the
        // input thread: a bad end-effector frame name must fail here,
        // before any takeover. THE controller is always the same; only the
        // reference source differs per run (Controller.h).
        TrackingController controller(controlled_model);

        std::unique_ptr<ReferenceSource> reference;
        TrajectorySource* trajectory_source = nullptr;
        if (playback) {
            PlaybackSettings settings;
            settings.kp_s_inv = config::kPlaybackKp;
            settings.start_mismatch_limit_deg = config::kStartMismatchLimitDeg;
            auto owned = std::make_unique<TrajectorySource>(
                std::move(*trajectory), settings);
            trajectory_source = owned.get();
            reference = std::move(owned);
        } else {
            // With kUseFixedTarget the compiled target is handed to the
            // source NOW and applies from the first cycle after takeover
            // (with the takeover orientation); it does NOT go through the
            // store, whose pre-takeover content is deliberately discarded.
            std::optional<PoseTarget> fixed_target;
            if (config::kUseFixedTarget) {
                // Freshness gate on the COMPILED target: the arm can have
                // been moved (dashboard jog, physical push) any time after
                // this binary was built, and the controller would otherwise
                // drive the full gap at clip speed from the first cycle.
                // 2026-08-04: the arm was jogged 37 cm between compile and
                // run; only an unrelated failure stopped the takeover.
                {
                    Eigen::Matrix<double, 7, 1> q_now_rad;
                    for (int j = 0; j < 7; ++j)
                        q_now_rad[j] =
                            initial.actuators(j).position() * M_PI / 180.0;
                    KinematicsWorkspace gate_workspace(
                        controlled_model.dynamics());
                    const Eigen::Vector3d ee_now =
                        controlled_model
                            .RightPoseAndJacobian(q_now_rad, gate_workspace)
                            .position;
                    const Eigen::Vector3d target_m(config::kFixedTargetM[0],
                                                   config::kFixedTargetM[1],
                                                   config::kFixedTargetM[2]);
                    const double gap_m = (target_m - ee_now).norm();
                    if (gap_m > config::kMaxFixedTargetDistanceM) {
                        std::cout
                            << "robot NOT ready: the compiled fixed target is "
                            << gap_m << " m from the CURRENT end-effector "
                            << "position (limit "
                            << config::kMaxFixedTargetDistanceM
                            << " m, kMaxFixedTargetDistanceM).\n"
                            << "  The arm has likely been moved since this "
                            << "target was chosen. Recompile with a target "
                            << "near the pose printed above.\n";
                        return 1;
                    }
                    std::cout << "fixed-target distance gate: PASS ("
                              << gap_m << " m to travel)\n";
                }
                PoseTarget target;
                target.p_desired = Eigen::Vector3d(config::kFixedTargetM[0],
                                                   config::kFixedTargetM[1],
                                                   config::kFixedTargetM[2]);
                // Empty rotation = keep the takeover orientation.
                if (config::kFixedTargetUseRpy)
                    target.rotation =
                        RotationFromRpy(config::kFixedTargetRpyRad[0],
                                        config::kFixedTargetRpyRad[1],
                                        config::kFixedTargetRpyRad[2]);
                fixed_target = target;
            }
            reference =
                std::make_unique<PoseTargetSource>(pose_targets, fixed_target);
        }
        PositionIntegration actuation(config::kCommandLeadLimitDeg);

        // Operator input threads. With kUseFixedTarget there is no stdin
        // thread — the arm drives to the fixed target as soon as the
        // takeover completes. The trajectory source has no operator targets
        // (Ctrl+C is the signal handler, not stdin).
        std::thread input_thread;
        if (!playback && config::kUseFixedTarget) {
            std::cout << "FIXED TARGET (Config.h kFixedTargetM): "
                      << config::kFixedTargetM[0] << " "
                      << config::kFixedTargetM[1] << " "
                      << config::kFixedTargetM[2]
                      << " m — THE ARM MOVES THERE IMMEDIATELY after the "
                         "takeover; Ctrl+C to stop\n";
            if (config::kFixedTargetUseRpy)
                std::cout << "  AND ROTATES to rpy "
                          << config::kFixedTargetRpyRad[0] << " "
                          << config::kFixedTargetRpyRad[1] << " "
                          << config::kFixedTargetRpyRad[2]
                          << " rad (kFixedTargetRpyRad) — compare against the "
                             "measured rpy printed above\n";
            else
                std::cout << "  orientation target unchanged "
                             "(kFixedTargetUseRpy = false)\n";
        } else if (!playback) {
            input_thread =
                std::thread(RunPoseTargetInput, std::ref(pose_targets), std::cref(g_stop));
        }
        InputThreadJoiner input_thread_joiner{input_thread};

        // Optional second target input (operator source only, target_file
        // in the config): edit+save the watched file to retarget. Its
        // content at startup is ignored — only in-session edits become
        // targets.
        std::thread target_file_thread;
        if (!playback && config::kTargetFile[0] != '\0') {
            std::cout << "watching target file: " << config::kTargetFile
                      << " (current content ignored; edit and save to retarget)\n";
            target_file_thread = std::thread(RunPoseTargetFileInput, std::ref(pose_targets),
                                             config::kTargetFile, std::cref(g_stop));
        }
        InputThreadJoiner target_file_thread_joiner{target_file_thread};

        // MOVES THE ARM (toward typed positions): servoing mode is entered
        // and restored inside the Runner, on every exit path (T2/D3).
        const LoopResult result = RunControlLoop(
            connection.base(), connection.base_cyclic(), *reference,
            controller, actuation,
            log, g_stop, config::kCyclePeriod, config::kQdotLimitDegS,
            config::kFollowingErrorLimitDeg, robot_ready);

        g_stop = true; // loop may have exited on a fault, not Ctrl+C
        if (input_thread.joinable())
            input_thread.join();
        if (target_file_thread.joinable())
            target_file_thread.join();

        // Final drain — everything before this was already on disk.
        log_writer.Stop();
        // Exit trailer: the last line of the file names how the run ended and
        // when, so a CSV is self-describing without the console output. A '#'
        // line, like the preamble, so every existing parser skips it.
        csv << "# exit_reason = " << StopReasonName(result.reason)
            << "\n# exit_time_s = " << result.stop_t_s
            << "\n# exit_cycle = " << result.cycles
            << "\n# faults_observed = " << (result.faults_observed ? 1 : 0)
            << "\n";
        csv.flush();
        std::cout << log_writer.rows_written() << " samples written";
        if (log.dropped() > 0)
            std::cout << " — WARNING: " << log.dropped()
                      << " samples never reached the file (the CSV writer "
                         "could not keep up with the loop; the gap is "
                         "visible in time_s)";
        // Full path on its own line, ready to paste into an analysis request.
        std::cout << "\nlog: " << log_file << "\n";

        // Playback outcome, stated plainly for the run record.
        if (trajectory_source) {
            if (trajectory_source->refused())
                std::cout << "playback outcome: REFUSED at takeover (no "
                             "motion commanded) — exit status is nonzero\n";
            else if (trajectory_source->completed())
                std::cout << "playback outcome: trajectory completed\n";
            else
                std::cout << "playback outcome: stopped before the last "
                             "sample\n";
        }

        // Only a clean operator stop with no observed faults is success —
        // faults the loop was told to ignore still taint the exit code.
        if (result.faults_observed)
            std::cout << "note: faults occurred during run — exit status is nonzero\n";
        const bool playback_refused =
            trajectory_source && trajectory_source->refused();
        return result.reason == LoopStop::kUserStop && !result.faults_observed &&
                       !playback_refused
                   ? 0
                   : 1;
    } catch (k_api::KDetailedException& e) {
        std::cerr << "Kortex error: " << e.what() << " (sub-code "
                  << k_api::SubErrorCodes_Name(static_cast<k_api::SubErrorCodes>(
                         e.getErrorInfo().getError().error_sub_code()))
                  << ")\n";
        return 1;
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
