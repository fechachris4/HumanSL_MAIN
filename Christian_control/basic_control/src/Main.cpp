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

namespace k_api = Kinova::Api;

// ---------------------------------------------------------------
// run configuration echo (startup print + CSV preamble)
// ---------------------------------------------------------------

// --log is the only runtime-selectable value; every controller setting is
// compiled in Config.h. The key = value lines below become the
// '#'-prefixed CSV preamble that makes every data file self-describing
// (parsers skip '#' lines).

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
    line("kp", FormatDouble(config::kKpCartesian));
    line("dls_lambda", FormatDouble(config::kDlsLambda));
    line("kp_rot", FormatDouble(config::kKpRotation));
    line("kd_pos", FormatDouble(config::kKdPosition));
    line("kd_rot", FormatDouble(config::kKdRotation));
    line("null_gain", FormatDouble(config::kNullGain));
    line("orientation_enabled", config::kOrientationEnabled ? "true" : "false");
    line("velocity_term_enabled", config::kVelocityTermEnabled ? "true" : "false");
    line("null_space_enabled", config::kNullSpaceEnabled ? "true" : "false");
    line("fixed_target_m", FormatDouble(config::kFixedTargetM[0]) + " " +
                               FormatDouble(config::kFixedTargetM[1]) + " " +
                               FormatDouble(config::kFixedTargetM[2]));
    line("fixed_target_use_rpy", config::kFixedTargetUseRpy ? "true" : "false");
    if (config::kFixedTargetUseRpy)
        line("fixed_target_rpy_rad",
             FormatDouble(config::kFixedTargetRpyRad[0]) + " " +
                 FormatDouble(config::kFixedTargetRpyRad[1]) + " " +
                 FormatDouble(config::kFixedTargetRpyRad[2]));
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
    csv << "# log_format = 6 (compiled)\n";
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
 *     -> run the control loop (RunControlLoop above — MOVES THE ARM:
 *        takeover sequence T1-T5, the fixed-target reference source feeding
 *        the TrackingController + PositionIntegration actuation,
 *        single-level servoing restored on every exit path)
 *     -> drain the last of the loop log (the rest
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
        // Print the measured orientation for diagnosis. Position-only targets
        // preserve this takeover orientation rather than accepting RPY input.
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
// observes it. Both signals must leave through
// the same graceful path — stop report, CSV drain, servoing restore —
// or an IDE stop kills the process mid-takeover with none of them.
std::atomic<bool> g_stop{false};
void on_stop_signal(int)
{
    g_stop = true;
}

namespace
{
    // The input thread only polls stdin, but it must never outlive the
    // mailbox it writes. This guard covers normal loop exit and exceptions
    // from RunControlLoop alike.
    class InputThreadStopJoiner
    {
    public:
        InputThreadStopJoiner(std::thread& thread, std::atomic<bool>& stop)
            : thread_(thread), stop_(stop)
        {}

        ~InputThreadStopJoiner()
        {
            Join();
        }

        void Join()
        {
            stop_.store(true, std::memory_order_relaxed);
            if (thread_.joinable())
                thread_.join();
        }

    private:
        std::thread& thread_;
        std::atomic<bool>& stop_;
    };
} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_stop_signal);
    std::signal(SIGTERM, on_stop_signal);

    const std::string log_file_arg = ParseLogFileArg(argc, argv);

    try {
        // The full configuration is embedded in the CSV preamble
        // (WriteConfigLines below) — the log stays self-describing
        // without printing thirty lines at every start.
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
        // READ-ONLY hard-speed gate before any servoing takeover. This is
        // intentionally independent of kSkipStartupGates: skipping
        // configuration writes does not authorize a qdot clip above the
        // robot's reported hard speed limit.
        if (!connection.VerifyKinematicHardLimits(std::cout))
            return 1;
        // Re-assert the configured JOINT_LIMIT thresholds: they do not
        // survive a robot power cycle, and a degenerate 0/0 band makes the
        // firmware fault any motion away from zero (Config.h). Only joints
        // with non-zero config entries are touched, so this is a handful of
        // fast RPCs. (The separate control-mode verification gate was
        // removed 2026-08-04: actuators boot in POSITION, nothing in this
        // program ever sets another mode, and probing the mode cost a long
        // RPC timeout per unreachable actuator on every start.)
        if (config::kSkipStartupGates)
            std::cout
                << "WARNING: STARTUP GATE SKIPPED (config::kSkipStartupGates)"
                   " — the JOINT_LIMIT bands are NOT restored. A band left at"
                   " 0/0 faults outward motion at the firmware level.\n";
        else {
            if (!connection.EnsureJointLimits(std::cout))
                return 1;
        }
        PrintRobotState(initial, controlled_model);

        // All logging memory is allocated here, before the loop starts. This
        // is the handoff queue to the writer thread, not the run's record —
        // the record is the CSV, written as the run happens.
        // cycles per second = 1e6 / period_us (500 at the 500 Hz default)
        LoopLog log(config::kLogBufferSeconds *
                    (1'000'000 / static_cast<std::size_t>(config::kCyclePeriod.count())));

        std::cout << "reactive-pose position integration at " << config::kControlFrequencyHz
                  << " Hz (full settings in the CSV preamble)\n";
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

        // Controller + reference source + actuation: a bad end-effector
        // frame name must fail here, before any takeover.
        TrackingController controller(controlled_model);

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
        // All targets, including the compiled first target, are position
        // only in base_link and preserve the orientation captured at
        // takeover.
        PoseTargetMailbox pose_targets;
        PoseTargetSource reference(target, pose_targets);
        PositionIntegration actuation(config::kCommandLeadLimitDeg);

        // The arm drives to the fixed target first. Stdin targets may queue
        // immediately, but each waits for an arrival edge before activation.
        std::cout << "FIXED TARGET (Config.h kFixedTargetM): "
                  << config::kFixedTargetM[0] << " "
                  << config::kFixedTargetM[1] << " "
                  << config::kFixedTargetM[2]
                  << " m — THE ARM MOVES THERE IMMEDIATELY after the "
                     "takeover; Ctrl+C to stop\n";
        std::cout << "  type x y z (metres, base_link) to queue up to eight "
                     "position-only targets; the takeover orientation holds\n";

        std::thread input_thread(RunPoseTargetInput, std::ref(pose_targets),
                                 std::cref(g_stop));
        InputThreadStopJoiner input_thread_joiner(input_thread, g_stop);

        // MOVES THE ARM: servoing mode is entered and restored inside the
        // Runner on every exit path (T2/D1).
        const LoopResult result = RunControlLoop(
            connection.base(), connection.base_cyclic(), reference,
            controller, actuation,
            log, g_stop, config::kCyclePeriod, config::kQdotLimitDegS,
            config::kFollowingErrorLimitDeg, robot_ready);

        input_thread_joiner.Join();

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

        // Only a clean operator stop with no observed faults is success —
        // faults the loop was told to ignore still taint the exit code.
        if (result.faults_observed)
            std::cout << "note: faults occurred during run — exit status is nonzero\n";
        return result.reason == LoopStop::kUserStop && !result.faults_observed
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
