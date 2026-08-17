//
// Main — the program: parse --arm (required) and --log, load one Pinocchio
// model per selected arm, connect, check readiness, wire the reference
// source to the controller, run the loop, report. RunOneArm below is the
// whole per-arm story; main() only selects which arm(s) --arm names,
// acquires every lock and loads every model before any hardware contact,
// and — for --arm both — runs one RunOneArm per arm on its own thread. The
// loop itself lives in Runner.h/.cpp.
//

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <tuple>
#include <vector>

#include <KDetailedException.h>

#include "BasePose.h"
#include "CartesianTrajectoryMailbox.h"
#include "Config.h"
#include "ExecutionConfig.h"
#include "ExecutionCore.h"
#include "FramePrint.h"
#include "Hardware.h"
#include "InProcessPlanner.h"
#include "Kinematics.h"
#include "MainArgs.h"
#include "ProcessLock.h"
#include "PlannerRuntime.h"
#include "Runner.h"
#include "Safety.h"
#include "State.h"
#include "ViconSource.h"

namespace k_api = Kinova::Api;

// ---------------------------------------------------------------
// run configuration echo (startup print + CSV preamble)
// ---------------------------------------------------------------

// --arm and --log are the only runtime-selectable values; every controller
// setting is compiled in Config.h. The key = value lines below become the
// '#'-prefixed CSV preamble that makes every data file self-describing
// (parsers skip '#' lines).

namespace {
[[noreturn]] void UsageAndExit(const std::string& error) {
    if (!error.empty()) std::cerr << "error: " << error << "\n";
    std::cerr << "usage: controller --arm <right|left|both> [--log <file>]\n"
              << "  --arm <right|left|both>  required — which physical arm(s)\n"
              << "                           this run controls (config::kRightArmConfig\n"
              << "                           / kLeftArmConfig, Config.h). No default: every\n"
              << "                           run states its target arm(s) explicitly.\n"
              << "  --log <file>             CSV filename (default: timestamped run file,\n"
              << "                           per-arm prefixed). Not valid with --arm both —\n"
              << "                           one filename can't name two arms' logs.\n"
              << "All other controller settings are compiled in src/Config.h.\n";
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
    line("limit_avoid_zone_deg", FormatDouble(config::kLimitAvoidZoneDeg));
    line("limit_avoid_gain", FormatDouble(config::kLimitAvoidGain));
    line("orientation_enabled", config::kOrientationEnabled ? "true" : "false");
    line("velocity_term_enabled", config::kVelocityTermEnabled ? "true" : "false");
    line("null_space_enabled", config::kNullSpaceEnabled ? "true" : "false");
    line("reference_source", "world_cartesian_trajectory");
    line("startup_hold", "first_fresh_world_pose");
    line("target_hold_s", FormatDouble(config::kTargetHoldS));
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
    line("arrival_orientation_tolerance_rad",
         FormatDouble(config::kArrivalOrientationToleranceRad));
    line("nonfinite_stop_cycles", std::to_string(config::kNonFiniteStopCycles));
    line("overrun_stop_cycles", std::to_string(config::kOverrunStopCycles));
    line("stale_feedback_stop_cycles",
         std::to_string(config::kStaleFeedbackStopCycles));
    line("overrun_factor", FormatDouble(config::kOverrunFactor));
    line("takeover_hold_s", FormatDouble(config::kTakeoverHoldS));
    line("status_print_period_s", FormatDouble(config::kStatusPrintPeriodS));
    out << prefix << "log_file = " << (log_file.empty() ? "<timestamped>" : log_file)
        << (log_file.empty() ? " (default)\n" : " (--log)\n");
    line("control_dt_s", FormatDouble(config::kControlDtS));
    line("world_fresh_max_age_s", FormatDouble(config::kWorldFreshMaxAgeS));
    line("vicon_mount_twist_filter_tau_s",
         FormatDouble(config::kViconMountTwistFilterTauS));
    line("vicon_mount_twist_reset_gap_s",
         FormatDouble(config::kViconMountTwistResetGapS));
    line("world_prolonged_stale_s",
         FormatDouble(config::kWorldProlongedStaleS));
}

void WriteCsvPreamble(const std::string& log_file, const config::ArmConfig& arm_config,
                      std::ostream& csv) {
    csv << "# controller run config — parsers skip '#' lines\n";
    csv << "# log_format = 13 (compiled)\n";
    // Why a log's vicon_* columns are all-NaN: source compiled out
    // ("absent"), or compiled in but never connected (age stays NaN).
    csv << "# vicon_source = " << kViconSourceBuildMode << " ("
        << kViconSourceHost << ")\n";
    csv << "# arm = " << arm_config.name << " (" << arm_config.ip << ")\n";
    csv << "# planner_handoff = in_process_typed_world_cartesian\n";
    WriteConfigLines(log_file, csv, "# ");
}
} // namespace

/*
 * The story of the program, told at a high level:
 *
 *   parse --arm (required: right, left, or both) and --log (the only other
 *   runtime argument; everything else is Config.h)
 *     -> acquire every selected arm's ProcessLock and load its Pinocchio
 *        dynamics model, BEFORE any hardware contact (main(), below)
 *     -> for each selected arm, run RunOneArm — on its own thread when
 *        --arm both selects two: bind measured joints + the other arm's
 *        nominal joints through the explicit DualArmKinematics 14-DoF-
 *        model/7-controller adapter; connect to that arm only (TCP + UDP,
 *        Connect); readiness check on one feedback frame (before any
 *        takeover); print the current joint state and end-effector
 *        position; run the control loop (RunControlLoop — MOVES THE ARM:
 *        takeover sequence T1-T6, one ArmExecutionCore composing the
 *        world-Cartesian reference source, tracking controller and
 *        position integration from one immutable production configuration
 *        snapshot, single-level servoing restored on every exit path);
 *        drain the
 *        last of the loop log (the rest of the CSV was written by the
 *        writer thread as the run happened); a non-clean stop on either
 *        arm signals the shared stop flag, so a fault on one arm brings
 *        both down through the same graceful path
 *     -> RAII teardown, exit 0 only if every run arm stopped cleanly with
 *        no faults observed.
 *
 * Usage: ./controller --arm <right|left|both> [--log <file>]
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

    // The measured controlled-arm configuration in the mount frame,
    // alongside the OTHER arm at its compiled nominal. That other row is
    // OPEN-LOOP: it has no connection and no feedback in this process, so
    // it reports where the model says an arm at its nominal would be, not
    // where anything measured is.
    void PrintMeasuredMountFrame(DualArmKinematics& model,
                                 const Eigen::Matrix<double, 7, 1>& controlled_q_rad,
                                 const config::ArmConfig& arm_config)
    {
        Eigen::Matrix<double, 7, 1> other_q_rad;
        for (int i = 0; i < 7; ++i)
            other_q_rad[i] = model.other_arm_nominal_rad()[static_cast<std::size_t>(i)];
        const bool right_controlled = model.controlled_arm() == Arm::kRight;
        const Eigen::Matrix<double, 7, 1>& right_q_rad =
            right_controlled ? controlled_q_rad : other_q_rad;
        const Eigen::Matrix<double, 7, 1>& left_q_rad =
            right_controlled ? other_q_rad : controlled_q_rad;
        PrintDualArmFkTable(
            model, right_q_rad, left_q_rad,
            std::string("mount-frame FK at the measured ") + arm_config.name +
                " configuration (other arm at nominal, model-only — not measured):");
    }

    // Current joint state plus the end-effector position (our FK) — the
    // printed p is what a "hold here" desired position looks like, so the
    // operator can start from it and edit one coordinate.
    void PrintRobotState(const k_api::BaseCyclic::Feedback& feedback,
                         DualArmKinematics& model, const config::ArmConfig& arm_config)
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
        const PoseJacobian ee = model.ControlledPoseAndJacobian(q_rad, workspace);
        // Print the measured orientation for diagnosis. Position-only targets
        // preserve this takeover orientation rather than accepting RPY input.
        const Eigen::Vector3d zyx = ee.rotation.eulerAngles(2, 1, 0);
        std::cout << arm_config.name << " end-effector (" << arm_config.end_effector_frame
                  << " in " << arm_config.base_frame << "): "
                  << std::fixed << std::setprecision(4)
                  << ee.position.x() << " " << ee.position.y() << " "
                  << ee.position.z()
                  << " (m, " << arm_config.name << "-arm base frame)\n"
                  << "  orientation rpy: " << zyx.z() << " " << zyx.y() << " "
                  << zyx.x() << " (rad, R = Rz*Ry*Rx)"
                  << std::defaultfloat << "\n";

        // Same measured configuration, in mount. This is the line to check
        // against the physical rig: the base-frame position above is the one
        // the Kinova dashboard's tool_pose agrees with, so if that matches and
        // this does not, the mount<-base mounting transform is what is wrong.
        PrintMeasuredMountFrame(model, q_rad, arm_config);
    }

} // namespace

// Ctrl+C (SIGINT) and SIGTERM (CLion Stop, kill) set this flag; every
// running RunOneArm observes it. Both signals must leave through the same
// graceful path — stop report, CSV drain, servoing restore — or an IDE stop
// kills the process mid-takeover with none of them. Shared across every arm
// this process runs: one Ctrl+C stops all of them, and RunOneArm also sets
// it on its own non-clean exit, so a fault on one arm stops the other too.
std::atomic<bool> g_stop{false};
void on_stop_signal(int)
{
    g_stop = true;
}

namespace
{
    // The planner worker never touches the robot connection, but it must never
    // outlive the typed request/trajectory slots it uses. This guard covers
    // normal loop exit and exceptions from RunControlLoop alike.
    class WorkerThreadStopJoiner
    {
    public:
        WorkerThreadStopJoiner(std::thread& thread, std::atomic<bool>& stop)
            : thread_(thread), stop_(stop)
        {}

        ~WorkerThreadStopJoiner()
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

    // Everything one physical arm's run needs, from a fresh Connect through
    // teardown. Called directly for --arm right/left; called once per arm,
    // each on its own thread, for --arm both (main() has already acquired
    // every arm's ProcessLock and loaded every arm's Dynamics before any of
    // these run, so no run here ever begins a takeover while another arm's
    // startup is still contesting a lock or a model load).
    //
    // Catches every exception itself — nothing escapes this function, so an
    // exception from one arm's thread can never reach std::terminate at the
    // thread boundary and skip the other arm's teardown.
    //
    // Returns 0 only on a clean operator stop with no faults observed.
    int RunOneArm(const config::ArmConfig& arm_config, Arm controlled_arm,
                  Dynamics& dynamics, const std::string& log_file_arg)
    {
        const std::string tag = std::string("[") + arm_config.name + "] ";
        try {
            // Explicit right-arm-or-left-arm adapter over the shared
            // mounted model. The adapter validates nq=nv=14 and the exact
            // named mapping to the seven-wide controller interface.
            DualArmKinematics controlled_model(
                dynamics, controlled_arm, arm_config.other_arm_nominal_rad,
                config::kRightBaseFrame, config::kRightEndEffectorFrame);

            Connect connection(arm_config.ip);

            // Clearing faults (TrajectoryExecution's pattern): unconditional,
            // immediately after connecting, before anything else talks to the
            // arm. Commands no motion; a latched leftover from a previous run
            // clears, and anything LIVE simply re-latches and is caught by the
            // readiness gate below, which runs on a fresh post-clear read.
            try {
                connection.base()->ClearFaults();
            } catch (...) {
                std::cout << tag << "Unable to clear robot faults" << std::endl;
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
            // fast RPCs.
            if (config::kSkipStartupGates)
                std::cout << tag
                    << "WARNING: STARTUP GATE SKIPPED (config::kSkipStartupGates)"
                       " — the JOINT_LIMIT bands are NOT restored. A band left at"
                       " 0/0 faults outward motion at the firmware level.\n";
            else {
                if (!connection.EnsureJointLimits(std::cout))
                    return 1;
            }
            std::cout << tag << "== " << arm_config.name << " arm (" << arm_config.ip
                      << ") ==\n";
            PrintRobotState(initial, controlled_model, arm_config);

            // All logging memory is allocated here, before the loop starts. This
            // is the handoff queue to the writer thread, not the run's record —
            // the record is the CSV, written as the run happens.
            // cycles per second = 1e6 / period_us (500 at the 500 Hz default)
            LoopLog log(config::kLogBufferSeconds *
                        (1'000'000 / static_cast<std::size_t>(config::kCyclePeriod.count())));

            std::cout << tag << "reactive-pose position integration at "
                      << config::kControlFrequencyHz
                      << " Hz (full settings in the CSV preamble)\n";
            // Open the run's CSV BEFORE the takeover: a hardware run must never
            // end with zero evidence because the file could not be created. The
            // '#' config preamble makes every data file self-describing (F3).
            // The rows follow during the run, from the writer thread below.
            // Default: <repo>/runs/YYYY-MM-DD/ (RUNS_ROOT_DIR, baked in by
            // CMake) — the layout the plot scripts search, filename prefixed
            // per arm (arm_config.log_prefix) so --arm both never collides.
            // An explicit --log filename is used verbatim (refused with
            // --arm both, since one filename can't name two arms' logs).
            std::string log_file = log_file_arg;
            if (log_file.empty()) {
                const std::string run_dir = dated_run_dir(RUNS_ROOT_DIR);
                std::error_code dir_error;
                std::filesystem::create_directories(run_dir, dir_error);
                if (dir_error) {
                    std::cerr << tag << "Error: cannot create " << run_dir << " ("
                              << dir_error.message() << ") — not starting\n";
                    return 1;
                }
                log_file = run_dir + "/" + timestamped_csv_name(arm_config.log_prefix);
            }
            std::ofstream csv(log_file);
            if (!csv) {
                std::cerr << tag << "Error: cannot open " << log_file << " — not starting\n";
                return 1;
            }
            WriteCsvPreamble(log_file_arg, arm_config, csv);

            // World-pose input. Slot + acquisition thread
            // are per-arm because the slot is strictly single-reader and
            // --arm both runs one RunOneArm per arm: two arms mean two SDK
            // connections, which DataStream serves fine. The source starts
            // (and keeps retrying) in its own thread — an unreachable
            // Vicon never delays or stops the takeover; the stub build
            // returns nullptr and the columns record honest absence.
            BasePoseSlot base_pose_slot;
            std::unique_ptr<ViconSource> vicon_source =
                StartViconSource(base_pose_slot);
            std::cout << tag << "vicon world-pose source: "
                      << kViconSourceBuildMode << " (" << kViconSourceHost
                      << ", controller measurement + telemetry)\n";

            // From here the CSV writes itself: the writer thread drains the log
            // to disk every kLogDrainInterval for as long as it is alive. It is
            // declared after `csv` and before the loop, so it is torn down
            // first on every exit path — including an exception — and its
            // destructor performs the final drain.
            LoopLogWriter log_writer(log, csv, config::kLogDrainInterval);

            // The execution core: ONE immutable production configuration
            // snapshot (the only place config:: constants enter the
            // per-cycle path) feeding the controller, reference source and
            // position integration it composes. A bad end-effector frame
            // name or a physically meaningless configuration value must
            // fail here, before any takeover. The nominal period below
            // must be — and is — the same config::kCyclePeriod the loop
            // paces with.
            CartesianTrajectoryMailbox trajectory_targets;
            ArmExecutionCore execution_core(
                controlled_model, ProductionExecutionConfig(),
                trajectory_targets,
                std::chrono::duration<double>(config::kCyclePeriod).count());

            Eigen::Matrix<double, 7, 1> q_now_rad;
            for (int j = 0; j < 7; ++j)
                q_now_rad[j] = initial.actuators(j).position() * M_PI / 180.0;
            KinematicsWorkspace startup_workspace(controlled_model.dynamics());
            const PoseJacobian ee_now = controlled_model.ControlledPoseAndJacobian(
                q_now_rad, startup_workspace);
            if (!ee_now.position.allFinite() || !ee_now.rotation.allFinite()) {
                std::cerr << tag << "Error: current FK pose is non-finite — not starting\n";
                return 1;
            }
            csv << "# startup_position_m = " << ee_now.position.x() << " "
                << ee_now.position.y() << " " << ee_now.position.z()
                << " (measured FK, " << arm_config.base_frame << ")\n";
            // The same measurement in mount, so a log can be read in either
            // frame without needing the mounting transform to hand. Derived from
            // the model, not a stored constant: change the URDF and this follows.
            const Eigen::Vector3d startup_mount =
                controlled_model.PointBaseToMount(controlled_arm, ee_now.position);
            csv << "# startup_position_mount_m = " << startup_mount.x() << " "
                << startup_mount.y() << " " << startup_mount.z()
                << " (measured FK, mount)\n";
            csv << "# startup_joint_deg =";
            for (int j = 0; j < 7; ++j)
                csv << " " << initial.actuators(j).position();
            csv << " (measured Kortex actuator order)\n";
            // Both frames, because this is the line read immediately before a
            // run: the base-frame value is what the Kinova dashboard shows,
            // and the mount-frame value is the one a goal file is written in.
            // Printing only one means mentally converting at the worst moment.
            // Reuses the preamble's startup_mount above — one conversion, so
            // the log and the terminal cannot disagree.
            std::cout << tag << "current startup pose: " << ee_now.position.x() << " "
                      << ee_now.position.y() << " " << ee_now.position.z()
                      << " m in " << arm_config.base_frame << " = "
                      << startup_mount.x() << " " << startup_mount.y() << " "
                      << startup_mount.z() << " m in mount (goal-file frame)"
                      << "; the arm will hold here\n";
            // Before the first fresh world sample the core's reference
            // source returns the measured world pose each cycle (zero
            // Cartesian error). The first fresh sample captures a fixed
            // world hold; validated trajectories splice into that same
            // reference path through the mailbox constructed above.
            PlanningRequestSlot planning_requests;

            std::cout << tag << "HOLD AT START: zero-error Cartesian hold until "
                         "the first fresh world sample, then fixed WORLD pose; "
                         "Ctrl+C to stop\n";
            const PlannerRuntimeConfig planner_config{
                PLANNER_GOAL_FILE,
                PLANNER_CONFIG_FILE,
                PLANNER_JOINT_LIMITS_FILE,
                PLANNER_RIGHT_DH_FILE,
                PLANNER_LEFT_DH_FILE,
                RUNS_ROOT_DIR};
            std::thread planner_thread(
                RunInProcessPlanner, std::ref(planning_requests),
                std::ref(trajectory_targets), std::cref(planner_config),
                std::cref(g_stop), &SolveWorldTrajectoryForRequest);
            WorkerThreadStopJoiner planner_thread_joiner(planner_thread, g_stop);

            // MOVES THE ARM: servoing mode is entered and restored inside the
            // Runner on every exit path (T2/D1).
            const LoopResult result = RunControlLoop(
                connection.base(), connection.base_cyclic(), execution_core,
                log, g_stop, config::kCyclePeriod,
                config::kFollowingErrorLimitDeg, robot_ready,
                controlled_arm == Arm::kLeft ? PlanningArm::kLeft
                                              : PlanningArm::kRight,
                &planning_requests, &base_pose_slot);

            planner_thread_joiner.Join();

            // A non-clean stop on this arm signals the shared stop flag, so
            // it also stops any other arm this process is running. A
            // --arm both run shares one physical workspace and one
            // operator's attention: a fault, a following-error stop, or
            // lost servoing on either arm is a reason to bring both down
            // through the normal graceful path, not to leave the other one
            // running unsupervised. Solo runs: nothing else is listening,
            // so this is a no-op.
            if (result.reason != LoopStop::kUserStop || result.faults_observed)
                g_stop.store(true, std::memory_order_relaxed);

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
            std::cout << tag << log_writer.rows_written() << " samples written";
            if (log.dropped() > 0)
                std::cout << " — WARNING: " << log.dropped()
                          << " samples never reached the file (the CSV writer "
                             "could not keep up with the loop; the gap is "
                             "visible in time_s)";
            // Full path on its own line, ready to paste into an analysis request.
            std::cout << "\n" << tag << "log: " << log_file << "\n";

            // Only a clean operator stop with no observed faults is success —
            // faults the loop was told to ignore still taint the exit code.
            if (result.faults_observed)
                std::cout << tag << "note: faults occurred during run — exit status is nonzero\n";
            return result.reason == LoopStop::kUserStop && !result.faults_observed
                       ? 0
                       : 1;
        } catch (k_api::KDetailedException& e) {
            std::cerr << tag << "Kortex error: " << e.what() << " (sub-code "
                      << k_api::SubErrorCodes_Name(static_cast<k_api::SubErrorCodes>(
                             e.getErrorInfo().getError().error_sub_code()))
                      << ")\n";
            g_stop.store(true, std::memory_order_relaxed);
            return 1;
        } catch (std::exception& e) {
            std::cerr << tag << "Error: " << e.what() << "\n";
            g_stop.store(true, std::memory_order_relaxed);
            return 1;
        }
    }
} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_stop_signal);
    std::signal(SIGTERM, on_stop_signal);

    ParsedMainArgs args;
    try {
        args = ParseMainArgs(std::vector<std::string>(argv + 1, argv + argc));
    } catch (const std::exception& error) {
        UsageAndExit(error.what());
    }

    std::vector<const config::ArmConfig*> arm_configs;
    std::vector<Arm> controlled_arms;
    if (args.arm == "right" || args.arm == "both") {
        arm_configs.push_back(&config::kRightArmConfig);
        controlled_arms.push_back(Arm::kRight);
    }
    if (args.arm == "left" || args.arm == "both") {
        arm_configs.push_back(&config::kLeftArmConfig);
        controlled_arms.push_back(Arm::kLeft);
    }
    const std::size_t arm_count = arm_configs.size();

    try {
        // Acquire EVERY selected arm's lock before any hardware contact or
        // model load: a --arm both run must never take over one arm while
        // the other's lock is still contested by another process. Held for
        // main()'s whole scope (destroyed only after every thread below has
        // joined), not passed into RunOneArm — it needs to exist, not be used.
        std::vector<std::unique_ptr<ProcessLock>> locks;
        locks.reserve(arm_count);
        for (std::size_t i = 0; i < arm_count; ++i) {
            const config::ArmConfig& arm_config = *arm_configs[i];
            locks.push_back(std::make_unique<ProcessLock>(
                arm_config.lock_path,
                std::string("another basic_control process is already controlling ") +
                    arm_config.name + " (" + arm_config.ip + ")"));
        }

        // One Dynamics (Pinocchio model) per arm, constructed sequentially,
        // here, before any thread starts: pinocchio::urdf::buildModel's
        // thread-safety for two concurrent constructions is unverified, so
        // this never races two constructions against each other. A bad URDF
        // path also fails once, loudly, before either arm's takeover.
        std::vector<std::unique_ptr<Dynamics>> dynamics_list;
        dynamics_list.reserve(arm_count);
        for (std::size_t i = 0; i < arm_count; ++i)
            dynamics_list.push_back(std::make_unique<Dynamics>(GEN3_DUAL_URDF_PATH));

        // Full configuration is embedded in each arm's own CSV preamble
        // (WriteConfigLines) — the log stays self-describing without
        // printing thirty lines at every start.
        std::vector<int> exit_codes(arm_count, 1);
        std::vector<std::thread> threads;
        threads.reserve(arm_count);
        for (std::size_t i = 0; i < arm_count; ++i) {
            threads.emplace_back([&, i]() {
                exit_codes[i] = RunOneArm(*arm_configs[i], controlled_arms[i],
                                          *dynamics_list[i], args.log_file);
            });
        }
        for (std::thread& t : threads)
            t.join();

        // 0 only if every run arm stopped cleanly with no faults observed.
        int exit_code = 0;
        for (int code : exit_codes)
            exit_code = std::max(exit_code, code);
        return exit_code;
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
