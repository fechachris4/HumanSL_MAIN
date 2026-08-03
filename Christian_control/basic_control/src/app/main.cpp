/*
 * main.cpp — the story of the program, told at a high level:
 *
 *   parse options (CLI > TOML > compiled defaults, app/Options)
 *     -> load the mounted dual-arm URDF (Pinocchio; exactly 14 velocity vars)
 *     -> bind measured right joints + nominal left joints through the explicit
 *        DualArmKinematics 14-DoF-model/7-controller adapter
 *     -> connect only to the right arm (TCP + UDP, Connect)
 *     -> readiness check on one feedback frame (before any takeover)
 *     -> print the current joint state and end-effector position
 *     -> start the desired-target input thread (stdin: x y z meters, plus
 *        roll pitch yaw radians for the reactive-pose law)
 *     -> run the control loop (loop/Runner.cpp — MOVES THE ARM: takeover
 *        sequence T1-T6, the selected controller (ResolvedRate or
 *        ReactivePose) + PositionIntegration actuation, single-level
 *        servoing restored on every exit path)
 *     -> stop the input thread, drain the last of the loop log (the rest
 *        of the CSV was written by the writer thread as the run happened)
 *     -> RAII teardown, exit 0 only on a clean operator stop.
 *
 * Usage: ./controller
 */

#include <atomic>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <system_error>
#include <thread>
#include <tuple>

#include "app/Config.h"
#include "app/Options.h"
#include "actuation/PositionIntegration.h"
#include "control/ReactivePose.h"
#include "control/ResolvedRate.h"
#include "control/Target.h"
#include "control/TrajectoryFile.h"
#include "control/TrajectoryPlayback.h"
#include "hardware/Connect.h"
#include "hardware/Measure.h"
#include "hardware/Record.h"
#include "loop/Runner.h"
#include "math/DualArmKinematics.h"
#include "math/Kinematics.h"
#include "safety/FaultReport.h" // StopReasonName, for the CSV exit trailer
#include "safety/Supervisor.h"
#include "Dynamics.h"

#include <KDetailedException.h>

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
        std::cout << "right end-effector (" << config::kRightEndEffectorFrame
                  << " in " << config::kRightBaseFrame << "): "
                  << std::fixed << std::setprecision(4)
                  << ee.position.x() << " " << ee.position.y() << " "
                  << ee.position.z()
                  << " (m, right-arm base frame)"
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

    const RunOptions options = ParseRunOptions(argc, argv);

    try {
        EchoConfig(options, std::cout);
        const bool playback = std::string(config::kController) == "playback";
        const bool reactive = std::string(config::kController) == "reactive-pose";
        if (!playback && !reactive && std::string(config::kController) != "resolved-rate")
            throw std::runtime_error("Config.h kController must be resolved-rate, reactive-pose, or playback");
        if (config::kTargetFile[0] != '\0' && !reactive)
            throw std::runtime_error("Config.h kTargetFile requires reactive-pose");
        if (playback && config::kTrajectoryFile[0] == '\0')
            throw std::runtime_error("Config.h kTrajectoryFile is required for playback");

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

        // Readiness check on a standalone read, BEFORE the takeover: a live
        // fault means we never enter low-level servoing at all.
        const k_api::BaseCyclic::Feedback initial = read_feedback(connection.base_cyclic());
        const bool robot_ready = RobotReadyForTakeover(initial, std::cout); // T1
        if (!robot_ready)
            return 1;
        if (!connection.EnsurePositionControlModes(std::cout))
            return 1;
        PrintRobotState(initial, controlled_model);

        if (playback) {
            // Start-state gate on the pre-takeover read: the arm must
            // already BE at the trajectory's first row (wrapped compare —
            // the file's continuous angles vs the arm's [0,360) feedback).
            // TrajectoryPlayback::Reset re-checks at the takeover itself.
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
        // cycles per second = 1e6 / period_us (1000 at the 1 kHz default)
        LoopLog log(config::kLogBufferSeconds *
                    (1'000'000 / static_cast<std::size_t>(config::kCyclePeriod.count())));
        TargetStore targets;          // resolved-rate (position only)
        PoseTargetStore pose_targets; // reactive-pose (position + orientation)

        if (playback)
            std::cout << "PLAYBACK RUN: the arm will follow the validated "
                         "trajectory as soon as the takeover completes — "
                      << trajectory_summary.duration_s
                      << " s of motion, then a hold at the final point. "
                         "Ctrl+C stops at any time (the position servo holds "
                         "where the command stopped).\n";
        else if (reactive)
            std::cout << "type a desired end-effector target and press Enter; Ctrl+C to "
                         "stop:\n"
                         "  x y z                  (meters, right-arm base frame; "
                         "orientation target "
                         "unchanged)\n"
                         "  x y z roll pitch yaw   (meters + radians, R = Rz·Ry·Rx, "
                         "right-arm base frame)\n"
                      << "reactive-pose position integration at " << config::kControlFrequencyHz
                      << " Hz (full settings echoed above and in the CSV preamble)\n";
        else
            std::cout << "type a desired right end-effector position (x y z, meters, "
                         "right-arm base frame) and press Enter; Ctrl+C to stop\n"
                      << "resolved-rate position integration at " << config::kControlFrequencyHz
                      << " Hz (full settings echoed above and in the CSV preamble)\n";
        // Open the run's CSV BEFORE the takeover: a hardware run must never
        // end with zero evidence because the file could not be created. The
        // '#' config preamble makes every data file self-describing (F3).
        // The rows follow during the run, from the writer thread below.
        // Default: <repo>/runs/YYYY-MM-DD/ (RUNS_ROOT_DIR, baked in by
        // CMake) — the layout the plot scripts search; an explicit
        // An explicit --log filename is used verbatim.
        std::string log_file = options.log_file;
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
        WriteCsvPreamble(options, csv);

        // From here the CSV writes itself: the writer thread drains the log
        // to disk every kLogDrainInterval for as long as it is alive. It is
        // declared after `csv` and before the loop, so it is torn down
        // first on every exit path — including an exception — and its
        // destructor performs the final drain.
        LoopLogWriter log_writer(log, csv, config::kLogDrainInterval);

        // Controller + actuation, constructed before the input thread: a bad
        // end-effector frame name must fail here, before any takeover.
        std::unique_ptr<Controller> controller;
        TrajectoryPlayback* playback_controller = nullptr;
        if (playback) {
            PlaybackSettings settings;
            settings.kp_s_inv = config::kPlaybackKp;
            settings.start_mismatch_limit_deg = config::kStartMismatchLimitDeg;
            auto owned = std::make_unique<TrajectoryPlayback>(
                std::move(*trajectory), settings);
            playback_controller = owned.get();
            controller = std::move(owned);
        } else if (reactive) {
            ReactivePoseGains gains;
            gains.kp_position_s_inv = config::kKpCartesian;
            gains.kp_rotation_s_inv = config::kKpRotation;
            gains.kd_position = config::kKdPosition;
            gains.kd_rotation = config::kKdRotation;
            gains.null_gain_s_inv = config::kNullGain;
            gains.dls_lambda = config::kDlsLambda;
            gains.orientation_enabled = config::kOrientationEnabled;
            gains.velocity_enabled = config::kVelocityTermEnabled;
            gains.null_space_enabled = config::kNullSpaceEnabled;
            Eigen::Matrix<double, 7, 1> midpoint_rad;
            Eigen::Matrix<double, 7, 1> centering_mask;
            for (int i = 0; i < 7; ++i) {
                midpoint_rad[i] = config::kNullMidpointDeg[i] * M_PI / 180.0;
                centering_mask[i] = config::kNullCenteringMask[i];
            }
            controller = std::make_unique<ReactivePose>(
                controlled_model, pose_targets, gains, config::kArrivalToleranceM,
                midpoint_rad, centering_mask, config::kNullRampDurationS);
        } else {
            controller = std::make_unique<ResolvedRate>(
                controlled_model, targets, config::kKpCartesian, config::kDlsLambda,
                config::kArrivalToleranceM);
        }
        PositionIntegration actuation(config::kCommandLeadLimitDeg);

        // Playback has no operator targets: no input thread at all (Ctrl+C
        // is the signal handler, not stdin).
        std::thread input_thread;
        if (!playback)
            input_thread =
                reactive
                    ? std::thread(RunPoseTargetInput, std::ref(pose_targets), std::cref(g_stop))
                    : std::thread(RunTargetInput, std::ref(targets), std::cref(g_stop));
        InputThreadJoiner input_thread_joiner{input_thread};

        // Optional second target source (reactive-pose only, target_file in
        // the config): edit+save the watched file to retarget. Its content
        // at startup is ignored — only in-session edits become targets.
        std::thread target_file_thread;
        if (reactive && config::kTargetFile[0] != '\0') {
            std::cout << "watching target file: " << config::kTargetFile
                      << " (current content ignored; edit and save to retarget)\n";
            target_file_thread = std::thread(RunPoseTargetFileInput, std::ref(pose_targets),
                                             config::kTargetFile, std::cref(g_stop));
        }
        InputThreadJoiner target_file_thread_joiner{target_file_thread};

        // MOVES THE ARM (toward typed positions): servoing mode is entered
        // and restored inside the Runner, on every exit path (T2/D3).
        const StopPolicy stop_policy{
            config::kStopOnFault, config::kNonFiniteStopCycles,
            config::kOverrunStopCycles, config::kOverrunFactor};
        const LoopResult result = RunControlLoop(
            connection.base(), connection.base_cyclic(), *controller, actuation,
            log, g_stop, config::kCyclePeriod, config::kQdotLimitDegS,
            config::kFollowingErrorLimitDeg, stop_policy, robot_ready);

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
        if (playback_controller) {
            if (playback_controller->refused())
                std::cout << "playback outcome: REFUSED at takeover (no "
                             "motion commanded) — exit status is nonzero\n";
            else if (playback_controller->completed())
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
            playback_controller && playback_controller->refused();
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
