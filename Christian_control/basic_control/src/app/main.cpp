/*
 * main.cpp — the story of the program, told at a high level:
 *
 *   parse options (CLI > TOML > compiled defaults, app/Options)
 *     -> load configuration + URDF model (Pinocchio; must have 7 velocity vars)
 *     -> connect (TCP + UDP, Connect)
 *     -> readiness check on one feedback frame (before any takeover)
 *     -> print the current joint state and end-effector position
 *     -> start the desired-target input thread (stdin: x y z meters, plus
 *        roll pitch yaw radians for the reactive-pose law)
 *     -> run the control loop (loop/Runner.cpp — MOVES THE ARM: takeover
 *        sequence T1-T6, the selected controller (ResolvedRate or
 *        ReactivePose) + PositionIntegration actuation, single-level
 *        servoing restored on every exit path)
 *     -> stop the input thread, write the loop log to CSV
 *     -> RAII teardown, exit 0 only on a clean operator stop.
 *
 * Usage: ./controller   (no flags — settings live in Config.h)
 */

#include <atomic>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <tuple>

#include "app/Config.h"
#include "app/Options.h"
#include "actuation/PositionIntegration.h"
#include "control/ReactivePose.h"
#include "control/ResolvedRate.h"
#include "control/Target.h"
#include "hardware/Connect.h"
#include "hardware/Measure.h"
#include "hardware/Record.h"
#include "loop/Runner.h"
#include "math/Kinematics.h"
#include "safety/Supervisor.h"
#include "Dynamics.h"

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
    void PrintRobotState(const k_api::BaseCyclic::Feedback& feedback, Dynamics& dynamics)
    {
        JointVector position_deg;
        JointVector velocity_deg_s;
        Eigen::VectorXd q_rad(static_cast<int>(position_deg.size()));
        for (size_t i = 0; i < position_deg.size(); ++i) {
            position_deg[i] = feedback.actuators(i).position();
            velocity_deg_s[i] = feedback.actuators(i).velocity();
            q_rad[static_cast<int>(i)] = position_deg[i] * M_PI / 180.0;
        }
        PrintJointHeader();
        PrintRow("position deg", position_deg);
        PrintRow("velocity deg/s", velocity_deg_s);

        Pose ee = forward_kinematics(dynamics, dynamics.convertJointAnglesToConfig(q_rad),
                                     config::kEndEffectorFrame);
        std::cout << "end-effector (" << config::kEndEffectorFrame << "): " << std::fixed
                  << std::setprecision(4) << ee.position.x() << " " << ee.position.y() << " "
                  << ee.position.z() << " (m, base frame)" << std::defaultfloat << "\n";
    }

} // namespace

// Ctrl+C sets this flag; the loop and the input thread both observe it.
std::atomic<bool> g_stop{false};
void on_sigint(int)
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
    std::signal(SIGINT, on_sigint);

    const EffectiveConfig cfg = ParseOptions(argc, argv);

    try {
        EchoConfig(cfg, std::cout);

        // Model + configuration before any hardware session. The controller
        // maps 3 Cartesian velocities onto 7 joint velocities: the model
        // must agree on that 7.
        Dynamics dynamics(GEN3_URDF_PATH);
        if (dynamics.model_.nv != static_cast<int>(std::tuple_size_v<JointVector>))
            throw std::runtime_error("URDF model has " + std::to_string(dynamics.model_.nv) +
                                     " velocity variables, expected 7");

        Connect connection(config::kRobotIp);

        // Readiness check on a standalone read, BEFORE the takeover: a live
        // fault means we never enter low-level servoing at all.
        const k_api::BaseCyclic::Feedback initial = read_feedback(connection.base_cyclic());
        const bool robot_ready = RobotReadyForTakeover(initial, std::cout); // T1
        if (!robot_ready)
            return 1;
        PrintRobotState(initial, dynamics);

        // All logging memory is allocated here, before the loop starts.
        // cycles per second = 1e6 / period_us (100 at the 100 Hz default)
        LoopLog log(config::kLogCapacitySeconds *
                    (1'000'000 / static_cast<std::size_t>(config::kCyclePeriod.count())));
        TargetStore targets;          // resolved-rate (position only)
        PoseTargetStore pose_targets; // reactive-pose (position + orientation)
        const bool reactive = cfg.controller == "reactive-pose";

        if (reactive)
            std::cout << "type a desired end-effector target and press Enter; Ctrl+C to "
                         "stop:\n"
                         "  x y z                  (meters, base frame; orientation target "
                         "unchanged)\n"
                         "  x y z roll pitch yaw   (meters + radians, R = Rz·Ry·Rx, base "
                         "frame)\n"
                      << "reactive-pose position integration at " << config::kControlFrequencyHz
                      << " Hz (full settings echoed above and in the CSV preamble)\n";
        else
            std::cout << "type a desired end-effector position (x y z, meters, base frame) and "
                         "press Enter; Ctrl+C to stop\n"
                      << "resolved-rate position integration at " << config::kControlFrequencyHz
                      << " Hz (full settings echoed above and in the CSV preamble)\n";
        // Open the run's CSV BEFORE the takeover: a hardware run must never
        // end with zero evidence because the file could not be created. The
        // '#' config preamble makes every data file self-describing (F3).
        const std::string log_file = cfg.log_file.empty()
                                         ? timestamped_csv_name(config::kLoopLogPrefix)
                                         : cfg.log_file;
        std::ofstream csv(log_file);
        if (!csv) {
            std::cerr << "Error: cannot open " << log_file << " — not starting\n";
            return 1;
        }
        WriteCsvPreamble(cfg, csv);

        // Controller + actuation, constructed before the input thread: a bad
        // end-effector frame name must fail here, before any takeover.
        std::unique_ptr<Controller> controller;
        if (reactive) {
            ReactivePoseGains gains;
            gains.kp_position_s_inv = cfg.kp;
            gains.kp_rotation_s_inv = cfg.kp_rot;
            gains.kd_position = cfg.kd_pos;
            gains.kd_rotation = cfg.kd_rot;
            gains.null_gain_s_inv = cfg.null_gain;
            gains.dls_lambda = cfg.dls_lambda;
            gains.orientation_enabled = cfg.orientation_enabled;
            gains.velocity_enabled = cfg.velocity_term_enabled;
            gains.null_space_enabled = cfg.null_space_enabled;
            Eigen::Matrix<double, 7, 1> midpoint_rad;
            Eigen::Matrix<double, 7, 1> centering_mask;
            for (int i = 0; i < 7; ++i) {
                midpoint_rad[i] = config::kNullMidpointDeg[i] * M_PI / 180.0;
                centering_mask[i] = config::kNullCenteringMask[i];
            }
            controller = std::make_unique<ReactivePose>(
                dynamics, pose_targets, gains, cfg.arrival_tolerance_m,
                config::kEndEffectorFrame, midpoint_rad, centering_mask);
        } else {
            controller = std::make_unique<ResolvedRate>(
                dynamics, targets, cfg.kp, cfg.dls_lambda, cfg.arrival_tolerance_m,
                config::kEndEffectorFrame);
        }
        PositionIntegration actuation;

        std::thread input_thread =
            reactive
                ? std::thread(RunPoseTargetInput, std::ref(pose_targets), std::cref(g_stop))
                : std::thread(RunTargetInput, std::ref(targets), std::cref(g_stop));
        InputThreadJoiner input_thread_joiner{input_thread};

        // MOVES THE ARM (toward typed positions): servoing mode is entered
        // and restored inside the Runner, on every exit path (T2/D3).
        const StopPolicy stop_policy{
            config::kStopOnFault, cfg.nonfinite_stop_cycles,
            cfg.saturation_stop_cycles, cfg.overrun_stop_cycles,
            cfg.overrun_factor};
        const LoopResult result = RunControlLoop(
            connection.base(), connection.base_cyclic(), *controller, actuation,
            log, g_stop, config::kCyclePeriod, config::kQdotLimitDegS,
            cfg.following_error_limit_deg, stop_policy, robot_ready);

        g_stop = true; // loop may have exited on a fault, not Ctrl+C
        input_thread.join();

        // Flush the log — one file per run (opened before the loop above).
        log.WriteCsv(csv);
        std::cout << log.size() << " samples written to " << log_file;
        if (log.total_pushed() > log.size())
            std::cout << " (" << (log.total_pushed() - log.size())
                      << " oldest samples overwritten by the ring buffer)";
        std::cout << "\n";

        // Only a clean operator stop with no observed faults is success —
        // faults the loop was told to ignore still taint the exit code.
        if (result.faults_observed)
            std::cout << "note: faults occurred during run — exit status is nonzero\n";
        return result.reason == LoopStop::kUserStop && !result.faults_observed ? 0 : 1;
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
