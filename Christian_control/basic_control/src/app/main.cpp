/*
 * main.cpp — the story of the program, told at a high level:
 *
 *   load configuration + URDF model (Pinocchio; must have 7 velocity vars)
 *     -> connect (TCP + UDP, Connect)
 *     -> readiness check on one feedback frame (before any takeover)
 *     -> print the current joint state and end-effector position
 *     -> start the desired-position input thread (stdin: x y z, meters)
 *     -> run the resolved-rate loop (Loop.cpp — MOVES THE ARM; it enters
 *        LOW_LEVEL_SERVOING, streams position setpoints with the actuators
 *        in their default POSITION mode, and restores single-level
 *        servoing on every exit path)
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
#include <thread>
#include <tuple>

#include "app/Config.h"
#include "control/Loop.h"
#include "control/Target.h"
#include "hardware/Connect.h"
#include "hardware/Measure.h"
#include "hardware/Record.h"
#include "math/Kinematics.h"
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

int main()
{
    std::signal(SIGINT, on_sigint);

    try {
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
        if (!RobotReadyForTakeover(initial, std::cout))
            return 1;
        PrintRobotState(initial, dynamics);

        // All logging memory is allocated here, before the loop starts.
        // cycles per second = 1e6 / period_us (100 at the 100 Hz default)
        LoopLog log(config::kLogCapacitySeconds *
                    (1'000'000 / static_cast<std::size_t>(config::kCyclePeriod.count())));
        TargetStore targets;

        std::cout << "type a desired end-effector position (x y z, meters, base frame) and "
                     "press Enter; Ctrl+C to stop\n"
                  << "resolved-rate position integration at " << config::kControlFrequencyHz
                  << " Hz — Kp " << config::kKpCartesian << " 1/s, lambda "
                  << config::kDlsLambda << ", per-joint q-dot clip "
                  << config::kQdotLimitDegS[0] << "/" << config::kQdotLimitDegS[4]
                  << " deg/s (joints 1-4/5-7), following-error stop "
                  << config::kFollowingErrorLimitDeg << " deg, arrival notice at "
                  << config::kArrivalToleranceM * 1000.0 << " mm\n";
        // Open the run's CSV BEFORE the takeover: a hardware run must never
        // end with zero evidence because the file could not be created.
        const std::string log_file = timestamped_csv_name(config::kLoopLogPrefix);
        std::ofstream csv(log_file);
        if (!csv) {
            std::cerr << "Error: cannot open " << log_file << " — not starting\n";
            return 1;
        }

        std::thread input_thread(RunTargetInput, std::ref(targets), std::cref(g_stop));
        InputThreadJoiner input_thread_joiner{input_thread};

        // MOVES THE ARM (toward typed positions): servoing mode is entered
        // and restored inside the loop, on every exit path.
        const LoopResult result = RunResolvedRateLoop(
            connection.base(), connection.base_cyclic(), dynamics, targets, log, g_stop,
            config::kCyclePeriod, config::kKpCartesian, config::kDlsLambda,
            config::kQdotLimitDegS, config::kFollowingErrorLimitDeg,
            config::kArrivalToleranceM, config::kEndEffectorFrame);

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
