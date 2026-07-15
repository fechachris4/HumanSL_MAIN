//
// Motion: low-level cyclic motion through BaseCyclic — all 7 joints at once.
//
// Pattern follows TrajectoryExecution/src/KinovaTrajectory.cpp
// (joint_position_control): LOW_LEVEL_SERVOING + one command frame per cycle.
//

#include "Motion.h"

#include "Record.h"   // MoveLogSample / write_move_log_*

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <thread>
#include <stdexcept>
#include <filesystem>

#include <KDetailedException.h>   // Kortex API call failures

ServoingModeGuard::ServoingModeGuard(k_api::Base::BaseClient* base) : base_(base)
{
    k_api::Base::ServoingModeInformation mode;
    mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base_->SetServoingMode(mode);
}

ServoingModeGuard::~ServoingModeGuard()
{
    try {
        k_api::Base::ServoingModeInformation mode;
        mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base_->SetServoingMode(mode);
    } catch (...) {
        // Destructors must not throw; nothing more we can do here.
        std::cerr << "warning: failed to restore SINGLE_LEVEL_SERVOING\n";
    }
}

namespace {

constexpr int NUM_JOINTS = 7;

// One bounded step from `current` toward `target`: never overshoots, and the
// final (short) step lands exactly on the target.
double move_towards(double current, double target, double max_step)
{
    double remaining = target - current;
    if (std::abs(remaining) <= max_step)
        return target;
    return current + (remaining > 0 ? max_step : -max_step);
}

// The robot expects command angles in [0, 360).
double wrap_0_360(double angle_deg)
{
    double wrapped = std::fmod(angle_deg, 360.0);
    return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
}

// A joint that must move but may not (speed <= 0) could never arrive.
// `context` names the caller (file path or function) in the error message.
void validate_move_request(const JointVector& deltas_deg,
                           const JointVector& speeds_deg_s,
                           const std::string& context)
{
    for (int i = 0; i < NUM_JOINTS; ++i)
        if (deltas_deg[i] != 0.0 && speeds_deg_s[i] <= 0.0)
            throw std::invalid_argument(
                context + ": joint " + std::to_string(i + 1) +
                " has a nonzero delta but speed <= 0");
}

// Advance every joint one bounded step; true once every command sits
// exactly on its target.
bool advance_commands(JointVector& command_angles,
                      const JointVector& target_angles,
                      const JointVector& speeds_deg_s, double period_s)
{
    bool all_reached = true;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        command_angles[i] = move_towards(command_angles[i], target_angles[i],
                                         speeds_deg_s[i] * period_s);
        if (command_angles[i] != target_angles[i])
            all_reached = false;
    }
    return all_reached;
}

// Copy one cycle's feedback into a log row. Feedback positions are wrapped
// to [0, 360) but our commands are continuous, so each measurement is
// shifted by whole turns next to its command — error math and plots then
// live on the same axis.
void fill_sample(MoveLogSample& s, const k_api::BaseCyclic::Feedback& fb,
                 const JointVector& command_angles)
{
    for (int i = 0; i < NUM_JOINTS; ++i) {
        const auto& a = fb.actuators(i);
        s.commanded_deg[i] = command_angles[i];
        s.measured_deg[i] = command_angles[i] +
            std::remainder(a.position() - command_angles[i], 360.0);
        s.error_deg[i] = s.commanded_deg[i] - s.measured_deg[i];
        s.velocity_deg_s[i] = a.velocity();
        s.torque_nm[i] = a.torque();
        s.current_a[i] = a.current_motor();
        s.fault_bank[i] = a.fault_bank_a();
        s.warning_bank[i] = a.warning_bank_a();
    }
    s.arm_state = fb.base().active_state();
    s.base_fault_bank = fb.base().fault_bank_a();
    s.base_warning_bank = fb.base().warning_bank_a();
    s.refresh_ok = true;
}

// Full state of every moving joint — printed on any failure, so the
// terminal alone tells what was asked, commanded, measured, and missed.
void print_joint_report(const JointVector& deltas_deg,
                        const JointVector& start_angles,
                        const JointVector& target_angles,
                        const MoveLogSample& s)
{
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (deltas_deg[i] == 0.0)
            continue;
        std::cout << "  joint " << (i + 1)
                  << ": requested " << deltas_deg[i]
                  << " deg, moved " << (s.measured_deg[i] - start_angles[i])
                  << " deg | commanded " << s.commanded_deg[i]
                  << ", measured " << s.measured_deg[i]
                  << ", tracking error " << s.error_deg[i]
                  << ", target " << target_angles[i]
                  << ", to target " << (target_angles[i] - s.measured_deg[i])
                  << " deg\n";
    }
}

// True — after printing which fault bank fired — when the base or any
// actuator raises a fault bit.
bool report_faults(const MoveLogSample& s)
{
    bool fault = false;
    if (s.base_fault_bank != 0) {
        std::cout << "move failed: base fault bank " << s.base_fault_bank
                  << " at t=" << s.t_s << " s\n";
        fault = true;
    }
    for (int i = 0; i < NUM_JOINTS; ++i)
        if (s.fault_bank[i] != 0) {
            std::cout << "move failed: actuator " << (i + 1)
                      << " fault bank " << s.fault_bank[i]
                      << " at t=" << s.t_s << " s\n";
            fault = true;
        }
    return fault;
}

// A Kortex call failed mid-move: log a final row flagged refresh_ok=0,
// print the original error, fail the move.
bool abort_move(const char* kind, const char* what,
                std::ostream* log, MoveLogSample& sample)
{
    sample.refresh_ok = false;
    if (log)
        write_move_log_row(*log, sample);
    std::cout << "move failed: " << kind << ": " << what << "\n";
    return false;
}

} // namespace

k_api::BaseCyclic::Feedback send_positions(
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
    k_api::BaseCyclic::Command& command,
    const JointVector& angles)
{
    for (int i = 0; i < NUM_JOINTS; ++i)
        command.mutable_actuators(i)->set_position(wrap_0_360(angles[i]));
    command.set_frame_id((command.frame_id() + 1) % 65536);
    for (int i = 0; i < NUM_JOINTS; ++i)
        command.mutable_actuators(i)->set_command_id(command.frame_id());
    return base_cyclic->Refresh(command, 0);
}

JointVector broadcast_speed(double speed_deg_s)
{
    JointVector speeds;
    speeds.fill(speed_deg_s);
    return speeds;
}

namespace {

// Parse the rest of a config line into exactly 7 numbers.
JointVector parse_seven(std::istringstream& line, const std::string& key)
{
    JointVector v;
    for (int i = 0; i < NUM_JOINTS; ++i)
        if (!(line >> v[i]))
            throw std::invalid_argument(key + ": expected 7 numbers");
    double extra;
    if (line >> extra)
        throw std::invalid_argument(key + ": more than 7 numbers");
    return v;
}

} // namespace

std::string find_motion_config()
{
    namespace fs = std::filesystem;

    std::vector<fs::path> candidates = {fs::current_path() / "motion.txt"};

    // Where the running executable lives (Linux). Lets the file sit next to
    // the binary (build/) or one level up (basic_control/).
    std::error_code ec;
    fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        candidates.push_back(exe.parent_path() / "motion.txt");
        candidates.push_back(exe.parent_path().parent_path() / "motion.txt");
    }

    for (const auto& p : candidates)
        if (fs::exists(p))
            return fs::absolute(p).lexically_normal().string();
    return "";
}

MotionConfig load_motion_config(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
        throw std::invalid_argument("cannot open motion config: " + path);

    bool have_deltas = false, have_speeds = false;
    bool have_default_speed = false, have_limits = false;
    std::string mode = "joints";
    JointVector deltas{}, speeds{};
    JointVector limits = kDefaultSpeedLimits;
    double default_speed = 5.0;  // used if no speed line is given

    std::string raw;
    while (std::getline(file, raw)) {
        // Strip comments; skip blank lines.
        std::string text = raw.substr(0, raw.find('#'));
        std::istringstream line(text);
        std::string key;
        if (!(line >> key))
            continue;

        if (key == "mode:") {
            if (!(line >> mode) || (mode != "joints" && mode != "reactive"))
                throw std::invalid_argument(
                    path + ": mode must be 'joints' or 'reactive'");
        } else if (key == "deltas_deg:") {
            deltas = parse_seven(line, "deltas_deg");
            have_deltas = true;
        } else if (key == "speeds_deg_s:") {
            speeds = parse_seven(line, "speeds_deg_s");
            have_speeds = true;
        } else if (key == "default_speed_deg_s:") {
            if (!(line >> default_speed))
                throw std::invalid_argument("default_speed_deg_s: expected a number");
            have_default_speed = true;
        } else if (key == "speed_limits_deg_s:") {
            limits = parse_seven(line, "speed_limits_deg_s");
            have_limits = true;
        } else {
            throw std::invalid_argument("unknown line in " + path + ": " + raw);
        }
    }

    MotionConfig config;
    config.mode = mode;
    config.deltas = deltas;
    config.speeds = have_speeds ? speeds : broadcast_speed(default_speed);
    config.limits = limits;

    // Reactive mode takes its target and limits from Config.h. Reject the
    // joint move's lines instead of ignoring them, so the file can't look
    // like one mode and run the other.
    if (config.mode == "reactive") {
        if (have_deltas || have_speeds || have_default_speed || have_limits)
            throw std::invalid_argument(
                path + ": mode 'reactive' takes no joint-move lines "
                "(remove deltas_deg/speeds_deg_s/default_speed_deg_s/"
                "speed_limits_deg_s)");
        return config;
    }

    if (!have_deltas)
        throw std::invalid_argument(path + ": missing required line 'deltas_deg: D1 .. D7'");

    // Fail at load time (before connecting) rather than in the motion loop.
    validate_move_request(config.deltas, config.speeds, path);
    for (int i = 0; i < NUM_JOINTS; ++i)
        if (config.speeds[i] > config.limits[i])
            throw std::invalid_argument(
                path + ": joint " + std::to_string(i + 1) + " speed " +
                std::to_string(config.speeds[i]) + " deg/s exceeds limit " +
                std::to_string(config.limits[i]) + " deg/s");

    return config;
}

bool move_joints_relative(k_api::Base::BaseClient* base,
                          k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          const JointVector& deltas_deg,
                          const JointVector& speeds_deg_s,
                          const std::atomic<bool>& stop,
                          std::ostream* log,
                          std::chrono::microseconds period)
{
    const double period_s = std::chrono::duration<double>(period).count();

    // Hold this long after the ramp so the arm settles before the final
    // measured check, and overshoot/settling shows up in the log.
    constexpr auto kSettleWindow = std::chrono::seconds(1);

    validate_move_request(deltas_deg, speeds_deg_s, "move_joints_relative");

    if (log)
        write_move_log_header(*log);
    MoveLogSample sample;   // last cycle's row, reused for the error row

    // Every Kortex call sits inside this try: a failed send or a missing
    // reply aborts the move with the original error and `false`, never
    // silently. Unwinding destroys the guard, so the servoing mode is
    // restored on every path.
    try {
        // From here until the guard is destroyed, WE are the controller.
        ServoingModeGuard servoing(base);

        // Read the joints AFTER the mode switch, never before: the round
        // trip gives the base time to finish entering LOW_LEVEL_SERVOING
        // (sending the first command too early fails with
        // WRONG_SERVOING_MODE). Commands start at the measurement, so
        // taking over causes no jump; zero delta means that joint holds.
        k_api::BaseCyclic::Feedback feedback = base_cyclic->RefreshFeedback();
        JointVector start_angles, target_angles, command_angles;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            start_angles[i] = feedback.actuators(i).position();
            target_angles[i] = start_angles[i] + deltas_deg[i];
            command_angles[i] = start_angles[i];
        }

        k_api::BaseCyclic::Command command;
        for (int i = 0; i < NUM_JOINTS; ++i)
            command.add_actuators();
        send_positions(base_cyclic, command, start_angles);  // hold in place

        using clock = std::chrono::steady_clock;
        const auto t_start = clock::now();
        auto t_prev = t_start;
        auto next_cycle = t_start;
        clock::time_point settle_end{};  // set once all commands arrived
        int bad_cycles = 0;  // consecutive cycles with a big tracking error

        // The controller, once per `period`:
        //   step commands -> exchange with robot -> log -> fault check ->
        //   tracking check -> settle, then verify -> wait for the next slot.
        while (!stop) {
            next_cycle += period;

            bool all_reached = advance_commands(command_angles, target_angles,
                                                speeds_deg_s, period_s);
            feedback = send_positions(base_cyclic, command, command_angles);

            auto t_now = clock::now();
            sample.t_s = std::chrono::duration<double>(t_now - t_start).count();
            sample.dt_s = std::chrono::duration<double>(t_now - t_prev).count();
            t_prev = t_now;
            fill_sample(sample, feedback, command_angles);
            if (log)
                write_move_log_row(*log, sample);

            if (report_faults(sample)) {
                print_joint_report(deltas_deg, start_angles, target_angles,
                                   sample);
                return false;
            }

            // Tracking watchdog: the arm stopped following the stream. The
            // persistence filter keeps the ~2 deg start-up transient and
            // single-cycle blips from tripping it.
            bool tracking_bad = false;
            for (int i = 0; i < NUM_JOINTS; ++i)
                if (deltas_deg[i] != 0.0 &&
                    std::abs(sample.error_deg[i]) > kTrackingErrorLimitDeg)
                    tracking_bad = true;
            bad_cycles = tracking_bad ? bad_cycles + 1 : 0;
            if (bad_cycles >= kTrackingErrorCycleLimit) {
                std::cout << "move failed: tracking error above "
                          << kTrackingErrorLimitDeg << " deg for "
                          << kTrackingErrorCycleLimit << " cycles at t="
                          << sample.t_s << " s\n";
                print_joint_report(deltas_deg, start_angles, target_angles,
                                   sample);
                // Freeze where the arm actually is, so we don't leave a
                // setpoint it is still straining toward.
                send_positions(base_cyclic, command, sample.measured_deg);
                return false;
            }

            // Ramp done: hold through the settle window, then believe only
            // the measurement.
            if (all_reached) {
                if (settle_end == clock::time_point{}) {
                    settle_end = clock::now() + kSettleWindow;
                } else if (clock::now() >= settle_end) {
                    bool arrived = true;
                    for (int i = 0; i < NUM_JOINTS; ++i)
                        if (deltas_deg[i] != 0.0 &&
                            std::abs(target_angles[i] - sample.measured_deg[i]) >
                                kReachedToleranceDeg)
                            arrived = false;
                    if (arrived) {
                        std::cout << "move ok: all moving joints measured "
                                     "within " << kReachedToleranceDeg
                                  << " deg of target\n";
                        return true;
                    }
                    std::cout << "move failed: final measured position "
                                 "missed the target\n";
                    print_joint_report(deltas_deg, start_angles,
                                       target_angles, sample);
                    return false;
                }
            }

            // Fixed-rate pacing on a grid (sleep_until, so errors don't add
            // up); after a stall, continue instead of bursting to catch up.
            auto now = clock::now();
            if (next_cycle > now)
                std::this_thread::sleep_until(next_cycle);
            else
                next_cycle = now;
        }

        std::cout << "move stopped by user (Ctrl+C) before reaching target\n";
        return false;
    }
    catch (k_api::KDetailedException& ex) {
        return abort_move("Kortex API error", ex.what(), log, sample);
    }
    catch (std::runtime_error& ex) {
        return abort_move("communication error", ex.what(), log, sample);
    }
}
