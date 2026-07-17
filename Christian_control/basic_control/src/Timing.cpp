//
// Timing: latency benchmarks for the control loop.
//

#include "Timing.h"

#include "Measure.h" // read_feedback — the single robot state reader

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

namespace
{

    int64_t now_us()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    // Write min / mean / max / stddev (jitter) / 99th percentile of the samples.
    void print_stats(const char* name, std::vector<int64_t> samples_us, std::ostream& out)
    {
        std::sort(samples_us.begin(), samples_us.end());
        const size_t n = samples_us.size();

        double mean = 0;
        for (int64_t s : samples_us)
            mean += s;
        mean /= n;

        double var = 0;
        for (int64_t s : samples_us)
            var += (s - mean) * (s - mean);
        double stddev = std::sqrt(var / n);

        out << name << "  (" << n << " cycles)\n"
            << "  min   " << samples_us.front() << " us\n"
            << "  mean  " << mean << " us\n"
            << "  p99   " << samples_us[n * 99 / 100] << " us\n"
            << "  max   " << samples_us.back() << " us\n"
            << "  jitter (stddev) " << stddev << " us\n";
    }

} // namespace

void time_feedback_roundtrip(k_api::BaseCyclic::BaseCyclicClient* base_cyclic, std::ostream& out,
                             int cycles)
{
    std::vector<int64_t> samples;
    samples.reserve(cycles);

    for (int c = 0; c < cycles; ++c) {
        int64_t t0 = now_us();
        k_api::BaseCyclic::Feedback fb = read_feedback(base_cyclic);
        // Touch the data so the timing includes having the angles "in hand".
        volatile double angle = fb.actuators(0).position();
        (void)angle;
        samples.push_back(now_us() - t0);
    }
    print_stats("feedback round-trip (RefreshFeedback)", samples, out);
}

void time_control_cycle(k_api::Base::BaseClient* base,
                        k_api::BaseCyclic::BaseCyclicClient* base_cyclic, std::ostream& out,
                        int cycles)
{
    // We become the controller for the duration of the test; the command we
    // stream is "stay exactly where you are", so the arm does not move.
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    try {
        k_api::BaseCyclic::Feedback feedback = read_feedback(base_cyclic);
        k_api::BaseCyclic::Command command;
        for (int i = 0; i < feedback.actuators_size(); ++i)
            command.add_actuators()->set_position(feedback.actuators(i).position());

        std::vector<int64_t> samples;
        samples.reserve(cycles);

        for (int c = 0; c < cycles; ++c) {
            command.set_frame_id((command.frame_id() + 1) % 65536);
            for (int i = 0; i < feedback.actuators_size(); ++i)
                command.mutable_actuators(i)->set_command_id(command.frame_id());

            int64_t t0 = now_us();
            feedback = base_cyclic->Refresh(command, 0);
            samples.push_back(now_us() - t0);
        }
        print_stats("full control cycle (Refresh: command out + feedback in)", samples, out);
    } catch (std::exception& ex) {
        std::cerr << "time_control_cycle: error: " << ex.what() << "\n";
    }

    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
}

void time_dynamics_solve(Dynamics& dynamics, const Eigen::VectorXd& q_pin, std::ostream& out,
                         int cycles)
{
    std::vector<int64_t> samples;
    samples.reserve(cycles);

    for (int c = 0; c < cycles; ++c) {
        int64_t t0 = now_us();
        Eigen::VectorXd tau_g = dynamics.gravity_m(q_pin);
        volatile double touch = tau_g(0);
        (void)touch;
        samples.push_back(now_us() - t0);
    }
    print_stats("dynamics solve (gravity torques, Pinocchio)", samples, out);
}
