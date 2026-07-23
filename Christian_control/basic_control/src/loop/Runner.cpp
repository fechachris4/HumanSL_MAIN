//
// Runner: the cyclic control loop (sequence spec in Runner.h).
//

#include "loop/Runner.h"

#include "hardware/Cyclic.h"
#include "math/Dls.h" // ClampedCycleDt
#include "safety/FaultReport.h"
#include "safety/ServoingGuard.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <tuple>

#include <KDetailedException.h>

namespace
{
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    // Copy one cycle's data into a log row. Feedback positions are wrapped
    // to [0, 360) but the integrated command is continuous, so each
    // measurement is shifted by whole turns next to its command — tracking
    // error (cmd - meas) and plots then live on the same axis.
    void FillSample(LoopLogSample& s, const k_api::BaseCyclic::Feedback& fb,
                    const JointVector& commanded_deg,
                    const JointVector& commanded_velocity_deg_s,
                    const Eigen::Vector3d& p_desired, const Eigen::Vector3d& p_current)
    {
        for (int i = 0; i < 3; ++i)
        {
            s.p_desired_m[i] = p_desired[i];
            s.p_current_m[i] = p_current[i];
        }
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            const auto& a = fb.actuators(i);
            s.commanded_deg[i] = commanded_deg[i];
            s.commanded_velocity_deg_s[i] = commanded_velocity_deg_s[i];
            s.measured_deg[i] =
                commanded_deg[i] + std::remainder(a.position() - commanded_deg[i], 360.0);
            s.measured_raw_deg[i] = a.position();
            s.velocity_deg_s[i] = a.velocity();
            s.torque_nm[i] = a.torque();
            s.fault_bank[i] = a.fault_bank_a();
        }
        s.arm_state = fb.base().active_state();
        s.base_fault_bank = fb.base().fault_bank_a();
        s.refresh_ok = true;
    }
} // namespace

LoopResult RunControlLoop(k_api::Base::BaseClient* base,
                          k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          Controller& controller, Actuation& actuation,
                          LoopLog& log, const std::atomic<bool>& stop,
                          std::chrono::microseconds period,
                          const JointVector& qdot_limit_deg_s,
                          double following_error_limit_deg,
                          const StopPolicy& policy, bool robot_ready)
{
    // T1: the readiness gate is a hard precondition (unreachable from main,
    // which returns before calling us when the gate fails).
    assert(robot_ready);
    if (!robot_ready)
        throw std::logic_error("RunControlLoop called without a passed readiness gate");

    if (!policy.stop_on_fault)
        std::cout << "WARNING: FAULT-STOP DISABLED (config::kStopOnFault = false) — live "
            "fault bits will NOT stop the loop; the following-error guard and the "
            "operator are the backstops. Attended use only.\n";

    const double nominal_dt_s = std::chrono::duration<double>(period).count();

    LoopStop reason = LoopStop::kUserStop;
    bool faults_observed = false; // live fault seen at any point (taints exit)
    LoopLogSample sample; // reused every cycle
    long cycle = 0;
    bool joint_fault_was_latched = false;
    CycleCounters counters;

    CyclicSession cyclic(base_cyclic);

    // T2: from here until the guard's destructor runs, WE are the controller.
    ServoingGuard servoing_guard(base);

    try
    {
        // T3: seed AFTER the mode switch (the round trip gives the base time
        // to finish entering LOW_LEVEL_SERVOING). The only standalone read.
        k_api::BaseCyclic::Feedback feedback = cyclic.Seed();

        RobotState state;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            state.q_rad[i] = feedback.actuators(i).position() * kDegToRad;
            state.qdot_rad_s[i] = feedback.actuators(i).velocity() * kDegToRad;
        }

        // T4: the integrator seed, q_command = q_measured — the ONLY time
        // (resolved-rate-position-integration.md, "state distinction").
        actuation.Prepare(state);

        // T5: the controller seeds its own hold-here state.
        controller.Reset(state);

        // Fault-change printing state, seeded from the startup read so a
        // bank already latched at entry (allowed by RobotReadyForTakeover
        // for the JOINT_FAULT summary) does not print as a fresh event.
        std::array<std::uint32_t, 7> prev_joint_banks{};
        for (int i = 0; i < NUM_JOINTS; ++i)
            prev_joint_banks[i] = feedback.actuators(i).fault_bank_a();
        std::uint32_t prev_base_bank = feedback.base().fault_bank_a();
        int fault_prints = 0;

        JointVector commanded_deg;
        JointVector commanded_velocity_deg_s{};
        for (int i = 0; i < NUM_JOINTS; ++i)
            commanded_deg[i] = state.q_rad[i] * kRadToDeg;

        // T6: first unchanged holding frame (command == measured); its reply
        // is the loop's first input.
        feedback = cyclic.Send(commanded_deg);

        using clock = std::chrono::steady_clock;
        const auto t_start = clock::now();
        auto t_prev = t_start;
        auto next_cycle = t_start;

        ControllerStatus status;

        while (!stop)
        {
            next_cycle += period;

            // dt = measured elapsed cycle time (nominal on the first cycle),
            // clamped so a stall cannot integrate one large jump. Sampled at
            // cycle start — dt is an input to the controller.
            const auto t_now = clock::now();
            const double dt_s =
                cycle == 0
                    ? nominal_dt_s
                    : ClampedCycleDt(
                        std::chrono::duration<double>(t_now - t_prev).count(),
                        nominal_dt_s);
            if (cycle > 0 &&
                std::chrono::duration<double>(t_now - t_prev).count() >
                    policy.overrun_factor * nominal_dt_s)
            {
                ++counters.overrun;
                ++counters.overrun_total;
            }
            else
                counters.overrun = 0;

            // Measured state from the previous exchange; degrees -> radians
            // at this boundary.
            for (int i = 0; i < NUM_JOINTS; ++i)
            {
                state.q_rad[i] = feedback.actuators(i).position() * kDegToRad;
                state.qdot_rad_s[i] = feedback.actuators(i).velocity() * kDegToRad;
            }
            state.t_s = std::chrono::duration<double>(t_now - t_start).count();

            // The controller: pure computation, desired q̇ before clamping.
            status = ControllerStatus{};
            Eigen::Matrix<double, 7, 1> qdot_raw_rad_s =
                controller.DesiredVelocity(state, dt_s, status);

            // Non-finite output never reaches the integrator: hold this
            // cycle and count it (decision 12).
            if (!qdot_raw_rad_s.allFinite())
            {
                qdot_raw_rad_s.setZero();
                ++counters.nonfinite;
            }
            else
                counters.nonfinite = 0;

            // Arrival notice (edge-triggered data from the controller; the
            // print lives out here — controllers do no I/O).
            if (status.arrived_edge)
                std::cout << "target reached: " << status.p_desired[0] << " "
                          << status.p_desired[1] << " " << status.p_desired[2]
                          << " m, within " << status.arrival_error_m * 1000.0
                          << " mm — holding\n";

            // Per-joint clamp — the program's single speed limit — then the
            // actuation integrates and produces this cycle's setpoints.
            Eigen::Matrix<double, 7, 1> qdot_clamped_rad_s;
            bool any_joint_saturated = false;
            for (int i = 0; i < NUM_JOINTS; ++i)
            {
                const double desired_deg_s = qdot_raw_rad_s[i] * kRadToDeg;
                if (desired_deg_s < -qdot_limit_deg_s[i] ||
                    desired_deg_s > qdot_limit_deg_s[i])
                    any_joint_saturated = true;
                qdot_clamped_rad_s[i] =
                    std::clamp(desired_deg_s, -qdot_limit_deg_s[i],
                               qdot_limit_deg_s[i]) *
                    kDegToRad;
            }
            counters.saturated = any_joint_saturated ? counters.saturated + 1 : 0;
            actuation.Apply(qdot_clamped_rad_s, dt_s, commanded_deg,
                            commanded_velocity_deg_s);

            // The one exchange: send this cycle's position command, receive
            // the feedback the next iteration will use.
            feedback = cyclic.Send(commanded_deg);
            ++cycle;

            sample.t_s = state.t_s;
            sample.dt_s = std::chrono::duration<double>(t_now - t_prev).count();
            t_prev = t_now;
            FillSample(sample, feedback, commanded_deg, commanded_velocity_deg_s,
                       status.p_desired, status.p_current);
            sample.sigma_min = status.sigma_min;
            joint_fault_was_latched =
                joint_fault_was_latched || (sample.base_fault_bank & kJointFaultBit) != 0;

            // Every fault-bank change prints as it happens (edge-triggered,
            // capped — see PrintFaultChange). This is live visibility, not
            // policy: whether the loop stops is still ClassifyStop's call.
            if (sample.base_fault_bank != prev_base_bank ||
                sample.fault_bank != prev_joint_banks)
            {
                if (fault_prints < kMaxFaultChangePrints)
                {
                    PrintFaultChange(sample, cycle, prev_joint_banks, prev_base_bank);
                    if (++fault_prints == kMaxFaultChangePrints)
                        std::cout << "further fault-bank changes not printed (limit "
                                  << kMaxFaultChangePrints
                                  << "); every cycle's banks are in the CSV\n";
                }
                prev_base_bank = sample.base_fault_bank;
                prev_joint_banks = sample.fault_bank;
            }

            // Stop policy: the following-error, arm-state and communication
            // exits are unconditional; fault stops obey policy.stop_on_fault
            // (config::kStopOnFault, compile-time only — F2). With
            // fault-stop disabled (the 2026-07-20 experiment), bank changes
            // still print above, every cycle's banks stay in the CSV, and
            // observed faults still taint the exit code (decision 3).
            if (ClassifyStop(sample, following_error_limit_deg, reason))
            {
                if (reason == LoopStop::kRobotFault)
                    faults_observed = true;
                if (reason != LoopStop::kRobotFault || policy.stop_on_fault)
                {
                    log.push(sample);
                    break;
                }
                reason = LoopStop::kUserStop; // ignored fault: not a stop reason
            }
            // Decision-12 counters, checked AFTER ClassifyStop so the guard
            // and live faults keep priority.
            if (const auto counter_stop = ClassifyCounters(counters, policy))
            {
                reason = *counter_stop;
                log.push(sample);
                break;
            }
            log.push(sample);

            // Fixed-rate pacing on a grid (sleep_until, so errors don't add
            // up); after a stall, continue instead of bursting.
            const auto now = clock::now();
            if (next_cycle > now)
                std::this_thread::sleep_until(next_cycle);
            else
                next_cycle = now;
        }
    }
    catch (k_api::KDetailedException& ex)
    {
        reason = LoopStop::kCommunication;
        sample.refresh_ok = false;
        log.push(sample);
        std::cout << "Kortex API error: " << ex.what() << "\n";
    }
    catch (std::runtime_error& ex)
    {
        reason = LoopStop::kCommunication;
        sample.refresh_ok = false;
        log.push(sample);
        std::cout << "communication error: " << ex.what() << "\n";
    }
    // Catch-alls: an exception of any other type (Pinocchio logic_error,
    // bad_alloc, ...) must not skip the report — and the servoing restore
    // no longer even depends on being caught (D3 runs by unwinding).
    catch (std::exception& ex)
    {
        reason = LoopStop::kInternalError;
        sample.refresh_ok = false;
        log.push(sample);
        std::cout << "internal error: " << ex.what() << "\n";
    }
    catch (...)
    {
        reason = LoopStop::kInternalError;
        sample.refresh_ok = false;
        log.push(sample);
        std::cout << "internal error: unknown exception type\n";
    }

    // D1 -> D2; D3 is the guard's destructor, after the return statement.
    actuation.Restore();
    PrintStopReport(reason, sample, cycle, following_error_limit_deg);
    std::cout << "cycle overruns: " << counters.overrun_total << " of " << cycle
              << " cycles (dt > " << policy.overrun_factor << " x nominal)\n";
    if (joint_fault_was_latched)
        std::cout << "note: base JOINT_FAULT was latched during the run (stale summary "
            "diagnostic unless a joint fault is shown above; not cleared here)\n";

    return {reason, faults_observed};
}
