//
// Runner — implementation of the loop declared in Runner.h.
//

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <tuple>

#include <KDetailedException.h>

#include "Runner.h"

namespace k_api = Kinova::Api;

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
                    std::uint32_t command_frame_id,
                    const JointVector& commanded_deg,
                    const JointVector& commanded_velocity_deg_s,
                    const ControllerStatus& status)
    {
        for (int i = 0; i < 3; ++i)
        {
            s.p_desired_m[i] = status.p_desired[i];
            s.p_current_m[i] = status.p_current[i];
        }
        s.sigma_min = status.sigma_min;
        s.rot_error_rad = status.rot_error_rad;
        for (int i = 0; i < 4; ++i)
            s.tool_quat_xyzw[i] = status.tool_quat.coeffs()[i]; // Eigen order x,y,z,w
        const Eigen::Vector3d right_base_origin{
            config::kRightBaseOriginControlM[0],
            config::kRightBaseOriginControlM[1],
            config::kRightBaseOriginControlM[2]
        };
        s.pd_beyond_reach = (status.p_desired - right_base_origin).norm() >
            config::kReachRadiusM - config::kReachMarginM;
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
            s.actuator_command_ack[i] = a.command_id();
            s.actuator_status_flags[i] = a.status_flags();
            s.actuator_jitter_us[i] = a.jitter_comm();
        }
        s.command_frame_id = command_frame_id;
        s.feedback_frame_id = fb.frame_id();
        s.arm_state = fb.base().active_state();
        s.base_fault_bank = fb.base().fault_bank_a();
        s.refresh_ok = true;
        for (int i = 0; i < NUM_JOINTS; ++i)
            s.ref_deg[i] = status.q_ref_deg[i];
        s.playback_t_s = status.playback_t_s;
        s.playback_state = status.playback_state;
    }
} // namespace

LoopResult RunControlLoop(k_api::Base::BaseClient* base,
                          k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          ReferenceSource& reference,
                          TrackingController& controller,
                          PositionIntegration& actuation,
                          LoopLog& log, const std::atomic<bool>& stop,
                          std::chrono::microseconds period,
                          const JointVector& qdot_limit_deg_s,
                          double following_error_limit_deg, bool robot_ready)
{
    // T1: the readiness gate is a hard precondition (unreachable from main,
    // which returns before calling us when the gate fails).
    assert(robot_ready);
    if (!robot_ready)
        throw std::logic_error("RunControlLoop called without a passed readiness gate");

    // Stop policy is COMPILE-TIME ONLY (F2, approved 2026-07-22): the
    // config:: constants below are read directly and no CLI flag or file
    // may change them. kStopOnFault = false reproduces the 2026-07-20
    // fault-ignoring experiment (bank changes still print, every cycle's
    // banks are logged, observed faults still force a nonzero exit).
    if (!config::kStopOnFault)
        std::cout << "WARNING: FAULT-STOP DISABLED (config::kStopOnFault = false) — live "
            "fault bits will NOT stop the loop; the following-error guard and the "
            "operator are the backstops. Attended use only.\n";
    if (config::kDisableFollowingErrorStop)
        std::cout << "WARNING: FOLLOWING-ERROR STOP DISABLED "
            "(config::kDisableFollowingErrorStop) — a joint that stops following "
            "its setpoint will NOT stop the loop.\n";
    if (!config::kStopOnFault && config::kDisableFollowingErrorStop)
        std::cout << "WARNING: BOTH the fault stop and the following-error stop are "
            "off. The ONLY automatic stop left is loss of low-level servoing. "
            "YOU are the safety system — hand on the e-stop for this run.\n";

    const double nominal_dt_s = std::chrono::duration<double>(period).count();

    LoopStop reason = LoopStop::kUserStop;
    bool faults_observed = false; // live fault seen at any point (taints exit)
    LoopLogSample sample; // reused every cycle
    long cycle = 0;
    bool joint_fault_was_latched = false;
    CycleCounters counters;
    FeedbackFreshnessMonitor freshness_monitor;

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

        // T4: one holding frame (command == measured), whose reply reseeds
        // the integrator and the controller at the exact pose control will
        // start from. (Until 2026-08-03 this was a 0.5 s streaming
        // handshake window with its own stop classification — trimmed to a
        // single frame; the guards in the loop below cover the same ground
        // from the very first control cycle.)
        feedback = cyclic.Send(commanded_deg);
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            state.q_rad[i] = feedback.actuators(i).position() * kDegToRad;
            state.qdot_rad_s[i] = feedback.actuators(i).velocity() * kDegToRad;
            commanded_deg[i] = feedback.actuators(i).position();
        }
        actuation.Prepare(state);
        controller.Reset(state);
        reference.Reset(state);
        feedback = cyclic.Send(commanded_deg);

        using clock = std::chrono::steady_clock;

        // T5: normal control begins only after the mode gate and the seed.
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
                config::kOverrunFactor * nominal_dt_s)
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

            // Reference then controller: both pure computation. The source
            // says WHERE to be this cycle; the controller turns that into
            // the desired q̇ before clamping.
            status = ControllerStatus{};
            const Reference cycle_reference =
                reference.Get(state, dt_s, status);
            Eigen::Matrix<double, 7, 1> qdot_raw_rad_s =
                controller.DesiredVelocity(state, cycle_reference, dt_s,
                                           status);

            // The requested velocity for the record is the controller's own
            // output, captured BEFORE the non-finite hold and the speed
            // clamp below can modify it (a non-finite value is logged as
            // such — that is the evidence for the kNonFiniteCommand stop).
            JointVector requested_velocity_deg_s{};
            for (int i = 0; i < NUM_JOINTS; ++i)
                requested_velocity_deg_s[i] = qdot_raw_rad_s[i] * kRadToDeg;

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
            // Playback notices, same pattern (TrajectorySource).
            if (status.playback_refused_edge)
                std::cout << "PLAYBACK REFUSED: measured start no longer "
                    "matches the trajectory start (arm moved since "
                    "the pre-takeover gate?) — holding here; Ctrl+C "
                    "to stop\n";
            if (status.playback_done_edge)
                std::cout << "trajectory complete at t=" << status.playback_t_s
                    << " s — holding the final setpoint; Ctrl+C to "
                    "stop\n";

            // Per-joint clamp — the program's single speed limit — then the
            // actuation integrates and produces this cycle's setpoints.
            // A pinned clamp is allowed indefinitely (no saturation stop):
            // far targets transit at clip speed by design.
            Eigen::Matrix<double, 7, 1> qdot_clamped_rad_s;
            for (int i = 0; i < NUM_JOINTS; ++i)
                qdot_clamped_rad_s[i] =
                    std::clamp(qdot_raw_rad_s[i] * kRadToDeg,
                               -qdot_limit_deg_s[i], qdot_limit_deg_s[i]) *
                    kDegToRad;
            const PositionIntegration::ApplyStatus actuation_status =
                actuation.Apply(qdot_clamped_rad_s, state, dt_s, commanded_deg,
                                commanded_velocity_deg_s);

            // The one exchange: send this cycle's position command, receive
            // the feedback the next iteration will use. Stamped on both
            // sides so analysis can match command and feedback by clock.
            const auto t_send = clock::now();
            feedback = cyclic.Send(commanded_deg);
            const auto t_recv = clock::now();
            ++cycle;

            sample.t_s = state.t_s;
            sample.dt_s = std::chrono::duration<double>(t_now - t_prev).count();
            sample.t_send_s = std::chrono::duration<double>(t_send - t_start).count();
            sample.t_recv_s = std::chrono::duration<double>(t_recv - t_start).count();
            t_prev = t_now;
            FillSample(sample, feedback, cyclic.last_command_frame_id(),
                       commanded_deg, commanded_velocity_deg_s, status);
            sample.cycle = cycle;
            sample.requested_deg = actuation_status.requested_deg;
            sample.requested_velocity_deg_s = requested_velocity_deg_s;
            sample.lead_limited = actuation_status.lead_limited;
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
            // exits are unconditional; fault stops obey config::kStopOnFault
            // (compile-time only — F2). With fault-stop disabled (the
            // 2026-07-20 experiment), bank changes still print above, every
            // cycle's banks stay in the CSV, and observed faults still
            // taint the exit code (decision 3).
            if (ClassifyStop(sample, following_error_limit_deg, reason))
            {
                if (reason == LoopStop::kRobotFault)
                    faults_observed = true;
                if (reason != LoopStop::kRobotFault || config::kStopOnFault)
                {
                    log.push(sample);
                    break;
                }
                reason = LoopStop::kUserStop; // ignored fault: not a stop reason
            }
            // Feedback-freshness evidence is RECORDED, never acted on: a
            // repeated acknowledgement ID and a stationary joint are logged
            // (ack_unchanged_j*, req/cmd/meas) and left for offline review.
            // Neither ends the run — see Config.h kCommandLeadLimitDeg.
            freshness_monitor.Update(sample.actuator_command_ack);
            sample.ack_unchanged_cycles = freshness_monitor.unchanged_cycles();
            // Decision-12 counters, checked AFTER ClassifyStop so the guard
            // and live faults keep priority; N <= 0 disables one. There is
            // deliberately NO saturation stop: a pinned velocity clamp is
            // normal transit toward a far target (removed 2026-07-23).
            if (config::kNonFiniteStopCycles > 0 &&
                counters.nonfinite >= config::kNonFiniteStopCycles)
            {
                reason = LoopStop::kNonFiniteCommand;
                log.push(sample);
                break;
            }
            if (config::kOverrunStopCycles > 0 &&
                counters.overrun >= config::kOverrunStopCycles)
            {
                reason = LoopStop::kOverrun;
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
        std::cout << "Kortex API error: " << ex.what() << " (sub-code "
            << k_api::SubErrorCodes_Name(static_cast<k_api::SubErrorCodes>(
                ex.getErrorInfo().getError().error_sub_code()))
            << ")\n";
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

    // D1 -> D2 (with destructor retry if D2 fails) -> D3.
    actuation.Restore();
    // TrajectoryExecution restores explicitly immediately after its cyclic
    // loop. Do the same here (with RAII retry still retained) so any Kortex
    // sub-error is visible and the base gets its documented settling time.
    servoing_guard.Restore(std::cout);
    PrintStopReport(reason, sample, cycle, following_error_limit_deg);
    std::cout << "cycle overruns: " << counters.overrun_total << " of " << cycle
        << " cycles (dt > " << config::kOverrunFactor << " x nominal)\n";
    if (joint_fault_was_latched)
        std::cout << "note: base JOINT_FAULT was latched during the run (stale summary "
            "diagnostic unless a joint fault is shown above; not cleared here)\n";

    return {reason, faults_observed, sample.t_s, cycle};
}

