//
// Runner — implementation of the loop declared in Runner.h.
//

#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <thread>
#include <tuple>

#include <KDetailedException.h>

#include "Runner.h"
#include "StopPriority.h"

namespace k_api = Kinova::Api;

namespace
{
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    struct TakeoverStop
    {
        LoopStop reason;
        bool log_sample = true;
    };

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
        s.null_leak_m_s = status.null_leak_m_s;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            s.qdot_task_deg_s[i] = status.qdot_task_rad_s[i] * kRadToDeg;
            s.qdot_null_deg_s[i] = status.qdot_null_rad_s[i] * kRadToDeg;
        }
        for (int i = 0; i < 4; ++i)
            s.tool_quat_xyzw[i] = status.tool_quat.coeffs()[i]; // Eigen order x,y,z,w
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
        s.joint_traj_activated = status.joint_traj_activated;
        s.joint_traj_rejected = status.joint_traj_rejected;
        s.joint_traj_complete_edge = status.joint_traj_complete_edge;
        s.joint_traj_start_error_deg = status.joint_traj_start_error_deg;
        s.joint_following_error_stop = status.joint_following_error_stop;
        s.joint_following_error_deg = status.joint_following_error_deg;
        s.posture_error_deg = status.posture_error_deg;
        s.base_comp_m = status.base_comp_m;
        s.base_estimate_fresh = status.base_estimate_fresh;
        s.joint_limit_margin_deg = status.joint_limit_margin_deg;
        s.replan_advised = status.replan_advised;
        s.command_frame_id = command_frame_id;
        s.feedback_frame_id = fb.frame_id();
        s.arm_state = fb.base().active_state();
        s.base_fault_bank = fb.base().fault_bank_a();
        s.refresh_ok = true;
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
                          double following_error_limit_deg, bool robot_ready,
                          const char* base_frame)
{
    // T1: the readiness gate is a hard precondition (unreachable from main,
    // which returns before calling us when the gate fails).
    if (!robot_ready)
        throw std::logic_error("RunControlLoop called without a passed readiness gate");

    // Stop policy is compile-time only: the config constants below are read
    // directly and no CLI flag or file may weaken them. Fault-bank changes
    // are printed and logged regardless of the selected stop policy.
    if (!config::kStopOnFault)
        std::cout << "WARNING: FAULT-STOP DISABLED (config::kStopOnFault = false) — live "
            "fault bits will NOT stop the loop; following-error, low-level-servoing, "
            "joint-boundary, stale-feedback, non-finite, and overrun guards still can. "
            "Attended use only.\n";
    if (config::kDisableFollowingErrorStop)
        std::cout << "WARNING: FOLLOWING-ERROR STOP DISABLED "
            "(config::kDisableFollowingErrorStop) — a joint whose MEASURED position "
            "stops following its command will NOT stop the loop. The joint-trajectory "
            "tracking gate (config::kTrajFollowingErrorStopDeg) is separate and still "
            "stops it.\n";
    if (!config::kStopOnFault)
        if (config::kDisableFollowingErrorStop)
            std::cout << "WARNING: BOTH the fault stop and the following-error stop are "
                "off. Low-level-servoing, joint-boundary, stale-feedback, non-finite, "
                "and overrun guards remain; YOU are the safety system — hand on the e-stop "
                "for this run.\n";

    const double nominal_dt_s = std::chrono::duration<double>(period).count();

    LoopStop reason = LoopStop::kUserStop;
    bool faults_observed = false; // live fault seen at any point (taints exit)
    LoopLogSample sample; // reused every cycle
    long cycle = 0;
    int joint_limit_warning_joint = -1;
    int stale_feedback_joint = -1;
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
            commanded_deg[i] = feedback.actuators(i).position();

        // Fault visibility is shared by the takeover hold and normal loop:
        // every bank change is still printed, but policy is resolved from
        // the independently observed sample facts below.
        const auto print_fault_change = [&]()
        {
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
        };

        using clock = std::chrono::steady_clock;
        // CSV time covers the entire low-level takeover, while RobotState
        // time starts only once controller integration is permitted below.
        const auto log_start = clock::now();

        // T4: fixed POSITION hold before either controller state or the
        // integrator can advance. The loop count is derived from the same
        // cyclic period that paces normal control, so this is exactly 0.5 s
        // at the compiled 500 Hz rate.
        auto hold_next = log_start;
        auto hold_prev = log_start;
        ControllerStatus hold_status;
        for (std::size_t hold_cycle = 0;
             hold_cycle < config::kTakeoverHoldCycles && !stop;
             ++hold_cycle)
        {
            hold_next += period;
            const auto t_now = clock::now();
            const auto t_send = clock::now();
            feedback = cyclic.Send(commanded_deg);
            const auto t_recv = clock::now();

            const double hold_dt_s = hold_cycle == 0
                ? nominal_dt_s
                : std::chrono::duration<double>(t_now - hold_prev).count();
            sample.t_s = std::chrono::duration<double>(t_now - log_start).count();
            sample.dt_s = hold_dt_s;
            sample.t_send_s = std::chrono::duration<double>(t_send - log_start).count();
            sample.t_recv_s = std::chrono::duration<double>(t_recv - log_start).count();
            hold_prev = t_now;
            if (hold_cycle > 0 &&
                hold_dt_s > config::kOverrunFactor * nominal_dt_s)
            {
                ++counters.overrun;
                ++counters.overrun_total;
            }
            else
                counters.overrun = 0;
            FillSample(sample, feedback, cyclic.last_command_frame_id(),
                       commanded_deg, commanded_velocity_deg_s, hold_status);
            sample.cycle = 0;
            sample.requested_deg = commanded_deg;
            sample.requested_velocity_deg_s = JointVector{};
            sample.lead_limited = {};
            joint_fault_was_latched =
                joint_fault_was_latched || (sample.base_fault_bank & kJointFaultBit) != 0;
            print_fault_change();

            // The holding command is exactly the fixed Seed measurement, so
            // no integrator proposal exists and the joint-boundary fact is
            // false. All other normal-loop precedence remains active.
            freshness_monitor.Update(sample.actuator_command_ack);
            sample.ack_unchanged_cycles = freshness_monitor.unchanged_cycles();
            const std::optional<int> stale_acknowledgement_joint =
                StaleAcknowledgementJoint(sample.ack_unchanged_cycles,
                                          config::kStaleFeedbackStopCycles);
            const StopPriorityDecision priority = ResolveStopPriority(
                FollowingErrorExceeded(sample, following_error_limit_deg),
                HasLiveFault(sample),
                sample.arm_state != k_api::Common::ArmState::ARMSTATE_SERVOING_LOW_LEVEL,
                false,
                config::kStopOnFault,
                stale_acknowledgement_joint.has_value());
            faults_observed = faults_observed || priority.live_fault_observed;
            switch (priority.reason)
            {
            case StopPriorityReason::kFollowingError:
                reason = LoopStop::kFollowingError;
                break;
            case StopPriorityReason::kLeftLowLevel:
                reason = LoopStop::kLeftLowLevel;
                break;
            case StopPriorityReason::kRobotFault:
                reason = LoopStop::kRobotFault;
                break;
            case StopPriorityReason::kStaleFeedback:
                stale_feedback_joint = *stale_acknowledgement_joint;
                reason = LoopStop::kStaleFeedback;
                break;
            case StopPriorityReason::kJointLimitWarning:
            case StopPriorityReason::kNone:
                break;
            }
            if (priority.reason != StopPriorityReason::kNone)
                throw TakeoverStop{reason};
            // A controller non-finite output and a joint-boundary proposal
            // cannot exist during the fixed hold, but timing stalls still
            // use the normal configured consecutive-cycle guard. It follows
            // live-state and stale-acknowledgement precedence above.
            if (config::kOverrunStopCycles > 0 &&
                counters.overrun >= config::kOverrunStopCycles)
            {
                reason = LoopStop::kOverrun;
                throw TakeoverStop{reason};
            }
            log.push(sample);

            const auto now = clock::now();
            if (hold_next > now)
                std::this_thread::sleep_until(hold_next);
            else
                hold_next = now;
        }
        if (stop)
            throw TakeoverStop{LoopStop::kUserStop, false};

        std::cout << "takeover hold: PASS (" << config::kTakeoverHoldS
            << " s unchanged POSITION command)\n";

        // T5: only after a complete healthy hold do the integrator and
        // controller capture the final measured pose. This prevents any
        // harmless takeover drift becoming an initial command jump.
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            state.q_rad[i] = feedback.actuators(i).position() * kDegToRad;
            state.qdot_rad_s[i] = feedback.actuators(i).velocity() * kDegToRad;
            commanded_deg[i] = feedback.actuators(i).position();
        }
        actuation.Prepare(state);
        controller.Reset(state);
        feedback = cyclic.Send(commanded_deg);

        // T6: normal control begins only after the mode gate and full hold.
        const auto control_start = clock::now();
        auto t_prev = control_start;
        auto next_cycle = control_start;

        ControllerStatus status;

        // Periodic status line: cadence in whole cycles from the same grid
        // that paces the loop; 0 = disabled. The saturation tally counts
        // cycles (not joints) where any clip engaged since the last line.
        const long status_period_cycles = config::kStatusPrintPeriodS > 0.0
            ? std::lround(config::kStatusPrintPeriodS *
                          config::kControlFrequencyHz)
            : 0;
        long cycles_since_status = 0;
        int saturated_cycles_in_window = 0;

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
            state.t_s = std::chrono::duration<double>(t_now - control_start).count();

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

            // Arrival edge: the mailbox advance stays here (pure state, no
            // I/O). The notices themselves print in the post-exchange slack
            // below, with the trajectory edges — a print must never sit
            // between this cycle's compute and its Send.
            if (status.arrived_edge)
                reference.OnArrivalEdge(status);

            // Per-joint clamp — the program's single speed limit — then the
            // actuation integrates and produces this cycle's setpoints.
            // A pinned clamp is allowed indefinitely (no saturation stop):
            // far targets transit at clip speed by design.
            const JointVelocityClampResult clamp_result =
                ClampJointVelocity(qdot_raw_rad_s, qdot_limit_deg_s);
            const Eigen::Matrix<double, 7, 1>& qdot_clamped_rad_s =
                clamp_result.qdot_rad_s;
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

            sample.t_s = std::chrono::duration<double>(t_now - log_start).count();
            sample.dt_s = std::chrono::duration<double>(t_now - t_prev).count();
            sample.t_send_s = std::chrono::duration<double>(t_send - log_start).count();
            sample.t_recv_s = std::chrono::duration<double>(t_recv - log_start).count();
            t_prev = t_now;
            FillSample(sample, feedback, cyclic.last_command_frame_id(),
                       commanded_deg, commanded_velocity_deg_s, status);
            sample.cycle = cycle;
            sample.requested_deg = actuation_status.requested_deg;
            sample.requested_velocity_deg_s = requested_velocity_deg_s;
            sample.lead_limited = actuation_status.lead_limited;
            joint_fault_was_latched =
                joint_fault_was_latched || (sample.base_fault_bank & kJointFaultBit) != 0;

            print_fault_change();

            // Update and record acknowledgement freshness before resolving
            // this completed feedback sample. The count is evidence of a
            // stalled cyclic feedback path, not of physical motion.
            freshness_monitor.Update(sample.actuator_command_ack);
            sample.ack_unchanged_cycles = freshness_monitor.unchanged_cycles();
            const std::optional<int> stale_acknowledgement_joint =
                StaleAcknowledgementJoint(sample.ack_unchanged_cycles,
                                          config::kStaleFeedbackStopCycles);

            // Resolve every fact independently: an ignored fault must still
            // taint the exit and must never mask following error or loss of
            // low-level servoing. Communication exits in the Send catch
            // remain unconditional before this completed feedback sample.
            const StopPriorityDecision priority = ResolveStopPriority(
                FollowingErrorStopRequested(
                    FollowingErrorExceeded(sample, following_error_limit_deg),
                    status.joint_following_error_stop),
                HasLiveFault(sample),
                sample.arm_state != k_api::Common::ArmState::ARMSTATE_SERVOING_LOW_LEVEL,
                actuation_status.joint_limit_warning_joint.has_value(),
                config::kStopOnFault,
                stale_acknowledgement_joint.has_value());
            faults_observed = faults_observed || priority.live_fault_observed;
            switch (priority.reason)
            {
            case StopPriorityReason::kFollowingError:
                reason = LoopStop::kFollowingError;
                log.push(sample);
                break;
            case StopPriorityReason::kLeftLowLevel:
                reason = LoopStop::kLeftLowLevel;
                log.push(sample);
                break;
            case StopPriorityReason::kRobotFault:
                reason = LoopStop::kRobotFault;
                log.push(sample);
                break;
            case StopPriorityReason::kJointLimitWarning:
                joint_limit_warning_joint = *actuation_status.joint_limit_warning_joint;
                reason = LoopStop::kJointLimitWarning;
                log.push(sample);
                break;
            case StopPriorityReason::kStaleFeedback:
                stale_feedback_joint = *stale_acknowledgement_joint;
                reason = LoopStop::kStaleFeedback;
                log.push(sample);
                break;
            case StopPriorityReason::kNone:
                break;
            }
            if (priority.reason != StopPriorityReason::kNone)
                break;
            // Decision-12 counters, checked after the independent live-state
            // and stale-feedback priority so those guards keep priority; N <=
            // 0 disables one. There is deliberately NO saturation stop: a
            // pinned velocity clamp is normal transit toward a far target
            // (removed 2026-07-23).
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

            // Arrival notices (edge-triggered by the controller above;
            // controllers do no I/O, and the print waits for this slack).
            if (status.arrived_edge)
            {
                std::cout << "target reached: " << status.p_desired[0] << " "
                    << status.p_desired[1] << " " << status.p_desired[2]
                    << " m in " << base_frame
                    << ", within " << status.arrival_error_m * 1000.0
                    << " mm — holding\n";
            }
            else if (status.not_reached_edge)
            {
                std::cout << "target NOT reached: "
                    << status.arrival_error_m * 1000.0 << " mm short after "
                    << config::kTargetHoldS
                    << " s (holding; Ctrl+C to abort)\n";
            }

            // Joint-trajectory edges: one bounded line each, edge-triggered
            // by the source. Printed here, in the slack after the exchange
            // and the log push, for the same reason as the status line below
            // — a print must never sit between this cycle's compute and its
            // Send. A rejected plan is the failure the replanning loop must
            // never suffer silently, so it names the splice distance that
            // failed the guard.
            if (status.joint_traj_activated)
                std::cout << "trajectory activated: " << status.joint_traj_points
                    << " points, " << status.joint_traj_duration_s << " s\n";
            if (status.joint_traj_rejected)
                std::cout << "trajectory REJECTED: first point "
                    << status.joint_traj_start_error_deg
                    << " deg from the measured position (limit "
                    << config::kTrajStartToleranceDeg
                    << " deg); the previous reference keeps running\n";
            if (status.joint_traj_complete_edge)
                std::cout << "trajectory complete: holding the final point\n";

            // Status line, in the slack after the exchange and the log push
            // so it never delays a Send. Same thread as the arrival notice;
            // one bounded line per period.
            if (status_period_cycles > 0)
            {
                for (int i = 0; i < NUM_JOINTS; ++i)
                {
                    if (clamp_result.saturated[i])
                    {
                        ++saturated_cycles_in_window;
                        break;
                    }
                }
                if (++cycles_since_status >= status_period_cycles)
                {
                    StatusLineData status_line;
                    status_line.t_s = state.t_s;
                    status_line.position_error_m =
                        (status.p_desired - status.p_current).norm();
                    status_line.rot_error_rad = status.rot_error_rad;
                    status_line.task_speed_deg_s =
                        status.qdot_task_rad_s.norm() * kRadToDeg;
                    status_line.null_speed_deg_s =
                        status.qdot_null_rad_s.norm() * kRadToDeg;
                    status_line.null_leak_m_s = status.null_leak_m_s;
                    status_line.sigma_min = status.sigma_min;
                    status_line.saturated_cycles = saturated_cycles_in_window;
                    status_line.window_cycles =
                        static_cast<int>(cycles_since_status);
                    constexpr int kBoundedJoints[3] = {1, 3, 5}; // j2/j4/j6
                    for (int b = 0; b < 3; ++b)
                    {
                        status_line.bounded_q_deg[b] = std::remainder(
                            state.q_rad[kBoundedJoints[b]] * kRadToDeg, 360.0);
                        status_line.bounded_limit_deg[b] =
                            config::kJointSoftwareLimitDeg[kBoundedJoints[b]];
                    }
                    std::cout << FormatStatusLine(status_line) << "\n";
                    cycles_since_status = 0;
                    saturated_cycles_in_window = 0;
                }
            }

            // Fixed-rate pacing on a grid (sleep_until, so errors don't add
            // up); after a stall, continue instead of bursting.
            const auto now = clock::now();
            if (next_cycle > now)
                std::this_thread::sleep_until(next_cycle);
            else
                next_cycle = now;
        }
    }
    catch (const TakeoverStop& ex)
    {
        reason = ex.reason;
        if (ex.log_sample)
            log.push(sample);
        if (reason != LoopStop::kUserStop)
            std::cout << "takeover hold failed before controller motion began\n";
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
    // Catch-alls: any other exception type (Kinematics' runtime_error
    // validity checks, Pinocchio logic_error, bad_alloc, ...) must not skip
    // the report — the ServoingGuard destructor retries its restore during
    // unwinding if needed. Transport failures are KDetailedException above;
    // a bare runtime_error mid-loop is an internal failure, not the link, so
    // it must not report as "communication" and send diagnosis at the network.
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

    // D1 (with destructor retry if D1 fails) -> D2.
    // TrajectoryExecution restores explicitly immediately after its cyclic
    // loop. Do the same here (with RAII retry still retained) so any Kortex
    // sub-error is visible and the base gets its documented settling time.
    servoing_guard.Restore(std::cout);
    PrintStopReport(reason, sample, cycle, following_error_limit_deg,
                    joint_limit_warning_joint, stale_feedback_joint);
    std::cout << "cycle overruns: " << counters.overrun_total << " of " << cycle
        << " cycles (dt > " << config::kOverrunFactor << " x nominal)\n";
    if (joint_fault_was_latched)
        std::cout << "note: base JOINT_FAULT was latched during the run (stale summary "
            "diagnostic unless a joint fault is shown above; not cleared here)\n";

    return {reason, faults_observed, sample.t_s, cycle};
}
