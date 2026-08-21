//
// Runner — implementation of the loop declared in Runner.h.
//

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <thread>
#include <tuple>

#include <KDetailedException.h>

#include "Runner.h"

#include "BasePose.h"
#include "StopPriority.h"

namespace k_api = Kinova::Api;

namespace
{
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;
    constexpr double kRadToDeg = 180.0 / M_PI;
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

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
                    const ControllerStatus& status,
                    const RobotState& state,
                    const MeasuredCartesianState& measured,
                    const PoseReference& reference,
                    bool cartesian_available = true)
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
        s.cart_traj_activated = status.cartesian_traj_activated;
        s.cart_traj_rejected = status.cartesian_traj_rejected;
        s.cart_traj_complete = status.cartesian_traj_complete_edge;
        s.cart_traj_cancelled = status.cartesian_traj_cancelled_edge;
        s.cart_replan_requested = status.request_replan_edge;
        s.cart_start_position_error_m =
            status.cartesian_traj_start_position_error_m;
        s.cart_start_orientation_error_rad =
            status.cartesian_traj_start_orientation_error_rad;
        s.cart_trajectory_id = reference.trajectory_id;
        s.cart_planner_vicon_sequence = reference.planner_vicon_sequence;
        s.cart_reference_time_s = reference.t_from_start_s;
        const Eigen::Quaterniond ref_q(reference.ee_pose_world.rotation);
        const Eigen::Quaterniond measured_q(measured.ee_pose_world.rotation);
        for (int i = 0; i < 3; ++i) {
            s.cart_ref_position_world_m[i] = reference.ee_pose_world.position_m[i];
            s.cart_ref_linear_world_m_s[i] = reference.ee_twist_world.linear_m_s[i];
            s.cart_ref_angular_world_rad_s[i] =
                reference.ee_twist_world.angular_rad_s[i];
            s.cart_measured_position_world_m[i] =
                measured.ee_pose_world.position_m[i];
            s.cart_measured_linear_world_m_s[i] =
                measured.ee_twist_world.linear_m_s[i];
            s.cart_measured_angular_world_rad_s[i] =
                measured.ee_twist_world.angular_rad_s[i];
        }
        for (int i = 0; i < 4; ++i) {
            s.cart_ref_quat_world_xyzw[i] = ref_q.coeffs()[i];
            s.cart_measured_quat_world_xyzw[i] = measured_q.coeffs()[i];
        }
        for (int i = 0; i < 3; ++i) {
            s.measured_tcp_mount_m[i] = cartesian_available
                ? measured.ee_pose_mount.position_m[i] : kNaN;
            s.commanded_tcp_mount_m[i] = kNaN;
        }
        const Eigen::Quaterniond measured_mount_q(
            measured.ee_pose_mount.rotation);
        for (int i = 0; i < 4; ++i) {
            s.measured_tcp_quat_mount_xyzw[i] = cartesian_available
                ? measured_mount_q.coeffs()[i] : kNaN;
            s.commanded_tcp_quat_mount_xyzw[i] = kNaN;
        }
        for (int point = 0; point < 9; ++point)
            for (int axis = 0; axis < 3; ++axis)
                s.measured_arm_chain_mount_m[point][axis] = cartesian_available
                    ? measured.arm_chain_mount_m[point][axis] : kNaN;
        s.world_fresh = state.world_fresh;
        s.world_mount_twist_valid = state.world_mount_twist_valid;
        s.command_frame_id = command_frame_id;
        s.feedback_frame_id = fb.frame_id();
        s.arm_state = fb.base().active_state();
        s.base_fault_bank = fb.base().fault_bank_a();
        s.refresh_ok = true;
    }

    // Takeover rows run before world measurement/reference generation. They
    // use the same robot/actuation fields and leave Cartesian
    // evidence at its explicit default values.
    void FillSample(LoopLogSample& s, const k_api::BaseCyclic::Feedback& fb,
                    std::uint32_t command_frame_id,
                    const JointVector& commanded_deg,
                    const JointVector& commanded_velocity_deg_s,
                    const ControllerStatus& status)
    {
        FillSample(s, fb, command_frame_id, commanded_deg,
                   commanded_velocity_deg_s, status, RobotState{},
                   MeasuredCartesianState{}, PoseReference{}, false);
    }
} // namespace

LoopResult RunControlLoop(k_api::Base::BaseClient* base,
                          k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          ArmExecutionCore& core,
                          LoopLog& log, const std::atomic<bool>& stop,
                          std::chrono::microseconds period,
                          double following_error_limit_deg, bool robot_ready,
                          PlanningArm planning_arm,
                          PlanningRequestSlot* planning_requests,
                          BasePoseSlot* base_pose,
                          GoalCommandSlot* live_goals)
{
    // T1: the readiness gate is a hard precondition (unreachable from main,
    // which returns before calling us when the gate fails).
    if (!robot_ready)
        throw std::logic_error("RunControlLoop called without a passed readiness gate");

    // Stop policy is compile-time only: the config constants below are read
    // directly and no CLI flag or file may weaken them. Fault-bank changes
    // are printed and logged regardless of the selected stop policy.
    //
    // PARALLEL CONFIG SURFACE (recorded 2026-08-17, simplicity review I3):
    // this adapter reads config::kStopOnFault (warnings + takeover ladder),
    // config::kDisableFollowingErrorStop (warning; the takeover predicate's
    // gate lives in Safety.cpp), config::kOverrunFactor (takeover overrun
    // count + post-loop report), config::kOverrunStopCycles (takeover
    // consecutive-overrun stop) and config::kStaleFeedbackStopCycles (the
    // adapter-owned acknowledgement monitor, both phases) DIRECTLY, while
    // ArmExecutionCore consumes the same values through its
    // construction-time snapshot: ExecutionConfig fields for the first
    // three thresholds (ProductionExecutionConfig() is the pinned identity
    // mapping — tests/test_execution_config.cpp) and a construction-time
    // config::kOverrunFactor read (deferred ExecutionConfig field,
    // ExecutionCore.cpp). INVARIANT: production constructs the core from
    // ProductionExecutionConfig(), so both surfaces are the same compiled
    // constants and neither side has runtime reconfiguration. The
    // following_error_limit_deg parameter carries the same invariant
    // (Runner.h). kStaleFeedbackStopCycles never enters the core at all —
    // the acknowledgement monitor is adapter-owned, so it has one surface.
    if (!config::kStopOnFault)
        std::cout << "WARNING: FAULT-STOP DISABLED (config::kStopOnFault = false) — live "
            "fault bits will NOT stop the loop; following-error, low-level-servoing, "
            "joint-boundary, stale-feedback, non-finite, and overrun guards still can. "
            "Attended use only.\n";
    if (config::kDisableFollowingErrorStop)
        std::cout << "WARNING: FOLLOWING-ERROR STOP DISABLED "
            "(config::kDisableFollowingErrorStop) — a joint whose MEASURED position "
            "stops following its command will NOT stop the loop.\n";
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
    std::uint64_t next_planning_request_id = 1;
    GoalCommand latest_goal;
    bool have_goal = false;

    // The loop's only contact with Vicon: one coherent wait-free slot read
    // per cycle, used by both world measurement and the matching log row.
    // `base_pose_sample` is the reader-owned last copy: between Vicon
    // frames (~5 cycles at 100 Hz vs this loop's 500 Hz) the sequence
    // repeats and only the age advances — zero-order hold, per
    // docs/thesis/world-frame-hold-derivation.md §5. NOTHING may
    // finite-difference across a repeated sequence.
    BasePoseSample base_pose_sample;
    double base_pose_age_s = std::numeric_limits<double>::quiet_NaN();
    // The cycle's single slot read, at cycle start: the SAME sample then
    // feeds world control and the log row, so the evidence columns
    // describe exactly the input the controller acted on.
    const auto read_base_pose =
        [&](std::chrono::steady_clock::time_point now) {
            if (base_pose != nullptr)
                base_pose->ReadLatest(base_pose_sample);
            base_pose_age_s = BasePoseAgeS(
                base_pose_sample,
                std::chrono::duration<double>(now.time_since_epoch()).count());
        };
    const auto fill_vicon = [&](LoopLogSample& s,
                                std::chrono::steady_clock::time_point) {
        s.vicon_sequence = base_pose_sample.sequence;
        s.vicon_frame_number = base_pose_sample.vicon_frame_number;
        s.vicon_latency_s = base_pose_sample.latency_reported_s;
        s.vicon_age_s = base_pose_age_s;
        s.vicon_frame_rate_hz = base_pose_sample.frame_rate_hz;
        for (int i = 0; i < 3; ++i) {
            s.vicon_mount_linear_world_m_s[i] =
                base_pose_sample.mount_linear_world_m_s[i];
            s.vicon_mount_angular_world_rad_s[i] =
                base_pose_sample.mount_angular_world_rad_s[i];
        }
        s.vicon_mount_twist_valid = base_pose_sample.mount_twist_valid;
        for (int seg = 0; seg < kBasePoseSegmentCount; ++seg) {
            for (int i = 0; i < 3; ++i)
                s.vicon_seg_pos_m[seg][i] =
                    base_pose_sample.segments[seg].position_m[i];
            for (int i = 0; i < 4; ++i)
                s.vicon_seg_quat_xyzw[seg][i] =
                    base_pose_sample.segments[seg].quat_xyzw[i];
            s.vicon_seg_valid[seg] = base_pose_sample.segments[seg].valid;
        }
    };
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

        JointVector commanded_deg;
        JointVector commanded_velocity_deg_s{};
        for (int i = 0; i < NUM_JOINTS; ++i)
            commanded_deg[i] = feedback.actuators(i).position();

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
            read_base_pose(t_now);
            fill_vicon(sample, t_now);
            sample.cycle = 0;
            sample.requested_deg = commanded_deg;
            sample.requested_velocity_deg_s = JointVector{};
            sample.lead_limited = {};
            joint_fault_was_latched =
                joint_fault_was_latched || (sample.base_fault_bank & kJointFaultBit) != 0;
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

        // T5: only after a complete healthy hold does the execution core
        // capture the final measured joints — its first measurement AND its
        // integrator seed (q_command = q_measured, the only time command
        // state is seeded from measurement). The Cartesian reference
        // captures the first fresh measured world pose in T6, preventing
        // harmless takeover drift from becoming an initial command jump.
        JointVector seed_position_deg;
        JointVector seed_velocity_deg_s;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            seed_position_deg[i] = feedback.actuators(i).position();
            seed_velocity_deg_s[i] = feedback.actuators(i).velocity();
        }
        core.Seed(seed_position_deg, seed_velocity_deg_s);
        commanded_deg = seed_position_deg;
        feedback = cyclic.Send(commanded_deg);

        // T6: normal control begins only after the mode gate and full hold.
        const auto control_start = clock::now();
        auto t_prev = control_start;
        auto next_cycle = control_start;

        ArmExecutionInput input;

        while (!stop)
        {
            next_cycle += period;

            // dt is measured at cycle start and handed to the core RAW for
            // timing/freshness diagnostics only. The core uses Config.h's
            // fixed kControlDtS for all control mathematics and integration.
            const auto t_now = clock::now();
            input.dt_s =
                std::chrono::duration<double>(t_now - t_prev).count();

            // Measured state from the previous exchange, still in actuator
            // degrees — the degrees -> radians boundary lives inside the
            // core so the conversion stays identical to the frozen loop.
            for (int i = 0; i < NUM_JOINTS; ++i)
            {
                input.measured_position_deg[i] =
                    feedback.actuators(i).position();
                input.measured_velocity_deg_s[i] =
                    feedback.actuators(i).velocity();
            }

            // The cycle's single wait-free slot read, then the RAW world
            // sample: pose + validity + sequence + age + twist, BEFORE any
            // freshness verdict. Classification (fresh/stale, ZOH pose
            // freeze, stale-twist decay, prolonged-stale handling) is the
            // core's job — this adapter only reports what Vicon delivered.
            read_base_pose(t_now);
            {
                const BasePoseSegmentPose& mount =
                    base_pose_sample.segments[kBasePoseMount];
                input.world.mount_valid = mount.valid;
                input.world.mount_position_m = Eigen::Vector3d(
                    mount.position_m[0], mount.position_m[1],
                    mount.position_m[2]);
                for (int i = 0; i < 4; ++i)
                    input.world.mount_quat_xyzw[i] = mount.quat_xyzw[i];
                input.world.sequence = base_pose_sample.sequence;
                input.world.age_s = base_pose_age_s;
                input.world.mount_twist_valid =
                    base_pose_sample.mount_twist_valid;
                input.world.mount_linear_world_m_s = Eigen::Vector3d(
                    base_pose_sample.mount_linear_world_m_s[0],
                    base_pose_sample.mount_linear_world_m_s[1],
                    base_pose_sample.mount_linear_world_m_s[2]);
                input.world.mount_angular_world_rad_s = Eigen::Vector3d(
                    base_pose_sample.mount_angular_world_rad_s[0],
                    base_pose_sample.mount_angular_world_rad_s[1],
                    base_pose_sample.mount_angular_world_rad_s[2]);
            }

            input.goal_preempt = GoalPreemptCommand{};
            std::uint64_t reserved_request_id = 0;
            if (live_goals) {
                GoalCommand incoming;
                if (live_goals->TakeLatest(incoming)) {
                    latest_goal = incoming;
                    have_goal = true;
                    reserved_request_id = next_planning_request_id++;
                    input.goal_preempt.preempt = true;
                    input.goal_preempt.minimum_trajectory_id =
                        reserved_request_id;
                }
            }

            // One core step: measurement -> world classification ->
            // reference -> law -> non-finite hold -> clamp -> integration,
            // in the frozen pre-extraction order (fixed-size computation,
            // no locks or I/O). The returned command frame is transmitted
            // below BEFORE the stop verdict (send-then-resolve), so a stop
            // cycle still sends its held frame first.
            const ArmExecutionResult result = core.Step(input);
            if (result.overrun)
                ++counters.overrun_total;

            // Fixed-size, wait-free typed publication only. GPMP2 remains in
            // the in-process non-real-time planner worker. The replan EDGE is
            // core evidence; this publish is the adapter-to-worker handoff.
            if ((input.goal_preempt.preempt ||
                 result.controller_status.request_replan_edge) &&
                have_goal && planning_requests && result.state.world_fresh &&
                result.state.world_sequence != 0) {
                PlanningRequest request;
                request.request_id = input.goal_preempt.preempt
                                         ? reserved_request_id
                                         : next_planning_request_id++;
                request.arm = planning_arm;
                request.vicon_sequence = base_pose_sample.sequence;
                request.vicon_frame_number =
                    base_pose_sample.vicon_frame_number;
                request.receive_steady_s = base_pose_sample.t_receive_s;
                request.age_s = std::max(0.0, base_pose_age_s);
                request.world_T_mount = Eigen::Isometry3d::Identity();
                request.world_T_mount.translation() =
                    result.state.world_p_mountseg;
                request.world_T_mount.linear() =
                    result.state.world_R_mountseg;
                request.q_rad = result.state.q_rad;
                request.goal = latest_goal;
                planning_requests->Publish(request);
            }

            // The requested velocity for the record is the controller's own
            // raw output, captured by the core BEFORE its non-finite hold
            // and speed clamp (a non-finite value is logged as such — that
            // is the evidence for the kNonFiniteCommand stop).
            JointVector requested_velocity_deg_s{};
            for (int i = 0; i < NUM_JOINTS; ++i)
                requested_velocity_deg_s[i] =
                    result.qdot_raw_rad_s[i] * kRadToDeg;

            commanded_deg = result.commanded_deg;
            commanded_velocity_deg_s = result.commanded_velocity_deg_s;

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
                       commanded_deg, commanded_velocity_deg_s,
                       result.controller_status, result.state,
                       result.measured, result.reference);
            fill_vicon(sample, t_now);
            sample.cycle = cycle;
            sample.requested_deg = result.actuation_status.requested_deg;
            sample.requested_velocity_deg_s = requested_velocity_deg_s;
            sample.lead_limited = result.actuation_status.lead_limited;
            joint_fault_was_latched =
                joint_fault_was_latched || (sample.base_fault_bank & kJointFaultBit) != 0;

            // Update and record acknowledgement freshness before resolving
            // this completed feedback sample. The count is evidence of a
            // stalled cyclic feedback path, not of physical motion.
            freshness_monitor.Update(sample.actuator_command_ack);
            sample.ack_unchanged_cycles = freshness_monitor.unchanged_cycles();
            const std::optional<int> stale_acknowledgement_joint =
                StaleAcknowledgementJoint(sample.ack_unchanged_cycles,
                                          config::kStaleFeedbackStopCycles);

            // The reply's generic health facts, decoded HERE (Kortex fault
            // banks, the servoing-mode field and the acknowledgement
            // freshness monitor stay the adapter's). The core ranks them —
            // with its own following-error and joint-boundary facts —
            // through the frozen precedence, then the decision-12 counters.
            // An ignored fault must still taint the exit and must never
            // mask following error or loss of low-level servoing.
            // Communication exits in the Send catch remain unconditional
            // before this completed feedback sample.
            AdapterHealth health;
            health.live_fault = HasLiveFault(sample);
            health.low_level_state_lost =
                sample.arm_state !=
                k_api::Common::ArmState::ARMSTATE_SERVOING_LOW_LEVEL;
            health.stale_feedback = stale_acknowledgement_joint.has_value();
            JointVector reply_position_deg;
            for (int i = 0; i < NUM_JOINTS; ++i)
                reply_position_deg[i] = feedback.actuators(i).position();
            const ExecutionStopDecision stop_decision =
                core.ResolveStop(reply_position_deg, health);
            faults_observed = faults_observed ||
                              stop_decision.priority.live_fault_observed;
            switch (stop_decision.priority.reason)
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
                joint_limit_warning_joint =
                    *result.actuation_status.joint_limit_warning_joint;
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
            if (stop_decision.priority.reason != StopPriorityReason::kNone)
                break;
            // Decision-12 counter stops, evaluated by the core only when
            // the priority ladder found nothing; N <= 0 disables one.
            // There is deliberately NO saturation stop: a pinned velocity
            // clamp is normal transit toward a far target (removed
            // 2026-07-23).
            if (stop_decision.nonfinite_stop)
            {
                reason = LoopStop::kNonFiniteCommand;
                log.push(sample);
                break;
            }
            if (stop_decision.overrun_stop)
            {
                reason = LoopStop::kOverrun;
                log.push(sample);
                break;
            }
            // UI-only canonical FK: transmission and stop resolution are
            // complete. Selected stop paths above skip this work.
            const CartesianPose commanded_tcp_mount =
                core.CommandedTcpMount();
            for (int i = 0; i < 3; ++i)
                sample.commanded_tcp_mount_m[i] =
                    commanded_tcp_mount.position_m[i];
            const Eigen::Quaterniond commanded_tcp_q(
                commanded_tcp_mount.rotation);
            for (int i = 0; i < 4; ++i)
                sample.commanded_tcp_quat_mount_xyzw[i] =
                    commanded_tcp_q.coeffs()[i];
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
        const auto subcode = static_cast<k_api::SubErrorCodes>(
            ex.getErrorInfo().getError().error_sub_code());
        // WRONG_SERVOING_MODE is a robot-state/ownership failure, not a
        // transport failure. Preserve that distinction in the stop report;
        // the exception is still fatal and follows the same safe teardown.
        reason = subcode == k_api::WRONG_SERVOING_MODE
            ? LoopStop::kLeftLowLevel
            : LoopStop::kCommunication;
        sample.refresh_ok = false;
        log.push(sample);
        std::cout << "Kortex API error: " << ex.what() << " (sub-code "
            << k_api::SubErrorCodes_Name(subcode)
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
    // bad_alloc, ...) must not skip the report — the ServoingGuard destructor
    // retries its restore during unwinding if needed.
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
