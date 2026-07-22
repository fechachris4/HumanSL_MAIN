//
// Loop: the resolved-rate Cartesian controller (position integration).
//

#include "control/Loop.h"

#include "actuation/PositionIntegration.h"
#include "hardware/Cyclic.h"
#include "math/Dls.h"
#include "safety/ServoingGuard.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

#include <KDetailedException.h>

#include "safety/FaultReport.h"

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

LoopResult RunResolvedRateLoop(k_api::Base::BaseClient* base,
                             k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                             Dynamics& dynamics, TargetStore& targets, LoopLog& log,
                             const std::atomic<bool>& stop, std::chrono::microseconds period,
                             double kp, double dls_lambda,
                             const JointVector& qdot_limit_deg_s,
                             double following_error_limit_deg,
                             double arrival_tolerance_m,
                             const std::string& ee_frame_name)
{
    // Precondition: model_.nv == 7 — validated once in main.cpp, before any
    // hardware session is opened.
    if (!dynamics.model_.existFrame(ee_frame_name))
        throw std::runtime_error("no frame named '" + ee_frame_name + "' in the model");
    const pinocchio::FrameIndex ee_frame = dynamics.model_.getFrameId(ee_frame_name);
    KinematicsWorkspace kinematics_workspace(dynamics);

    const double nominal_dt_s = std::chrono::duration<double>(period).count();

    LoopStop reason = LoopStop::kUserStop;
    bool faults_observed = false; // live fault seen at any point (taints exit)
    LoopLogSample sample; // reused every cycle
    long cycle = 0;
    bool joint_fault_was_latched = false;

    CyclicSession cyclic(base_cyclic);
    PositionIntegration actuation;

    // From here until the guard's destructor runs, WE are the controller.
    ServoingGuard servoing_guard(base);

    try
    {
        // Seed AFTER the mode switch (the round trip gives the base time to
        // finish entering LOW_LEVEL_SERVOING). The only standalone read.
        k_api::BaseCyclic::Feedback feedback = cyclic.Seed();

        RobotState seed_state;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            seed_state.q_rad[i] = feedback.actuators(i).position() * kDegToRad;
            seed_state.qdot_rad_s[i] = feedback.actuators(i).velocity() * kDegToRad;
        }
        // The integrator seed, q_command = q_measured — the ONLY time
        // (resolved-rate-position-integration.md, "state distinction").
        actuation.Prepare(seed_state);

        Eigen::VectorXd q_measured_rad(NUM_JOINTS);

        // Seed the desired position with the CURRENT end-effector position,
        // so the controller holds until the operator types a target.
        PositionJacobian ee = position_and_jacobian(
            dynamics, dynamics.convertJointAnglesToConfig(seed_state.q_rad), ee_frame,
            kinematics_workspace);
        targets.Store(ee.position); // anything typed before takeover is discarded

        // Arrival notice state: armed only when the target sequence changes,
        // so the seeded hold target above never prints and each typed
        // target prints at most once.
        std::uint64_t last_target_sequence = targets.Get().sequence;
        bool arrival_reported = true;

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
            commanded_deg[i] = seed_state.q_rad[i] * kRadToDeg;

        // First unchanged holding frame (command == measured); its reply is
        // the loop's first input.
        feedback = cyclic.Send(commanded_deg);

        using clock = std::chrono::steady_clock;
        const auto t_start = clock::now();
        auto t_prev = t_start;
        auto next_cycle = t_start;

        while (!stop)
        {
            next_cycle += period;

            // Measured state from the previous exchange; degrees -> radians
            // at this boundary. FK and Jacobian use the SAME q_measured.
            for (int i = 0; i < NUM_JOINTS; ++i)
                q_measured_rad[i] = feedback.actuators(i).position() * kDegToRad;
            ee = position_and_jacobian(dynamics,
                                       dynamics.convertJointAnglesToConfig(q_measured_rad),
                                       ee_frame, kinematics_workspace);

            // e = p_desired - p(q_measured);  v_d = Kp e;
            // q̇_raw = DLS(Jp, v_d);  clip per joint;  integrate.
            const TargetStore::Snapshot target = targets.Get();
            const Eigen::Vector3d position_error_m = target.p_desired - ee.position;
            const Eigen::Vector3d v_desired = kp * position_error_m;

            // Arrival notice (edge-triggered, see Loop.h). FK already gives
            // p_current every cycle; "reached" is just its distance to the
            // target crossing under the tolerance.
            if (target.sequence != last_target_sequence)
            {
                last_target_sequence = target.sequence;
                arrival_reported = false;
            }
            if (!arrival_reported && position_error_m.norm() < arrival_tolerance_m)
            {
                arrival_reported = true;
                std::cout << "target reached: " << target.p_desired[0] << " "
                          << target.p_desired[1] << " " << target.p_desired[2]
                          << " m, within " << position_error_m.norm() * 1000.0
                          << " mm — holding\n";
            }
            const Eigen::Matrix<double, 7, 1> qdot_raw_rad_s =
                DampedLeastSquares(ee.jacobian_p, v_desired, dls_lambda);

            // dt = measured elapsed cycle time (nominal on the first cycle),
            // clamped so a stall cannot integrate one large jump.
            const auto t_now = clock::now();
            const double dt_s =
                cycle == 0
                    ? nominal_dt_s
                    : ClampedCycleDt(
                        std::chrono::duration<double>(t_now - t_prev).count(),
                        nominal_dt_s);

            // Per-joint clamp — the program's single speed limit — then the
            // actuation integrates and produces this cycle's setpoints.
            Eigen::Matrix<double, 7, 1> qdot_clamped_rad_s;
            for (int i = 0; i < NUM_JOINTS; ++i)
                qdot_clamped_rad_s[i] =
                    std::clamp(qdot_raw_rad_s[i] * kRadToDeg, -qdot_limit_deg_s[i],
                               qdot_limit_deg_s[i]) *
                    kDegToRad;
            actuation.Apply(qdot_clamped_rad_s, dt_s, commanded_deg,
                            commanded_velocity_deg_s);

            // The one exchange: send this cycle's position command, receive
            // the feedback the next iteration will use.
            feedback = cyclic.Send(commanded_deg);
            ++cycle;

            sample.t_s = std::chrono::duration<double>(t_now - t_start).count();
            sample.dt_s = std::chrono::duration<double>(t_now - t_prev).count();
            t_prev = t_now;
            FillSample(sample, feedback, commanded_deg, commanded_velocity_deg_s,
                       target.p_desired, ee.position);
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

            // TEMPORARY (Christian, 2026-07-20): fault-triggered exit
            // disabled for an experiment — actuator/base fault bits no
            // longer stop the loop; each bank change prints above and every
            // cycle's banks are logged. The arm-left-low-level exit is kept
            // (the stream is dead at that point; Refresh throws anyway),
            // and so is the following-error exit — it is the backstop
            // that bounds how far the integrator can run from a stopped
            // arm while faults are ignored (run log 2026-07-22).
            // RESTORE the ClassifyStop break before any unattended use.
            if (ClassifyStop(sample, following_error_limit_deg, reason))
            {
                if (reason != LoopStop::kRobotFault)
                {
                    log.push(sample);
                    break;
                }
                faults_observed = true;       // ignored here, but taints the exit code
                reason = LoopStop::kUserStop; // not a stop reason — the loop continues
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
    // bad_alloc, ...) must not skip the report and the servoing restore
    // below — before these existed it also hit std::thread's destructor in
    // main and aborted the whole program with the arm left in low-level.
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

    // Shutdown, in the approved teardown order: actuation restore first
    // (no-op for PositionIntegration — the position servo holds the last
    // setpoint), then the report; finally the ServoingGuard destructor
    // restores SINGLE_LEVEL by unwinding, so it can neither be skipped nor
    // overwrite the recorded stop reason.
    actuation.Restore();
    PrintStopReport(reason, sample, cycle, following_error_limit_deg);
    if (joint_fault_was_latched)
        std::cout << "note: base JOINT_FAULT was latched during the run (stale summary "
            "diagnostic unless a joint fault is shown above; not cleared here)\n";

    return {reason, faults_observed};
}
