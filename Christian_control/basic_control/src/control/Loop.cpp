//
// Loop: the resolved-rate Cartesian controller (position integration).
//

#include "control/Loop.h"

#include "hardware/Measure.h" // read_feedback — the single standalone reader
#include "math/Dls.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

#include <KDetailedException.h>
#include <ActuatorConfig.pb.h> // SafetyIdentifierBankA_Name (fault decoding)

namespace
{
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    constexpr std::uint32_t kJointFaultBit = k_api::Base::SafetyIdentifier::JOINT_FAULT;

    // A safety bank is a bitmask; each set bit is one named safety event.
    std::string DecodeBank(std::uint32_t bank, const std::string& (*name_of)(int))
    {
        if (bank == 0)
            return "0";
        std::string out = std::to_string(bank) + " (";
        for (std::uint32_t bit = 1; bit != 0; bit <<= 1)
            if (bank & bit)
            {
                const std::string& name = name_of(static_cast<int>(bit));
                if (out.back() != '(')
                    out += " | ";
                out += name.empty() ? "bit " + std::to_string(bit) : name;
            }
        return out + ")";
    }

    std::string DecodeBaseBank(std::uint32_t bank)
    {
        return DecodeBank(bank, [](int bit) -> const std::string&
        {
            return k_api::Base::SafetyIdentifier_Name(
                static_cast<k_api::Base::SafetyIdentifier>(bit));
        });
    }

    std::string DecodeActuatorBank(std::uint32_t bank)
    {
        return DecodeBank(bank, [](int bit) -> const std::string&
        {
            return k_api::ActuatorConfig::SafetyIdentifierBankA_Name(
                static_cast<k_api::ActuatorConfig::SafetyIdentifierBankA>(bit));
        });
    }

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

    // Live-fault policy (no printing — loop-safe). The base's latched
    // JOINT_FAULT summary bit alone does NOT stop the loop.
    bool ClassifyStop(const LoopLogSample& s, double following_error_limit_deg,
                      LoopStop& reason)
    {
        // Checked FIRST so the guard cannot be masked by the experiment
        // policy that ignores fault bits (ClassifyStop returns on the
        // first match). measured_deg sits within ±180° of the command
        // (FillSample) and the gap grows by well under a degree per
        // cycle, so at a small limit the comparison is unambiguous.
        for (int i = 0; i < NUM_JOINTS; ++i)
            if (std::abs(s.measured_deg[i] - s.commanded_deg[i]) >
                following_error_limit_deg)
            {
                reason = LoopStop::kFollowingError;
                return true;
            }
        for (int i = 0; i < NUM_JOINTS; ++i)
            if (s.fault_bank[i] != 0)
            {
                reason = LoopStop::kRobotFault;
                return true;
            }
        if ((s.base_fault_bank & ~kJointFaultBit) != 0)
        {
            reason = LoopStop::kRobotFault;
            return true;
        }
        if (s.arm_state != k_api::Common::ArmState::ARMSTATE_SERVOING_LOW_LEVEL)
        {
            reason = LoopStop::kLeftLowLevel;
            return true;
        }
        return false;
    }

    void PrintStopReport(LoopStop reason, const LoopLogSample& s, long cycle,
                         double following_error_limit_deg)
    {
        switch (reason)
        {
        case LoopStop::kUserStop:
            std::cout << "loop stopped by user (Ctrl+C)\n";
            break;
        case LoopStop::kRobotFault:
            std::cout << "loop stopped: robot fault at t=" << s.t_s << " s (cycle " << cycle
                << ")\n";
            break;
        case LoopStop::kFollowingError:
            {
                int worst = 0;
                double worst_gap = 0.0;
                for (int i = 0; i < NUM_JOINTS; ++i)
                {
                    const double gap = std::abs(s.measured_deg[i] - s.commanded_deg[i]);
                    if (gap > worst_gap)
                    {
                        worst_gap = gap;
                        worst = i;
                    }
                }
                std::cout << "loop stopped: following error at t=" << s.t_s << " s (cycle "
                    << cycle << "): joint " << (worst + 1) << " is " << worst_gap
                    << " deg from its command (limit " << following_error_limit_deg
                    << ") — the arm stopped following the integrated command\n";
                break;
            }
        case LoopStop::kLeftLowLevel:
            std::cout << "loop stopped: arm left low-level servoing at t=" << s.t_s
                << " s (cycle " << cycle << "): state " << s.arm_state << " ("
                << k_api::Common::ArmState_Name(
                    static_cast<k_api::Common::ArmState>(s.arm_state))
                << ")\n";
            break;
        case LoopStop::kCommunication:
            std::cout << "loop stopped: communication failure at t=" << s.t_s << " s (cycle "
                << cycle << ")\n";
            break;
        case LoopStop::kInternalError:
            std::cout << "loop stopped: internal error at t=" << s.t_s << " s (cycle "
                << cycle << ")\n";
            break;
        }
        std::cout << "  desired p:  " << s.p_desired_m[0] << " " << s.p_desired_m[1] << " "
            << s.p_desired_m[2] << " m,  current p: " << s.p_current_m[0] << " "
            << s.p_current_m[1] << " " << s.p_current_m[2] << " m\n";
        if (reason == LoopStop::kUserStop)
            return;
        std::cout << "  base:    fault " << DecodeBaseBank(s.base_fault_bank) << "\n";
        for (int i = 0; i < NUM_JOINTS; ++i)
            std::cout << "  joint " << (i + 1) << ": fault "
                << DecodeActuatorBank(s.fault_bank[i]) << ", commanded "
                << s.commanded_deg[i] << " deg (q̇ " << s.commanded_velocity_deg_s[i]
                << " deg/s), measured " << s.measured_deg[i] << " deg (raw "
                << s.measured_raw_deg[i] << ")\n";
    }

    // Faults are deliberately non-stopping during the current experiment;
    // instead EVERY fault-bank change prints immediately, decoded, once per
    // change (not per cycle — a persisting fault stays silent after its
    // edge). Bounded: after kMaxFaultChangePrints events the loop stops
    // printing (the CSV still has every cycle's banks).
    constexpr int kMaxFaultChangePrints = 20;

    void PrintFaultChange(const LoopLogSample& s, long cycle,
                          const std::array<std::uint32_t, 7>& prev_joint_banks,
                          std::uint32_t prev_base_bank)
    {
        std::cout << "fault change at t=" << s.t_s << " s (cycle " << cycle << "):\n";
        if (s.base_fault_bank != prev_base_bank)
            std::cout << "  base:    " << DecodeBaseBank(prev_base_bank) << " -> "
                      << DecodeBaseBank(s.base_fault_bank) << "\n";
        for (int i = 0; i < NUM_JOINTS; ++i)
            if (s.fault_bank[i] != prev_joint_banks[i])
                std::cout << "  joint " << (i + 1) << ": "
                          << DecodeActuatorBank(prev_joint_banks[i]) << " -> "
                          << DecodeActuatorBank(s.fault_bank[i]) << "\n";
    }
} // namespace

bool RobotReadyForTakeover(const k_api::BaseCyclic::Feedback& feedback, std::ostream& out)
{
    const std::uint32_t base_bank = feedback.base().fault_bank_a();
    bool actuator_fault = false;

    out << "arm state: "
        << k_api::Common::ArmState_Name(
            static_cast<k_api::Common::ArmState>(feedback.base().active_state()))
        << ", base fault bank " << DecodeBaseBank(base_bank) << "\n";
    for (int i = 0; i < feedback.actuators_size(); ++i)
    {
        const std::uint32_t bank = feedback.actuators(i).fault_bank_a();
        if (bank != 0)
        {
            actuator_fault = true;
            out << "  joint " << (i + 1) << ": fault " << DecodeActuatorBank(bank) << "\n";
        }
    }

    const std::uint32_t base_fatal = base_bank & ~kJointFaultBit;
    if (!actuator_fault && (base_bank & kJointFaultBit) != 0)
        out << "note: base JOINT_FAULT is latched but every actuator bank is clear — "
            "stale summary diagnostic, continuing (not cleared by this program)\n";
    if (actuator_fault || base_fatal != 0)
    {
        out << "robot NOT ready: live fault present — not taking over "
            "(clear deliberately via the Kinova web dashboard)\n";
        return false;
    }
    return true;
}

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
    if (dynamics.model_.nv != NUM_JOINTS)
        throw std::runtime_error("model has " + std::to_string(dynamics.model_.nv) +
            " velocity variables, expected 7");
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

    // From here until the restore below, WE are the controller.
    enter_low_level_servoing(base);

    try
    {
        // Seed AFTER the mode switch (the round trip gives the base time to
        // finish entering LOW_LEVEL_SERVOING). The only standalone read.
        k_api::BaseCyclic::Feedback feedback = read_feedback(base_cyclic);

        // ONLY place q_command is set from measurement: startup. From here
        // on it is the persistent integrator state — resetting it from
        // feedback each cycle would break the continuous integration.
        Eigen::Matrix<double, 7, 1> q_command_rad;
        Eigen::VectorXd q_measured_rad(NUM_JOINTS);
        for (int i = 0; i < NUM_JOINTS; ++i)
            q_command_rad[i] = feedback.actuators(i).position() * kDegToRad;

        // Seed the desired position with the CURRENT end-effector position,
        // so the controller holds until the operator types a target.
        PositionJacobian ee = position_and_jacobian(
            dynamics, dynamics.convertJointAnglesToConfig(q_command_rad), ee_frame,
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

        k_api::BaseCyclic::Command command;
        JointVector commanded_deg;
        JointVector commanded_velocity_deg_s{};
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            command.add_actuators();
            commanded_deg[i] = q_command_rad[i] * kRadToDeg;
        }

        // First unchanged holding frame (command == measured); its reply is
        // the loop's first input.
        feedback = send_positions(base_cyclic, command, commanded_deg);

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

            for (int i = 0; i < NUM_JOINTS; ++i)
            {
                const double qdot_clipped_rad_s =
                    std::clamp(qdot_raw_rad_s[i] * kRadToDeg, -qdot_limit_deg_s[i],
                               qdot_limit_deg_s[i]) *
                    kDegToRad;
                q_command_rad[i] += qdot_clipped_rad_s * dt_s;
                commanded_velocity_deg_s[i] = qdot_clipped_rad_s * kRadToDeg;
                commanded_deg[i] = q_command_rad[i] * kRadToDeg;
            }

            // The one exchange: send this cycle's position command, receive
            // the feedback the next iteration will use.
            feedback = send_positions(base_cyclic, command, commanded_deg);
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

    // Shutdown: q_command simply stops updating — in POSITION mode the arm
    // holds the last commanded setpoint. Report, then the single guarded
    // restore (cannot overwrite the recorded stop reason).
    PrintStopReport(reason, sample, cycle, following_error_limit_deg);
    if (joint_fault_was_latched)
        std::cout << "note: base JOINT_FAULT was latched during the run (stale summary "
            "diagnostic unless a joint fault is shown above; not cleared here)\n";

    restore_single_level_servoing(base);
    return {reason, faults_observed};
}
