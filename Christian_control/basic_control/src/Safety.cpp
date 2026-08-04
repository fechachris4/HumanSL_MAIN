//
// Safety — implementations for Safety.h.
//

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <tuple>
#include <ActuatorConfig.pb.h>
#include <KDetailedException.h>

#include "Safety.h"

// ---------------------------------------------------------------
// Supervisor — stop classification and the readiness gate
// ---------------------------------------------------------------

//
// Supervisor: stop classification and the pre-takeover readiness gate.
//




namespace
{
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;
} // namespace

void FeedbackFreshnessMonitor::Update(
    const std::array<std::uint32_t, 7>& command_ack)
{
    // The first frame has nothing to compare against: seed and report zero.
    if (!initialized_)
    {
        previous_ = command_ack;
        initialized_ = true;
        unchanged_cycles_.fill(0);
        return;
    }
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        if (command_ack[i] == previous_[i])
            ++unchanged_cycles_[i];
        else
            unchanged_cycles_[i] = 0;
        previous_[i] = command_ack[i];
    }
}

bool ClassifyStop(const LoopLogSample& s, double following_error_limit_deg,
                  LoopStop& reason)
{
    // Checked FIRST so the guard cannot be masked by the experiment
    // policy that ignores fault bits (ClassifyStop returns on the
    // first match). measured_deg sits within ±180° of the command
    // (FillSample) and the gap grows by well under a degree per
    // cycle, so at a small limit the comparison is unambiguous.
    //
    // config::kDisableFollowingErrorStop removes this stop entirely. With
    // kStopOnFault also false, the only automatic stop left below is
    // "the arm left low-level servoing".
    if (!config::kDisableFollowingErrorStop)
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

    // The arm's own state outranks any bank heuristic: IN_FAULT means the
    // base considers itself faulted RIGHT NOW, whatever the decodable banks
    // show — the stale-summary tolerance below must never talk past it
    // (2026-07-31: a run proceeded to takeover while the arm printed
    // ARMSTATE_IN_FAULT).
    const bool in_fault_state =
        static_cast<k_api::Common::ArmState>(feedback.base().active_state()) ==
        k_api::Common::ArmState::ARMSTATE_IN_FAULT;

    const std::uint32_t base_fatal = base_bank & ~kJointFaultBit;
    if (!actuator_fault && !in_fault_state && (base_bank & kJointFaultBit) != 0)
        out << "note: base JOINT_FAULT is latched but every actuator bank is clear — "
            "stale summary diagnostic, continuing (not cleared by this program)\n";
    if (actuator_fault || base_fatal != 0 || in_fault_state)
    {
        if (in_fault_state)
            out << "robot NOT ready: the arm reports ARMSTATE_IN_FAULT — not "
                "taking over, even with clear fault banks (clear deliberately "
                "via the Kinova web dashboard)\n";
        else
            out << "robot NOT ready: live fault present — not taking over "
                "(clear deliberately via the Kinova web dashboard)\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------
// FaultReport — fault-bank decoding and the post-loop reports
// ---------------------------------------------------------------

namespace
{
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
} // namespace

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

std::string StopReasonName(LoopStop reason)
{
    switch (reason)
    {
    case LoopStop::kUserStop:       return "user_stop";
    case LoopStop::kRobotFault:     return "robot_fault";
    case LoopStop::kFollowingError: return "following_error";
    case LoopStop::kLeftLowLevel:   return "left_low_level_servoing";
    case LoopStop::kCommunication:  return "communication";
    case LoopStop::kInternalError:  return "internal_error";
    case LoopStop::kNonFiniteCommand: return "nonfinite_command";
    case LoopStop::kOverrun:        return "overrun";
    }
    return "unknown";
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
    case LoopStop::kNonFiniteCommand:
        std::cout << "loop stopped: non-finite controller output (consecutive-cycle "
            "limit, output held at zero meanwhile) at t=" << s.t_s << " s (cycle "
            << cycle << ")\n";
        break;
    case LoopStop::kOverrun:
        std::cout << "loop stopped: cycle overruns hit the consecutive-cycle limit at t="
            << s.t_s << " s (cycle " << cycle << ")\n";
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

// ---------------------------------------------------------------
// ServoingGuard — RAII ownership of the servoing mode
// ---------------------------------------------------------------

//
// ServoingGuard: RAII ownership of the base's servoing mode (see header).
//




ServoingGuard::ServoingGuard(k_api::Base::BaseClient* base)
    : base_(base)
{
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base_->SetServoingMode(servoing_mode);
}

ServoingGuard::~ServoingGuard()
{
    if (restored_)
        return;
    if (!Restore(std::cerr))
        std::cerr << "WARNING: could not restore SINGLE_LEVEL servoing — the arm may "
            "still be in low-level mode; check it before running anything else\n";
}

bool ServoingGuard::Restore(std::ostream& out) noexcept
{
    try
    {
        auto servoing_mode = k_api::Base::ServoingModeInformation();
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base_->SetServoingMode(servoing_mode);
        // TrajectoryExecution waits after this transition. Keep the session
        // and client alive while the base settles before any teardown.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        restored_ = true;
        return true;
    }
    catch (k_api::KDetailedException& ex)
    {
        out << "SINGLE_LEVEL restore Kortex error: " << ex.what()
            << " (sub-code "
            << k_api::SubErrorCodes_Name(static_cast<k_api::SubErrorCodes>(
                   ex.getErrorInfo().getError().error_sub_code()))
            << ")\n";
    }
    catch (std::exception& ex)
    {
        out << "SINGLE_LEVEL restore error: " << ex.what() << "\n";
    }
    catch (...)
    {
        out << "SINGLE_LEVEL restore error: unknown exception type\n";
    }
    return false;
}
