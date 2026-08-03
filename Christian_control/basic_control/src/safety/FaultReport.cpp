//
// FaultReport: fault-bank decoding and the post-loop reports.
//

#include "safety/FaultReport.h"

#include <cmath>
#include <iostream>
#include <tuple>

#include <ActuatorConfig.pb.h> // SafetyIdentifierBankA_Name (fault decoding)

namespace
{
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;

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
