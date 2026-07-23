//
// Supervisor: stop classification and the pre-takeover readiness gate.
//

#include "safety/Supervisor.h"

#include "safety/FaultReport.h"

#include <cmath>
#include <cstdlib>

namespace
{
    constexpr int NUM_JOINTS = 7;
} // namespace

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

std::optional<LoopStop> ClassifyCounters(const CycleCounters& counters,
                                         const StopPolicy& policy)
{
    if (policy.nonfinite_stop_cycles > 0 &&
        counters.nonfinite >= policy.nonfinite_stop_cycles)
        return LoopStop::kNonFiniteCommand;
    if (policy.saturation_stop_cycles > 0 &&
        counters.saturated >= policy.saturation_stop_cycles)
        return LoopStop::kSaturation;
    if (policy.overrun_stop_cycles > 0 &&
        counters.overrun >= policy.overrun_stop_cycles)
        return LoopStop::kOverrun;
    return std::nullopt;
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
