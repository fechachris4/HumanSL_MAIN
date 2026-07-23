//
// Tests for the Supervisor's stop classification: check ordering (the
// following-error guard first, so no fault policy can mask it), live-fault
// priorities, and the JOINT_FAULT-summary tolerance. Links the Kortex
// static library (Linux hardware machine only — bundled libs are ELF).
//

#include <iostream>
#include <string>

#include "safety/Supervisor.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    LoopLogSample CleanSample()
    {
        LoopLogSample s;
        s.arm_state = k_api::Common::ArmState::ARMSTATE_SERVOING_LOW_LEVEL;
        for (int i = 0; i < 7; ++i)
        {
            s.commanded_deg[i] = 100.0 + i;
            s.measured_deg[i] = 100.0 + i; // zero tracking error
            s.fault_bank[i] = 0;
        }
        s.base_fault_bank = 0;
        return s;
    }
} // namespace

int main()
{
    const double limit = 3.0;
    LoopStop reason = LoopStop::kUserStop;

    Check(!ClassifyStop(CleanSample(), limit, reason), "clean sample does not stop");

    // Following error alone stops.
    LoopLogSample s = CleanSample();
    s.measured_deg[2] = s.commanded_deg[2] + 3.5;
    Check(ClassifyStop(s, limit, reason) && reason == LoopStop::kFollowingError,
          "following error beyond the limit stops the loop");

    // ORDERING: following error is classified FIRST even when fault bits are
    // set in the same sample — a fault-ignoring policy cannot mask the guard.
    s.fault_bank[0] = 4;
    s.base_fault_bank = 1;
    Check(ClassifyStop(s, limit, reason) && reason == LoopStop::kFollowingError,
          "following error outranks simultaneous fault bits (guard unmaskable)");

    // Actuator fault bit stops as kRobotFault.
    s = CleanSample();
    s.fault_bank[5] = 2;
    Check(ClassifyStop(s, limit, reason) && reason == LoopStop::kRobotFault,
          "actuator fault bit classifies as robot fault");

    // Base JOINT_FAULT summary alone does NOT stop (stale aggregate).
    s = CleanSample();
    s.base_fault_bank = kJointFaultBit;
    Check(!ClassifyStop(s, limit, reason),
          "latched base JOINT_FAULT summary alone does not stop");

    // Any other base fault bit stops.
    s.base_fault_bank = kJointFaultBit | 1;
    Check(ClassifyStop(s, limit, reason) && reason == LoopStop::kRobotFault,
          "non-summary base fault bit classifies as robot fault");

    // Leaving low-level servoing stops.
    s = CleanSample();
    s.arm_state = k_api::Common::ArmState::ARMSTATE_SERVOING_READY;
    Check(ClassifyStop(s, limit, reason) && reason == LoopStop::kLeftLowLevel,
          "leaving low-level servoing stops the loop");

    // Tracking error just inside the limit does not stop.
    s = CleanSample();
    s.measured_deg[0] = s.commanded_deg[0] + 2.9;
    Check(!ClassifyStop(s, limit, reason), "tracking lag inside the limit continues");

    // Decision-12 consecutive-cycle counters.
    StopPolicy policy; // defaults: 3 / 50 / 10
    CycleCounters c;
    Check(!ClassifyCounters(c, policy).has_value(), "zeroed counters do not stop");
    c.nonfinite = policy.nonfinite_stop_cycles - 1;
    Check(!ClassifyCounters(c, policy).has_value(), "below the non-finite limit continues");
    c.nonfinite = policy.nonfinite_stop_cycles;
    Check(ClassifyCounters(c, policy) == LoopStop::kNonFiniteCommand,
          "non-finite output at the limit stops");
    c = CycleCounters{};
    c.saturated = policy.saturation_stop_cycles;
    Check(ClassifyCounters(c, policy) == LoopStop::kSaturation,
          "sustained clamp saturation stops");
    c = CycleCounters{};
    c.overrun = policy.overrun_stop_cycles;
    Check(ClassifyCounters(c, policy) == LoopStop::kOverrun,
          "consecutive overruns stop");
    policy.overrun_stop_cycles = 0; // disabled
    Check(!ClassifyCounters(c, policy).has_value(), "N <= 0 disables a counter");

    if (failures == 0) {
        std::cout << "all supervisor tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
