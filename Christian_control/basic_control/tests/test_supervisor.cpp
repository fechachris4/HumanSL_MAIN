//
// Tests for the Supervisor's stop classification: check ordering (the
// following-error guard first, so no fault policy can mask it), live-fault
// priorities, and the JOINT_FAULT-summary tolerance. Links the Kortex
// static library (Linux hardware machine only — bundled libs are ELF).
//

#include <iostream>
#include <sstream>
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

    // A healthy pre-takeover feedback frame: single-level servoing, every
    // bank clear.
    k_api::BaseCyclic::Feedback CleanFeedback()
    {
        k_api::BaseCyclic::Feedback fb;
        fb.mutable_base()->set_active_state(
            k_api::Common::ArmState::ARMSTATE_SERVOING_READY);
        fb.mutable_base()->set_fault_bank_a(0);
        for (int i = 0; i < 7; ++i)
            fb.add_actuators()->set_fault_bank_a(0);
        return fb;
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

    // Decision-12 consecutive-cycle counters (no saturation counter — the
    // saturation stop was removed 2026-07-23; a pinned clamp is normal
    // transit toward a far target).
    StopPolicy policy; // defaults: 3 / 10
    CycleCounters c;
    Check(!ClassifyCounters(c, policy).has_value(), "zeroed counters do not stop");
    c.nonfinite = policy.nonfinite_stop_cycles - 1;
    Check(!ClassifyCounters(c, policy).has_value(), "below the non-finite limit continues");
    c.nonfinite = policy.nonfinite_stop_cycles;
    Check(ClassifyCounters(c, policy) == LoopStop::kNonFiniteCommand,
          "non-finite output at the limit stops");
    c = CycleCounters{};
    c.overrun = policy.overrun_stop_cycles;
    Check(ClassifyCounters(c, policy) == LoopStop::kOverrun,
          "consecutive overruns stop");
    policy.overrun_stop_cycles = 0; // disabled
    Check(!ClassifyCounters(c, policy).has_value(), "N <= 0 disables a counter");

    // FeedbackFreshnessMonitor is TELEMETRY ONLY (2026-08-03): it counts
    // consecutive repeated acknowledgement IDs per joint and never stops a
    // run. The old short-window "unchanged feedback" and "no measured
    // motion" stops were removed; the counters below replace their evidence.
    FeedbackFreshnessMonitor freshness;
    std::array<std::uint32_t, 7> ack{};
    freshness.Update(ack);
    for (int i = 0; i < 7; ++i)
        Check(freshness.unchanged_cycles()[i] == 0,
              "the seeding frame counts nothing as unchanged");

    for (auto& value : ack)
        ++value;
    freshness.Update(ack);
    for (int i = 0; i < 7; ++i)
        Check(freshness.unchanged_cycles()[i] == 0,
              "advancing acknowledgements keep every counter at zero");

    // Joint 3 (index 2) freezes while the others keep advancing.
    for (int cycle = 1; cycle <= 4; ++cycle)
    {
        for (int i = 0; i < 7; ++i)
            if (i != 2)
                ++ack[i];
        freshness.Update(ack);
        Check(freshness.unchanged_cycles()[2] == cycle,
              "a frozen acknowledgement accumulates one count per cycle");
        Check(freshness.unchanged_cycles()[0] == 0,
              "a moving joint's counter stays at zero alongside a frozen one");
    }

    // The counter is a live measure, not a latch: it clears the moment the
    // acknowledgement advances again, so the run record shows the gap and
    // its end rather than a permanent flag.
    ++ack[2];
    freshness.Update(ack);
    Check(freshness.unchanged_cycles()[2] == 0,
          "the counter clears when the acknowledgement advances again");

    freshness.Reset();
    for (int i = 0; i < 7; ++i)
        Check(freshness.unchanged_cycles()[i] == 0,
              "Reset clears every counter");

    // RobotReadyForTakeover: the pre-takeover gate.
    {
        std::ostringstream sink;
        Check(RobotReadyForTakeover(CleanFeedback(), sink),
              "clean feedback passes the readiness gate");

        // Latched JOINT_FAULT summary alone: noted, still ready.
        k_api::BaseCyclic::Feedback fb = CleanFeedback();
        fb.mutable_base()->set_fault_bank_a(kJointFaultBit);
        std::ostringstream note;
        Check(RobotReadyForTakeover(fb, note),
              "latched JOINT_FAULT with clear actuator banks is ready");
        Check(note.str().find("stale summary") != std::string::npos,
              "the latched-summary tolerance is stated, not silent");

        // Any actuator fault bit refuses.
        fb = CleanFeedback();
        fb.mutable_actuators(3)->set_fault_bank_a(2);
        Check(!RobotReadyForTakeover(fb, sink),
              "an actuator fault bit refuses takeover");

        // Any non-summary base bit refuses.
        fb = CleanFeedback();
        fb.mutable_base()->set_fault_bank_a(kJointFaultBit | 1);
        Check(!RobotReadyForTakeover(fb, sink),
              "a non-summary base fault bit refuses takeover");

        // ARMSTATE_IN_FAULT refuses even with every bank clear — the arm's
        // own state outranks the latched-summary heuristic, and the stale
        // note must not print against it (2026-07-31 finding).
        fb = CleanFeedback();
        fb.mutable_base()->set_active_state(
            k_api::Common::ArmState::ARMSTATE_IN_FAULT);
        fb.mutable_base()->set_fault_bank_a(kJointFaultBit);
        std::ostringstream in_fault_out;
        Check(!RobotReadyForTakeover(fb, in_fault_out),
              "ARMSTATE_IN_FAULT refuses takeover despite clear banks");
        Check(in_fault_out.str().find("stale summary") == std::string::npos,
              "no stale-summary claim while the arm reports IN_FAULT");
        Check(in_fault_out.str().find("ARMSTATE_IN_FAULT") != std::string::npos,
              "the refusal names the arm state as the reason");
    }

    if (failures == 0) {
        std::cout << "all supervisor tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
