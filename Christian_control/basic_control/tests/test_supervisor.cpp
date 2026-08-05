//
// Tests for the Supervisor's stop classification: check ordering (the
// following-error guard first, so no fault policy can mask it), live-fault
// priorities, and the JOINT_FAULT-summary tolerance. Links the Kortex
// static library (Linux hardware machine only — bundled libs are ELF).
//

#include <iostream>
#include <sstream>
#include <string>

#include "Safety.h"

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

    // The startup gate owns every bounded joint's power-cycle-volatile,
    // firmware-enforced position thresholds. Continuous joints remain
    // untouched.
    Check(config::kJointLimitWarnDeg == JointVector{0, 130, 0, 145, 0, 118, 0},
          "joint-limit warning contract configures bounded joints 2, 4, and 6");
    Check(config::kJointLimitErrorDeg == JointVector{0, 140, 0, 150, 0, 123, 0},
          "joint-limit error contract configures bounded joints 2, 4, and 6");
    Check(StopReasonName(LoopStop::kJointLimitWarning) == "joint_limit_warning",
          "joint-limit warning has a distinct CSV exit token");
    Check(StopReasonName(LoopStop::kStaleFeedback) == "stale_feedback",
          "stale feedback has a distinct CSV exit token");

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

    // Decision-12 consecutive-cycle stops now live inline in the Runner
    // (the StopPolicy/ClassifyCounters indirection was trimmed 2026-08-03):
    // it compares CycleCounters directly against the Config.h thresholds.
    // Pin the configuration those inline checks rely on — a disabled (<= 0)
    // or absurd threshold would silently defang the stops.
    static_assert(config::kNonFiniteStopCycles > 0,
                  "non-finite stop must be enabled");
    static_assert(config::kNonFiniteStopCycles <= 10,
                  "non-finite output must stop within a few cycles");
    static_assert(config::kOverrunStopCycles > 0, "overrun stop must be enabled");
    static_assert(config::kOverrunFactor > 1.0,
                  "overrun factor must sit above nominal dt");
    static_assert(config::kStaleFeedbackStopCycles == 25,
                  "stale feedback must stop after 25 completed replies");
    CycleCounters c;
    Check(c.nonfinite == 0 && c.overrun == 0 && c.overrun_total == 0,
          "fresh counters start at zero");

    // FeedbackFreshnessMonitor counts consecutive repeated acknowledgements
    // per joint. The Runner stops when a count reaches the configured 25;
    // it does not infer this from unchanged position or physical motion.
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

    {
        // The periodic status line: every number the 2026-08-05 stall
        // diagnosis needed, in the units the operator reasons in. The
        // formatter is pure so this test pins the exact rendering.
        StatusLineData d;
        d.t_s = 12.04;
        d.position_error_m = 0.1438;   // -> mm
        d.rot_error_rad = 0.0452;      // -> mrad
        d.task_speed_deg_s = 610.23;
        d.null_speed_deg_s = 2497.04;
        d.null_leak_m_s = 0.4153;
        d.sigma_min = 0.0421;
        d.saturated_cycles = 3;
        d.window_cycles = 500;
        d.bounded_q_deg = {54.71, 116.24, 45.08};
        d.bounded_limit_deg = {126.9, 145.0, 118.0};
        const std::string line = FormatStatusLine(d);
        Check(line.find("t=12.0s") != std::string::npos,
              "status line reports loop time in seconds");
        Check(line.find("err=143.8mm") != std::string::npos,
              "status line reports the position error in mm");
        Check(line.find("rot=45.2mrad") != std::string::npos,
              "status line reports the rotation error in mrad");
        Check(line.find("task=610.2") != std::string::npos,
              "status line reports the task-velocity norm");
        Check(line.find("null=2497.0") != std::string::npos,
              "status line reports the null-velocity norm");
        Check(line.find("leak=0.415m/s") != std::string::npos,
              "status line reports the null-space leak speed");
        Check(line.find("sig=0.042") != std::string::npos,
              "status line reports sigma_min");
        Check(line.find("sat=3/500") != std::string::npos,
              "status line reports clip saturation over the window");
        Check(line.find("j2=54.7/126.9") != std::string::npos,
              "status line reports joint 2 against its software limit");
        Check(line.find("j4=116.2/145.0") != std::string::npos,
              "status line reports joint 4 against its software limit");
        Check(line.find("j6=45.1/118.0") != std::string::npos,
              "status line reports joint 6 against its software limit");
        Check(line.find('\n') == std::string::npos,
              "status line is a single line (caller adds the newline)");
    }

    if (failures == 0) {
        std::cout << "all supervisor tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
