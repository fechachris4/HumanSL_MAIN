//
// clear_faults: read the arm's fault state, invoke the SDK's supported
// whole-arm recovery (Base::ClearFaults — the same operation as the Kinova
// web dashboard's "Clear faults" button), and read back the result.
//
// NO MOTION is commanded and no servoing mode is changed: the only writes
// are the session login and the ClearFaults RPC itself. Exit 0 only if,
// after clearing, the arm reports SERVOING_READY with every fault bank zero.
//
//   ./clear_faults [--ip <address>]   (default: config::kRightRobotIp)
//

#include "Config.h"
#include "Hardware.h"
#include "Safety.h"
#include "ToolArgs.h"

#include <chrono>
#include <iostream>
#include <thread>

namespace
{
    void PrintState(const char* tag, const k_api::BaseCyclic::Feedback& fb)
    {
        std::cout << tag << ": arm state "
            << k_api::Common::ArmState_Name(
                static_cast<k_api::Common::ArmState>(fb.base().active_state()))
            << ", base fault bank " << DecodeBaseBank(fb.base().fault_bank_a())
            << "\n";
        for (int i = 0; i < fb.actuators_size(); ++i)
            if (fb.actuators(i).fault_bank_a() != 0)
                std::cout << "  joint " << (i + 1) << ": fault "
                    << DecodeActuatorBank(fb.actuators(i).fault_bank_a())
                    << " at " << fb.actuators(i).position() << " deg (raw)\n";
    }

    // The same criterion the controller's readiness gate applies
    // (Safety.cpp RobotReadyForTakeover), and for the same reason: the
    // base's JOINT_FAULT (16) summary bit is a LATCHED HISTORICAL
    // aggregate, not a live interlock. With it set, every actuator bank
    // clear and the arm SERVOING_READY, the Kinova Web App moves the arm
    // normally (fault-handling-hardening.md, correction of 2026-07-20).
    //
    // This tool used to treat any nonzero base bank as fatal and print "Do
    // not proceed to a takeover", contradicting the controller it is meant
    // to support and sending the operator hunting a fault that was not
    // there (2026-08-04).
    bool AllClear(const k_api::BaseCyclic::Feedback& fb)
    {
        if ((fb.base().fault_bank_a() & ~kJointFaultBit) != 0)
            return false;
        for (int i = 0; i < fb.actuators_size(); ++i)
            if (fb.actuators(i).fault_bank_a() != 0)
                return false;
        return static_cast<k_api::Common::ArmState>(fb.base().active_state()) ==
            k_api::Common::ArmState::ARMSTATE_SERVOING_READY;
    }

    // True when the only thing left is that latched summary bit — worth
    // saying out loud, because ClearFaults will never clear it.
    bool OnlyStaleJointFault(const k_api::BaseCyclic::Feedback& fb)
    {
        return (fb.base().fault_bank_a() & kJointFaultBit) != 0 &&
               AllClear(fb);
    }
} // namespace

int main(int argc, char** argv)
{
    const std::string ip = ParseIpArg(argc, argv, "clear_faults");
    try
    {
        Connect connection(ip);
        k_api::BaseCyclic::Feedback fb =
            connection.base_cyclic()->RefreshFeedback();
        PrintState("before", fb);

        if (AllClear(fb))
        {
            std::cout << (OnlyStaleJointFault(fb)
                              ? "nothing to clear — arm is READY, every actuator "
                                "bank clear; the base JOINT_FAULT bit is a stale "
                                "summary and will not clear (expected)\n"
                              : "nothing to clear — arm is READY with zero fault "
                                "banks\n");
            return 0;
        }

        std::cout << "issuing Base::ClearFaults (supported SDK recovery, "
            "no motion)...\n";
        connection.base()->ClearFaults();

        // The base takes a moment to re-arm the actuators; poll briefly
        // rather than trusting one immediate read.
        for (int attempt = 0; attempt < 10; ++attempt)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            fb = connection.base_cyclic()->RefreshFeedback();
            if (AllClear(fb))
                break;
        }
        PrintState("after", fb);

        if (!AllClear(fb))
        {
            std::cout << "NOT CLEAR after ClearFaults — an ACTUATOR bank is "
                "re-latching (joint likely outside its configured limit), a "
                "non-JOINT_FAULT base bit is set, or the arm is not READY. "
                "Do not proceed to a takeover.\n";
            return 1;
        }
        std::cout << (OnlyStaleJointFault(fb)
                          ? "clear: every actuator bank zero and arm "
                            "SERVOING_READY; the base JOINT_FAULT bit remains "
                            "latched, which is expected and does not block a "
                            "takeover\n"
                          : "all fault banks zero, arm SERVOING_READY\n");
        return 0;
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex API error: " << ex.what() << "\n";
        return 1;
    }
    catch (std::exception& ex)
    {
        std::cout << "error: " << ex.what() << "\n";
        return 1;
    }
}
