//
// clear_faults: read the arm's fault state, invoke the SDK's supported
// whole-arm recovery (Base::ClearFaults — the same operation as the Kinova
// web dashboard's "Clear faults" button), and read back the result.
//
// NO MOTION is commanded and no servoing mode is changed: the only writes
// are the session login and the ClearFaults RPC itself. Exit 0 only if,
// after clearing, the arm reports SERVOING_READY with every fault bank zero.
//
//   ./clear_faults
//

#include "Config.h"
#include "Hardware.h"
#include "Safety.h"

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

    bool AllClear(const k_api::BaseCyclic::Feedback& fb)
    {
        if (fb.base().fault_bank_a() != 0)
            return false;
        for (int i = 0; i < fb.actuators_size(); ++i)
            if (fb.actuators(i).fault_bank_a() != 0)
                return false;
        return static_cast<k_api::Common::ArmState>(fb.base().active_state()) ==
            k_api::Common::ArmState::ARMSTATE_SERVOING_READY;
    }
} // namespace

int main()
{
    try
    {
        Connect connection(config::kRightRobotIp);
        k_api::BaseCyclic::Feedback fb =
            connection.base_cyclic()->RefreshFeedback();
        PrintState("before", fb);

        if (AllClear(fb))
        {
            std::cout << "nothing to clear — arm is READY with zero fault banks\n";
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
            std::cout << "NOT CLEAR after ClearFaults — a fault is re-latching "
                "(joint likely outside its configured limit) or the arm is "
                "not READY. Do not proceed to a takeover.\n";
            return 1;
        }
        std::cout << "all fault banks zero, arm SERVOING_READY\n";
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
