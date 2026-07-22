//
// TEMPORARY diagnostic: where does the base JOINT_FAULT (bit 16) come from?
// Delete this file and its CMake target once the investigation is done.
//
// Walks the same startup sequence as the controller, printing every fault /
// warning bank (A and B, base and all 7 actuators), servoing mode and arm
// state at five checkpoints:
//   1. immediately after connecting            (fault here = pre-existing,
//                                               latched from before this run)
//   2. before switching to low-level servoing
//   3. immediately after switching
//   4. after the first unchanged holding command
//   5. after the first (tiny: +0.05 deg, joint 2) movement command
// then dumps the base's stored safety information (TCP, DeviceConfig), and
// finally — if a base fault is still latched — clears ONCE via the official
// ClearFaults(), re-reads, and reports whether bit 16 disappeared.
//
// MOVES THE ARM (checkpoint 5, 0.05 deg) — operator + e-stop required.
// The cyclic-command stamping is knowingly duplicated from Motion.cpp's
// send_positions: linking Motion here would drag in Record/Measure/Dynamics
// (Pinocchio) for a throwaway tool.
//

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <ActuatorConfig.pb.h> // SafetyIdentifierBankA (actuator fault bit names)
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <KDetailedException.h>
#include <google/protobuf/descriptor.h>

#include "app/Config.h"
#include "hardware/Connect.h"

namespace
{

    // "16 (JOINT_FAULT)" instead of "16". `names` may be null (bank B has no
    // published enum in this API version) — then bits print numerically.
    std::string decode(std::uint32_t bank, const google::protobuf::EnumDescriptor* names)
    {
        if (bank == 0)
            return "0";
        std::string out = std::to_string(bank) + " (";
        bool first = true;
        for (std::uint32_t bit = 1; bit != 0; bit <<= 1) {
            if (!(bank & bit))
                continue;
            if (!first)
                out += " | ";
            first = false;
            const auto* value = names ? names->FindValueByNumber(static_cast<int>(bit)) : nullptr;
            out += value ? value->name() : "bit " + std::to_string(bit);
        }
        return out + ")";
    }

    void checkpoint(const char* label, k_api::Base::BaseClient* base,
                    k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
    {
        const auto* base_ids = k_api::Base::SafetyIdentifier_descriptor();
        const auto* act_ids = k_api::ActuatorConfig::SafetyIdentifierBankA_descriptor();

        auto fb = base_cyclic->RefreshFeedback();
        std::cout << "\n=== " << label << " ===\n";
        try {
            std::cout << "servoing mode: "
                      << k_api::Base::ServoingMode_Name(base->GetServoingMode().servoing_mode())
                      << "\n";
        } catch (std::exception& ex) {
            std::cout << "servoing mode: <query failed: " << ex.what() << ">\n";
        }
        std::cout << "arm state:     "
                  << k_api::Common::ArmState_Name(
                         static_cast<k_api::Common::ArmState>(fb.base().active_state()))
                  << "\n";
        std::cout << "base  fault A " << decode(fb.base().fault_bank_a(), base_ids) << "  B "
                  << decode(fb.base().fault_bank_b(), nullptr) << "\n"
                  << "      warn  A " << decode(fb.base().warning_bank_a(), base_ids) << "  B "
                  << decode(fb.base().warning_bank_b(), nullptr) << "\n";
        for (int i = 0; i < fb.actuators_size(); ++i) {
            const auto& a = fb.actuators(i);
            std::cout << "joint " << (i + 1) << " (" << a.position() << " deg)"
                      << "  fault A " << decode(a.fault_bank_a(), act_ids) << "  B "
                      << decode(a.fault_bank_b(), nullptr) << "  warn A "
                      << decode(a.warning_bank_a(), act_ids) << "  B "
                      << decode(a.warning_bank_b(), nullptr) << "\n";
        }
        // This API version exposes no fault/warning banks on the interconnect
        // cyclic feedback (grep InterconnectCyclic.pb.h) — nothing to print.
    }

    // One cyclic exchange (duplicated from Motion::send_positions — see top).
    void refresh_once(k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                      k_api::BaseCyclic::Command& command)
    {
        command.set_frame_id((command.frame_id() + 1) % 65536);
        for (int i = 0; i < command.actuators_size(); ++i)
            command.mutable_actuators(i)->set_command_id(command.frame_id());
        base_cyclic->Refresh(command, 0);
    }

} // namespace

int main()
{
    try {
        Connect connection(config::kRobotIp);
        auto* base = connection.base();
        auto* base_cyclic = connection.base_cyclic();

        checkpoint("1: immediately after connecting", base, base_cyclic);
        bool pre_existing = base_cyclic->RefreshFeedback().base().fault_bank_a() != 0;
        std::cout << "\nfault existed BEFORE this program attempted motion: "
                  << (pre_existing ? "YES (latched from an earlier run/event)" : "no") << "\n";

        checkpoint("2: before switching to low-level servoing", base, base_cyclic);

        auto servoing_mode = k_api::Base::ServoingModeInformation();
        try {
            servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
            base->SetServoingMode(servoing_mode);
            checkpoint("3: immediately after switching to low-level", base, base_cyclic);

            auto fb = base_cyclic->RefreshFeedback();
            k_api::BaseCyclic::Command command;
            for (int i = 0; i < fb.actuators_size(); ++i)
                command.add_actuators()->set_position(fb.actuators(i).position());

            refresh_once(base_cyclic, command); // hold exactly in place
            checkpoint("4: after the first unchanged holding command", base, base_cyclic);

            // +0.05 deg on joint 2: one cycle's worth of real movement.
            command.mutable_actuators(1)->set_position(fb.actuators(1).position() + 0.05);
            refresh_once(base_cyclic, command);
            checkpoint("5: after the first movement command", base, base_cyclic);
        } catch (std::exception& ex) {
            std::cout << "\nlow-level sequence stopped early: " << ex.what() << "\n"
                      << "(a latched base fault refuses LOW_LEVEL_SERVOING — expected "
                         "if checkpoint 1 already showed the fault)\n";
        }
        try {
            servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
            base->SetServoingMode(servoing_mode);
        } catch (...) {
            std::cout << "WARNING: could not restore SINGLE_LEVEL servoing\n";
        }

        // Stored safety information over TCP — richer than the cyclic banks
        // (per-safety status, latched flags, limits).
        std::cout << "\n=== stored safety information (DeviceConfig, TCP) ===\n";
        try {
            k_api::DeviceConfig::DeviceConfigClient device_config(connection.router());
            auto list = device_config.GetAllSafetyInformation();
            for (const auto& info : list.information())
                if (info.can_change_safety_state() || info.has_warning_threshold() ||
                    info.has_error_threshold())
                    std::cout << "  " << info.ShortDebugString() << "\n";
        } catch (std::exception& ex) {
            std::cout << "  <query failed: " << ex.what() << ">\n";
        }

        // Clear ONCE, officially, only if a base fault is actually latched.
        auto fault_a = base_cyclic->RefreshFeedback().base().fault_bank_a();
        if (fault_a != 0) {
            std::cout << "\n=== clearing latched faults once (ClearFaults) ===\n";
            base->ClearFaults();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            auto after = base_cyclic->RefreshFeedback().base().fault_bank_a();
            std::cout << "base fault bank A before: "
                      << decode(fault_a, k_api::Base::SafetyIdentifier_descriptor())
                      << "  after: "
                      << decode(after, k_api::Base::SafetyIdentifier_descriptor()) << "\n"
                      << (after == 0 ? "fault cleared — it was latched, not active.\n"
                                     : "fault STILL PRESENT — an actuator or safety is actively "
                                       "faulting; do not retry moves, inspect the arm.\n");
        } else {
            std::cout << "\nno base fault latched at end of diagnostic — nothing to clear.\n";
        }
        return 0;
    } catch (std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
