//
// read_safety_limits — READ-ONLY: print every actuator's configured safety
// thresholds straight from the robot (DeviceManager + DeviceConfig), next
// to its live position. No motion, no writes, no servoing change.
//
// This answers, from the robot itself rather than any file: where are the
// configured JOINT_LIMIT_HIGH / JOINT_LIMIT_LOW positions that latched
// faults on joints 4 and 6 (runs 2026-08-03 13:39 and 14:05)?
//
//   ./read_safety_limits
//

#include "Config.h"
#include "Hardware.h"
#include "Safety.h" // DecodeBaseBank / DecodeActuatorBank

#include <DeviceConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <ActuatorConfig.pb.h>

#include <iomanip>
#include <iostream>
#include <map>

namespace k_api = Kinova::Api;

int main()
{
    try
    {
        Connect connection(config::kRightRobotIp);

        // This is the same owned ControlConfig client and read-only
        // hard-speed verification used before controller takeover. It sends
        // no motion or servoing command, and also makes this diagnostic a
        // pre-session way to test the GetKinematicHardLimits RPC.
        if (!connection.VerifyKinematicHardLimits(std::cout))
            return 1;
        std::cout << "\n";

        // Live joint positions for side-by-side comparison.
        const k_api::BaseCyclic::Feedback fb =
            connection.base_cyclic()->RefreshFeedback();

        // The live fault picture FIRST, from the real-time path, before any
        // per-device configuration read. This is what says whether a fault
        // is live: it comes from the cyclic feedback, which keeps working
        // even when an actuator's configuration service does not answer.
        std::cout << "arm state "
                  << k_api::Common::ArmState_Name(static_cast<k_api::Common::ArmState>(
                         fb.base().active_state()))
                  << ", base fault bank " << DecodeBaseBank(fb.base().fault_bank_a())
                  << "\n";
        for (int i = 0; i < fb.actuators_size(); ++i)
        {
            const std::uint32_t bank = fb.actuators(i).fault_bank_a();
            std::cout << "  joint " << (i + 1) << "  position "
                      << std::fixed << std::setprecision(2)
                      << fb.actuators(i).position() << " deg  "
                      << std::defaultfloat
                      << (bank == 0 ? "no fault"
                                    : "FAULT " + DecodeActuatorBank(bank))
                      << "\n";
        }
        std::cout << "\n";

        // DeviceManager is not owned by Connect, so build it here. DeviceConfig
        // IS owned by Connect — building a second one on the same router fails
        // with "notification callback is already registered".
        k_api::DeviceManager::DeviceManagerClient device_manager(
            connection.tcp_router());
        k_api::DeviceConfig::DeviceConfigClient& device_config =
            *connection.device_config();

        const k_api::DeviceManager::DeviceHandles devices =
            device_manager.ReadAllDevices();

        int actuator_index = 0; // 0-based Kortex actuator order
        for (int d = 0; d < devices.device_handle_size(); ++d)
        {
            const auto& handle = devices.device_handle(d);
            const auto type = handle.device_type();
            if (type != k_api::Common::DeviceTypes::BIG_ACTUATOR &&
                type != k_api::Common::DeviceTypes::MEDIUM_ACTUATOR &&
                type != k_api::Common::DeviceTypes::SMALL_ACTUATOR)
                continue;

            ++actuator_index;
            // Actuator device identifiers on a Gen3 are 1..7 in joint order;
            // the DeviceHandles list itself is NOT in joint order.
            const int joint = static_cast<int>(handle.device_identifier());
            const double position_deg =
                joint >= 1 && joint <= fb.actuators_size()
                    ? fb.actuators(joint - 1).position()
                    : -1.0;

            std::cout << "joint " << joint << " (device "
                << handle.device_identifier() << ", "
                << k_api::Common::DeviceTypes_Name(type)
                << "), live position " << std::fixed << std::setprecision(2)
                << position_deg << " deg raw:\n" << std::defaultfloat;

            // One unresponsive actuator must not hide the other six. A
            // configuration RPC that times out is itself a finding — the
            // actuator is alive on the real-time bus (its position printed
            // above) but its config service is not answering — so report it
            // and carry on down the chain.
            std::map<unsigned, std::pair<float, float>> configured; // id -> (warn, err)
            k_api::DeviceConfig::SafetyInformationList infos;
            try
            {
                const auto configs = device_config.GetAllSafetyConfiguration(
                    handle.device_identifier());
                for (int c = 0; c < configs.configuration_size(); ++c)
                {
                    const auto& sc = configs.configuration(c);
                    configured[sc.handle().identifier()] = {
                        sc.warning_threshold(), sc.error_threshold()};
                }
                infos = device_config.GetAllSafetyInformation(
                    handle.device_identifier());
            }
            catch (std::exception& ex)
            {
                std::cout << "  CONFIGURATION SERVICE DID NOT ANSWER: "
                          << ex.what() << "\n"
                          << "  (the actuator still reports position on the "
                             "cyclic path, so this is the config channel, "
                             "not a dead joint)\n";
                continue;
            }

            for (int s = 0; s < infos.information_size(); ++s)
            {
                const auto& si = infos.information(s);
                const unsigned id = si.handle().identifier();
                const std::string name =
                    k_api::ActuatorConfig::SafetyIdentifierBankA_IsValid(
                        static_cast<int>(id))
                        ? k_api::ActuatorConfig::SafetyIdentifierBankA_Name(
                              static_cast<k_api::ActuatorConfig::
                                              SafetyIdentifierBankA>(id))
                        : ("id_0x" + std::to_string(id));
                std::cout << "  " << std::left << std::setw(28) << name
                    << std::right;
                if (si.has_error_threshold())
                {
                    const auto it = configured.find(id);
                    std::cout << " error_thr="
                        << (it != configured.end() ? it->second.second : -1.0f)
                        << " (default " << si.default_error_threshold() << ")";
                    if (si.has_warning_threshold() && it != configured.end())
                        std::cout << " warn_thr=" << it->second.first
                            << " (default " << si.default_warning_threshold()
                            << ")";
                }
                std::cout << " hard=[" << si.lower_hard_limit() << ", "
                    << si.upper_hard_limit() << "]"
                    << " unit=" << k_api::Common::Unit_Name(si.unit())
                    << " status="
                    << k_api::Common::SafetyStatusValue_Name(si.status())
                    << "\n";
            }
        }
        if (actuator_index == 0)
        {
            std::cout << "no actuator devices reported\n";
            return 1;
        }
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
