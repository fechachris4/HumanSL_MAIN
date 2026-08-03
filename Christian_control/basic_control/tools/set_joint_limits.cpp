//
// set_joint_limits — WRITES ROBOT SAFETY CONFIGURATION for joints 4 and 6.
//
// Why this exists (2026-08-03): read_safety_limits showed that on this arm
// joint 2 carries real JOINT_LIMIT thresholds (+/-140 error, +/-130 warning)
// while every other actuator is configured 0/0. For the two other BOUNDED
// joints, 4 and 6, that is a degenerate empty band: the firmware latched
// JOINT_LIMIT_LOW on joint 4 and JOINT_LIMIT_HIGH on joint 6 on any motion
// further away from zero, wherever the joint happened to be. Joints
// 1/3/5/7 are continuous and are deliberately NOT touched.
//
// The values below are a BACKSTOP that sits just OUTSIDE the rated model
// range, so the software limits stay the primary constraint:
//
//   joint 4  rated +/-147.8 deg (Kinova spec; URDF +/-147.25)
//            -> warning +/-145.0, error +/-150.0
//   joint 6  rated +/-120.3 deg (Kinova spec; URDF +/-119.75)
//            -> warning +/-118.0, error +/-123.0
//
// This is deliberately TIGHTER than joint 2's factory-style margin (error
// ~11 deg beyond rated). Nothing here widens a limit past the joint's rated
// range; it replaces an empty band with the rated one.
//
// The write is persistent in the actuator's configuration. To undo, run
// with --restore-degenerate (puts 0/0 back), or use the Kinova web
// dashboard. NO MOTION is commanded by this tool.
//
//   ./set_joint_limits --apply
//   ./set_joint_limits              (dry run: prints current values only)
//   ./set_joint_limits --restore-degenerate --apply
//

#include "Config.h"
#include "Hardware.h"

#include <DeviceConfigClientRpc.h>
#include <ActuatorConfig.pb.h>

#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>

namespace k_api = Kinova::Api;

namespace
{
    struct LimitSpec
    {
        int joint;          // 1-based joint number == device identifier
        double warning_deg; // magnitude; sign applied per HIGH/LOW
        double error_deg;
    };

    // Kortex safety identifiers for the two position limits.
    constexpr unsigned kHigh = k_api::ActuatorConfig::SafetyIdentifierBankA::JOINT_LIMIT_HIGH;
    constexpr unsigned kLow = k_api::ActuatorConfig::SafetyIdentifierBankA::JOINT_LIMIT_LOW;

    void PrintPair(k_api::DeviceConfig::DeviceConfigClient& client, int joint)
    {
        for (const unsigned id : {kHigh, kLow})
        {
            k_api::Common::SafetyHandle handle;
            handle.set_identifier(id);
            const auto cfg = client.GetSafetyConfiguration(
                handle, static_cast<std::uint32_t>(joint));
            std::cout << "    joint " << joint << " "
                << (id == kHigh ? "JOINT_LIMIT_HIGH" : "JOINT_LIMIT_LOW ")
                << "  warning=" << std::setw(8) << cfg.warning_threshold()
                << "  error=" << std::setw(8) << cfg.error_threshold() << "\n";
        }
    }
} // namespace

int main(int argc, char** argv)
{
    bool apply = false;
    bool restore_degenerate = false;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--apply") == 0)
            apply = true;
        else if (std::strcmp(argv[i], "--restore-degenerate") == 0)
            restore_degenerate = true;
        else
        {
            std::cerr << "unknown argument: " << argv[i] << "\n";
            return 2;
        }
    }

    std::vector<LimitSpec> specs = {
        {4, 145.0, 150.0},
        {6, 118.0, 123.0},
    };
    if (restore_degenerate)
        for (auto& s : specs)
        {
            s.warning_deg = 0.0;
            s.error_deg = 0.0;
        }

    try
    {
        Connect connection(config::kRightRobotIp);
        k_api::DeviceConfig::DeviceConfigClient device_config(
            connection.tcp_router());

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "BEFORE:\n";
        for (const auto& s : specs)
            PrintPair(device_config, s.joint);

        if (!apply)
        {
            std::cout << "\ndry run — nothing written. Re-run with --apply to "
                "write the values below:\n";
            for (const auto& s : specs)
                std::cout << "    joint " << s.joint << "  warning +/-"
                    << s.warning_deg << "  error +/-" << s.error_deg << "\n";
            return 0;
        }

        std::cout << "\nWRITING safety configuration (no motion)...\n";
        for (const auto& s : specs)
            for (const unsigned id : {kHigh, kLow})
            {
                const double sign = (id == kHigh) ? 1.0 : -1.0;

                // Read-modify-write: preserve whatever else the message
                // carries (enable state), change only the two thresholds.
                k_api::Common::SafetyHandle handle;
                handle.set_identifier(id);
                k_api::DeviceConfig::SafetyConfiguration cfg =
                    device_config.GetSafetyConfiguration(
                        handle, static_cast<std::uint32_t>(s.joint));
                cfg.mutable_handle()->set_identifier(id);
                cfg.set_warning_threshold(
                    static_cast<float>(sign * s.warning_deg));
                cfg.set_error_threshold(static_cast<float>(sign * s.error_deg));
                device_config.SetSafetyConfiguration(
                    cfg, static_cast<std::uint32_t>(s.joint));
            }

        std::cout << "\nAFTER (read back from the robot):\n";
        bool ok = true;
        for (const auto& s : specs)
        {
            PrintPair(device_config, s.joint);
            for (const unsigned id : {kHigh, kLow})
            {
                const double sign = (id == kHigh) ? 1.0 : -1.0;
                k_api::Common::SafetyHandle handle;
                handle.set_identifier(id);
                const auto cfg = device_config.GetSafetyConfiguration(
                    handle, static_cast<std::uint32_t>(s.joint));
                if (std::abs(cfg.error_threshold() - sign * s.error_deg) > 0.01 ||
                    std::abs(cfg.warning_threshold() - sign * s.warning_deg) > 0.01)
                    ok = false;
            }
        }
        if (!ok)
        {
            std::cout << "\nREADBACK MISMATCH — the robot did not accept one or "
                "more values. Do not rely on these limits.\n";
            return 1;
        }
        std::cout << "\nreadback matches every requested value\n";
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
