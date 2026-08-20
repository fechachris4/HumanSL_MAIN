//
// set_joint_limits — WRITES ROBOT SAFETY CONFIGURATION for joints 2, 4, and 6.
//
// Why this exists: every bounded joint needs a non-degenerate JOINT_LIMIT
// band. A live read found joint 2's HIGH/LOW warning and error thresholds
// at 0/0, which faults outward motion; joints 4 and 6 have the same
// power-cycle-volatile configuration risk. Joints 1/3/5/7 are continuous
// and are deliberately NOT touched.
//
// Firmware JOINT_LIMIT is the actual joint-position enforcement; the
// controller has no separate client-side joint-position clamp. Its velocity
// clip, reach screen, and following-error stop are separate protections.
// Warnings ARE the physical Kinova limits, derived from the one
// authoritative table (planning/config/joint_limits.yaml) since 2026-08-20;
// they are no longer three hand-authored numbers. Errors stay hand-authored
// and sit outside those limits, as the last robot-side protection:
//
//   joint 2  warning +/-128.9, error +/-140.0
//   joint 4  warning +/-147.8, error +/-150.0
//   joint 6  warning +/-120.3, error +/-123.0
//
// The software layers stop first: the planner 2 deg inside the physical
// limit, the controller 1 deg inside it, then this warning, then the error.
//
// The values match Config.h, which the controller re-applies and verifies on
// every connection. DeviceConfig writes are not durable across a power cycle.
//
// The write affects the current actuator configuration; it is not durable
// across a power cycle. To deliberately restore a 0/0 band, run with
// --restore-degenerate. NO MOTION is commanded by this tool.
//
//   ./set_joint_limits --apply
//   ./set_joint_limits              (dry run: prints current values only)
//   ./set_joint_limits --restore-degenerate --apply
//   ./set_joint_limits --ip <address> --apply   (default: config::kRightRobotIp)
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
    std::string ip = config::kRightRobotIp;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--apply") == 0)
            apply = true;
        else if (std::strcmp(argv[i], "--restore-degenerate") == 0)
            restore_degenerate = true;
        else if (std::strcmp(argv[i], "--ip") == 0)
        {
            if (++i >= argc)
            {
                std::cerr << "--ip needs an address\n";
                return 2;
            }
            ip = argv[i];
        }
        else
        {
            std::cerr << "unknown argument: " << argv[i] << "\n";
            return 2;
        }
    }

    std::vector<LimitSpec> specs = {
        {2, config::kJointLimitWarnDeg[1], config::kJointLimitErrorDeg[1]},
        {4, config::kJointLimitWarnDeg[3], config::kJointLimitErrorDeg[3]},
        {6, config::kJointLimitWarnDeg[5], config::kJointLimitErrorDeg[5]},
    };
    if (restore_degenerate)
        for (auto& s : specs)
        {
            s.warning_deg = 0.0;
            s.error_deg = 0.0;
        }

    try
    {
        Connect connection(ip);
        k_api::DeviceConfig::DeviceConfigClient& device_config =
            *connection.device_config();

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
