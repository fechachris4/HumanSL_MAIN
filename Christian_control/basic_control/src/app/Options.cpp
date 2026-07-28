//
// Options: CLI + TOML parsing and the effective-config echo (see header).
//

#include "app/Options.h"

#include "app/Config.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>

#include <unistd.h>

#include "tomlplusplus/toml.hpp"

namespace
{
    [[noreturn]] void UsageAndExit(const std::string& error)
    {
        if (!error.empty())
            std::cerr << "error: " << error << "\n";
        std::cerr <<
            "usage: controller [options]\n"
            "  --controller <name>   control law (valid: resolved-rate, reactive-pose)\n"
            "  --kp <value>          Cartesian position P gain, 1/s (both laws)\n"
            "  --log <file>          CSV filename (default: run_<timestamp>.csv)\n"
            "  --config <path>       TOML file with gains/thresholds; without this\n"
            "                        flag the compiled default file\n"
#ifdef DEFAULT_CONFIG_PATH
            "                        " DEFAULT_CONFIG_PATH "\n"
#endif
            "                        is loaded when it exists\n"
            "  --help\n"
            "precedence: CLI > TOML > compiled defaults (src/app/Config.h)\n"
            "TOML keys: controller, kp, dls_lambda, following_error_limit_deg,\n"
            "  arrival_tolerance_m, nonfinite_stop_cycles,\n"
            "  overrun_stop_cycles, overrun_factor;\n"
            "  reactive-pose only: kp_rot, kd_pos, kd_rot, null_gain,\n"
            "  orientation_enabled, velocity_term_enabled, null_space_enabled,\n"
            "  target_file (watched pose-target file; edit+save to retarget)\n"
            "safety policy is NOT configurable at runtime (config::kStopOnFault\n"
            "is compile-time only)\n";
        std::exit(2);
    }

    std::string FormatDouble(double v)
    {
        std::ostringstream out;
        out << v;
        return out.str();
    }

    void ApplyToml(EffectiveConfig& cfg, const std::string& path)
    {
        toml::table table;
        try
        {
            table = toml::parse_file(path);
        }
        catch (const toml::parse_error& err)
        {
            std::ostringstream what;
            what << path << ": " << err.description() << " (line "
                 << err.source().begin.line << ")";
            UsageAndExit(what.str());
        }

        for (const auto& entry : table)
        {
            const std::string name(entry.first.str());
            const toml::node& node = entry.second; // named (not a structured
                                        // binding): lambdas capture it in C++17
            const auto number = [&](double& field)
            {
                if (const auto v = node.value<double>())
                    field = *v;
                else
                    UsageAndExit(path + ": key '" + name + "' must be a number");
                cfg.source[name] = "toml";
            };
            const auto integer = [&](int& field)
            {
                if (const auto v = node.value<int64_t>())
                    field = static_cast<int>(*v);
                else
                    UsageAndExit(path + ": key '" + name + "' must be an integer");
                cfg.source[name] = "toml";
            };
            const auto boolean = [&](bool& field)
            {
                if (const auto v = node.value<bool>())
                    field = *v;
                else
                    UsageAndExit(path + ": key '" + name + "' must be true or false");
                cfg.source[name] = "toml";
            };
            const auto text = [&](std::string& field)
            {
                if (const auto v = node.value<std::string>())
                    field = *v;
                else
                    UsageAndExit(path + ": key '" + name + "' must be a string");
                cfg.source[name] = "toml";
            };
            if (name == "controller") text(cfg.controller);
            else if (name == "target_file") text(cfg.target_file);
            else if (name == "kp") number(cfg.kp);
            else if (name == "dls_lambda") number(cfg.dls_lambda);
            else if (name == "kp_rot") number(cfg.kp_rot);
            else if (name == "kd_pos") number(cfg.kd_pos);
            else if (name == "kd_rot") number(cfg.kd_rot);
            else if (name == "null_gain") number(cfg.null_gain);
            else if (name == "orientation_enabled") boolean(cfg.orientation_enabled);
            else if (name == "velocity_term_enabled") boolean(cfg.velocity_term_enabled);
            else if (name == "null_space_enabled") boolean(cfg.null_space_enabled);
            else if (name == "following_error_limit_deg")
                number(cfg.following_error_limit_deg);
            else if (name == "arrival_tolerance_m") number(cfg.arrival_tolerance_m);
            else if (name == "nonfinite_stop_cycles") integer(cfg.nonfinite_stop_cycles);
            else if (name == "overrun_stop_cycles") integer(cfg.overrun_stop_cycles);
            else if (name == "overrun_factor") number(cfg.overrun_factor);
            else if (name == "stop_on_fault")
                UsageAndExit(path + ": 'stop_on_fault' is compile-time only "
                             "(config::kStopOnFault) and cannot be set here");
            else
                UsageAndExit(path + ": unknown key '" + name +
                             "' (a typo must not silently fall back to a default)");
        }
    }
} // namespace

EffectiveConfig ParseOptions(int argc, char** argv)
{
    EffectiveConfig cfg;
    cfg.kp = config::kKpCartesian;
    cfg.dls_lambda = config::kDlsLambda;
    cfg.kp_rot = config::kKpRotation;
    cfg.kd_pos = config::kKdPosition;
    cfg.kd_rot = config::kKdRotation;
    cfg.null_gain = config::kNullGain;
    cfg.orientation_enabled = config::kOrientationEnabled;
    cfg.velocity_term_enabled = config::kVelocityTermEnabled;
    cfg.null_space_enabled = config::kNullSpaceEnabled;
    cfg.following_error_limit_deg = config::kFollowingErrorLimitDeg;
    cfg.arrival_tolerance_m = config::kArrivalToleranceM;
    cfg.nonfinite_stop_cycles = config::kNonFiniteStopCycles;
    cfg.overrun_stop_cycles = config::kOverrunStopCycles;
    cfg.overrun_factor = config::kOverrunFactor;
    for (const char* key :
         {"controller", "kp", "dls_lambda", "kp_rot", "kd_pos", "kd_rot",
          "null_gain", "orientation_enabled", "velocity_term_enabled",
          "null_space_enabled", "following_error_limit_deg",
          "arrival_tolerance_m", "nonfinite_stop_cycles", "overrun_stop_cycles",
          "overrun_factor", "log_file", "target_file", "config_file"})
        cfg.source[key] = "compiled";

    // First pass: locate --config so TOML applies before CLI overrides.
    std::string config_path;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help")
            UsageAndExit("");
        if (arg == "--config")
        {
            if (i + 1 >= argc)
                UsageAndExit("--config needs a path");
            config_path = argv[++i];
            cfg.source["config_file"] = "cli";
        }
    }
#ifdef DEFAULT_CONFIG_PATH
    // No --config: load the compiled default file when it exists. A fixed
    // absolute path baked in at build time — behavior never depends on the
    // working directory (runtime-config.md, "Default config file").
    if (config_path.empty() && ::access(DEFAULT_CONFIG_PATH, R_OK) == 0)
    {
        config_path = DEFAULT_CONFIG_PATH;
        cfg.source["config_file"] = "default";
    }
#endif
    cfg.config_path = config_path;
    if (!config_path.empty())
        ApplyToml(cfg, config_path);

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string
        {
            if (i + 1 >= argc)
                UsageAndExit(arg + " needs a value");
            return argv[++i];
        };
        if (arg == "--config")
        {
            ++i; // consumed in the first pass
        }
        else if (arg == "--controller")
        {
            cfg.controller = value();
            cfg.source["controller"] = "cli";
        }
        else if (arg == "--kp")
        {
            try
            {
                cfg.kp = std::stod(value());
            }
            catch (...)
            {
                UsageAndExit("--kp needs a number");
            }
            cfg.source["kp"] = "cli";
        }
        else if (arg == "--log")
        {
            cfg.log_file = value();
            cfg.source["log_file"] = "cli";
        }
        else
            UsageAndExit("unknown option '" + arg + "'");
    }

    if (cfg.controller != "resolved-rate" && cfg.controller != "reactive-pose")
        UsageAndExit("unknown controller '" + cfg.controller +
                     "' (valid: resolved-rate, reactive-pose)");
    if (!cfg.target_file.empty() && cfg.controller != "reactive-pose")
        UsageAndExit("target_file requires controller = \"reactive-pose\" "
                     "(the watched file carries pose targets)");
    return cfg;
}

namespace
{
    void WriteConfigLines(const EffectiveConfig& cfg, std::ostream& out,
                          const char* prefix)
    {
        const auto line = [&](const std::string& key, const std::string& value)
        {
            const auto source = cfg.source.find(key);
            out << prefix << key << " = " << value << " ("
                << (source != cfg.source.end() ? source->second : "compiled") << ")\n";
        };
        line("config_file", cfg.config_path.empty() ? "<none>" : cfg.config_path);
        out << prefix << "controlled_arm = right (compile-time only)\n";
        out << prefix << "robot_ip = " << config::kRightRobotIp
            << " (compiled right-only hardware map)\n";
        out << prefix << "urdf = " << GEN3_DUAL_URDF_PATH
            << " (compiled dual mounted model)\n";
        out << prefix << "end_effector_frame = " << config::kRightEndEffectorFrame
            << " (compiled; dual-model world/common mount frame)\n";
        out << prefix << "left_nominal_rad = 0,0,0,0,0,0,0"
            << " (compiled model-only state)\n";
        line("controller", cfg.controller);
        line("target_file", cfg.target_file.empty() ? "<none>" : cfg.target_file);
        line("kp", FormatDouble(cfg.kp));
        line("dls_lambda", FormatDouble(cfg.dls_lambda));
        line("kp_rot", FormatDouble(cfg.kp_rot));
        line("kd_pos", FormatDouble(cfg.kd_pos));
        line("kd_rot", FormatDouble(cfg.kd_rot));
        line("null_gain", FormatDouble(cfg.null_gain));
        line("orientation_enabled", cfg.orientation_enabled ? "true" : "false");
        line("velocity_term_enabled", cfg.velocity_term_enabled ? "true" : "false");
        line("null_space_enabled", cfg.null_space_enabled ? "true" : "false");
        line("following_error_limit_deg", FormatDouble(cfg.following_error_limit_deg));
        line("arrival_tolerance_m", FormatDouble(cfg.arrival_tolerance_m));
        line("nonfinite_stop_cycles", std::to_string(cfg.nonfinite_stop_cycles));
        line("overrun_stop_cycles", std::to_string(cfg.overrun_stop_cycles));
        line("overrun_factor", FormatDouble(cfg.overrun_factor));
        line("log_file", cfg.log_file.empty() ? "<timestamped>" : cfg.log_file);
        out << prefix << "stop_on_fault = " << (config::kStopOnFault ? "true" : "false")
            << " (compile-time only)\n";
        out << prefix << "control_dt_s = " << FormatDouble(config::kControlDtS)
            << " (compiled)\n";
        out << prefix << "qdot_clip_deg_s = " << FormatDouble(config::kQdotLimitDegS[0])
            << "/" << FormatDouble(config::kQdotLimitDegS[4])
            << " (compiled model limits, joints 1-4/5-7)\n";
    }
} // namespace

void EchoConfig(const EffectiveConfig& cfg, std::ostream& out)
{
    out << "effective config (CLI > TOML > compiled):\n";
    WriteConfigLines(cfg, out, "  ");
}

void WriteCsvPreamble(const EffectiveConfig& cfg, std::ostream& csv)
{
    csv << "# controller run config — parsers skip '#' lines\n";
    // Bump when columns change so scripts detect the format without
    // sniffing headers. 2 = t_send/t_recv + quaternion + pd_beyond_reach
    // + reactive-pose rotation error (hardware/Record.h).
    csv << "# log_format = 2 (compiled)\n";
    WriteConfigLines(cfg, csv, "# ");
}
