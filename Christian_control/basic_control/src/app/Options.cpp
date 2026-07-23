//
// Options: CLI + TOML parsing and the effective-config echo (see header).
//

#include "app/Options.h"

#include "app/Config.h"

#include <cstdlib>
#include <iostream>
#include <sstream>

#include "tomlplusplus/toml.hpp"

namespace
{
    [[noreturn]] void UsageAndExit(const std::string& error)
    {
        if (!error.empty())
            std::cerr << "error: " << error << "\n";
        std::cerr <<
            "usage: controller [options]\n"
            "  --controller <name>   control law (valid: resolved-rate)\n"
            "  --kp <value>          Cartesian P gain, 1/s\n"
            "  --log <file>          CSV filename (default: run_<timestamp>.csv)\n"
            "  --config <path>       TOML file with gains/thresholds (explicit\n"
            "                        path only — never auto-discovered)\n"
            "  --help\n"
            "precedence: CLI > TOML > compiled defaults (src/app/Config.h)\n"
            "TOML keys: kp, dls_lambda, following_error_limit_deg,\n"
            "  arrival_tolerance_m, nonfinite_stop_cycles,\n"
            "  saturation_stop_cycles, overrun_stop_cycles, overrun_factor\n"
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
            if (name == "kp") number(cfg.kp);
            else if (name == "dls_lambda") number(cfg.dls_lambda);
            else if (name == "following_error_limit_deg")
                number(cfg.following_error_limit_deg);
            else if (name == "arrival_tolerance_m") number(cfg.arrival_tolerance_m);
            else if (name == "nonfinite_stop_cycles") integer(cfg.nonfinite_stop_cycles);
            else if (name == "saturation_stop_cycles") integer(cfg.saturation_stop_cycles);
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
    cfg.following_error_limit_deg = config::kFollowingErrorLimitDeg;
    cfg.arrival_tolerance_m = config::kArrivalToleranceM;
    cfg.nonfinite_stop_cycles = config::kNonFiniteStopCycles;
    cfg.saturation_stop_cycles = config::kSaturationStopCycles;
    cfg.overrun_stop_cycles = config::kOverrunStopCycles;
    cfg.overrun_factor = config::kOverrunFactor;
    for (const char* key :
         {"controller", "kp", "dls_lambda", "following_error_limit_deg",
          "arrival_tolerance_m", "nonfinite_stop_cycles", "saturation_stop_cycles",
          "overrun_stop_cycles", "overrun_factor", "log_file"})
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
        }
    }
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

    if (cfg.controller != "resolved-rate")
        UsageAndExit("unknown controller '" + cfg.controller +
                     "' (valid: resolved-rate)");
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
        line("controller", cfg.controller);
        line("kp", FormatDouble(cfg.kp));
        line("dls_lambda", FormatDouble(cfg.dls_lambda));
        line("following_error_limit_deg", FormatDouble(cfg.following_error_limit_deg));
        line("arrival_tolerance_m", FormatDouble(cfg.arrival_tolerance_m));
        line("nonfinite_stop_cycles", std::to_string(cfg.nonfinite_stop_cycles));
        line("saturation_stop_cycles", std::to_string(cfg.saturation_stop_cycles));
        line("overrun_stop_cycles", std::to_string(cfg.overrun_stop_cycles));
        line("overrun_factor", FormatDouble(cfg.overrun_factor));
        line("log_file", cfg.log_file.empty() ? "<timestamped>" : cfg.log_file);
        out << prefix << "stop_on_fault = " << (config::kStopOnFault ? "true" : "false")
            << " (compile-time only)\n";
        out << prefix << "control_dt_s = " << FormatDouble(config::kControlDtS)
            << " (compiled)\n";
        out << prefix << "qdot_clip_deg_s = " << FormatDouble(config::kQdotLimitDegS[0])
            << "/" << FormatDouble(config::kQdotLimitDegS[4])
            << " (compiled, derived joints 1-4/5-7)\n";
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
    WriteConfigLines(cfg, csv, "# ");
}
