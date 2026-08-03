#include "app/Options.h"
#include "app/Config.h"

#include <cstdlib>
#include <iostream>
#include <sstream>

namespace {
[[noreturn]] void UsageAndExit(const std::string& error) {
    if (!error.empty()) std::cerr << "error: " << error << "\n";
    std::cerr << "usage: controller [--log <file>]\n"
              << "  --log <file>          CSV filename (default: timestamped run file)\n"
              << "All controller settings are compiled in src/app/Config.h.\n";
    std::exit(2);
}

std::string FormatDouble(double value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

void WriteConfigLines(const RunOptions& options, std::ostream& out, const char* prefix) {
    const auto line = [&](const char* key, const std::string& value) {
        out << prefix << key << " = " << value << " (Config.h)\n";
    };
    line("controller", config::kController);
    line("target_file", config::kTargetFile[0] ? config::kTargetFile : "<none>");
    line("trajectory_file", config::kTrajectoryFile[0] ? config::kTrajectoryFile : "<none>");
    line("playback_kp", FormatDouble(config::kPlaybackKp));
    line("start_mismatch_limit_deg", FormatDouble(config::kStartMismatchLimitDeg));
    line("kp", FormatDouble(config::kKpCartesian));
    line("dls_lambda", FormatDouble(config::kDlsLambda));
    line("kp_rot", FormatDouble(config::kKpRotation));
    line("kd_pos", FormatDouble(config::kKdPosition));
    line("kd_rot", FormatDouble(config::kKdRotation));
    line("null_gain", FormatDouble(config::kNullGain));
    line("orientation_enabled", config::kOrientationEnabled ? "true" : "false");
    line("velocity_term_enabled", config::kVelocityTermEnabled ? "true" : "false");
    line("null_space_enabled", config::kNullSpaceEnabled ? "true" : "false");
    line("following_error_limit_deg", FormatDouble(config::kFollowingErrorLimitDeg));
    line("arrival_tolerance_m", FormatDouble(config::kArrivalToleranceM));
    line("nonfinite_stop_cycles", std::to_string(config::kNonFiniteStopCycles));
    line("overrun_stop_cycles", std::to_string(config::kOverrunStopCycles));
    line("overrun_factor", FormatDouble(config::kOverrunFactor));
    out << prefix << "log_file = " << (options.log_file.empty() ? "<timestamped>" : options.log_file)
        << (options.log_file.empty() ? " (default)\n" : " (--log)\n");
    line("control_dt_s", FormatDouble(config::kControlDtS));
}
} // namespace

RunOptions ParseRunOptions(int argc, char** argv) {
    RunOptions options;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) != "--log") UsageAndExit("unknown option '" + std::string(argv[i]) + "'");
        if (++i >= argc) UsageAndExit("--log needs a filename");
        options.log_file = argv[i];
    }
    return options;
}

void EchoConfig(const RunOptions& options, std::ostream& out) {
    out << "controller config (Config.h):\n";
    WriteConfigLines(options, out, "  ");
}

void WriteCsvPreamble(const RunOptions& options, std::ostream& csv) {
    csv << "# controller run config — parsers skip '#' lines\n";
    csv << "# log_format = 5 (compiled)\n";
    WriteConfigLines(options, csv, "# ");
}
