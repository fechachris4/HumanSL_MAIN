//
// Options: the runtime configuration front-end. Precedence: CLI > TOML
// (--config <path>, explicit only — no auto-discovery) > compiled defaults
// (app/Config.h). Runtime keys are gains, thresholds, controller selection
// and the log filename — NEVER safety policy: config::kStopOnFault is
// compile-time only (F2, approved 2026-07-22).
// Decision record: docs/decisions/runtime-config.md
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_OPTIONS_H
#define HUMANSL_MASTERS_PROJECT_2025_OPTIONS_H

#include <map>
#include <ostream>
#include <string>

// The merged, effective configuration, with per-key source tracking
// ("compiled" / "toml" / "cli") for the startup echo and the CSV preamble.
struct EffectiveConfig {
    std::string controller = "resolved-rate"; // or "reactive-pose"
    double kp = 0.0;                       // filled from config:: in ParseOptions
    double dls_lambda = 0.0;
    // Reactive-pose law only (ignored by resolved-rate); kp above doubles as
    // its position P gain.
    double kp_rot = 0.0;
    double kd_pos = 0.0;
    double kd_rot = 0.0;
    double null_gain = 0.0;
    bool orientation_enabled = true;
    bool velocity_term_enabled = false;
    bool null_space_enabled = false;
    double following_error_limit_deg = 0.0;
    double arrival_tolerance_m = 0.0;
    int nonfinite_stop_cycles = 0;
    int saturation_stop_cycles = 0;
    int overrun_stop_cycles = 0;
    double overrun_factor = 0.0;
    std::string log_file; // empty -> timestamped default (Record)

    std::map<std::string, std::string> source; // key -> where its value came from
};

// Parse CLI + optional TOML; unknown flags and unknown TOML keys are hard
// errors (exit 2 — a typo must never silently fall back to a default).
// Flags: --controller <name>, --kp <v>, --log <file>, --config <path>, --help.
EffectiveConfig ParseOptions(int argc, char** argv);

// The startup echo: every runtime value with its source, plus the
// compile-time-only safety facts (stop_on_fault, the derived qdot clip).
void EchoConfig(const EffectiveConfig& cfg, std::ostream& out);

// The same information as '#'-prefixed preamble lines, written above the
// CSV header row so every data file is self-describing (F3). Parsers skip
// '#' lines; the plot scripts loud-fail on files without a preamble.
void WriteCsvPreamble(const EffectiveConfig& cfg, std::ostream& csv);

#endif // HUMANSL_MASTERS_PROJECT_2025_OPTIONS_H
