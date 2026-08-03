// Runtime arguments that do not affect controller behaviour.

#ifndef HUMANSL_MASTERS_PROJECT_2025_OPTIONS_H
#define HUMANSL_MASTERS_PROJECT_2025_OPTIONS_H

#include <ostream>
#include <string>

// The output filename is the only runtime-selectable value.
struct RunOptions {
    std::string log_file;    // empty -> timestamped default (Record)
    // Reactive-pose law only (ignored by resolved-rate); kp above doubles as
    // its position P gain.
};

// Parse the log-output convenience only; all controller settings belong to
// app/Config.h and unknown arguments are hard errors.
RunOptions ParseRunOptions(int argc, char** argv);

// The startup echo: every runtime value with its source, plus the
// compile-time-only safety facts (stop_on_fault, the derived qdot clip).
void EchoConfig(const RunOptions& options, std::ostream& out);

// The same information as '#'-prefixed preamble lines, written above the
// CSV header row so every data file is self-describing (F3). Parsers skip
// '#' lines; the plot scripts loud-fail on files without a preamble.
void WriteCsvPreamble(const RunOptions& options, std::ostream& csv);

#endif // HUMANSL_MASTERS_PROJECT_2025_OPTIONS_H
