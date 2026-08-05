#pragma once
#include <iosfwd>
#include <string>
#include <vector>

// Runs one plan: resolves the start state, solves, validates, and writes
// one target line per waypoint to `targets` (the stream the operator
// connects to the controller's stdin). Diagnostics go to `diagnostics`
// (stderr in main). Returns a process exit code: 0 emitted, 1 bad
// arguments, 2 start-state unavailable, 3 solve failed, 4 validation
// rejected the plan. NOTHING is written to `targets` on any non-zero path.
// During the solve, the legacy optimizer's own stdout chatter is
// redirected into `diagnostics` so it can never land in `targets`.
int RunBridge(const std::vector<std::string>& args, std::ostream& targets,
              std::ostream& diagnostics);
