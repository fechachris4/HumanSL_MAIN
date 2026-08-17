#pragma once
#include <iosfwd>
#include <string>
#include <vector>

// Offline preview wrapper: resolves the start state, solves, validates, and
// writes one text CART_TRAJ block to `targets`. This stream is not connected
// to the production controller; production uses the typed runtime API.
// Diagnostics go to `diagnostics` (stderr in main). Returns a process exit code: 0 = targets emitted, OR
// --help/-h was requested (usage text goes to `diagnostics`, nothing to
// `targets` either way); 1 bad arguments (including a --box outside the
// SDF grid volume); 2 start-state unavailable; 3 solve failed; 4
// validation rejected the plan. NOTHING is written to `targets` on any
// non-zero path. During the solve, the legacy optimizer's own stdout
// chatter is redirected into `diagnostics` so it can never land in
// `targets`.
int RunBridge(const std::vector<std::string>& args, std::ostream& targets,
              std::ostream& diagnostics);
