//
// MainArgs — controller's command-line contract, split out of Main.cpp so
// it is hardware-free testable: it never talks to Kortex, never touches a
// file, and never calls std::exit (main() does that once, after catching
// the exception this throws — see BridgeMain.cpp's ParseArgs/RunBridge for
// the same pattern in planner_bridge).
//

#pragma once

#include <string>
#include <vector>

// --arm <right|left|both> (required) and --log <file> (optional, refused
// with --arm both — one filename can't name two arms' logs).
struct ParsedMainArgs {
    std::string arm;      // "right", "left", or "both"
    std::string log_file; // empty = timestamped default
};

// args excludes argv[0]. Throws std::invalid_argument naming exactly what
// was wrong (unknown flag, missing value, missing/invalid --arm, --log with
// --arm both) — main() turns that into the usage text and exit(2).
ParsedMainArgs ParseMainArgs(const std::vector<std::string>& args);
