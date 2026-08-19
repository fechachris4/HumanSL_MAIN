//
// MainArgs — controller's command-line contract, split out of Main.cpp so
// it is hardware-free testable: it never talks to Kortex, never touches a
// file, and never calls std::exit (main() does that once, after catching
// the exception this throws — see BridgeMain.cpp's ParseArgs/RunBridge for
// the same pattern in planner_bridge).
//

#pragma once

#include <array>
#include <string>
#include <vector>

// The session's four independent choices, plus --log:
//   --arm <right|left|both>   required — which physical arm(s) to control
//   --mount <fixed|vicon>     required — where world_T_mount comes from.
//                             Same policy as --arm: every run states its
//                             world-pose source explicitly, no default.
//   --fixed-pose x y z qx qy qz qw
//                             optional, --mount fixed only: world_T_mount as
//                             translation (m) + quaternion. Omitted =
//                             identity (world ≡ mount, the bench case).
//   --plan <on|off>           default on — run the in-process planner worker
//   --record <on|off>         default on — write the run CSV
//   --log <file>              optional, refused with --arm both
struct ParsedMainArgs {
    std::string arm;      // "right", "left", or "both"
    std::string log_file; // empty = timestamped default
    std::string mount;    // "fixed" or "vicon"
    bool plan = true;
    bool record = true;
    // world_T_mount for --mount fixed: x y z (m), then quaternion x y z w.
    // Identity unless --fixed-pose was given. Meaningless with --mount vicon.
    std::array<double, 7> fixed_world_t_mount{0, 0, 0, 0, 0, 0, 1};
};

// args excludes argv[0]. Throws std::invalid_argument naming exactly what
// was wrong — main() turns that into the usage text and exit(2). The
// quaternion in --fixed-pose must be within 1e-3 of unit norm (it is then
// normalised exactly); a wilder value is a typo, not a rotation.
ParsedMainArgs ParseMainArgs(const std::vector<std::string>& args);
