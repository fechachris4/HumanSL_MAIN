//
// Hardware-free tests for the controller's command-line
// contract (MainArgs.h). No Kortex, no Pinocchio, no robot — this is pure
// string parsing, and it must never touch the real `controller` binary:
// building the controller executable is fine, but running it
// (even just to probe argument parsing) is a hardware-affecting action this
// test suite must not take.
//

#include <iostream>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include "MainArgs.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    bool Throws(const std::vector<std::string>& args)
    {
        try {
            (void)ParseMainArgs(args);
            return false;
        } catch (const std::invalid_argument&) {
            return true;
        }
    }
} // namespace

int main()
{
    // --arm and --mount are both required: no flags at all, each alone,
    // and every other flag without them.
    Check(Throws({}), "no arguments refuses (missing --arm)");
    Check(Throws({"--log", "foo.csv"}), "--log alone refuses (missing --arm)");
    Check(Throws({"--arm", "right"}), "--arm alone refuses (missing --mount)");
    Check(Throws({"--mount", "fixed"}), "--mount alone refuses (missing --arm)");

    // The three valid arm values, each round-tripping into ParsedMainArgs,
    // and the defaults: planning on, recording on, identity fixed pose.
    for (const std::string arm : {"right", "left", "both"}) {
        const ParsedMainArgs parsed = ParseMainArgs({"--arm", arm, "--mount", "fixed"});
        Check(parsed.arm == arm, "--arm " + arm + " is accepted and recorded");
        Check(parsed.log_file.empty(), "--arm " + arm + " alone leaves log_file empty");
        Check(parsed.mount == "fixed", "--mount fixed recorded");
        Check(parsed.plan && parsed.record, "planning and recording default on");
        const std::array<double, 7> identity{0, 0, 0, 0, 0, 0, 1};
        Check(parsed.fixed_world_t_mount == identity,
              "fixed pose defaults to identity (world = mount)");
    }

    // An unrecognized --arm value is refused, not silently accepted.
    Check(Throws({"--arm", "up", "--mount", "fixed"}), "--arm up (not right/left/both) refuses");
    Check(Throws({"--arm"}), "--arm with no following value refuses");

    // --mount accepts exactly fixed and vicon.
    Check(ParseMainArgs({"--arm", "right", "--mount", "vicon"}).mount == "vicon",
          "--mount vicon is accepted and recorded");
    Check(Throws({"--arm", "right", "--mount", "gps"}),
          "--mount gps (not fixed/vicon) refuses");

    // --plan / --record take exactly on or off.
    {
        const ParsedMainArgs parsed = ParseMainArgs(
            {"--arm", "right", "--mount", "fixed", "--plan", "off", "--record", "off"});
        Check(!parsed.plan, "--plan off recorded");
        Check(!parsed.record, "--record off recorded");
    }
    Check(Throws({"--arm", "right", "--mount", "fixed", "--plan", "maybe"}),
          "--plan maybe refuses");
    Check(Throws({"--arm", "right", "--mount", "fixed", "--record", "1"}),
          "--record 1 refuses (on/off only)");

    // --fixed-pose: 7 finite numbers, quaternion near unit, fixed mount only.
    {
        const ParsedMainArgs parsed = ParseMainArgs(
            {"--arm", "right", "--mount", "fixed", "--fixed-pose",
             "0.1", "-0.2", "0.3", "0", "0", "0", "1"});
        Check(parsed.fixed_world_t_mount[0] == 0.1 &&
                  parsed.fixed_world_t_mount[1] == -0.2 &&
                  parsed.fixed_world_t_mount[2] == 0.3 &&
                  parsed.fixed_world_t_mount[6] == 1.0,
              "--fixed-pose values recorded");
    }
    Check(Throws({"--arm", "right", "--mount", "vicon", "--fixed-pose",
                  "0", "0", "0", "0", "0", "0", "1"}),
          "--fixed-pose with --mount vicon refuses");
    Check(Throws({"--arm", "right", "--mount", "fixed", "--fixed-pose",
                  "0", "0", "0", "0", "0", "0"}),
          "--fixed-pose with only 6 numbers refuses");
    Check(Throws({"--arm", "right", "--mount", "fixed", "--fixed-pose",
                  "0", "0", "zebra", "0", "0", "0", "1"}),
          "--fixed-pose with a non-number refuses");
    Check(Throws({"--arm", "right", "--mount", "fixed", "--fixed-pose",
                  "0", "0", "0", "0.5", "0.5", "0.5", "0.9"}),
          "--fixed-pose with a non-unit quaternion refuses");
    {
        // A quaternion within tolerance is normalised exactly, so the
        // rotation built from it downstream is orthonormal.
        const ParsedMainArgs parsed = ParseMainArgs(
            {"--arm", "right", "--mount", "fixed", "--fixed-pose",
             "0", "0", "0", "0", "0", "0", "1.0005"});
        Check(parsed.fixed_world_t_mount[6] == 1.0,
              "a near-unit quaternion is normalised");
    }

    // --log is accepted and recorded for a single-arm run, either order.
    {
        const ParsedMainArgs parsed = ParseMainArgs({"--arm", "right", "--mount", "fixed", "--log", "run.csv"});
        Check(parsed.arm == "right", "--arm right --log <file>: arm recorded");
        Check(parsed.log_file == "run.csv", "--arm right --log <file>: log_file recorded");
    }
    {
        const ParsedMainArgs parsed = ParseMainArgs({"--log", "run.csv", "--arm", "left", "--mount", "vicon"});
        Check(parsed.arm == "left", "--log <file> --arm left: arm recorded regardless of order");
        Check(parsed.log_file == "run.csv",
              "--log <file> --arm left: log_file recorded regardless of order");
    }
    Check(Throws({"--arm", "right", "--mount", "fixed", "--log"}), "--log with no following value refuses");

    // --log is refused with --arm both: one filename cannot name two arms'
    // logs (Main.cpp — RunOneArm prefixes each arm's default file instead).
    Check(Throws({"--arm", "both", "--mount", "fixed", "--log", "run.csv"}),
          "--log is refused when --arm is both");
    // Order must not matter for that refusal either.
    Check(Throws({"--log", "run.csv", "--mount", "fixed", "--arm", "both"}),
          "--log before --arm both is still refused");

    // Unknown flags are refused, not silently ignored.
    Check(Throws({"--arm", "right", "--mount", "fixed", "--bogus"}), "an unrecognized flag refuses");

    // --planner: default current, baseline requires the frozen binary's
    // path, and a path without the mode (or a bogus mode) is refused —
    // an argument nothing would read must not parse.
    {
        const ParsedMainArgs parsed =
            ParseMainArgs({"--arm", "left", "--mount", "fixed"});
        Check(parsed.planner == "current" && parsed.baseline_bridge.empty(),
              "planner defaults to current with no baseline binary");
    }
    {
        const ParsedMainArgs parsed = ParseMainArgs(
            {"--arm", "left", "--mount", "fixed", "--planner", "baseline",
             "--baseline-bridge", "/opt/baseline/planner_bridge"});
        Check(parsed.planner == "baseline" &&
                  parsed.baseline_bridge == "/opt/baseline/planner_bridge",
              "--planner baseline with --baseline-bridge is recorded");
    }
    Check(Throws({"--arm", "left", "--mount", "fixed", "--planner", "baseline"}),
          "--planner baseline without --baseline-bridge refuses");
    Check(Throws({"--arm", "left", "--mount", "fixed",
                  "--baseline-bridge", "/opt/x"}),
          "--baseline-bridge without --planner baseline refuses");
    Check(Throws({"--arm", "left", "--mount", "fixed", "--planner", "gpmp2"}),
          "an unknown --planner value refuses");

    if (failures == 0) {
        std::cout << "all MainArgs tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
