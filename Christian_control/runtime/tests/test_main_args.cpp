//
// Hardware-free tests for the controller's --arm/--log command-line
// contract (MainArgs.h). No Kortex, no Pinocchio, no robot — this is pure
// string parsing, and it must never touch the real `controller` binary:
// building the controller executable is fine, but running it
// (even just to probe argument parsing) is a hardware-affecting action this
// test suite must not take.
//

#include <iostream>
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
    // --arm is required: no flags at all, and every other flag without it.
    Check(Throws({}), "no arguments refuses (missing --arm)");
    Check(Throws({"--log", "foo.csv"}), "--log alone refuses (missing --arm)");

    // The three valid values, each round-tripping into ParsedMainArgs.
    for (const std::string arm : {"right", "left", "both"}) {
        const ParsedMainArgs parsed = ParseMainArgs({"--arm", arm});
        Check(parsed.arm == arm, "--arm " + arm + " is accepted and recorded");
        Check(parsed.log_file.empty(), "--arm " + arm + " alone leaves log_file empty");
    }

    // An unrecognized --arm value is refused, not silently accepted.
    Check(Throws({"--arm", "up"}), "--arm up (not right/left/both) refuses");
    Check(Throws({"--arm"}), "--arm with no following value refuses");

    // --log is accepted and recorded for a single-arm run, either order.
    {
        const ParsedMainArgs parsed = ParseMainArgs({"--arm", "right", "--log", "run.csv"});
        Check(parsed.arm == "right", "--arm right --log <file>: arm recorded");
        Check(parsed.log_file == "run.csv", "--arm right --log <file>: log_file recorded");
    }
    {
        const ParsedMainArgs parsed = ParseMainArgs({"--log", "run.csv", "--arm", "left"});
        Check(parsed.arm == "left", "--log <file> --arm left: arm recorded regardless of order");
        Check(parsed.log_file == "run.csv",
              "--log <file> --arm left: log_file recorded regardless of order");
    }
    Check(Throws({"--arm", "right", "--log"}), "--log with no following value refuses");

    // --log is refused with --arm both: one filename cannot name two arms'
    // logs (Main.cpp — RunOneArm prefixes each arm's default file instead).
    Check(Throws({"--arm", "both", "--log", "run.csv"}),
          "--log is refused when --arm is both");
    // Order must not matter for that refusal either.
    Check(Throws({"--log", "run.csv", "--arm", "both"}),
          "--log before --arm both is still refused");

    // Unknown flags are refused, not silently ignored.
    Check(Throws({"--arm", "right", "--bogus"}), "an unrecognized flag refuses");

    if (failures == 0) {
        std::cout << "all MainArgs tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
