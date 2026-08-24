#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "MainArgs.h"

namespace {
int failures = 0;

void Check(bool condition, const char* description)
{
    if (!condition) {
        std::printf("FAIL: %s\n", description);
        ++failures;
    }
}

void CheckUnknownOption(const std::vector<std::string>& args,
                        const std::string& expected)
{
    try {
        (void)ParseMainArgs(args);
        Check(false, "retired option is rejected");
    } catch (const std::invalid_argument& error) {
        Check(error.what() == expected, "retired option reports unknown option");
    }
}
} // namespace

int main()
{
    const ParsedMainArgs normal = ParseMainArgs({
        "--arm", "right", "--mount", "fixed"});
    Check(normal.arm == "right" && normal.mount == "fixed" && normal.plan,
          "normal fixed-mount invocation parses with planning on");

    CheckUnknownOption({"--arm", "right", "--mount", "fixed",
                        "--planner", "current"},
                       "unknown option '--planner'");
    CheckUnknownOption({"--arm", "right", "--mount", "fixed",
                        "--baseline-bridge", "/x"},
                       "unknown option '--baseline-bridge'");

    return failures == 0 ? 0 : 1;
}
