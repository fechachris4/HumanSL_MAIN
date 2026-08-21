#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "BridgeMain.h"

namespace
{

    int failures = 0;

    void Check(bool condition, const std::string& what)
    {
        if (!condition) {
            std::printf("FAIL: %s\n", what.c_str());
            ++failures;
        }
    }

} // namespace

int main()
{
    std::ostringstream targets;
    std::ostringstream diagnostics;
    const int code = RunBridge({"--box", "0", "0", "0", "1", "1", "1"}, targets, diagnostics);

    Check(code == 1, "retired flag rejected");
    Check(targets.str().empty(), "no trajectory emitted");
    Check(diagnostics.str().find("unrecognized flag: '--box'") != std::string::npos,
          "diagnostic names flag");

    if (failures == 0)
        std::puts("test_bridge_args: all assertions passed");
    return failures == 0 ? 0 : 1;
}
