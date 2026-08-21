#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "BridgeMain.h"

namespace
{

    int failures = 0;
    int goal_file_number = 0;

    void Check(bool condition, const std::string& what)
    {
        if (!condition) {
            std::printf("FAIL: %s\n", what.c_str());
            ++failures;
        }
    }

    void CheckGoalFileBoxRejected(const std::string& box_key)
    {
        const std::filesystem::path path =
            std::filesystem::temp_directory_path() /
            ("humansl_retired_goal_box_" + std::to_string(goal_file_number++) + ".yaml");
        {
            std::ofstream file(path);
            file << "session_arms: right\n"
                 << "right:\n"
                 << "  frame: mount\n"
                 << "  goal: [0.5, 0.0, 0.4]\n"
                 << "  " << box_key << ":\n"
                 << "    center: [0.0, 0.0, 0.0]\n"
                 << "    half_extent: [0.1, 0.1, 0.1]\n";
        }

        std::ostringstream targets;
        std::ostringstream diagnostics;
        const int code =
            RunBridge({"--arm", "right", "--goal-file", path.string()}, targets, diagnostics);
        std::filesystem::remove(path);

        Check(code == 1, box_key + " goal box rejected");
        Check(targets.str().empty(), box_key + " goal box emits no trajectory");
        Check(diagnostics.str().find("box is retired — edit obstacles.scene in planner.yaml") !=
                  std::string::npos,
              box_key + " goal box gets migration guidance");
        Check(diagnostics.str().rfind("error: goal file ", 0) == 0,
              box_key + " goal box rejected before model or solve");
    }

} // namespace

int main()
{
    std::ostringstream targets;
    std::ostringstream diagnostics;
    const int code = RunBridge({"--box", "0", "0", "0", "1", "1", "1"}, targets, diagnostics);

    Check(code == 1, "retired flag rejected");
    Check(targets.str().empty(), "no trajectory emitted");
    Check(diagnostics.str().find("--box is retired — edit obstacles.scene in planner.yaml") !=
              std::string::npos,
          "retired flag gets migration guidance");

    CheckGoalFileBoxRejected("box");
    CheckGoalFileBoxRejected("\"box\"");
    CheckGoalFileBoxRejected("'box'");

    if (failures == 0)
        std::puts("test_bridge_args: all assertions passed");
    return failures == 0 ? 0 : 1;
}
