#include <cstdio>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "BridgeMain.h"

// Replays the 2026-08-23 left-arm run that exposed two traced-circle
// defects (runs/2026-08-23/session_153439): the measured start
// configuration and the circle geometry the arm actually traced.
//
// 1. The `--circle` centre and normal must be parsed in the order given.
//    The old construction `Eigen::Vector3d(ParseDouble(next()), ...)` left
//    the evaluation order to the compiler, and GCC filled the vector
//    back-to-front: the run's requested vertical circle (normal +x) was
//    planned as a horizontal one (normal +z) around a swapped centre. The
//    declaration line now echoes centre and normal so a session log —
//    and this test — can see the geometry the planner actually built.
//
// 2. A traced plan must end in the configuration its own IK walk reached.
//    Pinning the independently solved terminal branch forced a 150.6 deg
//    joint-space bridge into the final samples, which validation could
//    only cure by stretching the whole 14.1 s plan to 100.1 s. With the
//    walk's own end configuration as the terminal, the requested pace
//    needs at most a mild duration repair, never a many-fold stretch.

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

    // Reads the number between `prefix` and the next " s" in `text`, or
    // a negative value when the prefix never appears.
    double NumberAfter(const std::string& text, const std::string& prefix)
    {
        const std::size_t at = text.find(prefix);
        if (at == std::string::npos) return -1.0;
        return std::atof(text.c_str() + at + prefix.size());
    }

} // namespace

int main(int argc, char** argv)
{
    Check(argc == 2, "usage: test_traced_circle dh_flange.yaml");
    if (argc != 2) return 1;

    const std::filesystem::path runs_root =
        std::filesystem::temp_directory_path() / "humansl_test_traced_circle";
    std::filesystem::create_directories(runs_root);

    std::ostringstream targets;
    std::ostringstream diagnostics;
    const int code = RunBridge(
        {"--arm", "left", "--dh", argv[1],
         "--start-deg", "271.298", "81.8539", "331.742", "21.109",
         "38.0453", "31.8926", "216.611",
         "--circle", "0.5213", "0.686", "0.39", "0.15", "0", "0", "1", "12",
         "--circle-orientation", "fixed",
         "--goal-rpy-rad", "1.5707963267948966", "0", "1.5707963267948966",
         "--world-mount-pose-m-quat", "0", "0", "0", "0", "0", "0", "1",
         "--vicon-sequence", "1", "--trajectory-id", "1",
         "--planner-config", "../config/planner.yaml",
         "--joint-limits", "../config/joint_limits.yaml",
         "--runs-root", runs_root.string()},
        targets, diagnostics);
    const std::string log = diagnostics.str();

    Check(code == 0, "traced circle plan is emitted (exit 0), diagnostics:\n" + log);
    Check(!targets.str().empty(), "traced circle writes a trajectory block");

    // Geometry declaration: the parsed centre and normal, in the order the
    // command line gave them.
    Check(log.find("centre 0.5213 0.686 0.39 m") != std::string::npos,
          "declaration echoes the circle centre in argument order");
    Check(log.find("normal 0 0 1") != std::string::npos,
          "declaration echoes the circle normal in argument order");

    // Pace: approach plus one 12 s lap. A mild dynamic repair is
    // acceptable; the 7x whole-plan stretch the terminal pin forced is not.
    const double duration_s = NumberAfter(log, "result: REACHED, duration ");
    Check(duration_s > 0.0, "plan summary reports a REACHED duration");
    Check(duration_s < 30.0,
          "traced circle keeps the requested pace (got " +
              std::to_string(duration_s) + " s, limit 30 s)");

    std::filesystem::remove_all(runs_root);
    if (failures == 0)
        std::puts("test_traced_circle: all assertions passed");
    return failures == 0 ? 0 : 1;
}
