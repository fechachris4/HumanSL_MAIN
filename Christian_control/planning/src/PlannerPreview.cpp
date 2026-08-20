#include "BridgeMain.h"

#include "PlannerRuntime.h"
#include "WorldCartesianTrajectoryWire.h"

int RunBridge(const std::vector<std::string>& args, std::ostream& targets,
              std::ostream& diagnostics)
{
    PlannerSolveResult result = SolvePlan(args, diagnostics);
    if (result.trajectory)
        targets << FormatWorldCartesianTrajectoryBlock(*result.trajectory);
    return result.exit_code;
}
