//
// The posture-preference field of WorldCartesianTrajectory: contract rules
// only, no robot model. Posture is either absent from every point or
// present and finite on every point — a trajectory that flips mid-stream
// would make the controller's null-space attractor appear and vanish
// between cycles.
//

#include <cstdio>
#include <limits>
#include <string>

#include "WorldCartesianTrajectory.h"

namespace {

int failures = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++failures;
    }
}

// The smallest trajectory the validator accepts: two points, second one
// arrival-eligible and stationary.
WorldCartesianTrajectory MinimalValid() {
    WorldCartesianTrajectory trajectory;
    WorldCartesianTrajectoryPoint first;
    first.t_from_start_s = 0.0;
    WorldCartesianTrajectoryPoint last;
    last.t_from_start_s = 1.0;
    last.arrival_eligible = true;
    trajectory.points = {first, last};
    return trajectory;
}

}  // namespace

int main() {
    {
        const WorldCartesianTrajectory trajectory = MinimalValid();
        Check(!ValidateWorldCartesianTrajectory(trajectory).has_value(),
              "posture-free trajectory stays valid (slice-1 compatibility)");
    }
    {
        WorldCartesianTrajectory trajectory = MinimalValid();
        for (auto& point : trajectory.points) {
            point.has_posture = true;
            point.posture_rad.setConstant(0.5);
        }
        Check(!ValidateWorldCartesianTrajectory(trajectory).has_value(),
              "posture on every point is valid");
    }
    {
        WorldCartesianTrajectory trajectory = MinimalValid();
        trajectory.points.back().has_posture = true;
        Check(ValidateWorldCartesianTrajectory(trajectory).has_value(),
              "posture on only some points is rejected");
    }
    {
        WorldCartesianTrajectory trajectory = MinimalValid();
        for (auto& point : trajectory.points) point.has_posture = true;
        trajectory.points.front().posture_rad(3) =
            std::numeric_limits<double>::quiet_NaN();
        Check(ValidateWorldCartesianTrajectory(trajectory).has_value(),
              "non-finite posture is rejected");
    }

    if (failures == 0)
        std::printf("test_trajectory_posture: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
