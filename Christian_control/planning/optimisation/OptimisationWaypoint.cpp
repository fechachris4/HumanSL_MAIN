#include "OptimisationWaypoint.h"

#include <cmath>

bool WaypointTimesAreMonotonic(const std::vector<OptimisationWaypoint>& waypoints,
                               std::size_t* first_bad_index) {
    const auto fail = [&](std::size_t index) {
        if (first_bad_index) *first_bad_index = index;
        return false;
    };
    if (waypoints.empty()) return fail(0);
    if (!std::isfinite(waypoints[0].time_s) || waypoints[0].time_s != 0.0)
        return fail(0);
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        if (!std::isfinite(waypoints[i].time_s) ||
            waypoints[i].time_s <= waypoints[i - 1].time_s)
            return fail(i);
    }
    return true;
}
