#include <cassert>
#include "Waypoints.h"
#include "Targets.h"  // basic_control — ParsePoseTarget round-trip

int main(int argc, char** argv) {
    assert(argc == 2);
    const PlannerModel model = LoadPlannerModel(argv[1]);

    // Straight-line joint path: q2 sweeps 0 -> 1 rad over 21 support states.
    std::vector<gtsam::Vector> path;
    for (int i = 0; i <= 20; ++i) {
        gtsam::Vector q = gtsam::Vector::Zero(7);
        q(1) = 0.05 * i;
        path.push_back(q);
    }
    assert(!ValidateJointPath(path).has_value());

    const auto waypoints = SampleCartesianWaypoints(model, path);
    assert(!waypoints.empty() && waypoints.size() <= 8);
    for (std::size_t i = 1; i < waypoints.size(); ++i)
        assert((waypoints[i] - waypoints[i - 1]).norm() >= 0.05 - 1e-9);
    Eigen::Matrix<double, 7, 1> q_last(path.back());
    assert((waypoints.back() - ToolPositionInBaseLink(model, q_last)).norm() < 1e-9);

    // Limit violation: q4 at 150 deg exceeds the 145 deg warn limit.
    auto bad = path;
    bad.back()(3) = 150.0 * M_PI / 180.0;
    assert(ValidateJointPath(bad).has_value());

    // Every emitted line must be accepted verbatim by the controller parser.
    for (const auto& waypoint : waypoints) {
        std::string error;
        assert(ParsePoseTarget(FormatTargetLine(waypoint), error).has_value());
    }
    return 0;
}
