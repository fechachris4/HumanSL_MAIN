// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
#include <sstream>
#include "Waypoints.h"
#include "Targets.h"  // basic_control — ParsePoseTarget round-trip
#include "Config.h"   // basic_control — pin ValidationLimitsDeg() against it

int main(int argc, char** argv) {
    assert(argc == 2);
    const PlannerModel model = LoadPlannerModel(argv[1]);

    // The bridge's validation table must never drift from the controller's
    // real software stop (Config.h kJointSoftwareLimitDeg) — that stop, not
    // the wider firmware warn limit, is what the controller actually
    // enforces per-joint.
    const auto& validation_limits = ValidationLimitsDeg();
    for (int j = 0; j < 7; ++j)
        assert(validation_limits[static_cast<std::size_t>(j)] ==
               config::kJointSoftwareLimitDeg[static_cast<std::size_t>(j)]);

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

    // Limit violation: q4 at 150 deg exceeds the 145 deg software limit.
    auto bad = path;
    bad.back()(3) = 150.0 * M_PI / 180.0;
    assert(ValidateJointPath(bad).has_value());

    // Limit violation: q2 at 127 deg exceeds the tighter 126.9 deg software
    // stop (Table 39's 128.9 deg upper limit minus the 2 deg margin — the
    // bound that actually binds for j2, below its 130 deg warn limit).
    auto bad_j2 = path;
    bad_j2.back()(1) = 127.0 * M_PI / 180.0;
    assert(ValidateJointPath(bad_j2).has_value());

    // Every emitted line must be accepted verbatim by the controller parser.
    for (const auto& waypoint : waypoints) {
        std::string error;
        assert(ParsePoseTarget(FormatTargetLine(waypoint), error).has_value());
    }

    // GPMP2 trajectories cluster support states near the goal (near-zero
    // terminal velocity): append a tight tail cluster within a fraction of
    // a millimetre of the linear sweep's endpoint, then re-designate the
    // very last of those as the goal. The last kept intermediate from the
    // sweep now sits closer to the goal than min_spacing_m, so the goal
    // must be able to evict it and still satisfy the spacing invariant on
    // every consecutive pair, including the final one. max_count is raised
    // well past the state count so the cap itself cannot be the reason the
    // last two kept points end up far apart — only the spacing logic can.
    auto clustered = path;  // 21 states, ends at q2 = 1.0 rad.
    for (int k = 1; k <= 5; ++k) {
        gtsam::Vector q = gtsam::Vector::Zero(7);
        q(1) = 1.0 + 0.0001 * k;
        clustered.push_back(q);
    }
    assert(!ValidateJointPath(clustered).has_value());
    const auto clustered_waypoints =
        SampleCartesianWaypoints(model, clustered, /*max_count=*/100, 0.05);
    assert(!clustered_waypoints.empty() && clustered_waypoints.size() <= 100);
    for (std::size_t i = 1; i < clustered_waypoints.size(); ++i)
        assert((clustered_waypoints[i] - clustered_waypoints[i - 1]).norm() >=
               0.05 - 1e-9);
    Eigen::Matrix<double, 7, 1> q_clustered_last(clustered.back());
    assert((clustered_waypoints.back() -
            ToolPositionInBaseLink(model, q_clustered_last))
               .norm() < 1e-9);
    for (const auto& waypoint : clustered_waypoints) {
        std::string error;
        assert(ParsePoseTarget(FormatTargetLine(waypoint), error).has_value());
    }

    // 7-field orientation-carrying overload: identity quaternion round-trips
    // to "0 0 0 1" at 6 decimals, and the first three fields match the
    // 3-field overload exactly.
    {
        const Eigen::Vector3d p(0.4, 0.1, 0.3);
        const std::string line3 = FormatTargetLine(p);
        const std::string line7 = FormatTargetLine(p, Eigen::Quaterniond::Identity());
        assert(line7 == line3 + " 0.000000 0.000000 0.000000 1.000000");
        int field_count = 0;
        std::istringstream fields(line7);
        double value = 0.0;
        while (fields >> value) ++field_count;
        assert(field_count == 7);
    }
    return 0;
}
