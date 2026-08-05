// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
#include "PathValidation.h"
#include "Config.h"   // basic_control — pin ValidationLimitsDeg() against it

int main(int argc, char** argv) {
    assert(argc == 2);
    const PlannerModel model = LoadPlannerModel(argv[1]);
    (void)model;  // loaded to prove the generated DH YAML still parses

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

    // A continuous joint (j1) drifted a full revolution past straight-up
    // must be rejected here too, mirroring the controller's own ±360 deg
    // ingest gate (Targets.cpp's kContinuousJointBoundDeg) — this is the
    // scenario where GPMP2 seeds from an unwrapped start and settles a
    // solution a full turn away from the physical pose it planned.
    auto bad_j1 = path;
    bad_j1.back()(0) = 365.0 * M_PI / 180.0;
    assert(ValidateJointPath(bad_j1).has_value());

    // A path whose support states cluster near the goal (GPMP2's near-zero
    // terminal velocity does this) is still a valid path — validation is
    // per-state and must not care about spacing. Spacing mattered only to
    // the Cartesian waypoint sampler, retired in stage 2.
    auto clustered = path;
    for (int k = 1; k <= 5; ++k) {
        gtsam::Vector q = gtsam::Vector::Zero(7);
        q(1) = 1.0 + 0.0001 * k;
        clustered.push_back(q);
    }
    assert(!ValidateJointPath(clustered).has_value());

    return 0;
}
