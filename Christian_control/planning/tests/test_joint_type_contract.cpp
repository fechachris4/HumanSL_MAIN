//
// The planner's joint-type contract, and which layer's limits this gate
// enforces. Both are read from config::limits — generated at build time
// from planning/config/joint_limits.yaml, which matches the URDF's
// continuous/revolute declarations — so nothing here restates a fact about
// the robot. If the validator and the robot model ever disagree again,
// these checks fail.
//
// A Kinova Gen3's joints 1/3/5/7 rotate continuously: no mechanical stop
// exists, so no absolute angular bound may reject a plan on them.
// Joints 2/4/6 are bounded and must lie inside the CONTROLLER's software
// stop — not the planner's stricter one, and not the physical limit.
// Excessive rotation on a continuous joint is a trajectory continuity,
// velocity or acceleration question, measured by ValidatePlannedPath and
// PathIk's joint-step metric, never a position limit.
//

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include "Config.h"
#include "PathValidation.h"
#include "utils.h"

namespace {

int failures = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++failures;
    }
}

// One support state, written in degrees because every limit in this area is
// quoted in degrees; ValidateJointPath consumes radians.
gtsam::Vector SupportDeg(const std::array<double, 7>& degrees) {
    gtsam::Vector q(7);
    for (int j = 0; j < 7; ++j) q(j) = degrees[j] * M_PI / 180.0;
    return q;
}

// A state with one joint moved and every other joint at zero.
gtsam::Vector OnlyJointAt(std::size_t joint, double degrees) {
    std::array<double, 7> q{};
    q[joint] = degrees;
    return SupportDeg(q);
}

}  // namespace

int main() {
    const PlannerJointLimits limits = createJointLimits("../config/joint_limits.yaml");
    Check(std::abs(limits.hardware_acceleration_rad_s2.upper(0) - 5.2) < 1e-12 &&
              std::abs(limits.hardware_acceleration_rad_s2.upper(3) - 5.2) < 1e-12 &&
              std::abs(limits.hardware_acceleration_rad_s2.upper(4) - 10.0) < 1e-12 &&
              std::abs(limits.hardware_acceleration_rad_s2.upper(6) - 10.0) < 1e-12,
          "hardware acceleration uses the Gen3 1-4/5-7 split");
    Check((limits.effective_acceleration_rad_s2.upper -
           limits.hardware_acceleration_rad_s2.upper).cwiseAbs().maxCoeff() < 1e-12,
          "effective acceleration equals fraction times hardware acceleration");

    for (std::size_t j = 0; j < config::limits::kBoundedMask.size(); ++j) {
        const std::string name = "joint " + std::to_string(j + 1);

        if (!config::limits::kBoundedMask[j]) {
            // Three full revolutions. Kinematically the same configuration
            // as zero, physically reachable, and not a position-limit
            // question. Whether GETTING there is sensible is a continuity
            // and speed question, checked elsewhere against the
            // trajectory, never here against an angle.
            Check(!ValidateJointPath({OnlyJointAt(j, 1080.0)}).has_value(),
                  "continuous " + name +
                      " at three revolutions must not be rejected");
            continue;
        }

        const double upper = config::limits::kControllerUpperDeg[j];
        const double lower = config::limits::kControllerLowerDeg[j];

        Check(!ValidateJointPath({OnlyJointAt(j, upper - 0.5)}).has_value(),
              name + " just inside its upper stop must be accepted");
        Check(!ValidateJointPath({OnlyJointAt(j, lower + 0.5)}).has_value(),
              name + " just inside its lower stop must be accepted");

        for (const double outside : {upper + 0.5, lower - 0.5}) {
            const std::optional<std::string> error =
                ValidateJointPath({OnlyJointAt(j, outside)});
            Check(error.has_value(), name + " past its stop must be rejected");
            if (error)
                Check(error->find(name) != std::string::npos,
                      "the rejection must name " + name);
        }

        // Which layer this gate belongs to. The planner stops earlier than
        // the controller by design (joint_limits.yaml's two margins), and
        // GPMP2's limit factors are soft, so a plan may legitimately settle
        // just outside the planner's stop. This gate asks the CONTROLLER's
        // question, so such a state must pass here.
        Check(config::limits::kPlannerUpperDeg[j] < upper,
              "the planner must stop before the controller does, " + name);
        Check(!ValidateJointPath(
                   {OnlyJointAt(j, config::limits::kPlannerUpperDeg[j] + 0.1)})
                   .has_value(),
              name + " outside the PLANNER stop but inside the controller's "
                     "must be accepted");
        // ... and the controller must still stop before the hardware does.
        Check(upper < config::limits::kPhysicalUpperDeg[j],
              "the controller must stop before the physical limit, " + name);
    }

    // The specific bound this test exists to keep deleted: +-360 deg on
    // every continuous joint at once.
    Check(!ValidateJointPath({SupportDeg({400.0, 0.0, 400.0, 0.0,
                                          400.0, 0.0, 400.0})}).has_value(),
          "continuous joints past 360 deg must not be rejected");

    // A violation anywhere in the path is a violation, not just at entry.
    Check(ValidateJointPath({SupportDeg({0, 0, 0, 0, 0, 0, 0}),
                             SupportDeg({0, 0, 0, 200.0, 0, 0, 0})}).has_value(),
          "a bounded-joint violation at a later support state is rejected");

    if (failures == 0)
        std::printf("test_joint_type_contract: all checks passed\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
