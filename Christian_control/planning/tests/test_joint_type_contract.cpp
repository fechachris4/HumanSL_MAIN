#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "Config.h"
#include "utils.h"

namespace {
int failures = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++failures;
    }
}
}  // namespace

int main() {
    const PlannerJointLimits limits = createJointLimits("../config/joint_limits.yaml");
    const YAML::Node config = YAML::LoadFile("../config/joint_limits.yaml");
    const double acceleration_fraction =
        config["margins"]["acceleration_planner_fraction"].as<double>();

    Check(std::abs(limits.hardware_acceleration_rad_s2.upper(0) - 5.2) < 1e-12 &&
              std::abs(limits.hardware_acceleration_rad_s2.upper(3) - 5.2) < 1e-12 &&
              std::abs(limits.hardware_acceleration_rad_s2.upper(4) - 10.0) < 1e-12 &&
              std::abs(limits.hardware_acceleration_rad_s2.upper(6) - 10.0) < 1e-12,
          "hardware acceleration uses the Gen3 1-4/5-7 split");
    Check((limits.effective_acceleration_rad_s2.upper -
           acceleration_fraction * limits.hardware_acceleration_rad_s2.upper)
              .cwiseAbs()
              .maxCoeff() < 1e-12,
          "effective acceleration equals fraction times hardware acceleration");

    Check(config::limits::kBoundedMask ==
              std::array<double, 7>{0, 1, 0, 1, 0, 1, 0},
          "generated joint-type mask matches the seven-DoF Gen3 contract");

    for (std::size_t joint = 0; joint < config::limits::kBoundedMask.size(); ++joint) {
        if (!config::limits::kBoundedMask[joint]) {
            continue;
        }
        Check(config::limits::kPlannerLowerDeg[joint] >
                  config::limits::kPhysicalLowerDeg[joint],
              "planner lower bound is inside the physical lower bound");
        Check(config::limits::kPlannerUpperDeg[joint] <
                  config::limits::kPhysicalUpperDeg[joint],
              "planner upper bound is inside the physical upper bound");
        Check(config::limits::kControllerLowerDeg[joint] <=
                  config::limits::kPlannerLowerDeg[joint] &&
                  config::limits::kPlannerUpperDeg[joint] <=
                      config::limits::kControllerUpperDeg[joint],
              "planner bounds remain within controller bounds");
    }

    if (failures == 0)
        std::printf("test_joint_type_contract: all checks passed\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
