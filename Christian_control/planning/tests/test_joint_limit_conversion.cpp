// Guard on createJointLimits' degree-to-radian boundary
// (optimisation/utils.cpp): joint_limits.yaml's position_limits
// are authored in degrees (2026-08-18), converted to radians exactly once
// on load, while velocity_limits stay untouched (already rad/s). Without
// this test nothing asserts the conversion happened exactly once — a
// missing conversion or a double conversion would both still produce a
// finite JointLimits the rest of the planner would silently accept.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdio>

#include "utils.h"

int main(int argc, char** argv) {
    assert(argc == 2 && "usage: test_joint_limit_conversion <joint_limits.yaml>");
    const auto [pos_limits, vel_limits] = createJointLimits(argv[1]);

    // --- A known degree limit becomes the correct internal radian value.
    // actuator_2 (index 1) is documented in joint_limits.yaml as +-128.9
    // deg, matching control/Config.h's kJointUpperDeg[1]. If the
    // conversion were applied twice, this would come out near 2.25 rad
    // instead of ~2.25 already being 128.9 deg -- double-converting
    // 128.9 deg once more would land near 0.0392 rad, off by two orders
    // of magnitude, so this bound alone also proves "exactly once".
    const double expected_actuator2_rad = 128.9 * M_PI / 180.0;
    assert(std::abs(pos_limits.upper(1) - expected_actuator2_rad) < 1e-9 &&
           "actuator_2 upper position limit must equal 128.9 deg in radians");
    assert(std::abs(pos_limits.lower(1) + expected_actuator2_rad) < 1e-9 &&
           "actuator_2 lower position limit must equal -128.9 deg in radians");

    const double expected_actuator4_rad = 147.8 * M_PI / 180.0;
    assert(std::abs(pos_limits.upper(3) - expected_actuator4_rad) < 1e-9 &&
           "actuator_4 upper position limit must equal 147.8 deg in radians");

    const double expected_actuator6_rad = 120.3 * M_PI / 180.0;
    assert(std::abs(pos_limits.upper(5) - expected_actuator6_rad) < 1e-9 &&
           "actuator_6 upper position limit must equal 120.3 deg in radians");

    // --- Continuous joints (1/3/5/7, indices 0/2/4/6) remain unbounded:
    // `continuous: true` in the yaml, no lower_limit/upper_limit fields,
    // and createJointLimits must still hand back the same +-1e20 rad
    // sentinel every existing consumer (JointLimitFactorVector, the
    // grid-coverage envelope) already treats as "no limit".
    for (int continuous_index : {0, 2, 4, 6}) {
        assert(pos_limits.lower(continuous_index) < -1e10 &&
               "continuous joint must report an unbounded lower position limit");
        assert(pos_limits.upper(continuous_index) > 1e10 &&
               "continuous joint must report an unbounded upper position limit");
    }

    // --- velocity_limits are unaffected by the position conversion: still
    // the raw rad/s figures joint_limits.yaml has always stored, including
    // for continuous joints, which DO have a velocity bound.
    assert(std::abs(vel_limits.lower(0) - (-1.3265)) < 1e-9 &&
           "actuator_1 velocity limit must stay untouched (rad/s, not converted)");
    assert(std::abs(vel_limits.upper(1) - 1.3265) < 1e-9 &&
           "actuator_2 velocity limit must stay untouched (rad/s, not converted)");

    std::printf("joint limit conversion: bounded position limits convert "
                "degrees->radians exactly once, continuous joints stay "
                "unbounded, velocity limits untouched\n");
    return 0;
}
