// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

// Holds config/planner.yaml to two properties, which together are what the
// file buys over compiled constants:
//
//  1. The checked-in values equal the compiled defaults. Without this the
//     struct defaults in TrajectoryOptimization.h and PlannerConfig.h drift
//     away from the file, and "what does the planner actually do" stops
//     having one answer.
//  2. A malformed file is refused, naming the problem. An unknown key is
//     the case that matters most: it is what a typo looks like, and a
//     loader that ignores it turns "I changed it" into "and nothing
//     happened".

#include <cassert>
#include <cstdio>
#include <fstream>
#include <string>

#include "PlannerConfig.h"

namespace {

// Writes `text` to a scratch file and returns the loader's complaint, or
// "" if it accepted the file. Written beside the test binary rather than a
// fixed temp name so two concurrent ctest runs cannot collide.
std::string RejectionFor(const std::string& scratch_path, const std::string& text) {
    {
        std::ofstream file(scratch_path);
        file << text;
    }
    try {
        LoadPlannerConfig(scratch_path);
    } catch (const std::exception& error) {
        return error.what();
    }
    return "";
}

const char* kValidConfig = R"(
motion:
  nominal_speed_mps: 0.05
  min_duration_s: 4.0
  waypoints: 10
obstacles:
  epsilon_dist_m: 0.05
  collision_sigma: 0.0005
smoothness:
  qc_scale: 1.0
goal:
  position_sigma_xyz: [0.01, 0.1, 0.01]
  rotation_sigma_rpy: [0.01, 0.01, 0.01]
solver:
  max_iterations: 1000
path_following:
  position_prior_sigma_m: 0.005
  rotation_prior_sigma_rad: 0.01
  maximum_planning_error_m: 0.005
  maximum_orientation_error_rad: 0.1
  validation_dt_s: 0.002
  approach_velocity_fraction: 0.3
  approach_min_duration_s: 2.0
  approach_waypoints: 5
  max_chord_error_m: 0.001
seeding:
  ik_seed: 20260807
  randomised: false
)";

}  // namespace

int main(int argc, char** argv) {
    assert(argc == 3 &&
           "usage: test_planner_config <planner.yaml> <scratch_dir>");
    const std::string scratch = std::string(argv[2]) + "/planner_config_scratch.yaml";

    // ---- 1. the checked-in file matches the compiled defaults -----------
    const PlannerConfig config = LoadPlannerConfig(argv[1]);
    const MotionConfig motion_defaults;
    const OptimizerTuning tuning_defaults;

    assert(config.motion.nominal_speed_mps == motion_defaults.nominal_speed_mps);
    assert(config.motion.min_duration_s == motion_defaults.min_duration_s);
    assert(config.motion.waypoints == motion_defaults.waypoints);
    assert(config.optimizer.epsilon_dist_m == tuning_defaults.epsilon_dist_m);
    assert(config.optimizer.collision_sigma == tuning_defaults.collision_sigma);
    assert(config.optimizer.qc_scale == tuning_defaults.qc_scale);
    assert(config.optimizer.goal_position_sigma_xyz ==
           tuning_defaults.goal_position_sigma_xyz);
    assert(config.optimizer.goal_rotation_sigma_rpy ==
           tuning_defaults.goal_rotation_sigma_rpy);
    assert(config.optimizer.max_iterations == tuning_defaults.max_iterations);

    // The path-following section, same contract: the checked-in file must
    // equal the compiled defaults, so the YAML and the code cannot drift.
    const PathFollowingConfig path_defaults;
    assert(config.path_following.position_prior_sigma_m ==
           path_defaults.position_prior_sigma_m);
    assert(config.path_following.rotation_prior_sigma_rad ==
           path_defaults.rotation_prior_sigma_rad);
    assert(config.path_following.maximum_planning_error_m ==
           path_defaults.maximum_planning_error_m);
    assert(config.path_following.maximum_orientation_error_rad ==
           path_defaults.maximum_orientation_error_rad);
    assert(config.path_following.validation_dt_s == path_defaults.validation_dt_s);
    assert(config.path_following.approach_velocity_fraction ==
           path_defaults.approach_velocity_fraction);
    assert(config.path_following.approach_min_duration_s ==
           path_defaults.approach_min_duration_s);
    assert(config.path_following.approach_waypoints == path_defaults.approach_waypoints);
    assert(config.path_following.max_chord_error_m == path_defaults.max_chord_error_m);

    // Determinism is the DEFAULT, and the effective seed must equal the
    // configured one when not randomised — otherwise the report would name
    // a seed that did not produce the plan.
    const SeedingConfig seeding_defaults;
    assert(config.seeding.ik_seed == seeding_defaults.ik_seed);
    assert(config.seeding.randomised == false &&
           "the checked-in config must plan deterministically by default");
    assert(config.effective_ik_seed == config.seeding.ik_seed);

    // The prior SIGMA and the pass/fail REQUIREMENT are different numbers
    // with different meanings; nothing may quietly alias them together.
    assert(&config.path_following.position_prior_sigma_m !=
           &config.path_following.maximum_planning_error_m);

    assert(config.source_fnv1a64 != 0 && "digest must be computed");
    std::printf("checked-in planner.yaml matches the compiled defaults\n");

    // The echo must actually carry the values, or a run log records nothing
    // useful about what produced its trajectory.
    const std::string echo = EffectiveConfigText(config);
    assert(echo.find("nominal_speed_mps") != std::string::npos);
    assert(echo.find("digest") != std::string::npos);
    assert(echo.find("path_following.maximum_planning_error_m") != std::string::npos &&
           "the GATE must appear in the run's echoed config");
    assert(echo.find("EFFECTIVE_IK_SEED") != std::string::npos &&
           "every report must record the seed that produced the plan");

    // ---- 2. malformed files are refused, naming the problem -------------
    // A sanity check first: the fixture itself must load, so the rejections
    // below are caused by what each case changes and nothing else.
    assert(RejectionFor(scratch, kValidConfig).empty() &&
           "the valid fixture must load");

    // An unknown key — the typo case: `nominal_speed` for
    // `nominal_speed_mps`. The message must name both halves, so one read
    // says what was written and what was meant.
    {
        std::string text(kValidConfig);
        const std::string line = "  nominal_speed_mps: 0.05\n";
        const std::size_t at = text.find(line);
        assert(at != std::string::npos);
        text.replace(at, line.size(), "  nominal_speed: 0.05\n");
        const std::string rejection = RejectionFor(scratch, text);
        assert(!rejection.empty() && "an unknown key must be refused");
        assert(rejection.find("nominal_speed_mps") != std::string::npos);
        assert(rejection.find("nominal_speed:") != std::string::npos ||
               rejection.find("nominal_speed]") != std::string::npos);
        std::printf("unknown key rejected: %s\n", rejection.c_str());
    }

    // A missing key, so nothing can silently fall back to a default.
    {
        std::string text(kValidConfig);
        const std::size_t at = text.find("  waypoints: 10\n");
        assert(at != std::string::npos);
        text.erase(at, std::string("  waypoints: 10\n").size());
        const std::string rejection = RejectionFor(scratch, text);
        assert(!rejection.empty() && "a missing key must be refused");
        assert(rejection.find("waypoints") != std::string::npos);
        std::printf("missing key rejected: %s\n", rejection.c_str());
    }

    // Out of range: the ten-times-too-fast typo the speed rail exists for.
    {
        std::string text(kValidConfig);
        const std::size_t at = text.find("0.05\n");
        assert(at != std::string::npos);
        text.replace(at, std::string("0.05\n").size(), "5.0\n");
        const std::string rejection = RejectionFor(scratch, text);
        assert(!rejection.empty() && "an out-of-range speed must be refused");
        assert(rejection.find("nominal_speed_mps") != std::string::npos);
        std::printf("out-of-range value rejected: %s\n", rejection.c_str());
    }

    // A wrong-length vector, which would otherwise reach Eigen as garbage.
    {
        std::string text(kValidConfig);
        const std::size_t at = text.find("[0.01, 0.1, 0.01]");
        assert(at != std::string::npos);
        text.replace(at, std::string("[0.01, 0.1, 0.01]").size(), "[0.01, 0.1]");
        const std::string rejection = RejectionFor(scratch, text);
        assert(!rejection.empty() && "a short vector must be refused");
        assert(rejection.find("position_sigma_xyz") != std::string::npos);
        std::printf("wrong-length vector rejected: %s\n", rejection.c_str());
    }

    // A file that does not exist must say so rather than yielding defaults.
    {
        bool threw = false;
        try {
            LoadPlannerConfig(std::string(argv[2]) + "/no_such_planner_config.yaml");
        } catch (const std::exception&) {
            threw = true;
        }
        assert(threw && "a missing file must be an error, not a default config");
    }

    std::printf("test_planner_config: PASSED\n");
    return 0;
}
