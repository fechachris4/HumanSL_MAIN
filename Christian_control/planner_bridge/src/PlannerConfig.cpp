#include "PlannerConfig.h"

#include <random>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

[[noreturn]] void Fail(const std::string& message) {
    throw std::runtime_error("planner config: " + message);
}

std::string Join(const std::vector<std::string>& names) {
    std::ostringstream joined;
    for (std::size_t i = 0; i < names.size(); ++i)
        joined << (i ? ", " : "") << names[i];
    return joined.str();
}

// Every expected key present, and no others. Reporting BOTH lists in one
// message is what turns a typo into a one-read fix: a misspelt key shows up
// as its correct name missing AND the misspelling extra, side by side.
void RequireExactKeys(const YAML::Node& table, const std::set<std::string>& expected,
                      const std::string& location) {
    if (!table || !table.IsMap())
        Fail(location + " must be a table");
    std::set<std::string> actual;
    for (const auto& entry : table)
        actual.insert(entry.first.as<std::string>());

    std::vector<std::string> missing;
    std::vector<std::string> extra;
    std::set_difference(expected.begin(), expected.end(), actual.begin(), actual.end(),
                        std::back_inserter(missing));
    std::set_difference(actual.begin(), actual.end(), expected.begin(), expected.end(),
                        std::back_inserter(extra));
    if (missing.empty() && extra.empty())
        return;

    std::ostringstream message;
    message << location << " keys differ; missing = [" << Join(missing)
            << "], unknown = [" << Join(extra) << "]";
    Fail(message.str());
}

// Range bounds are stated, not implied: an out-of-range value reports the
// interval it had to be in, so the message alone says what to write.
double Number(const YAML::Node& table, const std::string& key,
              const std::string& location, double minimum, double maximum) {
    const std::string where = location + "." + key;
    double value = 0.0;
    try {
        value = table[key].as<double>();
    } catch (const YAML::Exception&) {
        Fail(where + " must be a number");
    }
    if (!std::isfinite(value))
        Fail(where + " must be finite");
    if (value < minimum || value > maximum) {
        std::ostringstream message;
        message << where << " must be within [" << minimum << ", " << maximum
                << "] (got " << value << ")";
        Fail(message.str());
    }
    return value;
}

int Integer(const YAML::Node& table, const std::string& key,
            const std::string& location, int minimum, int maximum) {
    const std::string where = location + "." + key;
    int value = 0;
    try {
        value = table[key].as<int>();
    } catch (const YAML::Exception&) {
        Fail(where + " must be a whole number");
    }
    if (value < minimum || value > maximum) {
        std::ostringstream message;
        message << where << " must be within [" << minimum << ", " << maximum
                << "] (got " << value << ")";
        Fail(message.str());
    }
    return value;
}

Eigen::Vector3d ReadVector3(const YAML::Node& table, const std::string& key,
                        const std::string& location, double minimum, double maximum) {
    const std::string where = location + "." + key;
    const YAML::Node node = table[key];
    if (!node || !node.IsSequence() || node.size() != 3)
        Fail(where + " must be a list of exactly three numbers");
    Eigen::Vector3d value;
    for (int i = 0; i < 3; ++i) {
        double component = 0.0;
        try {
            component = node[i].as<double>();
        } catch (const YAML::Exception&) {
            Fail(where + "[" + std::to_string(i) + "] must be a number");
        }
        if (!std::isfinite(component))
            Fail(where + "[" + std::to_string(i) + "] must be finite");
        if (component < minimum || component > maximum) {
            std::ostringstream message;
            message << where << "[" << i << "] must be within [" << minimum << ", "
                    << maximum << "] (got " << component << ")";
            Fail(message.str());
        }
        value[i] = component;
    }
    return value;
}

std::uint64_t Fnv1a64(const std::string& bytes) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const char raw : bytes) {
        hash ^= static_cast<unsigned char>(raw);
        hash *= 1099511628211ull;
    }
    return hash;
}

}  // namespace

PlannerConfig LoadPlannerConfig(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file)
        Fail("cannot open " + path);
    const std::string bytes((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());

    YAML::Node root;
    try {
        root = YAML::Load(bytes);
    } catch (const YAML::Exception& error) {
        Fail("cannot parse " + path + ": " + error.what());
    }
    RequireExactKeys(root, {"motion", "obstacles", "smoothness", "goal", "solver",
                            "path_following", "seeding"},
                     "root");

    PlannerConfig config;
    config.source_path = path;
    config.source_fnv1a64 = Fnv1a64(bytes);

    const YAML::Node motion = root["motion"];
    RequireExactKeys(motion, {"nominal_speed_mps", "min_duration_s", "waypoints"},
                     "motion");
    // Upper bounds here are bring-up sanity rails, not physical limits: a
    // 0.5 that was meant to be 0.05 is exactly the typo that would
    // otherwise produce a ten-times-faster move than intended.
    config.motion.nominal_speed_mps =
        Number(motion, "nominal_speed_mps", "motion", 1e-4, 0.25);
    config.motion.min_duration_s =
        Number(motion, "min_duration_s", "motion", 0.1, 600.0);
    config.motion.waypoints = Integer(motion, "waypoints", "motion", 2, 200);

    const YAML::Node obstacles = root["obstacles"];
    RequireExactKeys(obstacles, {"epsilon_dist_m", "collision_sigma"}, "obstacles");
    config.optimizer.epsilon_dist_m =
        Number(obstacles, "epsilon_dist_m", "obstacles", 0.0, 1.0);
    config.optimizer.collision_sigma =
        Number(obstacles, "collision_sigma", "obstacles", 1e-9, 1.0);

    const YAML::Node smoothness = root["smoothness"];
    RequireExactKeys(smoothness, {"qc_scale"}, "smoothness");
    config.optimizer.qc_scale = Number(smoothness, "qc_scale", "smoothness", 1e-6, 1e6);

    const YAML::Node goal = root["goal"];
    RequireExactKeys(goal, {"position_sigma_xyz", "rotation_sigma_rpy"}, "goal");
    config.optimizer.goal_position_sigma_xyz =
        ReadVector3(goal,"position_sigma_xyz", "goal", 1e-6, 1e3);
    config.optimizer.goal_rotation_sigma_rpy =
        ReadVector3(goal,"rotation_sigma_rpy", "goal", 1e-6, 1e3);

    const YAML::Node path_following = root["path_following"];
    RequireExactKeys(path_following,
                     {"position_prior_sigma_m", "rotation_prior_sigma_rad",
                      "maximum_planning_error_m", "maximum_orientation_error_rad",
                      "validation_dt_s", "approach_velocity_fraction",
                      "approach_min_duration_s", "approach_waypoints",
                      "max_chord_error_m"},
                     "path_following");
    config.path_following.position_prior_sigma_m =
        Number(path_following, "position_prior_sigma_m", "path_following", 1e-6, 1.0);
    config.path_following.rotation_prior_sigma_rad =
        Number(path_following, "rotation_prior_sigma_rad", "path_following", 1e-6, 10.0);
    config.path_following.maximum_planning_error_m =
        Number(path_following, "maximum_planning_error_m", "path_following", 1e-5, 1.0);
    config.path_following.maximum_orientation_error_rad =
        Number(path_following, "maximum_orientation_error_rad", "path_following", 1e-4, 3.15);
    config.path_following.validation_dt_s =
        Number(path_following, "validation_dt_s", "path_following", 1e-4, 0.1);
    config.path_following.approach_velocity_fraction =
        Number(path_following, "approach_velocity_fraction", "path_following", 0.01, 1.0);
    config.path_following.approach_min_duration_s =
        Number(path_following, "approach_min_duration_s", "path_following", 0.1, 120.0);
    config.path_following.approach_waypoints =
        Integer(path_following, "approach_waypoints", "path_following", 1, 200);
    config.path_following.max_chord_error_m =
        Number(path_following, "max_chord_error_m", "path_following", 1e-6, 0.1);

    const YAML::Node seeding = root["seeding"];
    RequireExactKeys(seeding, {"ik_seed", "randomised"}, "seeding");
    if (!seeding["ik_seed"] || !seeding["ik_seed"].IsScalar())
        throw std::runtime_error("seeding.ik_seed must be an integer");
    config.seeding.ik_seed = seeding["ik_seed"].as<std::uint64_t>();
    if (!seeding["randomised"] || !seeding["randomised"].IsScalar())
        throw std::runtime_error("seeding.randomised must be true or false");
    config.seeding.randomised = seeding["randomised"].as<bool>();
    // Resolve the effective seed HERE, once, so everything downstream reads
    // one value and the report cannot disagree with what was used.
    if (config.seeding.randomised) {
        std::random_device device;
        config.effective_ik_seed =
            (static_cast<std::uint64_t>(device()) << 32) ^ device();
    } else {
        config.effective_ik_seed = config.seeding.ik_seed;
    }

    const YAML::Node solver = root["solver"];
    RequireExactKeys(solver, {"max_iterations"}, "solver");
    config.optimizer.max_iterations = Integer(solver, "max_iterations", "solver", 1, 100000);

    return config;
}

std::string EffectiveConfigText(const PlannerConfig& config) {
    const Eigen::Vector3d& position = config.optimizer.goal_position_sigma_xyz;
    const Eigen::Vector3d& rotation = config.optimizer.goal_rotation_sigma_rpy;
    std::ostringstream text;
    text << std::setprecision(10);
    text << "planner config: " << config.source_path << "\n";
    text << "  digest(fnv1a64)          = " << std::hex << std::showbase
         << config.source_fnv1a64 << std::dec << std::noshowbase << "\n";
    text << "  motion.nominal_speed_mps = " << config.motion.nominal_speed_mps << "\n";
    text << "  motion.min_duration_s    = " << config.motion.min_duration_s << "\n";
    text << "  motion.waypoints         = " << config.motion.waypoints << "\n";
    text << "  obstacles.epsilon_dist_m = " << config.optimizer.epsilon_dist_m << "\n";
    text << "  obstacles.collision_sigma= " << config.optimizer.collision_sigma << "\n";
    text << "  smoothness.qc_scale      = " << config.optimizer.qc_scale << "\n";
    text << "  goal.position_sigma_xyz  = [" << position.x() << ", " << position.y()
         << ", " << position.z() << "]\n";
    text << "  goal.rotation_sigma_rpy  = [" << rotation.x() << ", " << rotation.y()
         << ", " << rotation.z() << "]\n";
    text << "  solver.max_iterations    = " << config.optimizer.max_iterations << "\n";
    const PathFollowingConfig& pf = config.path_following;
    text << "  path_following.position_prior_sigma_m     = " << pf.position_prior_sigma_m << "\n";
    text << "  path_following.rotation_prior_sigma_rad   = " << pf.rotation_prior_sigma_rad << "\n";
    text << "  path_following.maximum_planning_error_m   = " << pf.maximum_planning_error_m << "\n";
    text << "  path_following.maximum_orientation_error_rad = " << pf.maximum_orientation_error_rad << "\n";
    text << "  path_following.validation_dt_s            = " << pf.validation_dt_s << "\n";
    text << "  path_following.approach_velocity_fraction = " << pf.approach_velocity_fraction << "\n";
    text << "  path_following.approach_min_duration_s    = " << pf.approach_min_duration_s << "\n";
    text << "  path_following.approach_waypoints         = " << pf.approach_waypoints << "\n";
    text << "  path_following.max_chord_error_m          = " << pf.max_chord_error_m << "\n";
    // The seed EVERY report must carry. A plan whose seed is not recorded
    // cannot be reproduced, which is the whole point of fixing it.
    text << "  seeding.randomised       = "
         << (config.seeding.randomised ? "true (robustness testing)" : "false")
         << "\n";
    text << "  seeding.EFFECTIVE_IK_SEED = " << config.effective_ik_seed
         << "   <- replan with seeding.ik_seed set to this to reproduce\n";
    return text.str();
}
