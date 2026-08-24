#include "PlannerConfig.h"

#include <random>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
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

std::string MappingKey(const YAML::Node& key, const std::string& location) {
    if (!key || !key.IsScalar())
        Fail(location + " keys must be strings");
    try {
        return key.as<std::string>();
    } catch (const YAML::Exception&) {
        Fail(location + " keys must be strings");
    }
}

void RequireNoDuplicateKeys(const YAML::Node& table, const std::string& location) {
    if (!table || !table.IsMap())
        Fail(location + " must be a table");
    std::set<std::string> seen;
    for (const auto& entry : table) {
        const std::string key = MappingKey(entry.first, location);
        if (!seen.insert(key).second)
            Fail(location + "." + key + " is duplicated");
    }
}

// Every expected key present, and no others. Reporting BOTH lists in one
// message is what turns a typo into a one-read fix: a misspelt key shows up
// as its correct name missing AND the misspelling extra, side by side.
void RequireExactKeys(const YAML::Node& table, const std::set<std::string>& expected,
                      const std::string& location) {
    if (!table || !table.IsMap())
        Fail(location + " must be a table");
    RequireNoDuplicateKeys(table, location);
    std::set<std::string> actual;
    for (const auto& entry : table)
        actual.insert(MappingKey(entry.first, location));

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
                            const std::string& location,
                            double minimum = -std::numeric_limits<double>::infinity(),
                            double maximum = std::numeric_limits<double>::infinity()) {
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

double PositiveNumber(const YAML::Node& table, const std::string& key,
                      const std::string& location) {
    const std::string where = location + "." + key;
    double value = 0.0;
    try {
        value = table[key].as<double>();
    } catch (const YAML::Exception&) {
        Fail(where + " must be a number");
    }
    if (!std::isfinite(value))
        Fail(where + " must be finite");
    if (value <= 0.0)
        Fail(where + " must be strictly positive (got " + std::to_string(value) + ")");
    return value;
}

bool Boolean(const YAML::Node& table, const std::string& key,
             const std::string& location) {
    const std::string where = location + "." + key;
    if (!table[key] || !table[key].IsScalar())
        Fail(where + " must be true or false");
    try {
        return table[key].as<bool>();
    } catch (const YAML::Exception&) {
        Fail(where + " must be true or false");
    }
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
    RequireExactKeys(root, {"motion", "obstacles", "smoothness", "posture",
                            "solver", "path_following", "seeding"},
                     "root");

    PlannerConfig config;
    config.source_path = path;
    config.source_fnv1a64 = Fnv1a64(bytes);

    const YAML::Node motion = root["motion"];
    RequireExactKeys(motion, {"nominal_speed_mps", "min_duration_s", "waypoints"},
                     "motion");
    // Upper bounds here are sanity rails, not physical limits. 2.0 m/s is
    // above any tool speed the joint limits can actually produce, so the
    // joint-space validation is what truly binds; the rail now catches
    // only order-of-magnitude typos (a 5.0 meant as 0.05), no longer a
    // deliberate fast setting (decision 2026-08-13,
    // docs/motion-limits-map.md — the 0.25 it replaces was a bring-up
    // value that rejected configured speeds the hardware allows).
    config.motion.nominal_speed_mps =
        Number(motion, "nominal_speed_mps", "motion", 1e-4, 2.0);
    config.motion.min_duration_s =
        Number(motion, "min_duration_s", "motion", 0.1, 600.0);
    config.motion.waypoints = Integer(motion, "waypoints", "motion", 2, 200);

    const YAML::Node obstacles = root["obstacles"];
    RequireExactKeys(obstacles, {"minimum_clearance_m", "preferred_clearance_m", "collision_sigma", "scene"}, "obstacles");
    config.minimum_clearance_m =
        Number(obstacles, "minimum_clearance_m", "obstacles", 0.0, 10.0);
    config.optimizer.preferred_clearance_m =
        Number(obstacles, "preferred_clearance_m", "obstacles", 0.0, 10.0);
    if (config.minimum_clearance_m > config.optimizer.preferred_clearance_m)
        Fail("obstacles.minimum_clearance_m must not exceed preferred_clearance_m");
    config.optimizer.collision_sigma =
        Number(obstacles, "collision_sigma", "obstacles", 1e-9, 1.0);
    const YAML::Node scene = obstacles["scene"];
    if (!scene || !scene.IsMap())
        Fail("obstacles.scene must be a table");
    RequireNoDuplicateKeys(scene, "obstacles.scene");
    for (const auto& entry : scene) {
        const std::string id = MappingKey(entry.first, "obstacles.scene");
        const YAML::Node object = entry.second;
        const std::string location = "obstacles.scene." + id;
        if (!object || !object.IsMap())
            Fail(location + " must be a table");
        if (!object["shape"] || !object["shape"].IsScalar())
            Fail(location + ".shape must be a string");
        const std::string shape = object["shape"].as<std::string>();

        NamedStaticObstacle obstacle;
        obstacle.id = id;
        obstacle.enabled = Boolean(object, "enabled", location);
        if (shape == "cylinder") {
            RequireExactKeys(object,
                             {"enabled", "shape", "center_mount_m", "radius_m", "height_m", "permitted_sphere_groups"},
                             location);
            MountCylinder cylinder;
            cylinder.center_mount_m = ReadVector3(object, "center_mount_m", location);
            cylinder.radius_m = PositiveNumber(object, "radius_m", location);
            cylinder.height_m = PositiveNumber(object, "height_m", location);
            obstacle.geometry = cylinder;
        } else if (shape == "box") {
            RequireExactKeys(object,
                             {"enabled", "shape", "center_mount_m", "half_extent_m", "permitted_sphere_groups"},
                             location);
            AxisAlignedBox box;
            box.center = ReadVector3(object, "center_mount_m", location);
            box.half_extent = ReadVector3(object, "half_extent_m", location);
            if ((box.half_extent.array() <= 0.0).any())
                Fail(location + ".half_extent_m must have strictly positive components");
            obstacle.geometry = box;
        } else {
            Fail(location + ".shape must be box or cylinder (got " + shape + ")");
        }
        const YAML::Node groups = object["permitted_sphere_groups"];
        if (!groups || !groups.IsSequence())
            Fail(location + ".permitted_sphere_groups must be a list");
        for (const auto& group : groups) {
            const std::string name = group.as<std::string>();
            if (name == "mount_interface") obstacle.permitted_sphere_groups.push_back(CollisionSphereGroup::kMountInterface);
            else if (name == "proximal_arm") obstacle.permitted_sphere_groups.push_back(CollisionSphereGroup::kProximalArm);
            else if (name == "upper_arm") obstacle.permitted_sphere_groups.push_back(CollisionSphereGroup::kUpperArm);
            else if (name == "forearm") obstacle.permitted_sphere_groups.push_back(CollisionSphereGroup::kForearm);
            else if (name == "tool") obstacle.permitted_sphere_groups.push_back(CollisionSphereGroup::kTool);
            else Fail(location + ".permitted_sphere_groups contains unknown group '" + name + "'");
        }
        config.scene.push_back(std::move(obstacle));
    }

    const YAML::Node smoothness = root["smoothness"];
    RequireExactKeys(smoothness, {"qc_scale"}, "smoothness");
    config.optimizer.qc_scale = Number(smoothness, "qc_scale", "smoothness", 1e-6, 1e6);

    const YAML::Node posture = root["posture"];
    RequireExactKeys(posture, {"centering_sigma", "limit_threshold_deg"},
                     "posture");
    config.optimizer.centering_sigma =
        Number(posture, "centering_sigma", "posture", 0.1, 1e3);
    config.optimizer.position_limit_threshold_rad =
        Number(posture, "limit_threshold_deg", "posture", 0.0, 45.0) *
        (M_PI / 180.0);

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
    // Resolve the effective seed HERE, once, so the run report records one
    // unambiguous provenance value. Path IK's bounded search is deterministic
    // and does not use this as a random-restart control.
    if (config.seeding.randomised) {
        std::random_device device;
        config.effective_ik_seed =
            (static_cast<std::uint64_t>(device()) << 32) ^ device();
    } else {
        config.effective_ik_seed = config.seeding.ik_seed;
    }

    const YAML::Node solver = root["solver"];
    RequireExactKeys(solver,
                     {"max_iterations", "acceptance_graph_error", "max_restart_attempts"},
                     "solver");
    config.optimizer.max_iterations = Integer(solver, "max_iterations", "solver", 1, 100000);
    // Upper bound is a sanity rail, not a physical limit — gtsam graph
    // error has no natural ceiling; it just needs to catch a fat-fingered
    // magnitude. See PlannerConfig.h for what this gates.
    config.acceptance_graph_error =
        Number(solver, "acceptance_graph_error", "solver", 1e-6, 1e9);
    // See PlannerConfig.h for what this gates. Upper bound is a sanity
    // rail: the retained-terminal-candidate pool (kRetainedTerminalCandidates,
    // PlanSolver.cpp) caps how many distinct postures actually exist to
    // restart with, so a value far past that pool size cannot buy anything.
    config.max_restart_attempts =
        Integer(solver, "max_restart_attempts", "solver", 1, 100);

    return config;
}

std::string EffectiveConfigText(const PlannerConfig& config) {
    std::ostringstream text;
    text << std::setprecision(10);
    text << "planner config: " << config.source_path << "\n";
    text << "  digest(fnv1a64)          = " << std::hex << std::showbase
         << config.source_fnv1a64 << std::dec << std::noshowbase << "\n";
    text << "  motion.nominal_speed_mps = " << config.motion.nominal_speed_mps << "\n";
    text << "  motion.min_duration_s    = " << config.motion.min_duration_s << "\n";
    text << "  motion.waypoints         = " << config.motion.waypoints << "\n";
    text << "  obstacles.minimum_clearance_m = " << config.minimum_clearance_m << "\n";
    text << "  obstacles.preferred_clearance_m = " << config.optimizer.preferred_clearance_m << "\n";
    text << "  obstacles.collision_sigma= " << config.optimizer.collision_sigma << "\n";
    text << "  obstacles.scene.count    = " << config.scene.size() << "\n";
    for (const NamedStaticObstacle& obstacle : config.scene) {
        text << "  obstacles.scene." << obstacle.id << " = "
             << (obstacle.enabled ? "enabled" : "disabled") << " "
             << StaticObstacleShapeName(obstacle.geometry);
        if (const auto* box = std::get_if<AxisAlignedBox>(&obstacle.geometry)) {
            text << " center_mount_m=[" << box->center.x() << ", " << box->center.y()
                 << ", " << box->center.z() << "] half_extent_m=["
                 << box->half_extent.x() << ", " << box->half_extent.y() << ", "
                 << box->half_extent.z() << "]";
        } else {
            const auto& cylinder = std::get<MountCylinder>(obstacle.geometry);
            text << " center_mount_m=[" << cylinder.center_mount_m.x() << ", "
                 << cylinder.center_mount_m.y() << ", " << cylinder.center_mount_m.z()
                 << "] radius_m=" << cylinder.radius_m
                 << " height_m=" << cylinder.height_m;
        }
        text << "\n";
    }
    text << "  smoothness.qc_scale      = " << config.optimizer.qc_scale << "\n";
    text << "  solver.max_iterations    = " << config.optimizer.max_iterations << "\n";
    text << "  solver.acceptance_graph_error = " << config.acceptance_graph_error << "\n";
    text << "  solver.max_restart_attempts = " << config.max_restart_attempts << "\n";
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
