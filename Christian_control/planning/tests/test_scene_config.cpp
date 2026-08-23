// The persistent static scene is the planner's sole obstacle definition.
// These tests exercise the strict YAML boundary using a complete planner
// config, so a scene change cannot bypass any of the existing required keys.

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <variant>

#include "PlannerConfig.h"

namespace {

int failures = 0;
int file_number = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++failures;
    }
}

bool Near(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected) {
    return (actual - expected).cwiseAbs().maxCoeff() < 1e-12;
}

std::string Indent(const std::string& text, const std::string& prefix) {
    std::string indented = prefix;
    for (char character : text) {
        indented += character;
        if (character == '\n')
            indented += prefix;
    }
    return indented;
}

std::string WriteConfig(const std::string& scene_yaml) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("humansl_scene_config_" + std::to_string(file_number++) + ".yaml");
    std::ofstream file(path);
    if (!file)
        throw std::runtime_error("cannot write temporary planner config");
    file << R"(motion:
  nominal_speed_mps: 0.25
  min_duration_s: 1.0
  waypoints: 10
obstacles:
  minimum_clearance_m: 0.05
  preferred_clearance_m: 0.10
  collision_sigma: 0.0005
)";
    file << Indent(scene_yaml, "  ") << R"(
smoothness:
  qc_scale: 1.0
posture:
  centering_sigma: 2.0
  limit_threshold_deg: 20.0
solver:
  max_iterations: 1000
path_following:
  position_prior_sigma_m: 0.0012
  rotation_prior_sigma_rad: 0.01
  maximum_planning_error_m: 0.005
  maximum_orientation_error_rad: 0.1
  validation_dt_s: 0.002
  approach_velocity_fraction: 0.9
  approach_min_duration_s: 0.1
  approach_waypoints: 5
  max_chord_error_m: 0.001
seeding:
  ik_seed: 20260807
  randomised: false
)";
    return path.string();
}

void ExpectReject(const std::string& scene_yaml, const std::string& what,
                  const std::string& required_diagnostic = "") {
    const std::string path = WriteConfig(scene_yaml);
    try {
        static_cast<void>(LoadPlannerConfig(path));
        Check(false, what);
    } catch (const std::runtime_error& error) {
        Check(true, what);
        if (!required_diagnostic.empty())
            Check(std::string(error.what()).find(required_diagnostic) != std::string::npos,
                  what + " diagnostic names " + required_diagnostic);
    }
    std::filesystem::remove(path);
}

}  // namespace

int main() {
    const std::string empty_path = WriteConfig("scene: {}");
    const PlannerConfig empty = LoadPlannerConfig(empty_path);
    Check(empty.scene.empty(), "empty scene parses");
    std::filesystem::remove(empty_path);

    const std::string cylinder_path = WriteConfig(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.1, -0.2, 0.3]
    radius_m: 0.22
    height_m: 0.60
    permitted_sphere_groups: [mount_interface]
)");
    const PlannerConfig config = LoadPlannerConfig(cylinder_path);
    std::filesystem::remove(cylinder_path);
    Check(config.scene.size() == 1, "one cylinder");
    if (config.scene.size() == 1) {
        Check(config.scene[0].id == "torso", "mapping key is identity");
        Check(std::holds_alternative<MountCylinder>(config.scene[0].geometry),
              "cylinder selects cylinder geometry");
        if (std::holds_alternative<MountCylinder>(config.scene[0].geometry)) {
            const auto& torso = std::get<MountCylinder>(config.scene[0].geometry);
            Check(Near(torso.center_mount_m, Eigen::Vector3d(0.1, -0.2, 0.3)),
                  "mount centre");
            Check(torso.radius_m == 0.22 && torso.height_m == 0.60,
                  "radius/full height");
        }
        Check(config.scene[0].permitted_sphere_groups.size() == 1,
              "permitted sphere group round trips");
    }
    const std::string effective_text = EffectiveConfigText(config);
    Check(effective_text.find("obstacles.scene.count    = 1") != std::string::npos,
          "effective config reports scene count");
    Check(effective_text.find("obstacles.scene.torso = enabled cylinder") != std::string::npos,
          "effective config reports stable id, state, and shape");
    Check(effective_text.find("center_mount_m=[0.1, -0.2, 0.3]") != std::string::npos,
          "effective config reports mount centre");
    Check(effective_text.find("radius_m=0.22 height_m=0.6") != std::string::npos,
          "effective config reports cylinder dimensions");

    const std::string disabled_path = WriteConfig(R"(scene:
  backpack:
    enabled: false
    shape: box
    center_mount_m: [0.0, 0.0, 0.0]
    half_extent_m: [0.1, 0.2, 0.3]
    permitted_sphere_groups: []
)");
    const PlannerConfig disabled = LoadPlannerConfig(disabled_path);
    std::filesystem::remove(disabled_path);
    Check(disabled.scene.size() == 1, "disabled object is retained");
    if (disabled.scene.size() == 1) {
        Check(!disabled.scene[0].enabled, "disabled state is retained");
        Check(std::holds_alternative<AxisAlignedBox>(disabled.scene[0].geometry),
              "box selects box geometry");
        if (std::holds_alternative<AxisAlignedBox>(disabled.scene[0].geometry)) {
            const auto& box = std::get<AxisAlignedBox>(disabled.scene[0].geometry);
            Check(Near(box.center, Eigen::Vector3d::Zero()), "box mount centre");
            Check(Near(box.half_extent, Eigen::Vector3d(0.1, 0.2, 0.3)),
                  "box half extent");
        }
    }

    const std::string large_cylinder_path = WriteConfig(R"(scene:
  large_cylinder:
    enabled: false
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 5.1
    height_m: 6.2
    permitted_sphere_groups: []
)");
    const PlannerConfig large_cylinder = LoadPlannerConfig(large_cylinder_path);
    std::filesystem::remove(large_cylinder_path);
    Check(large_cylinder.scene.size() == 1, "large disabled cylinder parses");
    if (large_cylinder.scene.size() == 1) {
        Check(std::holds_alternative<MountCylinder>(large_cylinder.scene[0].geometry),
              "large disabled cylinder selects cylinder geometry");
    }
    if (large_cylinder.scene.size() == 1 &&
        std::holds_alternative<MountCylinder>(large_cylinder.scene[0].geometry)) {
        const auto& cylinder = std::get<MountCylinder>(large_cylinder.scene[0].geometry);
        Check(cylinder.radius_m == 5.1 && cylinder.height_m == 6.2,
              "large cylinder dimensions are retained");
    }

    const std::string large_box_path = WriteConfig(R"(scene:
  large_box:
    enabled: false
    shape: box
    center_mount_m: [0.0, 0.0, 0.0]
    half_extent_m: [5.1, 6.2, 7.3]
    permitted_sphere_groups: []
)");
    const PlannerConfig large_box = LoadPlannerConfig(large_box_path);
    std::filesystem::remove(large_box_path);
    Check(large_box.scene.size() == 1, "large disabled box parses");
    if (large_box.scene.size() == 1) {
        Check(std::holds_alternative<AxisAlignedBox>(large_box.scene[0].geometry),
              "large disabled box selects box geometry");
    }
    if (large_box.scene.size() == 1 &&
        std::holds_alternative<AxisAlignedBox>(large_box.scene[0].geometry)) {
        const auto& box = std::get<AxisAlignedBox>(large_box.scene[0].geometry);
        Check(Near(box.half_extent, Eigen::Vector3d(5.1, 6.2, 7.3)),
              "large box dimensions are retained");
    }

    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: sphere
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
)", "unknown shape is rejected");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
    height_m: 0.4
)", "missing shape is rejected", "obstacles.scene.torso.shape");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
)", "missing height is rejected", "obstacles.scene.torso");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
    height_m: 0.4
    ignored: 1
)", "extra shape key is rejected");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
    height_m: 0.4
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
    height_m: 0.4
)", "duplicate scene id is rejected", "obstacles.scene.torso");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
    radius_m: 0.3
    height_m: 0.4
)", "duplicate radius is rejected", "obstacles.scene.torso.radius_m");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    enabled: false
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
    height_m: 0.4
)", "duplicate enabled is rejected", "obstacles.scene.torso.enabled");
    ExpectReject("scene: []", "non-map scene is rejected");
    ExpectReject(R"(scene:
  torso:
    enabled: not_a_boolean
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
    height_m: 0.4
)", "non-boolean enabled is rejected");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [.nan, 0.0, 0.0]
    radius_m: 0.2
    height_m: 0.4
)", "non-finite centre is rejected");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.0
    height_m: 0.4
)", "non-positive cylinder radius is rejected");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
    height_m: -0.4
)", "non-positive cylinder height is rejected");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: .inf
    height_m: 0.4
)", "non-finite cylinder dimension is rejected");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: box
    center_mount_m: [0.0, 0.0, 0.0]
    half_extent_m: [0.1, 0.0, 0.3]
)", "non-positive box half extent is rejected");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
    height_m: 0.4
    permitted_sphere_groups: [unknown_group]
)", "unknown sphere group is rejected", "unknown group");
    ExpectReject(R"(scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.0, 0.0, 0.0]
    radius_m: 0.2
    height_m: 0.4
)", "missing sphere groups are rejected", "obstacles.scene.torso");

    if (failures == 0)
        std::puts("test_scene_config: all checks passed");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
