//
// test_dual_arm_mounting — holds GEN3_dual_mounted.urdf to
// config/dual_arm_mounting.yaml.
//
// The YAML is the source of truth for where the two bases sit relative to
// `world`; the URDF is what Pinocchio actually loads. Nothing generates one
// from the other, so this test is the link between them: it loads the URDF
// through the real Dynamics/DualArmKinematics path and checks the composed
// world->base transforms against the YAML's separation, tilt and origin
// convention. Edit the YAML, forget the URDF, and this fails naming the
// number that disagrees.
//
// The YAML is a flat 3-key file, so it is parsed here directly rather than
// pulling yaml-cpp into basic_control for it.
//

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>

#include "Config.h"
#include "Dynamics.h"
#include "Kinematics.h"

namespace
{
    int g_failures = 0;

    void Check(bool condition, const std::string& what)
    {
        if (condition) {
            std::printf("  ok   %s\n", what.c_str());
            return;
        }
        std::printf("  FAIL %s\n", what.c_str());
        ++g_failures;
    }

    // Minimal reader for `key: value` lines, ignoring blank lines and #
    // comments. Throws when the key is absent, so a renamed or deleted key
    // fails loudly rather than defaulting to something plausible.
    std::string ReadScalar(const std::string& path, const std::string& key)
    {
        std::ifstream file(path);
        if (!file)
            throw std::runtime_error("cannot open " + path);
        std::string line;
        while (std::getline(file, line)) {
            const std::size_t comment = line.find('#');
            if (comment != std::string::npos)
                line.resize(comment);
            const std::size_t colon = line.find(':');
            if (colon == std::string::npos)
                continue;
            std::string found = line.substr(0, colon);
            std::string value = line.substr(colon + 1);
            const auto trim = [](std::string& text) {
                const std::size_t first = text.find_first_not_of(" \t\r");
                const std::size_t last = text.find_last_not_of(" \t\r");
                text = (first == std::string::npos)
                           ? std::string()
                           : text.substr(first, last - first + 1);
            };
            trim(found);
            trim(value);
            if (found == key)
                return value;
        }
        throw std::runtime_error("no key '" + key + "' in " + path);
    }

    double ReadDouble(const std::string& path, const std::string& key)
    {
        const std::string text = ReadScalar(path, key);
        std::size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed != text.size())
            throw std::runtime_error("key '" + key + "' in " + path +
                                     " is not a plain number: '" + text + "'");
        return value;
    }

    // Roll about x recovered from a rotation matrix that is expected to BE a
    // roll — the pitch/yaw check below is what confirms that expectation.
    double RollX(const Eigen::Matrix3d& R) { return std::atan2(R(2, 1), R(2, 2)); }
} // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::fprintf(stderr, "usage: test_dual_arm_mounting <dual_arm_mounting.yaml>\n");
        return 2;
    }
    const std::string yaml_path(argv[1]);

    try {
        const double separation = ReadDouble(yaml_path, "base_separation_m");
        const double tilt = ReadDouble(yaml_path, "mount_tilt_rad");
        const std::string origin = ReadScalar(yaml_path, "world_origin");

        std::printf("mounting YAML: %s\n", yaml_path.c_str());
        std::printf("  base_separation_m %.6f, mount_tilt_rad %.6f, world_origin %s\n",
                    separation, tilt, origin.c_str());

        Check(separation > 0.0, "base_separation_m is positive");
        Check(std::abs(tilt) < M_PI, "mount_tilt_rad is a sane roll");
        // Only one convention is implemented. A new value here must come with
        // the URDF change that realises it, so refuse rather than silently
        // checking the wrong geometry.
        Check(origin == "base_midpoint",
              "world_origin is 'base_midpoint' (the only convention this "
              "test and the URDF implement)");
        if (origin != "base_midpoint") {
            std::printf("test_dual_arm_mounting: FAILED\n");
            return 1;
        }

        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, config::kLeftNominalRad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);

        const pinocchio::SE3& right = model.WorldFromBase(Arm::kRight);
        const pinocchio::SE3& left = model.WorldFromBase(Arm::kLeft);

        // Position: mirrored about world's XZ plane, half the separation each
        // way, nothing in x or z. That IS the base_midpoint convention.
        const double tolerance = 1e-9;
        Check((right.translation() -
               Eigen::Vector3d(0, -separation / 2.0, 0)).norm() < tolerance,
              "URDF right base origin matches base_separation_m");
        Check((left.translation() -
               Eigen::Vector3d(0, separation / 2.0, 0)).norm() < tolerance,
              "URDF left base origin matches base_separation_m");
        Check(((right.translation() + left.translation()) / 2.0).norm() < tolerance,
              "world origin is the midpoint of the two base origins");

        // Rotation: a pure roll of -+tilt. Checking the recovered roll alone
        // would pass for rotations that are not rolls at all, so compare the
        // whole matrix against the roll the YAML declares.
        const Eigen::Matrix3d right_expected =
            Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitX()).toRotationMatrix();
        const Eigen::Matrix3d left_expected =
            Eigen::AngleAxisd(-tilt, Eigen::Vector3d::UnitX()).toRotationMatrix();
        Check((right.rotation() - right_expected).norm() < tolerance,
              "URDF right base rotation is Rx(+mount_tilt_rad)");
        Check((left.rotation() - left_expected).norm() < tolerance,
              "URDF left base rotation is Rx(-mount_tilt_rad)");
        Check(std::abs(RollX(right.rotation()) - tilt) < tolerance,
              "recovered right roll equals mount_tilt_rad");

        // Reported, not required: the apex point the inherited geometry was
        // built around, which is where `world` used to sit. Under the
        // base_midpoint convention there is no base-to-world radius, so this
        // is only useful for recognising the original construction.
        // The apex-to-base displacement is [0, -+sep/2, -h], and it makes the
        // mount tilt with the y axis, so h = (sep/2) * tan(tilt).
        std::printf("  note: the two bases are %.6f m apart; the apex "
                    "equidistant from both lies %.6f m above the midpoint "
                    "(where `world` sat before 2026-08-05)\n",
                    separation, (separation / 2.0) * std::tan(tilt));
    } catch (const std::exception& exception) {
        std::printf("  FAIL %s\n", exception.what());
        ++g_failures;
    }

    std::printf("test_dual_arm_mounting: %s\n", g_failures == 0 ? "PASSED" : "FAILED");
    return g_failures == 0 ? 0 : 1;
}
