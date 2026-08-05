//
// print_dual_arm_fk — forward kinematics for BOTH arms at a chosen
// configuration, in the world frame and in each arm's own base_link, so the
// mounted model can be checked against the physical rig.
//
// Reads the URDF only. No Kortex, no connection, no motion: safe to run with
// the robot off, and it is deliberately not linked against Kortex so it
// cannot command anything.
//
// Usage:
//   ./print_dual_arm_fk                 both arms at q = 0
//   ./print_dual_arm_fk r1..r7          right arm at the given DEGREES,
//                                       left at config::kLeftNominalRad
//   ./print_dual_arm_fk r1..r7 l1..l7   both arms at the given DEGREES
//
// How to use the output: the right arm's base_link row is the one the Kinova
// web dashboard's tool_pose reports, so compare that first. Once it agrees,
// the world row is that same pose carried through the mounting transform, and
// a disagreement there is the mounting geometry in
// config/dual_arm_mounting.yaml — not the arm model.
//

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <utility>

#include "Config.h"
#include "Dynamics.h"
#include "FramePrint.h"
#include "Kinematics.h"

namespace
{
    // Parses a whole argv run of joint angles in degrees; returns false and
    // names the offender rather than silently accepting a partial number.
    bool ParseDegrees(int argc, char** argv, int first,
                      Eigen::Matrix<double, 7, 1>& q_rad)
    {
        for (int i = 0; i < 7; ++i) {
            const std::string text(argv[first + i]);
            std::size_t consumed = 0;
            double degrees = 0.0;
            try {
                degrees = std::stod(text, &consumed);
            } catch (const std::exception&) {
                std::fprintf(stderr, "not a number: '%s'\n", text.c_str());
                return false;
            }
            if (consumed != text.size()) {
                std::fprintf(stderr, "trailing characters in '%s'\n", text.c_str());
                return false;
            }
            q_rad[i] = degrees * M_PI / 180.0;
        }
        return true;
    }
} // namespace

int main(int argc, char** argv)
{
    if (argc != 1 && argc != 8 && argc != 15) {
        std::fprintf(stderr,
                     "usage: print_dual_arm_fk [right1..right7 [left1..left7]]"
                     "   (joint angles in DEGREES)\n");
        return 2;
    }

    Eigen::Matrix<double, 7, 1> right_q_rad = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> left_q_rad = Eigen::Matrix<double, 7, 1>::Zero();
    for (int i = 0; i < 7; ++i)
        left_q_rad[i] = config::kLeftNominalRad[static_cast<std::size_t>(i)];

    if (argc >= 8 && !ParseDegrees(argc, argv, 1, right_q_rad))
        return 2;
    if (argc == 15 && !ParseDegrees(argc, argv, 8, left_q_rad))
        return 2;

    try {
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, config::kLeftNominalRad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);

        std::printf("model: %s\n", GEN3_DUAL_URDF_PATH);
        for (const auto& [label, arm] :
             std::array<std::pair<const char*, Arm>, 2>{
                 {{"right base_link in world", Arm::kRight},
                  {"left  base_link in world", Arm::kLeft}}}) {
            const pinocchio::SE3& mount = model.WorldFromBase(arm);
            std::printf("%s: p [% .6f % .6f % .6f]  roll_x % .6f rad\n", label,
                        mount.translation().x(), mount.translation().y(),
                        mount.translation().z(),
                        std::atan2(mount.rotation()(2, 1), mount.rotation()(2, 2)));
        }
        std::printf("\n");
        std::fflush(stdout);

        const std::string caption =
            argc == 1
                ? "FK at q = 0 for both arms:"
                : "FK at the requested configuration (angles given in degrees):";
        PrintDualArmFkTable(model, right_q_rad, left_q_rad, caption);
    } catch (const std::exception& exception) {
        std::fprintf(stderr, "print_dual_arm_fk: %s\n", exception.what());
        return 1;
    }
    return 0;
}
