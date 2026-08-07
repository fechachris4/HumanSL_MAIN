//
// print_end_effector_pose — READ-ONLY: connect to the arm, take one cyclic
// feedback reading, run it through the controller's own forward kinematics,
// and print that arm's end-effector Cartesian pose. No motion, no writes,
// no servoing change.
//
//   ./print_end_effector_pose [--arm right|left]   (default: right)
//

#include "Config.h"
#include "Dynamics.h"
#include "Hardware.h"
#include "Kinematics.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

namespace k_api = Kinova::Api;

namespace
{
    // --arm selects BOTH the IP and the frame/adapter together, rather than
    // a bare --ip: an IP decoupled from the frame selection risks computing
    // FK in the wrong arm's frame while connecting to the intended one.
    const config::ArmConfig& ParseArmArg(int argc, char** argv, const char* usage_name)
    {
        std::string arm = "right";
        for (int i = 1; i < argc; ++i)
        {
            const std::string flag = argv[i];
            if (flag == "--arm")
            {
                if (++i >= argc)
                {
                    std::cerr << "error: --arm needs a value\n"
                              << "usage: " << usage_name << " [--arm right|left]\n";
                    std::exit(2);
                }
                arm = argv[i];
            }
            else
            {
                std::cerr << "error: unknown option '" << flag << "'\n"
                          << "usage: " << usage_name << " [--arm right|left]\n";
                std::exit(2);
            }
        }
        if (arm == "right") return config::kRightArmConfig;
        if (arm == "left") return config::kLeftArmConfig;
        std::cerr << "error: --arm must be 'right' or 'left'\n";
        std::exit(2);
    }
} // namespace

int main(int argc, char** argv)
{
    const config::ArmConfig& arm_config =
        ParseArmArg(argc, argv, "print_end_effector_pose");
    const Arm controlled_arm =
        arm_config.name == std::string("right") ? Arm::kRight : Arm::kLeft;
    try
    {
        // Same mounted-model + arm adapter the controller runs, so this
        // pose matches what the controller sees at takeover.
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, controlled_arm,
                                arm_config.other_arm_nominal_rad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);

        Connect connection(arm_config.ip);

        // The ONLY RefreshFeedback call this tool makes: one snapshot from
        // the 1 kHz cyclic (UDP) channel. Sends no command.
        const k_api::BaseCyclic::Feedback fb =
            connection.base_cyclic()->RefreshFeedback();

        Eigen::Matrix<double, 7, 1> q_rad;
        for (int i = 0; i < fb.actuators_size() && i < 7; ++i)
            q_rad[i] = fb.actuators(i).position() * M_PI / 180.0;

        KinematicsWorkspace workspace(model.dynamics());
        const PoseJacobian ee = model.ControlledPoseAndJacobian(q_rad, workspace);

        // Intrinsic Z-Y-X (yaw-pitch-roll) Euler angles, R = Rz*Ry*Rx —
        // the same convention the controller prints at takeover.
        const Eigen::Vector3d zyx = ee.rotation.eulerAngles(2, 1, 0);

        std::cout << arm_config.name << " end-effector (" << arm_config.end_effector_frame
                  << " in " << arm_config.base_frame << "):\n"
                  << std::fixed << std::setprecision(4)
                  << "  position xyz: " << ee.position.x() << " "
                  << ee.position.y() << " " << ee.position.z()
                  << " (m)\n"
                  << "  orientation rpy: " << zyx.z() << " " << zyx.y() << " "
                  << zyx.x() << " (rad, R = Rz*Ry*Rx)"
                  << std::defaultfloat << "\n";
        return 0;
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex API error: " << ex.what() << "\n";
        return 1;
    }
    catch (std::exception& ex)
    {
        std::cout << "error: " << ex.what() << "\n";
        return 1;
    }
}
