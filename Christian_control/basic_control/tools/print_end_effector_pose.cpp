//
// print_end_effector_pose — READ-ONLY: connect to the arm, take one cyclic
// feedback reading, run it through the controller's own forward kinematics,
// and print the right end-effector's Cartesian pose. No motion, no writes,
// no servoing change.
//
//   ./print_end_effector_pose
//

#include "Config.h"
#include "Dynamics.h"
#include "Hardware.h"
#include "Kinematics.h"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace k_api = Kinova::Api;

int main()
{
    try
    {
        // Same mounted-model + right-arm adapter the controller runs, so
        // this pose matches what the controller sees at takeover.
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, config::kLeftNominalRad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);

        Connect connection(config::kRightRobotIp);

        // The ONLY RefreshFeedback call this tool makes: one snapshot from
        // the 1 kHz cyclic (UDP) channel. Sends no command.
        const k_api::BaseCyclic::Feedback fb =
            connection.base_cyclic()->RefreshFeedback();

        Eigen::Matrix<double, 7, 1> q_rad;
        for (int i = 0; i < fb.actuators_size() && i < 7; ++i)
            q_rad[i] = fb.actuators(i).position() * M_PI / 180.0;

        KinematicsWorkspace workspace(model.dynamics());
        const PoseJacobian ee = model.RightPoseAndJacobian(q_rad, workspace);

        // Intrinsic Z-Y-X (yaw-pitch-roll) Euler angles, R = Rz*Ry*Rx —
        // the same convention the controller prints at takeover.
        const Eigen::Vector3d zyx = ee.rotation.eulerAngles(2, 1, 0);

        std::cout << "right end-effector (" << config::kRightEndEffectorFrame
                  << " in " << config::kRightBaseFrame << "):\n"
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
