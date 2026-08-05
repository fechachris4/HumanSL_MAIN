//
// compare_end_effector_pose — READ-ONLY diagnostic: on one cyclic feedback
// snapshot, print the end-effector pose two ways side by side —
//
//   1. firmware   : fb.base().tool_pose_* — the same pose the Kinova web app
//                   displays, computed on-device from its own configured
//                   Tool Frame (Kortex ControlConfig ToolConfiguration).
//   2. Pinocchio  : this repo's URDF forward kinematics to
//                   "EndEffector_Link", via DualArmKinematics.
//
// These are two independent implementations — firmware computes its pose
// on-device from its own Tool Frame config, unrelated to this codebase's
// Pinocchio/URDF path. If they disagree, the discrepancy is something the
// static URDF cannot know about (most likely a configured Tool Frame offset
// on the physical robot), not necessarily a coding bug.
//
// No motion, no writes, no servoing change.
//
//   ./compare_end_effector_pose
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
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, config::kLeftNominalRad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);

        Connect connection(config::kRightRobotIp);

        // The ONLY RefreshFeedback call this tool makes. Both numbers below
        // come from this single snapshot, so they describe the same instant.
        const k_api::BaseCyclic::Feedback fb =
            connection.base_cyclic()->RefreshFeedback();

        Eigen::Matrix<double, 7, 1> q_rad;
        for (int i = 0; i < fb.actuators_size() && i < 7; ++i)
            q_rad[i] = fb.actuators(i).position() * M_PI / 180.0;

        KinematicsWorkspace workspace(model.dynamics());
        const PoseJacobian ee = model.RightPoseAndJacobian(q_rad, workspace);
        const Eigen::Vector3d zyx = ee.rotation.eulerAngles(2, 1, 0);

        const double fw_x = fb.base().tool_pose_x();
        const double fw_y = fb.base().tool_pose_y();
        const double fw_z = fb.base().tool_pose_z();
        const double fw_tx_deg = fb.base().tool_pose_theta_x();
        const double fw_ty_deg = fb.base().tool_pose_theta_y();
        const double fw_tz_deg = fb.base().tool_pose_theta_z();

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "firmware tool_pose (what the web app shows), "
                  << config::kRightBaseFrame << " frame:\n"
                  << "  position xyz: " << fw_x << " " << fw_y << " " << fw_z
                  << " (m)\n"
                  << "  orientation (firmware theta_x,y,z, deg): "
                  << fw_tx_deg << " " << fw_ty_deg << " " << fw_tz_deg << "\n\n";

        std::cout << "Pinocchio FK to " << config::kRightEndEffectorFrame
                  << ", " << config::kRightBaseFrame << " frame:\n"
                  << "  position xyz: " << ee.position.x() << " "
                  << ee.position.y() << " " << ee.position.z() << " (m)\n"
                  << "  orientation rpy (Z*Y*X, rad): " << zyx.z() << " "
                  << zyx.y() << " " << zyx.x() << "\n\n";

        const Eigen::Vector3d delta_pinocchio(fw_x - ee.position.x(),
                                              fw_y - ee.position.y(),
                                              fw_z - ee.position.z());
        std::cout << "position delta firmware - Pinocchio:  "
                  << delta_pinocchio.transpose()
                  << "  magnitude " << delta_pinocchio.norm() << " m\n"
                  << std::defaultfloat;
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
