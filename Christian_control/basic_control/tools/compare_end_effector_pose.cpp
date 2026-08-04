//
// compare_end_effector_pose — READ-ONLY diagnostic: on one cyclic feedback
// snapshot, print the end-effector pose three ways side by side —
//
//   1. firmware   : fb.base().tool_pose_* — the same pose the Kinova web app
//                   displays, computed on-device from its own configured
//                   Tool Frame (Kortex ControlConfig ToolConfiguration).
//   2. Pinocchio  : this repo's URDF forward kinematics to
//                   "EndEffector_Link", via DualArmKinematics.
//   3. analytical : AnalyticalKinematics's closed-form chain — same URDF
//                   numbers, independent code path, no Pinocchio.
//
// (2) and (3) agree with each other by test_analytical_kinematics
// (hardware-free). So if this tool shows them agreeing with each other but
// NOT with (1), the discrepancy is not a coding bug in either FK — it is
// something the static URDF cannot know about (most likely a configured
// Tool Frame offset on the physical robot). If they disagree with each
// other too, that points at a genuine bug in one of the two FK paths.
//
// No motion, no writes, no servoing change.
//
//   ./compare_end_effector_pose
//

#include "AnalyticalKinematics.h"
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

        const Eigen::Isometry3d analytical = AnalyticalForwardKinematics(q_rad);
        const Eigen::Vector3d analytical_zyx =
            analytical.rotation().eulerAngles(2, 1, 0);

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

        std::cout << "analytical FK (AnalyticalKinematics.cpp), "
                  << config::kRightBaseFrame << " frame:\n"
                  << "  position xyz: " << analytical.translation().x() << " "
                  << analytical.translation().y() << " "
                  << analytical.translation().z() << " (m)\n"
                  << "  orientation rpy (Z*Y*X, rad): " << analytical_zyx.z()
                  << " " << analytical_zyx.y() << " " << analytical_zyx.x()
                  << "\n\n";

        const Eigen::Vector3d delta_pinocchio(fw_x - ee.position.x(),
                                              fw_y - ee.position.y(),
                                              fw_z - ee.position.z());
        const Eigen::Vector3d delta_analytical(
            fw_x - analytical.translation().x(),
            fw_y - analytical.translation().y(),
            fw_z - analytical.translation().z());
        const Eigen::Vector3d delta_fk_paths(
            ee.position - analytical.translation());
        std::cout << "position delta firmware - Pinocchio:  "
                  << delta_pinocchio.transpose()
                  << "  magnitude " << delta_pinocchio.norm() << " m\n"
                  << "position delta firmware - analytical: "
                  << delta_analytical.transpose()
                  << "  magnitude " << delta_analytical.norm() << " m\n"
                  << "position delta Pinocchio - analytical: "
                  << delta_fk_paths.transpose()
                  << "  magnitude " << delta_fk_paths.norm() << " m\n"
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
