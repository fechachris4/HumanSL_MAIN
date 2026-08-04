//
// Hardware-free cross-check: AnalyticalKinematics's closed-form FK must
// agree with Kinematics.cpp's Pinocchio-based FK (already diff-verified
// against Kinova's official URDF) at several joint configurations. No
// robot connection.
//

#include <cmath>
#include <iostream>
#include <string>

#include "AnalyticalKinematics.h"
#include "Config.h"
#include "Dynamics.h"
#include "Kinematics.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    void CheckPoseAgrees(Dynamics& dynamics, DualArmKinematics& model,
                         KinematicsWorkspace& workspace,
                         const Eigen::Matrix<double, 7, 1>& q_rad,
                         const std::string& label)
    {
        const PoseJacobian pinocchio_pose =
            model.RightPoseAndJacobian(q_rad, workspace);
        const Eigen::Isometry3d analytical_pose =
            AnalyticalForwardKinematics(q_rad);

        const double position_error_m =
            (pinocchio_pose.position - analytical_pose.translation()).norm();
        Check(position_error_m < 1e-9,
              label + ": position within 1nm (got " +
                  std::to_string(position_error_m) + " m)");

        const double rotation_error =
            (pinocchio_pose.rotation - analytical_pose.rotation()).norm();
        Check(rotation_error < 1e-9,
              label + ": rotation within 1e-9 (got " +
                  std::to_string(rotation_error) + ")");
    }
} // namespace

int main()
{
    try {
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, config::kLeftNominalRad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);
        KinematicsWorkspace workspace(dynamics);

        Eigen::Matrix<double, 7, 1> q_zero;
        q_zero.setZero();
        CheckPoseAgrees(dynamics, model, workspace, q_zero, "zero configuration");

        Eigen::Matrix<double, 7, 1> q_mixed;
        q_mixed << 0.11, -0.22, 0.33, -0.44, 0.55, -0.66, 0.77;
        CheckPoseAgrees(dynamics, model, workspace, q_mixed, "mixed configuration");

        Eigen::Matrix<double, 7, 1> q_large;
        q_large << 3.0, -1.9, 2.5, -2.1, 1.8, -1.5, 2.9;
        CheckPoseAgrees(dynamics, model, workspace, q_large, "near-limit configuration");

        Eigen::Matrix<double, 7, 1> q_wrapped;
        q_wrapped << 7.0, -0.5, -5.0, 0.9, 10.0, -0.3, -8.0;
        CheckPoseAgrees(dynamics, model, workspace, q_wrapped,
                        "unwrapped continuous-joint configuration");
    }
    catch (std::exception& ex) {
        std::cout << "FAIL: exception " << ex.what() << "\n";
        ++failures;
    }

    if (failures == 0) {
        std::cout << "PASS\n";
        return 0;
    }
    std::cout << failures << " failure(s)\n";
    return 1;
}
