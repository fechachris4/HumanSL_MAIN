//
// Hardware-free validation of the mounted 14-joint runtime model and the
// explicit right-arm 7-of-14 controller adapter.
//

#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>

#include "app/Config.h"
#include "math/DualArmKinematics.h"
#include "math/Kinematics.h"
#include "Dynamics.h"

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

    Eigen::Matrix3d RotX(double angle)
    {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        Eigen::Matrix3d rotation;
        rotation << 1, 0, 0,
                    0, c, -s,
                    0, s, c;
        return rotation;
    }
} // namespace

int main()
{
    static_assert(std::tuple_size_v<JointVector> == 7,
                  "hardware command interface must remain seven-wide");

    try {
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        Check(dynamics.model_.nq == 14, "dual model has nq = 14");
        Check(dynamics.model_.nv == 14, "dual model has nv = 14");
        Check(dynamics.model_.njoints == 15,
              "dual model has 14 movable joints plus Pinocchio universe");
        Check(dynamics.model_.existFrame("world"), "dual model retains world frame");
        Check(dynamics.model_.existFrame("base_link"),
              "dual model retains mounted right base");
        Check(dynamics.model_.existFrame("leftbase_link"),
              "dual model retains mounted left base");

        DualArmKinematics adapter(
            dynamics, config::kLeftNominalRad,
            config::kRightEndEffectorFrame);

        const std::array<const char*, 7> right_names{
            "Actuator1", "Actuator2", "Actuator3", "Actuator4",
            "Actuator5", "Actuator6", "Actuator7"
        };
        const std::array<const char*, 7> left_names{
            "leftActuator1", "leftActuator2", "leftActuator3", "leftActuator4",
            "leftActuator5", "leftActuator6", "leftActuator7"
        };

        Eigen::Matrix<double, 7, 1> right_q;
        right_q << 0.11, -0.22, 0.33, -0.44, 0.55, -0.66, 0.77;
        const Eigen::VectorXd& q_full =
            adapter.FullConfigurationForRight(right_q);
        for (int i = 0; i < 7; ++i) {
            const pinocchio::JointIndex right_id =
                dynamics.model_.getJointId(right_names[static_cast<std::size_t>(i)]);
            const pinocchio::JointIndex left_id =
                dynamics.model_.getJointId(left_names[static_cast<std::size_t>(i)]);
            Check(adapter.right_q_indices()[static_cast<std::size_t>(i)] ==
                      dynamics.model_.idx_qs[right_id],
                  "right q mapping follows Kortex joint order " +
                      std::to_string(i + 1));
            Check(adapter.right_v_indices()[static_cast<std::size_t>(i)] ==
                      dynamics.model_.idx_vs[right_id],
                  "right Jacobian mapping follows Kortex joint order " +
                      std::to_string(i + 1));
            Check(std::abs(q_full[dynamics.model_.idx_qs[right_id]] - right_q[i]) <
                      1e-12,
                  "right measured joint enters its named full-model slot " +
                      std::to_string(i + 1));
            Check(std::abs(q_full[dynamics.model_.idx_qs[left_id]] -
                           config::kLeftNominalRad[static_cast<std::size_t>(i)]) <
                      1e-12,
                  "left joint stays at nominal " + std::to_string(i + 1));
        }

        // Changing the right measured state must never mutate the left nominal.
        const Eigen::VectorXd& q_second =
            adapter.FullConfigurationForRight(
                Eigen::Matrix<double, 7, 1>::Constant(-0.25));
        for (int i = 0; i < 7; ++i)
            Check(std::abs(q_second[adapter.left_q_indices()[static_cast<std::size_t>(i)]] -
                           config::kLeftNominalRad[static_cast<std::size_t>(i)]) <
                      1e-12,
                  "left nominal survives right-state update " +
                      std::to_string(i + 1));

        // Exact fixed mounting geometry from the downloaded URDF.
        const Eigen::VectorXd& q_mount =
            adapter.FullConfigurationForRight(
                Eigen::Matrix<double, 7, 1>::Zero());
        const Pose right_base =
            forward_kinematics(dynamics, q_mount, "base_link");
        const Pose left_base =
            forward_kinematics(dynamics, q_mount, "leftbase_link");
        const Eigen::Matrix3d right_rotation = RotX(1.2085);
        const Eigen::Matrix3d left_rotation = RotX(-1.2085);
        Check((right_base.rotation - right_rotation).norm() < 1e-12,
              "right fixed mount rotation is +1.2085 rad about x");
        Check((left_base.rotation - left_rotation).norm() < 1e-12,
              "left fixed mount rotation is -1.2085 rad about x");
        Check((right_base.position -
               right_rotation * Eigen::Vector3d(0, -0.16, 0)).norm() < 1e-12,
              "right fixed mount keeps -0.16 m local-y translation");
        const Eigen::Vector3d configured_right_base{
            config::kRightBaseOriginCommonM[0],
            config::kRightBaseOriginCommonM[1],
            config::kRightBaseOriginCommonM[2]};
        Check((right_base.position - configured_right_base).norm() < 1e-12,
              "right-base-relative reach telemetry uses the mounted origin");
        Check((left_base.position -
               left_rotation * Eigen::Vector3d(0, 0.16, 0)).norm() < 1e-12,
              "left fixed mount keeps +0.16 m local-y translation");

        // Full 6x14 Jacobian is computed, then the adapter selects exactly the
        // seven named right columns. The separate left tree contributes zero
        // columns to the right end-effector and is never sent to the controller.
        KinematicsWorkspace workspace(dynamics);
        const PoseJacobian selected =
            adapter.RightPoseAndJacobian(right_q, workspace);
        for (int i = 0; i < 7; ++i) {
            Check((selected.jacobian.col(i) -
                   workspace.jacobian_full.col(
                       adapter.right_v_indices()[static_cast<std::size_t>(i)])).norm() <
                      1e-12,
                  "selected Jacobian column matches full right column " +
                      std::to_string(i + 1));
            Check(workspace.jacobian_full.col(
                      adapter.left_v_indices()[static_cast<std::size_t>(i)]).norm() <
                      1e-12,
                  "right end-effector has zero left-arm Jacobian column " +
                      std::to_string(i + 1));
        }
        Check(selected.position.allFinite() && selected.rotation.allFinite() &&
                  selected.jacobian.allFinite(),
              "mounted right pose and selected Jacobian are finite");
    } catch (const std::exception& error) {
        std::cout << "FAIL: dual-arm model validation threw: "
                  << error.what() << "\n";
        ++failures;
    }

    if (failures == 0) {
        std::cout << "all dual-arm model tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
