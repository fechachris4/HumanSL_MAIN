//
// Hardware-free validation of the mounted 14-joint runtime model and the
// explicit right-arm 7-of-14 controller adapter.
//

#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>

#include "Config.h"
#include "Kinematics.h"
#include "Dynamics.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

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

    void CheckStoredAngle(const Eigen::VectorXd& q, int q_index, int q_size,
                          double expected_rad, const std::string& what)
    {
        if (q_size == 1) {
            Check(std::abs(q[q_index] - expected_rad) < 1e-12, what);
            return;
        }
        Check(q_size == 2 &&
                  std::abs(q[q_index] - std::cos(expected_rad)) < 1e-12 &&
                  std::abs(q[q_index + 1] - std::sin(expected_rad)) < 1e-12,
              what);
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
        Check(dynamics.model_.nq == 22,
              "dual model has nq = 22 for eight continuous-joint cos/sin pairs");
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
            config::kRightBaseFrame,
            config::kRightEndEffectorFrame);

        const std::array<const char*, 7> right_names{
            "Actuator1", "Actuator2", "Actuator3", "Actuator4",
            "Actuator5", "Actuator6", "Actuator7"
        };
        const std::array<const char*, 7> left_names{
            "leftActuator1", "leftActuator2", "leftActuator3", "leftActuator4",
            "leftActuator5", "leftActuator6", "leftActuator7"
        };
        constexpr std::array<double, 7> official_velocity_rad_s{
            1.3963, 1.3963, 1.3963, 1.3963, 1.2218, 1.2218, 1.2218
        };
        constexpr std::array<double, 7> official_effort_nm{
            39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0
        };
        constexpr std::array<double, 7> official_position_limit_rad{
            0.0, 2.24, 0.0, 2.57, 0.0, 2.09, 0.0
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
            const int q_size =
                DualArmKinematics::kJointConfigurationSizes[static_cast<std::size_t>(i)];
            Check(dynamics.model_.nqs[right_id] == q_size &&
                      dynamics.model_.nqs[left_id] == q_size,
                  "both arms use the official joint type at actuator " +
                      std::to_string(i + 1));
            Check(dynamics.model_.nvs[right_id] == 1 &&
                      dynamics.model_.nvs[left_id] == 1,
                  "both arms keep one velocity DoF at actuator " +
                      std::to_string(i + 1));
            Check(adapter.right_q_indices()[static_cast<std::size_t>(i)] ==
                      dynamics.model_.idx_qs[right_id],
                  "right q mapping follows Kortex joint order " +
                      std::to_string(i + 1));
            Check(adapter.right_v_indices()[static_cast<std::size_t>(i)] ==
                      dynamics.model_.idx_vs[right_id],
                  "right Jacobian mapping follows Kortex joint order " +
                      std::to_string(i + 1));
            const int right_v = dynamics.model_.idx_vs[right_id];
            const int left_v = dynamics.model_.idx_vs[left_id];
            Check(std::abs(dynamics.model_.velocityLimit[right_v] -
                           official_velocity_rad_s[static_cast<std::size_t>(i)]) < 1e-12 &&
                      std::abs(dynamics.model_.velocityLimit[left_v] -
                               official_velocity_rad_s[static_cast<std::size_t>(i)]) < 1e-12,
                  "both arms use the official URDF velocity at actuator " +
                      std::to_string(i + 1));
            Check(std::abs(dynamics.model_.effortLimit[right_v] -
                           official_effort_nm[static_cast<std::size_t>(i)]) < 1e-12 &&
                      std::abs(dynamics.model_.effortLimit[left_v] -
                               official_effort_nm[static_cast<std::size_t>(i)]) < 1e-12,
                  "both arms use the official URDF effort at actuator " +
                      std::to_string(i + 1));
            if (q_size == 1) {
                const double limit =
                    official_position_limit_rad[static_cast<std::size_t>(i)];
                const int right_q_index = dynamics.model_.idx_qs[right_id];
                const int left_q_index = dynamics.model_.idx_qs[left_id];
                Check(std::abs(dynamics.model_.lowerPositionLimit[right_q_index] + limit) < 1e-12 &&
                          std::abs(dynamics.model_.upperPositionLimit[right_q_index] - limit) < 1e-12 &&
                          std::abs(dynamics.model_.lowerPositionLimit[left_q_index] + limit) < 1e-12 &&
                          std::abs(dynamics.model_.upperPositionLimit[left_q_index] - limit) < 1e-12,
                      "both arms use the official bounded range at actuator " +
                          std::to_string(i + 1));
            }
            CheckStoredAngle(q_full, dynamics.model_.idx_qs[right_id], q_size,
                             right_q[i],
                             "right measured joint enters its named full-model slot " +
                                 std::to_string(i + 1));
            CheckStoredAngle(
                q_full, dynamics.model_.idx_qs[left_id], q_size,
                config::kLeftNominalRad[static_cast<std::size_t>(i)],
                "left joint stays at nominal " + std::to_string(i + 1));
        }

        // Changing the right measured state must never mutate the left nominal.
        const Eigen::VectorXd& q_second =
            adapter.FullConfigurationForRight(
                Eigen::Matrix<double, 7, 1>::Constant(-0.25));
        for (int i = 0; i < 7; ++i) {
            const std::size_t joint = static_cast<std::size_t>(i);
            CheckStoredAngle(
                q_second, adapter.left_q_indices()[joint],
                DualArmKinematics::kJointConfigurationSizes[joint],
                config::kLeftNominalRad[joint],
                "left nominal survives right-state update " +
                    std::to_string(i + 1));
        }

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
        const Eigen::Vector3d configured_control_origin{
            config::kRightBaseOriginControlM[0],
            config::kRightBaseOriginControlM[1],
            config::kRightBaseOriginControlM[2]};
        Check(configured_control_origin.norm() < 1e-12,
              "right-base control frame has a zero reach origin");
        Check((left_base.position -
               left_rotation * Eigen::Vector3d(0, 0.16, 0)).norm() < 1e-12,
              "left fixed mount keeps +0.16 m local-y translation");

        // Full 6x14 Jacobian is computed, then the adapter selects exactly the
        // seven named right columns. The separate left tree contributes zero
        // columns to the right end-effector and is never sent to the controller.
        KinematicsWorkspace workspace(dynamics);
        const PoseJacobian selected =
            adapter.RightPoseAndJacobian(right_q, workspace);
        const Eigen::VectorXd q_eval =
            adapter.FullConfigurationForRight(right_q);
        const Pose world_tool = forward_kinematics(
            dynamics, q_eval, config::kRightEndEffectorFrame);
        const Eigen::Matrix3d base_R_world = right_base.rotation.transpose();
        const Eigen::Vector3d expected_base_position =
            base_R_world * (world_tool.position - right_base.position);
        const Eigen::Matrix3d expected_base_rotation =
            base_R_world * world_tool.rotation;
        Check((selected.position - expected_base_position).norm() < 1e-12,
              "right tool position is expressed in base_link");
        Check((selected.rotation - expected_base_rotation).norm() < 1e-12,
              "right tool orientation is expressed in base_link");

        KinematicsWorkspace world_workspace(dynamics);
        pinocchio::computeJointJacobians(dynamics.model_, dynamics.data_, q_eval);
        pinocchio::updateFramePlacements(dynamics.model_, dynamics.data_);
        pinocchio::getFrameJacobian(
            dynamics.model_, dynamics.data_, adapter.right_frame_id(),
            pinocchio::LOCAL_WORLD_ALIGNED, world_workspace.jacobian_full);
        for (int i = 0; i < 7; ++i) {
            const int right_v =
                adapter.right_v_indices()[static_cast<std::size_t>(i)];
            Eigen::Matrix<double, 6, 1> expected_base_column;
            expected_base_column.head<3>() =
                base_R_world * world_workspace.jacobian_full.col(right_v).head<3>();
            expected_base_column.tail<3>() =
                base_R_world * world_workspace.jacobian_full.col(right_v).tail<3>();
            Check((selected.jacobian.col(i) -
                   workspace.jacobian_full.col(right_v)).norm() < 1e-12,
                  "selected Jacobian column matches full right column " +
                      std::to_string(i + 1));
            Check((selected.jacobian.col(i) - expected_base_column).norm() < 1e-12,
                  "right Jacobian column is rotated into base_link axes " +
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
