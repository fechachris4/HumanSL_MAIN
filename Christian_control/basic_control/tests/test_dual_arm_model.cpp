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
#include "RobotModel.h"

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
        RobotModel robot_model(GEN3_DUAL_URDF_PATH);
        Check(robot_model.model_.nq == 22,
              "dual model has nq = 22 for eight continuous-joint cos/sin pairs");
        Check(robot_model.model_.nv == 14, "dual model has nv = 14");
        Check(robot_model.model_.njoints == 15,
              "dual model has 14 movable joints plus Pinocchio universe");
        Check(robot_model.model_.existFrame("mount"), "dual model retains mount frame");
        Check(robot_model.model_.existFrame("base_link"),
              "dual model retains mounted right base");
        Check(robot_model.model_.existFrame("leftbase_link"),
              "dual model retains mounted left base");

        DualArmKinematics adapter(
            robot_model, Arm::kRight, config::kLeftNominalRad,
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
            adapter.FullConfigurationForControlled(right_q);
        for (int i = 0; i < 7; ++i) {
            const pinocchio::JointIndex right_id =
                robot_model.model_.getJointId(right_names[static_cast<std::size_t>(i)]);
            const pinocchio::JointIndex left_id =
                robot_model.model_.getJointId(left_names[static_cast<std::size_t>(i)]);
            const int q_size =
                DualArmKinematics::kJointConfigurationSizes[static_cast<std::size_t>(i)];
            Check(robot_model.model_.nqs[right_id] == q_size &&
                      robot_model.model_.nqs[left_id] == q_size,
                  "both arms use the official joint type at actuator " +
                      std::to_string(i + 1));
            Check(robot_model.model_.nvs[right_id] == 1 &&
                      robot_model.model_.nvs[left_id] == 1,
                  "both arms keep one velocity DoF at actuator " +
                      std::to_string(i + 1));
            Check(adapter.right_q_indices()[static_cast<std::size_t>(i)] ==
                      robot_model.model_.idx_qs[right_id],
                  "right q mapping follows Kortex joint order " +
                      std::to_string(i + 1));
            Check(adapter.right_v_indices()[static_cast<std::size_t>(i)] ==
                      robot_model.model_.idx_vs[right_id],
                  "right Jacobian mapping follows Kortex joint order " +
                      std::to_string(i + 1));
            const int right_v = robot_model.model_.idx_vs[right_id];
            const int left_v = robot_model.model_.idx_vs[left_id];
            Check(std::abs(robot_model.model_.velocityLimit[right_v] -
                           official_velocity_rad_s[static_cast<std::size_t>(i)]) < 1e-12 &&
                      std::abs(robot_model.model_.velocityLimit[left_v] -
                               official_velocity_rad_s[static_cast<std::size_t>(i)]) < 1e-12,
                  "both arms use the official URDF velocity at actuator " +
                      std::to_string(i + 1));
            Check(std::abs(robot_model.model_.effortLimit[right_v] -
                           official_effort_nm[static_cast<std::size_t>(i)]) < 1e-12 &&
                      std::abs(robot_model.model_.effortLimit[left_v] -
                               official_effort_nm[static_cast<std::size_t>(i)]) < 1e-12,
                  "both arms use the official URDF effort at actuator " +
                      std::to_string(i + 1));
            if (q_size == 1) {
                const double limit =
                    official_position_limit_rad[static_cast<std::size_t>(i)];
                const int right_q_index = robot_model.model_.idx_qs[right_id];
                const int left_q_index = robot_model.model_.idx_qs[left_id];
                Check(std::abs(robot_model.model_.lowerPositionLimit[right_q_index] + limit) < 1e-12 &&
                          std::abs(robot_model.model_.upperPositionLimit[right_q_index] - limit) < 1e-12 &&
                          std::abs(robot_model.model_.lowerPositionLimit[left_q_index] + limit) < 1e-12 &&
                          std::abs(robot_model.model_.upperPositionLimit[left_q_index] - limit) < 1e-12,
                      "both arms use the official bounded range at actuator " +
                          std::to_string(i + 1));
            }
            CheckStoredAngle(q_full, robot_model.model_.idx_qs[right_id], q_size,
                             right_q[i],
                             "right measured joint enters its named full-model slot " +
                                 std::to_string(i + 1));
            CheckStoredAngle(
                q_full, robot_model.model_.idx_qs[left_id], q_size,
                config::kLeftNominalRad[static_cast<std::size_t>(i)],
                "left joint stays at nominal " + std::to_string(i + 1));
        }

        // Changing the right measured state must never mutate the left nominal.
        const Eigen::VectorXd& q_second =
            adapter.FullConfigurationForControlled(
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

        // Exact fixed mounting geometry. The numbers live in
        // config/dual_arm_mounting.yaml; test_dual_arm_mounting.cpp is what
        // holds the URDF to that file. Here we only pin the SHAPE the rest of
        // this test depends on: mount at the midpoint, mirrored about mount's
        // XZ plane, so each base sits at -+separation/2 along mount y with no
        // z offset.
        const Eigen::VectorXd& q_mount =
            adapter.FullConfigurationForControlled(
                Eigen::Matrix<double, 7, 1>::Zero());
        const Pose right_base =
            forward_kinematics(robot_model, q_mount, "base_link");
        const Pose left_base =
            forward_kinematics(robot_model, q_mount, "leftbase_link");
        const Eigen::Matrix3d right_rotation = RotX(1.2085);
        const Eigen::Matrix3d left_rotation = RotX(-1.2085);
        constexpr double kHalfSeparation = 0.113415 / 2.0;
        Check((right_base.rotation - right_rotation).norm() < 1e-12,
              "right fixed mount rotation is +1.2085 rad about x");
        Check((left_base.rotation - left_rotation).norm() < 1e-12,
              "left fixed mount rotation is -1.2085 rad about x");
        Check((right_base.position -
               Eigen::Vector3d(0, -kHalfSeparation, 0)).norm() < 1e-9,
              "right base sits half the separation along mount -y");
        Check((left_base.position -
               Eigen::Vector3d(0, kHalfSeparation, 0)).norm() < 1e-9,
              "left base sits half the separation along mount +y");
        Check(((right_base.position + left_base.position) / 2.0).norm() < 1e-12,
              "mount origin is the midpoint of the two base origins");

        // The mount accessors must agree with plain FK on the same frames, and
        // the point helpers must round-trip.
        Check((adapter.MountFromBase(Arm::kRight).translation() -
               right_base.position).norm() < 1e-12,
              "MountFromBase(right) matches FK on base_link");
        Check((adapter.MountFromBase(Arm::kLeft).translation() -
               left_base.position).norm() < 1e-12,
              "MountFromBase(left) matches FK on leftbase_link");
        const Eigen::Vector3d p_base(0.4, -0.1, 0.3);
        Check((adapter.PointMountToBase(
                   Arm::kRight, adapter.PointBaseToMount(Arm::kRight, p_base)) -
               p_base).norm() < 1e-12,
              "base->mount->base round-trips for the right arm");
        Check((adapter.PointBaseToMount(Arm::kRight, Eigen::Vector3d::Zero()) -
               right_base.position).norm() < 1e-12,
              "the right base origin maps to the mount position in mount");
        // A mount-frame tool pose must equal the base-frame one carried through
        // the mount — the identity the WORLD target path relies on.
        const Eigen::Matrix<double, 7, 1> q_probe =
            (Eigen::Matrix<double, 7, 1>() << 0.2, -0.4, 0.1, 0.9, -0.3, 0.5, 0.7)
                .finished();
        KinematicsWorkspace probe_workspace(robot_model);
        const PoseJacobian probe_base =
            adapter.ControlledPoseAndJacobian(q_probe, probe_workspace);
        const Pose probe_mount = adapter.ToolPoseInMount(Arm::kRight, q_probe);
        Check((adapter.PointBaseToMount(Arm::kRight, probe_base.position) -
               probe_mount.position).norm() < 1e-12,
              "mount tool position equals the base one through the mount");
        Check((adapter.PointMountToBase(Arm::kRight, probe_mount.position) -
               probe_base.position).norm() < 1e-12,
              "mount tool position converts back to the base one");

        // Full 6x14 Jacobian is computed, then the adapter selects exactly the
        // seven named right columns. The separate left tree contributes zero
        // columns to the right end-effector and is never sent to the controller.
        KinematicsWorkspace workspace(robot_model);
        const PoseJacobian selected =
            adapter.ControlledPoseAndJacobian(right_q, workspace);
        const Eigen::VectorXd q_eval =
            adapter.FullConfigurationForControlled(right_q);
        const Pose mount_tool = forward_kinematics(
            robot_model, q_eval, config::kRightEndEffectorFrame);
        const Eigen::Matrix3d base_R_mount = right_base.rotation.transpose();
        const Eigen::Vector3d expected_base_position =
            base_R_mount * (mount_tool.position - right_base.position);
        const Eigen::Matrix3d expected_base_rotation =
            base_R_mount * mount_tool.rotation;
        Check((selected.position - expected_base_position).norm() < 1e-12,
              "right tool position is expressed in base_link");
        Check((selected.rotation - expected_base_rotation).norm() < 1e-12,
              "right tool orientation is expressed in base_link");

        KinematicsWorkspace mount_workspace(robot_model);
        pinocchio::computeJointJacobians(robot_model.model_, robot_model.data_, q_eval);
        pinocchio::updateFramePlacements(robot_model.model_, robot_model.data_);
        pinocchio::getFrameJacobian(
            robot_model.model_, robot_model.data_, adapter.right_frame_id(),
            pinocchio::LOCAL_WORLD_ALIGNED, mount_workspace.jacobian_full);
        for (int i = 0; i < 7; ++i) {
            const int right_v =
                adapter.right_v_indices()[static_cast<std::size_t>(i)];
            Eigen::Matrix<double, 6, 1> expected_base_column;
            expected_base_column.head<3>() =
                base_R_mount * mount_workspace.jacobian_full.col(right_v).head<3>();
            expected_base_column.tail<3>() =
                base_R_mount * mount_workspace.jacobian_full.col(right_v).tail<3>();
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

        // --- left-controlled adapter: the same model, mirrored. Reuses
        // `robot_model` (safe: every call below fully recomputes robot_model.data_
        // before reading it, and nothing here interleaves with `adapter`
        // above). Right's own checks above already cover both arms' index
        // maps, velocity/effort/position limits and cos/sin sizing (those
        // depend only on the joint chain, not on controlled_arm), so this
        // block only re-tests what controlled_arm actually changes: which
        // half of q_full gets written every cycle, and which base/tool
        // frame ControlledPoseAndJacobian resolves into.
        DualArmKinematics left_adapter(
            robot_model, Arm::kLeft, config::kRightNominalRad,
            config::kRightBaseFrame, config::kRightEndEffectorFrame);
        Check(left_adapter.controlled_arm() == Arm::kLeft,
              "left-controlled adapter reports Arm::kLeft");
        Check(left_adapter.other_arm_nominal_rad() == config::kRightNominalRad,
              "left-controlled adapter's other-arm nominal is the right nominal");

        Eigen::Matrix<double, 7, 1> left_q;
        left_q << 0.12, -0.23, 0.34, -0.45, 0.56, -0.67, 0.78;
        const Eigen::VectorXd& left_q_full =
            left_adapter.FullConfigurationForControlled(left_q);
        for (int i = 0; i < 7; ++i) {
            const std::size_t joint = static_cast<std::size_t>(i);
            CheckStoredAngle(
                left_q_full, left_adapter.left_q_indices()[joint],
                DualArmKinematics::kJointConfigurationSizes[joint], left_q[i],
                "left measured joint enters its named full-model slot " +
                    std::to_string(i + 1));
            CheckStoredAngle(
                left_q_full, left_adapter.right_q_indices()[joint],
                DualArmKinematics::kJointConfigurationSizes[joint],
                config::kRightNominalRad[joint],
                "right joint stays at nominal under left control " +
                    std::to_string(i + 1));
        }

        KinematicsWorkspace left_workspace(robot_model);
        const PoseJacobian left_selected =
            left_adapter.ControlledPoseAndJacobian(left_q, left_workspace);
        const Eigen::VectorXd& left_q_eval =
            left_adapter.FullConfigurationForControlled(left_q);
        const Pose left_mount_tool = forward_kinematics(
            robot_model, left_q_eval, config::kLeftEndEffectorFrame);
        const Pose left_base_pose =
            forward_kinematics(robot_model, left_q_eval, "leftbase_link");
        const Eigen::Matrix3d left_base_R_mount =
            left_base_pose.rotation.transpose();
        const Eigen::Vector3d left_expected_position =
            left_base_R_mount * (left_mount_tool.position - left_base_pose.position);
        const Eigen::Matrix3d left_expected_rotation =
            left_base_R_mount * left_mount_tool.rotation;
        Check((left_selected.position - left_expected_position).norm() < 1e-12,
              "left-controlled tool position is expressed in leftbase_link");
        Check((left_selected.rotation - left_expected_rotation).norm() < 1e-12,
              "left-controlled tool orientation is expressed in leftbase_link");
        Check(left_selected.position.allFinite() && left_selected.rotation.allFinite() &&
                  left_selected.jacobian.allFinite(),
              "left-controlled pose and Jacobian are finite");

        // ToolPoseInMount's controlled-arm convenience overload must place
        // the measured angles into the LEFT slot now (it put them into the
        // right slot for the right-controlled `adapter` above).
        Eigen::Matrix<double, 7, 1> right_nominal_q;
        for (int i = 0; i < 7; ++i)
            right_nominal_q[i] = config::kRightNominalRad[static_cast<std::size_t>(i)];
        const Pose left_probe_mount =
            left_adapter.ToolPoseInMount(Arm::kLeft, left_q);
        const Pose left_probe_mount_explicit =
            left_adapter.ToolPoseInMount(Arm::kLeft, right_nominal_q, left_q);
        Check((left_probe_mount.position - left_probe_mount_explicit.position).norm() <
                  1e-12,
              "left ToolPoseInMount convenience overload matches the explicit one");
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
