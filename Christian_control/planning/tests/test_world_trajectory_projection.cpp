#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "PinocchioKinematicsAdapter.h"
#include "PlannerModel.h"
#include "WorldTrajectoryProjection.h"

namespace {

void CheckArm(const std::string& yaml_path, bool has_tool,
              const Eigen::Isometry3d& world_T_mount)
{
    const PlannerModel model =
        LoadPlannerModel(yaml_path, has_tool, world_T_mount);

    std::vector<gtsam::Vector> q(3, gtsam::Vector::Zero(7));
    q[0] << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7;
    q[1] << 0.11, -0.19, 0.28, -0.37, 0.46, -0.55, 0.64;
    q[2] << 0.12, -0.18, 0.26, -0.34, 0.42, -0.50, 0.58;

    std::vector<gtsam::Vector> qdot(3, gtsam::Vector::Zero(7));
    qdot[0] << 0.2, -0.1, 0.3, -0.2, 0.1, -0.3, 0.2;
    qdot[1] << -0.1, 0.2, -0.2, 0.3, -0.3, 0.1, -0.1;
    // Deliberately nonzero: the published terminal reference must still be
    // stationary and arrival-eligible.
    qdot[2].setConstant(0.5);

    const WorldCartesianTrajectory projected = ProjectWorldTrajectory(
        model, q, qdot, 0.004, /*trajectory_id=*/12,
        /*planner_vicon_sequence=*/99);
    assert(projected.trajectory_id == 12);
    assert(projected.planner_vicon_sequence == 99);
    assert(projected.points.size() == q.size() &&
           "dense projection must preserve every final state");
    assert(!ValidateWorldCartesianTrajectory(projected).has_value());

    const Eigen::Matrix3d world_R_base =
        world_T_mount.linear() *
        pinocchio_kinematics_adapter::MountFromBase(model.left_arm).linear();
    for (std::size_t i = 0; i < q.size(); ++i) {
        const Eigen::Matrix<double, 7, 1> q_i(q[i]);
        const Eigen::Matrix<double, 7, 1> qdot_i(qdot[i]);
        const auto base =
            pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
                q_i, model.end_effector_frame, model.left_arm);
        const Eigen::Vector3d expected_position =
            world_T_mount *
            pinocchio_kinematics_adapter::MountFromBase(model.left_arm) *
            base.position;
        const Eigen::Matrix3d expected_rotation =
            world_R_base * base.rotation;

        const auto& point = projected.points[i];
        assert(std::abs(point.t_from_start_s - 0.002 * i) < 1e-15);
        assert((point.position_world_m - expected_position).norm() < 1e-12);
        assert((point.orientation_world.toRotationMatrix() - expected_rotation).norm() <
               1e-12);
        assert(std::abs(point.orientation_world.norm() - 1.0) < 1e-15);

        if (i + 1 < q.size()) {
            const Eigen::Vector3d expected_linear =
                world_R_base * base.jacobian.topRows<3>() * qdot_i;
            const Eigen::Vector3d expected_angular =
                world_R_base * base.jacobian.bottomRows<3>() * qdot_i;
            assert((point.linear_velocity_world_m_s - expected_linear).norm() <
                   1e-12 &&
                   "linear twist must be independent J_W qdot evidence");
            assert((point.angular_velocity_world_rad_s - expected_angular).norm() <
                   1e-12 &&
                   "angular twist must be independent J_W qdot evidence");
            assert(!point.arrival_eligible);
        }
    }

    assert(projected.points.back().linear_velocity_world_m_s.isZero(0.0));
    assert(projected.points.back().angular_velocity_world_rad_s.isZero(0.0));
    assert(projected.points.back().arrival_eligible);
    for (std::size_t i = 1; i < projected.points.size(); ++i)
        assert(projected.points[i - 1].orientation_world.coeffs().dot(
                   projected.points[i].orientation_world.coeffs()) >= 0.0 &&
               "quaternion output must keep one hemisphere continuously");
}

bool ThrowsDimensionMismatch(const PlannerModel& model)
{
    try {
        ProjectWorldTrajectory(model, {gtsam::Vector::Zero(7),
                                       gtsam::Vector::Zero(6)},
                               {gtsam::Vector::Zero(7),
                                gtsam::Vector::Zero(7)},
                               0.01, 1, 1);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

} // namespace

int main(int argc, char** argv)
{
    assert(argc == 3 &&
           "usage: test_world_trajectory_projection <dh_tool> <dh_flange>");
    const Eigen::Isometry3d world_T_mount =
        Eigen::Translation3d(1.0, 2.0, 3.0) *
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    CheckArm(argv[1], /*has_tool=*/true, world_T_mount);
    CheckArm(argv[2], /*has_tool=*/false, world_T_mount);

    const PlannerModel right =
        LoadPlannerModel(argv[1], /*has_tool=*/true, world_T_mount);
    assert(ThrowsDimensionMismatch(right));

    std::printf("all world-trajectory projection tests passed\n");
    return 0;
}
