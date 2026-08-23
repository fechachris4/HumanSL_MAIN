// ProjectWorldTrajectory is the planner's ONLY mount->world conversion.
// This test proves the exit does exactly a rigid carry: every projected
// world pose equals world_T_mount composed with the model's mount-frame FK,
// and identity world_T_mount reproduces the mount numbers unchanged.

#include <cassert>
#include <cstdio>
#include <cmath>

#include "PlannerModel.h"
#include "WorldTrajectoryProjection.h"

int main(int argc, char** argv) {
    assert(argc == 3 && "usage: test_projection_exit dh_tool.yaml dh_flange.yaml");
    (void)argc;

    for (const bool left : {false, true}) {
        const PlannerModel model =
            LoadPlannerModel(argv[left ? 2 : 1], /*has_tool=*/!left);

        // Two joint states, mildly apart; velocities zero at the end so the
        // terminal sample is stationary as the contract requires.
        gtsam::Vector q0(7), q1(7), qd0(7), qd1(7);
        for (int j = 0; j < 7; ++j) {
            q0(j) = 0.1 * (j + 1);
            q1(j) = q0(j) + 0.05;
            qd0(j) = 0.02;
            qd1(j) = 0.0;
        }
        const std::vector<gtsam::Vector> positions = {q0, q1};
        const std::vector<gtsam::Vector> velocities = {qd0, qd1};

        Eigen::Isometry3d world_T_mount = Eigen::Isometry3d::Identity();
        world_T_mount.linear() =
            Eigen::AngleAxisd(0.6, Eigen::Vector3d(1, 1, 0).normalized())
                .toRotationMatrix();
        world_T_mount.translation() = Eigen::Vector3d(0.3, -0.2, 1.1);

        const WorldCartesianTrajectory projected = ProjectWorldTrajectory(
            model, world_T_mount, positions, velocities, 2.0, 7, 11);
        const WorldCartesianTrajectory in_mount = ProjectWorldTrajectory(
            model, Eigen::Isometry3d::Identity(), positions, velocities, 2.0,
            7, 11);
        assert(projected.points.size() == 2 && in_mount.points.size() == 2);

        for (std::size_t i = 0; i < projected.points.size(); ++i) {
            // Mount-frame truth from the model's own FK.
            const Eigen::Matrix<double, 7, 1> q(positions[i]);
            const gtsam::Pose3 mount_pose = ToolPoseInMount(model, q);

            // Identity projection == mount FK.
            assert((in_mount.points[i].position_world_m -
                    mount_pose.translation())
                       .cwiseAbs()
                       .maxCoeff() < 1e-9);

            // Non-identity projection == the same pose rigidly carried.
            const Eigen::Vector3d expected_p =
                world_T_mount * mount_pose.translation();
            assert((projected.points[i].position_world_m - expected_p)
                       .cwiseAbs()
                       .maxCoeff() < 1e-9);
            const Eigen::Matrix3d expected_R =
                world_T_mount.linear() * mount_pose.rotation().matrix();
            const Eigen::Matrix3d got_R =
                projected.points[i].orientation_world.toRotationMatrix();
            assert((got_R - expected_R).cwiseAbs().maxCoeff() < 1e-9);

            // Twists rotate; the terminal sample is exactly stationary.
            const Eigen::Vector3d rotated_v =
                world_T_mount.linear() *
                in_mount.points[i].linear_velocity_world_m_s;
            assert((projected.points[i].linear_velocity_world_m_s - rotated_v)
                       .cwiseAbs()
                       .maxCoeff() < 1e-9);
        }
        assert(projected.points.back().arrival_eligible);
        assert(projected.points.back()
                   .linear_velocity_world_m_s.cwiseAbs()
                   .maxCoeff() == 0.0);

        // The planner's redundancy decision crosses the boundary as a
        // posture preference: every projected point carries the exact joint
        // state it was computed from, regardless of world_T_mount.
        for (std::size_t i = 0; i < projected.points.size(); ++i) {
            assert(projected.points[i].has_posture);
            assert(in_mount.points[i].has_posture);
            assert((projected.points[i].posture_rad -
                    Eigen::Matrix<double, 7, 1>(positions[i]))
                       .cwiseAbs()
                       .maxCoeff() == 0.0);
        }
    }

    std::puts("test_projection_exit: all assertions passed");
    return 0;
}
