#include "PlanSolver.h"
#include <algorithm>
#include "GenerateTrajectory.h"
#include "TrajectoryInitiation.h"

PlanOutcome SolveToPosition(const PlannerModel& model, const PlanRequest& request,
                            const std::string& joint_limits_yaml) {
    PlanOutcome outcome;
    try {
        const auto [pos_limits, vel_limits] = createJointLimits(joint_limits_yaml);
        const gtsam::Pose3 start_pose = ToolPoseInBaseLink(model, request.q_start_rad);
        const gtsam::Pose3 goal_pose(start_pose.rotation(),
                                     gtsam::Point3(request.goal_position_m));
        // Slow by construction: Stage 1 plans are executed via profiled
        // point-to-point moves, so plan duration only shapes the solve.
        const double distance_m =
            (request.goal_position_m - start_pose.translation()).norm();
        const size_t total_time_step = 20;
        const double total_time_sec = std::max(4.0, distance_m / 0.05);

        InitializeTrajectory initializer(model.dh);
        const gtsam::Values init_values = initializer.initJointTrajectoryFromTarget(
            gtsam::Vector(request.q_start_rad), goal_pose, model.base_pose,
            total_time_step);
        const auto sdf = MakeWorldSdf(request.obstacle);
        outcome.result = optimizeJointTrajectory(
            *model.arm_model, sdf, init_values, goal_pose,
            gtsam::Vector(request.q_start_rad), pos_limits, vel_limits,
            total_time_step, total_time_sec);
        if (outcome.result.trajectory_pos.empty()) {
            outcome.error = "optimizer returned an empty trajectory";
            return outcome;
        }
        Eigen::Matrix<double, 7, 1> q_final(outcome.result.trajectory_pos.back());
        outcome.final_goal_error_m =
            (ToolPositionInBaseLink(model, q_final) - request.goal_position_m).norm();
        outcome.ok = true;
    } catch (const std::exception& exception) {
        outcome.error = exception.what();
    }
    return outcome;
}
