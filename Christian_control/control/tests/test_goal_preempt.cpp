#include <cmath>
#include <cstdio>
#include <memory>

#include "CartesianReference.h"
#include "GoalCommand.h"

namespace {

int failures = 0;

void Check(bool condition, const char* what)
{
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

std::unique_ptr<WorldCartesianTrajectory> Trajectory(
    std::uint64_t id, const Eigen::Vector3d& start)
{
    auto trajectory = std::make_unique<WorldCartesianTrajectory>();
    trajectory->trajectory_id = id;
    trajectory->planner_vicon_sequence = 10;
    trajectory->points.resize(2);
    trajectory->points[0].position_world_m = start;
    trajectory->points[0].orientation_world = Eigen::Quaterniond::Identity();
    trajectory->points[1] = trajectory->points[0];
    trajectory->points[1].t_from_start_s = 1.0;
    trajectory->points[1].position_world_m.x() += 0.1;
    trajectory->points[1].arrival_eligible = true;
    return trajectory;
}

void CheckGoalContract()
{
    GoalCommand point;
    point.command_id = 1;
    point.kind = GoalKind::kPoint;
    point.point_m = Eigen::Vector3d(0.4, 0.2, 0.3);
    Check(!ValidateGoalCommand(point), "finite mount-frame point accepted");

    GoalCommand circle;
    circle.command_id = 2;
    circle.kind = GoalKind::kCircle;
    circle.circle_centre_m = Eigen::Vector3d(0.4, 0.2, 0.3);
    circle.circle_radius_m = 0.1;
    circle.circle_normal = Eigen::Vector3d::UnitX();
    circle.circle_duration_s = 5.0;
    circle.orientation = GoalOrientation::kRadialInward;
    Check(!ValidateGoalCommand(circle), "finite mount-frame circle accepted");

    circle.circle_radius_m = 0.0;
    Check(ValidateGoalCommand(circle).has_value(),
          "zero-radius circle rejected");
}

void CheckPreemptHoldsAndRejectsSupersededPlan()
{
    CartesianTrajectoryMailbox mailbox;
    CartesianReferenceSource source(mailbox);
    RobotState state;
    state.world_fresh = true;
    state.world_sequence = 10;
    MeasuredCartesianState measured;
    measured.ee_pose_world.rotation = Eigen::Matrix3d::Identity();

    ControllerStatus startup;
    source.Get(state, measured, 0.002, 0.0, GoalPreemptCommand{}, startup);

    mailbox.Publish(Trajectory(1, measured.ee_pose_world.position_m));
    ControllerStatus activated;
    source.Get(state, measured, 0.002, 0.0, GoalPreemptCommand{}, activated);
    Check(activated.cartesian_traj_activated, "first trajectory activates");
    Check(source.state() == CartesianReferenceState::kTracking,
          "source tracks first trajectory");

    measured.ee_pose_world.position_m = Eigen::Vector3d(0.02, 0.03, 0.04);
    ControllerStatus preempted;
    const PoseReference hold = source.Get(
        state, measured, 0.002, 0.0, GoalPreemptCommand{true, 2}, preempted);
    Check(preempted.cartesian_traj_cancelled_edge,
          "new goal cancels active reference");
    Check(source.state() == CartesianReferenceState::kHolding,
          "controller holds while replacement is planned");
    Check((hold.ee_pose_world.position_m - measured.ee_pose_world.position_m).norm() <
              1e-12,
          "preempt hold is captured from current measurement");

    mailbox.Publish(Trajectory(1, measured.ee_pose_world.position_m));
    ControllerStatus stale;
    source.Get(state, measured, 0.002, 0.0, GoalPreemptCommand{}, stale);
    Check(stale.cartesian_traj_rejected,
          "superseded trajectory cannot reactivate");
    Check(source.state() == CartesianReferenceState::kHolding,
          "rejected stale plan leaves controller holding");

    mailbox.Publish(Trajectory(2, measured.ee_pose_world.position_m));
    ControllerStatus replacement;
    source.Get(state, measured, 0.002, 0.0, GoalPreemptCommand{}, replacement);
    Check(replacement.cartesian_traj_activated,
          "replacement trajectory activates");
    Check(source.state() == CartesianReferenceState::kTracking,
          "controller returns to tracking");
}

} // namespace

int main()
{
    CheckGoalContract();
    CheckPreemptHoldsAndRejectsSupersededPlan();
    if (failures == 0)
        std::puts("test_goal_preempt: all assertions passed");
    return failures == 0 ? 0 : 1;
}
