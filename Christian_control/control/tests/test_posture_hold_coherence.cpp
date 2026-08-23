//
// Posture through the CartesianReferenceSource state machine (review
// finding H2, 2026-08-23). Five hand-coordinated hold sites must keep the
// pose and the posture flag paired: a hold whose pose came from a PLAN
// keeps that plan's posture (completion keeps the terminal posture — the
// exact case where joint 6 drifted to its boundary); a hold whose pose is
// a MEASUREMENT carries no posture opinion. Hardware-free: mailbox and
// reference source only.
//

#include <cmath>
#include <cstdio>
#include <memory>

#include "CartesianReference.h"
#include "ExecutionConfig.h"
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

// A 1 s two-point trajectory; posture 0.1..0.8 rad per joint at the start,
// +0.2 at the end, so interpolation and the terminal value are separable.
std::unique_ptr<WorldCartesianTrajectory> PostureTrajectory(
    std::uint64_t id, const Eigen::Vector3d& start, bool with_posture)
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
    if (with_posture) {
        for (int j = 0; j < 7; ++j) {
            trajectory->points[0].posture_rad(j) = 0.1 * (j + 1);
            trajectory->points[1].posture_rad(j) = 0.1 * (j + 1) + 0.2;
        }
        trajectory->points[0].has_posture = true;
        trajectory->points[1].has_posture = true;
    }
    return trajectory;
}

ExecutionConfig TestConfig()
{
    ExecutionConfig config = ProductionExecutionConfig();
    config.world_prolonged_stale_s = 0.5;
    return config;
}

void CheckTrackingCompletionAndPreempt()
{
    CartesianTrajectoryMailbox mailbox;
    CartesianReferenceSource source(mailbox, TestConfig());
    RobotState state;
    state.world_fresh = true;
    state.world_sequence = 10;
    MeasuredCartesianState measured;
    measured.ee_pose_world.rotation = Eigen::Matrix3d::Identity();

    ControllerStatus startup;
    source.Get(state, measured, 0.002, 0.0, GoalPreemptCommand{}, startup);

    mailbox.Publish(
        PostureTrajectory(1, measured.ee_pose_world.position_m, true));
    ControllerStatus activated;
    const PoseReference at_start = source.Get(
        state, measured, 0.002, 0.0, GoalPreemptCommand{}, activated);
    Check(activated.cartesian_traj_activated, "posture trajectory activates");
    Check(at_start.has_posture, "tracking reference carries posture");
    Check(std::abs(at_start.posture_rad(0) - 0.1) < 0.01,
          "posture near the start value at activation");

    // Halfway: interpolated posture between 0.1 and 0.3 on joint 1.
    const PoseReference mid = source.Get(
        state, measured, 0.5, 0.0, GoalPreemptCommand{}, activated);
    Check(mid.has_posture, "mid-trajectory reference carries posture");
    Check(mid.posture_rad(0) > 0.1 && mid.posture_rad(0) < 0.3,
          "mid-trajectory posture is interpolated");

    // Run past the end: completion hold keeps the TERMINAL posture.
    ControllerStatus completing;
    source.Get(state, measured, 0.6, 0.0, GoalPreemptCommand{}, completing);
    ControllerStatus completed;
    const PoseReference final_hold = source.Get(
        state, measured, 0.6, 0.0, GoalPreemptCommand{}, completed);
    Check(source.state() == CartesianReferenceState::kHolding,
          "completed trajectory becomes a hold");
    Check(final_hold.has_posture, "completion hold keeps a posture");
    Check(std::abs(final_hold.posture_rad(0) - 0.3) < 1e-9,
          "completion hold posture is the terminal posture");

    // A goal preempt replaces the hold with the measured pose: no posture.
    ControllerStatus preempted;
    const PoseReference preempt_hold = source.Get(
        state, measured, 0.002, 0.0, GoalPreemptCommand{true, 2}, preempted);
    Check(!preempt_hold.has_posture,
          "preempt hold is a measured pose and carries no posture");
}

void CheckStalePauseKeepsPosture()
{
    CartesianTrajectoryMailbox mailbox;
    CartesianReferenceSource source(mailbox, TestConfig());
    RobotState state;
    state.world_fresh = true;
    state.world_sequence = 10;
    MeasuredCartesianState measured;
    measured.ee_pose_world.rotation = Eigen::Matrix3d::Identity();

    ControllerStatus status;
    source.Get(state, measured, 0.002, 0.0, GoalPreemptCommand{}, status);
    mailbox.Publish(
        PostureTrajectory(1, measured.ee_pose_world.position_m, true));
    source.Get(state, measured, 0.002, 0.0, GoalPreemptCommand{}, status);
    source.Get(state, measured, 0.4, 0.0, GoalPreemptCommand{}, status);
    Check(source.state() == CartesianReferenceState::kTracking,
          "tracking before the world goes stale");

    // World stale past the prolonged threshold: the pause hold keeps the
    // paused sample's posture.
    state.world_fresh = false;
    ControllerStatus paused_status;
    const PoseReference paused = source.Get(
        state, measured, 0.002, 1.0, GoalPreemptCommand{}, paused_status);
    Check(source.state() == CartesianReferenceState::kHolding,
          "prolonged stale world pauses the trajectory");
    Check(paused.has_posture, "stale-pause hold keeps the paused posture");

    // World recovers: the replan hold is a measured pose, posture dropped.
    state.world_fresh = true;
    state.world_sequence = 11;
    ControllerStatus recovered;
    const PoseReference replan_hold = source.Get(
        state, measured, 0.002, 0.0, GoalPreemptCommand{}, recovered);
    Check(recovered.request_replan_edge, "recovery requests a replan");
    Check(!replan_hold.has_posture,
          "recovery hold is a measured pose and carries no posture");
}

void CheckPostureFreeTrajectoryStaysPostureFree()
{
    CartesianTrajectoryMailbox mailbox;
    CartesianReferenceSource source(mailbox, TestConfig());
    RobotState state;
    state.world_fresh = true;
    state.world_sequence = 10;
    MeasuredCartesianState measured;
    measured.ee_pose_world.rotation = Eigen::Matrix3d::Identity();

    ControllerStatus status;
    source.Get(state, measured, 0.002, 0.0, GoalPreemptCommand{}, status);
    mailbox.Publish(
        PostureTrajectory(1, measured.ee_pose_world.position_m, false));
    const PoseReference tracking = source.Get(
        state, measured, 0.002, 0.0, GoalPreemptCommand{}, status);
    Check(status.cartesian_traj_activated, "posture-free trajectory activates");
    Check(!tracking.has_posture,
          "posture-free trajectory yields posture-free references");
    source.Get(state, measured, 0.6, 0.0, GoalPreemptCommand{}, status);
    const PoseReference hold = source.Get(
        state, measured, 0.6, 0.0, GoalPreemptCommand{}, status);
    Check(!hold.has_posture,
          "posture-free completion hold carries no posture");
}

} // namespace

int main()
{
    CheckTrackingCompletionAndPreempt();
    CheckStalePauseKeepsPosture();
    CheckPostureFreeTrajectoryStaysPostureFree();
    if (failures == 0)
        std::puts("test_posture_hold_coherence: all assertions passed");
    return failures == 0 ? 0 : 1;
}
