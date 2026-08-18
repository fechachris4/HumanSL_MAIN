#undef NDEBUG

#include <cassert>
#include <cmath>
#include <memory>

#include "CartesianReference.h"
#include "Config.h"

namespace {

MeasuredCartesianState Measured(const Eigen::Vector3d& position,
                                const Eigen::Matrix3d& rotation =
                                    Eigen::Matrix3d::Identity())
{
    MeasuredCartesianState measured;
    measured.ee_pose_world.position_m = position;
    measured.ee_pose_world.rotation = rotation;
    return measured;
}

RobotState FreshState(std::uint64_t sequence)
{
    RobotState state{};
    state.world_fresh = true;
    state.world_sequence = sequence;
    return state;
}

// Production's world-stale clock is owned by ArmExecutionCore
// (ExecutionCore.h): zeroed on every fresh cycle, advanced by the cycle dt
// while stale, and handed to Get post-update. This helper reproduces that
// contract so these direct policy tests drive Get exactly as the core does.
PoseReference Step(CartesianReferenceSource& source, double& stale_elapsed_s,
                   const RobotState& state,
                   const MeasuredCartesianState& measured, double dt_s,
                   ControllerStatus& status)
{
    stale_elapsed_s = state.world_fresh ? 0.0 : stale_elapsed_s + dt_s;
    return source.Get(state, measured, dt_s, stale_elapsed_s, status);
}

std::unique_ptr<WorldCartesianTrajectory> Trajectory(
    std::uint64_t id,
    std::uint64_t planner_sequence,
    const CartesianPose& start,
    const CartesianPose& finish)
{
    auto trajectory = std::make_unique<WorldCartesianTrajectory>();
    trajectory->trajectory_id = id;
    trajectory->planner_vicon_sequence = planner_sequence;
    WorldCartesianTrajectoryPoint first;
    first.position_world_m = start.position_m;
    first.orientation_world = Eigen::Quaterniond(start.rotation);
    first.linear_velocity_world_m_s = Eigen::Vector3d(0.2, 0.4, 0.6);
    first.angular_velocity_world_rad_s = Eigen::Vector3d(0.1, 0.2, 0.3);
    WorldCartesianTrajectoryPoint last;
    last.t_from_start_s = 1.0;
    last.position_world_m = finish.position_m;
    // Deliberately choose the opposite quaternion hemisphere. Interpolation
    // must still take the short physical rotation.
    last.orientation_world = Eigen::Quaterniond(finish.rotation);
    last.orientation_world.coeffs() *= -1.0;
    last.arrival_eligible = true;
    trajectory->points = {first, last};
    assert(!ValidateWorldCartesianTrajectory(*trajectory).has_value());
    return trajectory;
}

void CheckStartupAndFreshHold()
{
    CartesianTrajectoryMailbox mailbox;
    CartesianReferenceSource source(mailbox);
    double stale_s = 0.0;
    RobotState state{};
    ControllerStatus status;

    const auto first = Measured(Eigen::Vector3d(1.0, 2.0, 3.0));
    PoseReference reference = Step(source, stale_s, state, first, 0.1, status);
    assert((reference.ee_pose_world.position_m - first.ee_pose_world.position_m)
               .norm() == 0.0);
    assert(reference.ee_twist_world.linear_m_s.isZero(0.0));
    assert(!reference.arrival_eligible);
    assert(source.state() == CartesianReferenceState::kAwaitingWorld);

    const auto moved = Measured(Eigen::Vector3d(2.0, 3.0, 4.0));
    reference = Step(source, stale_s, state, moved, 0.1, status);
    assert((reference.ee_pose_world.position_m - moved.ee_pose_world.position_m)
               .norm() == 0.0 &&
           "without Vicon, startup reference must remain zero error");

    state = FreshState(10);
    status = ControllerStatus{};
    reference = Step(source, stale_s, state, moved, 0.1, status);
    assert(source.state() == CartesianReferenceState::kHolding);
    assert(status.request_replan_edge &&
           "first fresh world hold requests exactly one plan");
    const auto moved_again = Measured(Eigen::Vector3d(3.0, 4.0, 5.0));
    status = ControllerStatus{};
    reference = Step(source, stale_s, state, moved_again, 0.1, status);
    assert((reference.ee_pose_world.position_m - moved.ee_pose_world.position_m)
               .norm() == 0.0 &&
           "first fresh measurement captures one fixed world hold");
    assert(!status.request_replan_edge);
}

void CheckBriefStaleClockPauseAtControlRate()
{
    CartesianTrajectoryMailbox mailbox;
    CartesianReferenceSource source(mailbox);
    double stale_s = 0.0;
    RobotState state = FreshState(300);
    const MeasuredCartesianState measured =
        Measured(Eigen::Vector3d::Zero());
    CartesianPose finish;
    finish.position_m = Eigen::Vector3d(1.0, 0.0, 0.0);
    mailbox.Publish(Trajectory(30, 300, measured.ee_pose_world, finish));

    ControllerStatus status;
    Step(source, stale_s, state, measured, 0.0, status);
    for (int i = 0; i < 50; ++i) {
        status = ControllerStatus{};
        Step(source, stale_s, state, measured, 0.002, status);
    }

    state.world_fresh = false;
    status = ControllerStatus{};
    const PoseReference at_gap = Step(source, stale_s, state, measured, 0.002, status);
    assert(std::abs(at_gap.t_from_start_s - 0.100) < 1e-12);
    for (int i = 0; i < 49; ++i) {
        status = ControllerStatus{};
        const PoseReference paused = Step(source, stale_s, state, measured, 0.002, status);
        assert(std::abs(paused.t_from_start_s - 0.100) < 1e-12);
        assert((paused.ee_pose_world.position_m -
                at_gap.ee_pose_world.position_m).norm() < 1e-12);
        assert(!status.cartesian_traj_cancelled_edge);
    }

    state.world_fresh = true;
    status = ControllerStatus{};
    const PoseReference recovered = Step(source, stale_s, state, measured, 0.002, status);
    assert(std::abs(recovered.t_from_start_s - 0.100) < 1e-12);
    status = ControllerStatus{};
    const PoseReference resumed = Step(source, stale_s, state, measured, 0.002, status);
    assert(std::abs(resumed.t_from_start_s - 0.102) < 1e-12 &&
           "brief recovery resumes without wall-clock catch-up");
}

void CheckActivationInterpolationAndFinalHold()
{
    CartesianTrajectoryMailbox mailbox;
    CartesianReferenceSource source(mailbox);
    double stale_s = 0.0;
    RobotState state = FreshState(50);
    const MeasuredCartesianState measured =
        Measured(Eigen::Vector3d(1.0, 2.0, 3.0));
    CartesianPose finish;
    finish.position_m = Eigen::Vector3d(3.0, 4.0, 5.0);
    finish.rotation =
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    mailbox.Publish(Trajectory(7, 40, measured.ee_pose_world, finish));

    ControllerStatus status;
    PoseReference reference = Step(source, stale_s, state, measured, 0.5, status);
    assert(status.cartesian_traj_activated);
    assert(reference.trajectory_id == 7);
    assert(reference.planner_vicon_sequence == 40);
    assert(reference.t_from_start_s == 0.0);
    assert(!reference.arrival_eligible);
    assert((reference.ee_pose_world.position_m -
            measured.ee_pose_world.position_m).norm() < 1e-12);

    status = ControllerStatus{};
    reference = Step(source, stale_s, state, measured, 0.5, status);
    assert(std::abs(reference.t_from_start_s - 0.5) < 1e-12);
    assert((reference.ee_pose_world.position_m -
            Eigen::Vector3d(2.0, 3.0, 4.0)).norm() < 1e-12);
    assert((reference.ee_twist_world.linear_m_s -
            Eigen::Vector3d(0.1, 0.2, 0.3)).norm() < 1e-12);
    assert((reference.ee_twist_world.angular_rad_s -
            Eigen::Vector3d(0.05, 0.1, 0.15)).norm() < 1e-12);
    const Eigen::Matrix3d expected_mid =
        Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    assert((reference.ee_pose_world.rotation - expected_mid).norm() < 1e-12);
    assert(!reference.arrival_eligible);

    status = ControllerStatus{};
    reference = Step(source, stale_s, state, measured, 0.1, status);
    assert(status.cartesian_traj_complete_edge);
    assert(mailbox.DeleteRetired() == 1 &&
           "completed trajectory storage is reclaimed off the cyclic path");
    assert(reference.arrival_eligible);
    assert(reference.ee_twist_world.linear_m_s.isZero(0.0));
    assert(reference.ee_twist_world.angular_rad_s.isZero(0.0));
    assert((reference.ee_pose_world.position_m - finish.position_m).norm() <
           1e-12);

    status = ControllerStatus{};
    reference = Step(source, stale_s, state, measured, 1.0, status);
    assert(!status.cartesian_traj_complete_edge);
    assert(reference.ee_twist_world.linear_m_s.isZero(0.0));
    assert(reference.t_from_start_s == 1.0);
}

void CheckClockPausesAndBadReplacementCannotOverwrite()
{
    CartesianTrajectoryMailbox mailbox;
    CartesianReferenceSource source(mailbox);
    double stale_s = 0.0;
    RobotState state = FreshState(100);
    const MeasuredCartesianState measured =
        Measured(Eigen::Vector3d(0.0, 0.0, 0.0));
    CartesianPose finish;
    finish.position_m = Eigen::Vector3d(1.0, 0.0, 0.0);
    mailbox.Publish(Trajectory(10, 90, measured.ee_pose_world, finish));
    ControllerStatus status;
    Step(source, stale_s, state, measured, 0.25, status); // returns t=0, advances to .25

    state.world_fresh = false;
    PoseReference paused = Step(source, stale_s, state, measured, 0.05, status);
    assert(std::abs(paused.t_from_start_s - 0.25) < 1e-12);
    paused = Step(source, stale_s, state, measured, 0.05, status);
    assert(std::abs(paused.t_from_start_s - 0.25) < 1e-12);

    state.world_fresh = true;
    state.world_sequence = 100;
    auto future = Trajectory(11, 101, measured.ee_pose_world, finish);
    mailbox.Publish(std::move(future));
    status = ControllerStatus{};
    PoseReference still_old = Step(source, stale_s, state, measured, 0.0, status);
    assert(status.cartesian_traj_rejected);
    assert(still_old.trajectory_id == 10);

    CartesianPose discontinuous = measured.ee_pose_world;
    discontinuous.position_m.x() =
        config::kArrivalToleranceM * 2.0;
    mailbox.Publish(Trajectory(12, 95, discontinuous, finish));
    status = ControllerStatus{};
    still_old = Step(source, stale_s, state, measured, 0.0, status);
    assert(status.cartesian_traj_rejected);
    assert(status.cartesian_traj_start_position_error_m >
           config::kArrivalToleranceM);
    assert(still_old.trajectory_id == 10 &&
           "a rejected replacement cannot partially overwrite the active plan");

    assert(mailbox.DeleteRetired() == 2 &&
           "rejected candidate storage is reclaimed off the cyclic path");
}

void CheckProlongedStaleCancelsAndReplansOnRecovery()
{
    CartesianTrajectoryMailbox mailbox;
    CartesianReferenceSource source(mailbox);
    double stale_s = 0.0;
    RobotState state = FreshState(100);
    const MeasuredCartesianState measured =
        Measured(Eigen::Vector3d(0.0, 0.0, 0.0));
    CartesianPose finish;
    finish.position_m = Eigen::Vector3d(1.0, 0.0, 0.0);
    mailbox.Publish(Trajectory(20, 100, measured.ee_pose_world, finish));
    ControllerStatus status;
    Step(source, stale_s, state, measured, 0.1, status);
    assert(status.cartesian_traj_activated);

    state.world_fresh = false;
    status = ControllerStatus{};
    PoseReference paused = Step(source, stale_s, state, measured, 0.1, status);
    assert(!status.cartesian_traj_cancelled_edge);
    const double paused_time = paused.t_from_start_s;
    status = ControllerStatus{};
    paused = Step(source, stale_s, state, measured, 0.1, status);
    assert(status.cartesian_traj_cancelled_edge);
    assert(source.state() == CartesianReferenceState::kHolding);
    assert(paused.trajectory_id == 0);
    assert(std::abs(paused.t_from_start_s - paused_time) < 1e-12);

    // A candidate based on the pre-gap world snapshot may arrive while
    // stale. It must not resume after a potentially moving-base blackout.
    mailbox.Publish(Trajectory(21, 100, paused.ee_pose_world, finish));
    state = FreshState(200);
    const MeasuredCartesianState recovered =
        Measured(Eigen::Vector3d(0.3, 0.4, 0.5));
    status = ControllerStatus{};
    const PoseReference hold = Step(source, stale_s, state, recovered, 0.002, status);
    assert(status.request_replan_edge);
    assert(status.cartesian_traj_rejected);
    assert(hold.trajectory_id == 0);
    assert((hold.ee_pose_world.position_m - recovered.ee_pose_world.position_m)
               .norm() < 1e-12);
    assert(mailbox.DeleteRetired() == 2 &&
           "cancelled and rejected trajectories are reclaimed off-cycle");

    status = ControllerStatus{};
    Step(source, stale_s, state, recovered, 0.002, status);
    assert(!status.request_replan_edge &&
           "recovery emits exactly one replan edge");
}

} // namespace

int main()
{
    CheckStartupAndFreshHold();
    CheckActivationInterpolationAndFinalHold();
    CheckBriefStaleClockPauseAtControlRate();
    CheckClockPausesAndBadReplacementCannotOverwrite();
    CheckProlongedStaleCancelsAndReplansOnRecovery();
    return 0;
}
