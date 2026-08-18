#include "CartesianReference.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "ExecutionConfig.h"
#include "ReactiveLaw.h"

CartesianReferenceSource::CartesianReferenceSource(
    CartesianTrajectoryMailbox& mailbox, const ExecutionConfig& config)
    : mailbox_(mailbox),
      arrival_position_tolerance_m_(config.arrival_position_tolerance_m),
      arrival_orientation_tolerance_rad_(
          config.arrival_orientation_tolerance_rad),
      world_prolonged_stale_s_(config.world_prolonged_stale_s)
{
}

CartesianReferenceSource::CartesianReferenceSource(
    CartesianTrajectoryMailbox& mailbox)
    : CartesianReferenceSource(mailbox, ProductionExecutionConfig())
{
}

PoseReference CartesianReferenceSource::HoldingReference() const
{
    PoseReference reference;
    reference.ee_pose_world = hold_world_;
    reference.ee_twist_world = Twist{};
    reference.trajectory_id = hold_trajectory_id_;
    reference.planner_vicon_sequence = hold_planner_vicon_sequence_;
    reference.t_from_start_s = hold_reference_time_s_;
    reference.arrival_eligible = hold_arrival_eligible_;
    return reference;
}

PoseReference CartesianReferenceSource::TrackingReference(double time_s) const
{
    const auto& points = active_->points;
    PoseReference reference;
    reference.trajectory_id = active_->trajectory_id;
    reference.planner_vicon_sequence = active_->planner_vicon_sequence;

    if (time_s <= points.front().t_from_start_s) {
        const auto& point = points.front();
        reference.ee_pose_world.position_m = point.position_world_m;
        reference.ee_pose_world.rotation =
            point.orientation_world.toRotationMatrix();
        reference.ee_twist_world.linear_m_s =
            point.linear_velocity_world_m_s;
        reference.ee_twist_world.angular_rad_s =
            point.angular_velocity_world_rad_s;
        reference.t_from_start_s = point.t_from_start_s;
        reference.arrival_eligible = false;
        return reference;
    }

    if (time_s >= points.back().t_from_start_s) {
        const auto& point = points.back();
        reference.ee_pose_world.position_m = point.position_world_m;
        reference.ee_pose_world.rotation =
            point.orientation_world.toRotationMatrix();
        reference.ee_twist_world = Twist{};
        reference.t_from_start_s = point.t_from_start_s;
        reference.arrival_eligible = true;
        return reference;
    }

    const auto upper = std::upper_bound(
        points.begin(), points.end(), time_s,
        [](double time, const WorldCartesianTrajectoryPoint& point) {
            return time < point.t_from_start_s;
        });
    const auto& after = *upper;
    const auto& before = *(upper - 1);
    const double alpha =
        (time_s - before.t_from_start_s) /
        (after.t_from_start_s - before.t_from_start_s);
    reference.ee_pose_world.position_m =
        (1.0 - alpha) * before.position_world_m +
        alpha * after.position_world_m;
    Eigen::Quaterniond q_before = before.orientation_world;
    Eigen::Quaterniond q_after = after.orientation_world;
    if (q_before.coeffs().dot(q_after.coeffs()) < 0.0)
        q_after.coeffs() *= -1.0;
    reference.ee_pose_world.rotation =
        q_before.slerp(alpha, q_after).normalized().toRotationMatrix();
    reference.ee_twist_world.linear_m_s =
        (1.0 - alpha) * before.linear_velocity_world_m_s +
        alpha * after.linear_velocity_world_m_s;
    reference.ee_twist_world.angular_rad_s =
        (1.0 - alpha) * before.angular_velocity_world_rad_s +
        alpha * after.angular_velocity_world_rad_s;
    reference.t_from_start_s = time_s;
    reference.arrival_eligible = false;
    return reference;
}

bool CartesianReferenceSource::TryActivate(
    std::unique_ptr<WorldCartesianTrajectory> incoming,
    const RobotState& state,
    const MeasuredCartesianState& measured,
    ControllerStatus& status)
{
    const bool provenance_valid = incoming->trajectory_id != 0 &&
        incoming->planner_vicon_sequence != 0 && state.world_sequence != 0 &&
        incoming->planner_vicon_sequence <= state.world_sequence &&
        incoming->planner_vicon_sequence >= minimum_planner_vicon_sequence_;

    double position_error_m = std::numeric_limits<double>::infinity();
    double orientation_error_rad = std::numeric_limits<double>::infinity();
    if (!incoming->points.empty()) {
        const auto& first = incoming->points.front();
        position_error_m =
            (first.position_world_m - measured.ee_pose_world.position_m).norm();
        orientation_error_rad = RotationLog(
            first.orientation_world.toRotationMatrix() *
            measured.ee_pose_world.rotation.transpose()).norm();
    }
    status.cartesian_traj_start_position_error_m = position_error_m;
    status.cartesian_traj_start_orientation_error_rad = orientation_error_rad;

    const bool continuous =
        position_error_m <= arrival_position_tolerance_m_ &&
        orientation_error_rad <= arrival_orientation_tolerance_rad_;
    if (!provenance_valid || !continuous) {
        status.cartesian_traj_rejected = true;
        mailbox_.Retire(std::move(incoming));
        return false;
    }

    mailbox_.Retire(std::move(active_));
    active_ = std::move(incoming);
    state_ = CartesianReferenceState::kTracking;
    trajectory_time_s_ = 0.0;
    complete_reported_ = false;
    status.cartesian_traj_activated = true;
    status.cartesian_traj_points = static_cast<int>(active_->points.size());
    status.cartesian_traj_duration_s =
        active_->points.back().t_from_start_s;
    return true;
}

PoseReference CartesianReferenceSource::Get(
    const RobotState& state,
    const MeasuredCartesianState& measured,
    double dt_s,
    double world_stale_elapsed_s,
    ControllerStatus& status)
{
    const bool valid_dt = std::isfinite(dt_s) && dt_s > 0.0;
    if (!state.world_fresh) {
        if (!prolonged_stale_handled_ &&
            world_stale_elapsed_s >= world_prolonged_stale_s_ &&
            state_ != CartesianReferenceState::kAwaitingWorld) {
            if (state_ == CartesianReferenceState::kTracking && active_) {
                const PoseReference paused = TrackingReference(trajectory_time_s_);
                hold_world_ = paused.ee_pose_world;
                hold_reference_time_s_ = paused.t_from_start_s;
                mailbox_.Retire(std::move(active_));
                status.cartesian_traj_cancelled_edge = true;
            }
            hold_trajectory_id_ = 0;
            hold_planner_vicon_sequence_ = 0;
            hold_arrival_eligible_ = false;
            state_ = CartesianReferenceState::kHolding;
            needs_replan_on_recovery_ = true;
            prolonged_stale_handled_ = true;
        }
    } else {
        const bool recovered = !previous_world_fresh_;
        prolonged_stale_handled_ = false;
        if (recovered && needs_replan_on_recovery_) {
            hold_world_ = measured.ee_pose_world;
            hold_trajectory_id_ = 0;
            hold_planner_vicon_sequence_ = 0;
            hold_reference_time_s_ = 0.0;
            hold_arrival_eligible_ = false;
            state_ = CartesianReferenceState::kHolding;
            needs_replan_on_recovery_ = false;
            minimum_planner_vicon_sequence_ = state.world_sequence;
            status.request_replan_edge = true;
        }
    }

    if (state_ == CartesianReferenceState::kAwaitingWorld) {
        // Before a trusted world sample exists, update this every cycle so it
        // is exactly zero-error and cannot pull toward a fictitious origin.
        hold_world_ = measured.ee_pose_world;
        hold_trajectory_id_ = 0;
        hold_planner_vicon_sequence_ = 0;
        hold_reference_time_s_ = 0.0;
        hold_arrival_eligible_ = false;
        if (state.world_fresh && state.world_sequence != 0) {
            state_ = CartesianReferenceState::kHolding;
            status.request_replan_edge = true;
        }
    }

    // Leave a complete candidate in the mailbox while world is stale: there
    // is no trustworthy handover measurement or provenance clock yet.
    if (state.world_fresh && state.world_sequence != 0) {
        if (std::unique_ptr<WorldCartesianTrajectory> incoming = mailbox_.Take())
            TryActivate(std::move(incoming), state, measured, status);
    }

    PoseReference reference;
    if (state_ != CartesianReferenceState::kTracking) {
        reference = HoldingReference();
    } else {
        reference = TrackingReference(trajectory_time_s_);
        const double final_time = active_->points.back().t_from_start_s;
        if (trajectory_time_s_ >= final_time) {
            hold_world_ = reference.ee_pose_world;
            hold_trajectory_id_ = reference.trajectory_id;
            hold_planner_vicon_sequence_ = reference.planner_vicon_sequence;
            hold_reference_time_s_ = final_time;
            hold_arrival_eligible_ = true;
            state_ = CartesianReferenceState::kHolding;
            if (!complete_reported_) {
                complete_reported_ = true;
                status.cartesian_traj_complete_edge = true;
            }
            mailbox_.Retire(std::move(active_));
            reference = HoldingReference();
        } else if (state.world_fresh && valid_dt) {
            trajectory_time_s_ = std::min(final_time,
                                          trajectory_time_s_ + dt_s);
        }
    }

    status.cartesian_trajectory_id = reference.trajectory_id;
    status.planner_vicon_sequence = reference.planner_vicon_sequence;
    status.cartesian_reference_time_s = reference.t_from_start_s;
    previous_world_fresh_ = state.world_fresh;
    return reference;
}
