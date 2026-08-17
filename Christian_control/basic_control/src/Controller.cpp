#include "Controller.h"

#include <algorithm>
#include <cmath>

#include "ExecutionConfig.h"
#include "Frames.h"
#include "Kinematics.h"

namespace {

constexpr double kDegToRad = M_PI / 180.0;

} // namespace

TrackingController::TrackingController(DualArmKinematics& model,
                                       const ExecutionConfig& config)
    : model_(model),
      workspace_(std::make_unique<KinematicsWorkspace>(model.robot_model())),
      gains_(config.gains),
      zone_rad_(config.limit_avoid_zone_rad),
      arrival_position_tolerance_m_(config.arrival_position_tolerance_m),
      arrival_orientation_tolerance_rad_(
          config.arrival_orientation_tolerance_rad),
      null_ramp_duration_s_(config.null_ramp_duration_s),
      arrival_monitor_(config.arrival_dwell_s),
      timeout_monitor_(config.target_hold_s)
{
    for (int joint = 0; joint < 7; ++joint)
        limit_rad_[joint] = config.software_limit_deg[joint] * kDegToRad;
}

TrackingController::TrackingController(DualArmKinematics& model)
    : TrackingController(model, ProductionExecutionConfig())
{
}

TrackingController::~TrackingController() = default;

MeasuredCartesianState TrackingController::Measure(const RobotState& state)
{
    const PoseJacobian base =
        model_.ControlledPoseAndJacobian(state.q_rad, *workspace_);
    const pinocchio::SE3& mount_T_base =
        model_.MountFromBase(model_.controlled_arm());

    Twist mount_twist_world;
    if (state.world_mount_twist_valid) {
        mount_twist_world.linear_m_s = state.world_v_mountseg_m_s;
        mount_twist_world.angular_rad_s = state.world_w_mountseg_rad_s;
    }
    const world_frames::WorldArmState world =
        world_frames::ArmControllerState(
            {state.world_p_mountseg, state.world_R_mountseg},
            {mount_T_base.translation(), mount_T_base.rotation()},
            {base.position, base.rotation}, base.jacobian, state.qdot_rad_s,
            mount_twist_world);

    MeasuredCartesianState measured;
    measured.ee_pose_world = world.ee_pose_world;
    measured.jacobian_world = world.jacobian_world;
    measured.ee_twist_world = world.ee_twist_world;
    return measured;
}

Eigen::Matrix<double, 7, 1> TrackingController::DesiredVelocity(
    const RobotState& state,
    const MeasuredCartesianState& measured,
    const PoseReference& reference,
    double dt_s,
    ControllerStatus& status)
{
    if (!trajectory_seen_ || reference.trajectory_id != last_trajectory_id_) {
        trajectory_seen_ = true;
        last_trajectory_id_ = reference.trajectory_id;
        arrival_reported_ = false;
        arrival_monitor_.Rearm();
        timeout_monitor_.Rearm();
    }

    const Eigen::Vector3d e_pos =
        reference.ee_pose_world.position_m -
        measured.ee_pose_world.position_m;
    const Eigen::Vector3d e_rot = RotationLog(
        reference.ee_pose_world.rotation *
        measured.ee_pose_world.rotation.transpose());
    Eigen::Matrix<double, 6, 1> measured_twist;
    measured_twist.head<3>() = measured.ee_twist_world.linear_m_s;
    measured_twist.tail<3>() = measured.ee_twist_world.angular_rad_s;
    const Twist e_twist =
        TwistError(reference.ee_twist_world, measured_twist);

    const bool position_arrived =
        e_pos.norm() <= arrival_position_tolerance_m_;
    const bool orientation_arrived =
        !gains_.orientation_enabled ||
        e_rot.norm() <= arrival_orientation_tolerance_rad_;
    const bool in_tolerance = reference.arrival_eligible &&
                              position_arrived && orientation_arrived;
    if (!arrival_reported_ && arrival_monitor_.Update(in_tolerance, dt_s)) {
        arrival_reported_ = true;
        status.arrived_edge = true;
        status.arrival_error_m = e_pos.norm();
    }
    if (timeout_monitor_.Update(reference.arrival_eligible &&
                                    !arrival_reported_,
                                dt_s)) {
        status.not_reached_edge = true;
        status.arrival_error_m = e_pos.norm();
    }

    status.p_desired = reference.ee_pose_world.position_m;
    status.p_current = measured.ee_pose_world.position_m;
    status.rot_error_rad = e_rot.norm();
    status.tool_quat = Eigen::Quaterniond(measured.ee_pose_world.rotation);
    if (status.tool_quat.w() < 0.0)
        status.tool_quat.coeffs() = -status.tool_quat.coeffs();

    const Eigen::Matrix<double, 6, 6> jjt =
        measured.jacobian_world * measured.jacobian_world.transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>>
        eigensolver(jjt);
    status.sigma_min =
        std::sqrt(std::max(0.0, eigensolver.eigenvalues()(0)));

    ReactivePoseGains ramped_gains = gains_;
    ramped_gains.limit_avoid_gain_s_inv *=
        UnitRamp(state.t_s, null_ramp_duration_s_);
    const ReactiveSolution solution = SolveReactiveVelocityDetailed(
        measured.jacobian_world, e_pos, e_rot, e_twist.linear_m_s,
        e_twist.angular_rad_s, state.q_rad, limit_rad_, zone_rad_,
        ramped_gains);
    status.qdot_task_rad_s = solution.qdot_task_rad_s;
    status.qdot_null_rad_s = solution.qdot_null_rad_s;
    status.null_leak_m_s = solution.leak_twist.head<3>().norm();
    return solution.qdot_task_rad_s + solution.qdot_null_rad_s;
}
