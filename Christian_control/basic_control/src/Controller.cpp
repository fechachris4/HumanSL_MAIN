//
// Controller — TrackingController implementation. This is the only
// translation unit where the controller meets the Pinocchio model.
//

#include "Controller.h"

#include <limits>

#include "Config.h"
#include "Frames.h"
#include "Kinematics.h"

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

ReactivePoseGains ConfiguredGains()
{
    ReactivePoseGains gains;
    gains.kp_position_s_inv = config::kKpCartesian;
    gains.kp_rotation_s_inv = config::kKpRotation;
    gains.kd_position = config::kKdPosition;
    gains.kd_rotation = config::kKdRotation;
    gains.limit_avoid_gain_s_inv = config::kLimitAvoidGain;
    gains.dls_lambda = config::kDlsLambda;
    gains.orientation_enabled = config::kOrientationEnabled;
    gains.velocity_enabled = config::kVelocityTermEnabled;
    gains.null_space_enabled = config::kNullSpaceEnabled;
    return gains;
}
} // namespace

TrackingController::TrackingController(DualArmKinematics& model)
    : model_(model),
      workspace_(std::make_unique<KinematicsWorkspace>(model.dynamics())),
      gains_(ConfiguredGains()),
      world_hold_(config::kWorldHoldRampS, config::kWorldHoldMaxErrorM,
                  config::kWorldHoldMaxRotErrorRad,
                  config::kWorldHoldReanchorAfterS),
      arrival_monitor_(config::kArrivalDwellS),
      timeout_monitor_(config::kTargetHoldS)
{
    for (int i = 0; i < 7; ++i)
        limit_rad_[i] = config::kJointSoftwareLimitDeg[i] * kDegToRad;
    zone_rad_ = config::kLimitAvoidZoneDeg * kDegToRad;
}

TrackingController::~TrackingController() = default;

void TrackingController::Reset(const RobotState& state)
{
    const PoseJacobian ee =
        model_.ControlledPoseAndJacobian(state.q_rad, *workspace_);
    // Hold here: an empty reference (or a position-only target's missing
    // orientation) resolves to the pose measured at takeover.
    hold_position_ = ee.position;
    hold_rotation_ = ee.rotation;
    pose_sequence_seen_ = false;
    arrival_reported_ = true;
    arrival_monitor_.Rearm();
    timeout_monitor_.Rearm();
}

Eigen::Matrix<double, 7, 1>
TrackingController::DesiredVelocity(const RobotState& state,
                                    const Reference& reference, double dt_s,
                                    ControllerStatus& status)
{
    // Joint channel: tracked directly, with no model and no Cartesian
    // telemetry to fill — the command flows into the same clip and
    // integration path as the pose channel's.
    // Only an ACTIVELY SAMPLED trajectory owns the cycle outright. The
    // source's idle hold (startup pose / completed endpoint) is offered
    // to the world hold below first — production always carries a joint
    // reference, so gating on reference.joint alone made the world
    // hold unreachable (found on the first hardware run, 2026-08-13).
    if (reference.joint && !reference.joint_is_idle_hold) {
        followed_joint_reference_ = true;
        // Trajectory precedence (spec §2): the world anchor is dropped;
        // the first hold cycle after the trajectory re-engages fresh.
        world_hold_.Reset();
        const JointTrackingCommand command = SolveJointTracking(
            *reference.joint, state.q_rad, config::kKpJointTracking,
            config::kTrajFollowingErrorStopDeg * kDegToRad);
        status.joint_following_error_deg =
            command.max_abs_error_rad / kDegToRad;
        status.joint_following_error_stop = command.following_error_stop;
        const double not_computed = std::numeric_limits<double>::quiet_NaN();
        status.p_desired.setConstant(not_computed);
        status.p_current.setConstant(not_computed);
        return command.qdot_rad_s;
    }

    // Coming back to the pose channel after joint tracking, the hold pose
    // captured at takeover is stale by the whole trajectory: holding it would
    // command the arm back to where the run started. Re-seat it (and the
    // arrival gates) from the CURRENT measurement, exactly as takeover does,
    // before any pose command is computed.
    if (followed_joint_reference_) {
        followed_joint_reference_ = false;
        Reset(state);
    }

    // World assembly (Frames.h, ported from the simulation's frames.py —
    // the reference implementation). world_T_base = the ZOH Mount-segment
    // sample composed through MSeg_T_mount (≡ I until stage-2 calibration)
    // and the model's fixed mount→base offset. With the never-seen default
    // (identity sample) the "world" is simply the mount frame, and every
    // command below is algebraically identical to the pre-Vicon law —
    // rotating the frame of e and J changes no DLS solution.
    const pinocchio::SE3& mount_from_base =
        model_.MountFromBase(model_.controlled_arm());
    const world_frames::FramePose world_T_base = world_frames::ComposePose(
        {state.world_p_mountseg, state.world_R_mountseg},
        {mount_from_base.translation(), mount_from_base.rotation()});

    // A new operator target re-arms the arrival notice; the hold pose
    // never does.
    if (reference.pose &&
        (!pose_sequence_seen_ ||
         reference.pose->sequence != last_pose_sequence_))
    {
        pose_sequence_seen_ = true;
        last_pose_sequence_ = reference.pose->sequence;
        arrival_reported_ = false;
        arrival_monitor_.Rearm();
        timeout_monitor_.Rearm();
    }

    // The adapter composes the SAME full q from the controlled arm's
    // measured joints and the other arm's fixed nominal, then selects only
    // the controlled arm's 7 Jacobian columns.
    const PoseJacobian ee =
        model_.ControlledPoseAndJacobian(state.q_rad, *workspace_);

    // frames.py arm_controller_state: EE pose, Jacobian and measured twist
    // in the WORLD frame. The moving-body twist is ZERO this slice (the
    // feedback-only decision: no measured base twist exists yet), so the
    // transport term contributes nothing and V_base,E lands here later.
    const world_frames::WorldArmState world = world_frames::ArmControllerState(
        world_T_base, {}, {ee.position, ee.rotation}, ee.jacobian,
        state.qdot_rad_s, Twist{});

    // The desired pose, WORLD frame. Three producers, one convention:
    // an explicit pose reference (world semantics since the 2026-08-13
    // port), the auto-engaged world hold, or — Vicon untrusted — the
    // world image of the base-frame takeover hold, which tracks the base
    // exactly as the pre-Vicon controller did.
    world_frames::FramePose desired_world;
    if (reference.pose) {
        world_hold_.Reset(); // an explicit target outranks the hold
        hold_was_active_ = false;
        // Resolve the DECLARED frame to world once per cycle, exactly as
        // the sim's resolve_target_world does — a base-framed target
        // moves with the base (the pre-Vicon meaning), a world-framed
        // one holds in the room. Moving-body twist is zero this slice.
        world_frames::FramePose target_pose;
        target_pose.position_m = reference.pose->p_desired;
        target_pose.rotation =
            reference.pose->rotation
                ? *reference.pose->rotation
                : hold_rotation_; // hold orientation, declared frame's axes
        const world_frames::WorldTargetValue resolved =
            world_frames::ResolveTargetWorld(
                {state.world_p_mountseg, state.world_R_mountseg},
                {mount_from_base.translation(), mount_from_base.rotation()},
                Twist{},
                static_cast<world_frames::TargetFrame>(reference.pose->frame),
                target_pose, reference.pose->twist);
        desired_world = resolved.pose_world;
        if (!reference.pose->rotation) // hold orientation is base-frame FK
            desired_world.rotation =
                world_frames::ComposePose(world_T_base,
                                          {hold_position_, hold_rotation_})
                    .rotation;
        resolved_reference_twist_ = resolved.twist_world;
    } else {
        WorldHoldInput hold_in;
        hold_in.sample_fresh =
            config::kWorldHoldAutoEngage && state.world_fresh;
        hold_in.ee_pose_world = world.ee_pose_world;
        hold_in.t_s = state.t_s;
        const WorldHoldOutput hold = world_hold_.Update(hold_in);
        status.hold_state = static_cast<int>(hold.state);
        status.world_err_m = hold.error_norm_m;
        status.hold_reanchor_count = hold.reanchor_count;
        status.world_err_rot_rad = hold.error_rot_rad;
        if (hold.provides_target) {
            hold_was_active_ = true;
            world_hold_ever_engaged_ = true;
            status.hold_ramp = hold.ramp;
            desired_world = hold.target_world;
        } else {
            // Latch-off step guard (review finding, 2026-08-13): the
            // base-frame hold pose was seated at takeover and is stale
            // by however far the world hold has since moved the arm —
            // falling back to it verbatim would command a step. On the
            // transition out of an active hold, re-seat it from the
            // CURRENT measurement, exactly as the post-trajectory
            // re-seat does; the fallback then starts with zero error.
            if (hold_was_active_) {
                hold_was_active_ = false;
                hold_position_ = ee.position;
                hold_rotation_ = ee.rotation;
            }
            // Before the world hold has EVER engaged, an idle joint
            // reference is followed exactly as before this slice — the
            // no-Vicon behaviour is bit-for-bit today's. After an
            // engage, the source's idle q is stale by the whole held
            // motion (it would snap the arm back to startup joints), so
            // the fallback is the re-seated Cartesian hold instead.
            if (!world_hold_ever_engaged_ && reference.joint) {
                const JointTrackingCommand command = SolveJointTracking(
                    *reference.joint, state.q_rad,
                    config::kKpJointTracking,
                    config::kTrajFollowingErrorStopDeg * kDegToRad);
                status.joint_following_error_deg =
                    command.max_abs_error_rad / kDegToRad;
                status.joint_following_error_stop =
                    command.following_error_stop;
                const double nan = std::numeric_limits<double>::quiet_NaN();
                status.p_desired.setConstant(nan);
                status.p_current.setConstant(nan);
                return command.qdot_rad_s;
            }
            // Inactive or latched off: today's behaviour, expressed in
            // world so there is exactly one error convention below.
            desired_world = world_frames::ComposePose(
                world_T_base, {hold_position_, hold_rotation_});
        }
    }

    // Equation 1: pose error, reference minus actual — WORLD frame, as in
    // reactive_controller.pose_error (ReactiveLaw.h supplies the log map).
    const Eigen::Vector3d e_pos =
        desired_world.position_m - world.ee_pose_world.position_m;
    const Eigen::Vector3d e_rot = RotationLog(
        desired_world.rotation * world.ee_pose_world.rotation.transpose());

    // Equation 2: twist error, reference minus measured world twist —
    // reactive_controller.twist_error. The hold's reference twist is
    // genuinely zero; an explicit reference brings its own (world frame).
    Eigen::Matrix<double, 6, 1> measured_twist;
    measured_twist.head<3>() = world.ee_twist_world.linear_m_s;
    measured_twist.tail<3>() = world.ee_twist_world.angular_rad_s;
    const Twist reference_twist =
        reference.pose ? resolved_reference_twist_ : Twist{};
    const Twist e_twist = TwistError(reference_twist, measured_twist);

    // Arrival is edge-triggered only on a terminal source sample.  A moving
    // Cartesian profile may pass through a target-shaped point; it must not
    // advance the mailbox until it explicitly becomes arrival-eligible.
    const bool arrival_eligible = reference.pose && reference.pose->arrival_eligible;
    const bool position_arrived = e_pos.norm() <= config::kArrivalToleranceM;
    const bool orientation_arrived = !gains_.orientation_enabled ||
        e_rot.norm() <= config::kArrivalOrientationToleranceRad;
    const bool in_tolerance =
        arrival_eligible && position_arrived && orientation_arrived;

    // Positive: debounce arrival over kArrivalDwellS, then edge-fire once per
    // target so Runner/Targets keep their once-per-target semantics.
    const bool reported = arrival_monitor_.Update(in_tolerance, dt_s);
    if (!arrival_reported_ && reported) {
        arrival_reported_ = true;
        status.arrived_edge = true;
        status.arrival_error_m = e_pos.norm();
    }

    // Negative: while parked at a target and not yet arrived, fire a one-shot
    // non-arrival edge after kTargetHoldS. Reporting only — no motion change.
    const bool waiting = arrival_eligible && !arrival_reported_;
    if (timeout_monitor_.Update(waiting, dt_s)) {
        status.not_reached_edge = true;
        status.arrival_error_m = e_pos.norm();
    }
    // Log continuity: pd_*/p_* columns stay in the BASE frame as their
    // Hardware.h documentation says — the base image of the world target.
    status.p_desired = world_T_base.rotation.transpose() *
                       (desired_world.position_m - world_T_base.position_m);
    status.p_current = ee.position;
    status.rot_error_rad = e_rot.norm();
    status.tool_quat = Eigen::Quaterniond(ee.rotation);
    if (status.tool_quat.w() < 0.0)
        status.tool_quat.coeffs() = -status.tool_quat.coeffs();

    // σ_min of the full 6×7 task Jacobian — 6×6 self-adjoint solve, no
    // allocation. The Jacobian mixes meters-per-radian and
    // radian-per-radian rows, so watch this value's trend, not its size.
    const Eigen::Matrix<double, 6, 6> jjt =
        world.jacobian_world * world.jacobian_world.transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(jjt);
    status.sigma_min = std::sqrt(std::max(0.0, eigensolver.eigenvalues()(0)));

    // Equations 3-6: task twist -> DLS -> null-space (ReactiveLaw.h). The
    // decomposition goes out through status so the Runner can print and log
    // the two terms that the summed command hides.
    ReactivePoseGains ramped_gains = gains_;
    ramped_gains.limit_avoid_gain_s_inv *=
        UnitRamp(state.t_s, config::kNullRampDurationS);
    const ReactiveSolution solution = SolveReactiveVelocityDetailed(
        world.jacobian_world, e_pos, e_rot, e_twist.linear_m_s,
        e_twist.angular_rad_s, state.q_rad, limit_rad_, zone_rad_,
        ramped_gains);
    status.qdot_task_rad_s = solution.qdot_task_rad_s;
    status.qdot_null_rad_s = solution.qdot_null_rad_s;
    status.null_leak_m_s = solution.leak_twist.head<3>().norm();
    return solution.qdot_task_rad_s + solution.qdot_null_rad_s;
}
