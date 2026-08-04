//
// Controller — TrackingController implementation. This is the only
// translation unit where the controller meets the Pinocchio model.
//

#include "Controller.h"

#include "Config.h"
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
    gains.null_gain_s_inv = config::kNullGain;
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
      gains_(ConfiguredGains())
{
    for (int i = 0; i < 7; ++i) {
        null_midpoint_rad_[i] = config::kNullMidpointDeg[i] * kDegToRad;
        null_centering_mask_[i] = config::kNullCenteringMask[i];
    }
}

TrackingController::~TrackingController() = default;

void TrackingController::Reset(const RobotState& state)
{
    const PoseJacobian ee =
        model_.RightPoseAndJacobian(state.q_rad, *workspace_);
    // Hold here: an empty reference (or a position-only target's missing
    // orientation) resolves to the pose measured at takeover.
    hold_position_ = ee.position;
    hold_rotation_ = ee.rotation;
    pose_sequence_seen_ = false;
    arrival_reported_ = true;
}

Eigen::Matrix<double, 7, 1>
TrackingController::DesiredVelocity(const RobotState& state,
                                    const Reference& reference, double dt_s,
                                    ControllerStatus& status)
{
    // Pose channel, or the hold pose when the source gave no reference.
    const Eigen::Vector3d p_desired =
        reference.pose ? reference.pose->p_desired : hold_position_;
    const Eigen::Matrix3d rotation_desired =
        reference.pose && reference.pose->rotation
            ? *reference.pose->rotation
            : hold_rotation_;

    // A new operator target re-arms the arrival notice; the hold pose
    // never does.
    if (reference.pose &&
        (!pose_sequence_seen_ ||
         reference.pose->sequence != last_pose_sequence_))
    {
        pose_sequence_seen_ = true;
        last_pose_sequence_ = reference.pose->sequence;
        arrival_reported_ = false;
    }

    // The adapter composes the SAME full q from measured right joints and the
    // fixed left nominal, then selects only the right 7 Jacobian columns.
    const PoseJacobian ee =
        model_.RightPoseAndJacobian(state.q_rad, *workspace_);

    // Equation 1: pose error, reference minus actual (ReactiveLaw.h).
    const Eigen::Vector3d e_pos = p_desired - ee.position;
    const Eigen::Vector3d e_rot =
        RotationLog(rotation_desired * ee.rotation.transpose());

    // Equation 2: twist error, the source's reference velocity minus the
    // measured end-effector twist J q̇_measured. The hold pose is stationary,
    // so its reference twist is genuinely zero rather than a placeholder.
    const Eigen::Matrix<double, 6, 1> measured_twist =
        ee.jacobian * state.qdot_rad_s;
    const Twist reference_twist =
        reference.pose ? reference.pose->twist : Twist{};
    const Twist e_twist = TwistError(reference_twist, measured_twist);

    // Arrival is edge-triggered only on a terminal source sample.  A moving
    // Cartesian profile may pass through a target-shaped point; it must not
    // advance the mailbox until it explicitly becomes arrival-eligible.
    const bool arrival_eligible = reference.pose && reference.pose->arrival_eligible;
    const bool position_arrived = e_pos.norm() <= config::kArrivalToleranceM;
    const bool orientation_arrived = !gains_.orientation_enabled ||
        e_rot.norm() <= config::kArrivalOrientationToleranceRad;
    if (!arrival_reported_ && arrival_eligible && position_arrived &&
        orientation_arrived) {
        arrival_reported_ = true;
        status.arrived_edge = true;
        status.arrival_error_m = e_pos.norm();
    }
    status.p_desired = p_desired;
    status.p_current = ee.position;
    status.rot_error_rad = e_rot.norm();
    status.tool_quat = Eigen::Quaterniond(ee.rotation);
    if (status.tool_quat.w() < 0.0)
        status.tool_quat.coeffs() = -status.tool_quat.coeffs();

    // σ_min of the full 6×7 task Jacobian — 6×6 self-adjoint solve, no
    // allocation. The Jacobian mixes meters-per-radian and
    // radian-per-radian rows, so watch this value's trend, not its size.
    const Eigen::Matrix<double, 6, 6> jjt = ee.jacobian * ee.jacobian.transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(jjt);
    status.sigma_min = std::sqrt(std::max(0.0, eigensolver.eigenvalues()(0)));

    // Equations 3-6: task twist -> DLS -> null-space (ReactiveLaw.h).
    ReactivePoseGains ramped_gains = gains_;
    ramped_gains.null_gain_s_inv *=
        UnitRamp(state.t_s, config::kNullRampDurationS);
    return SolveReactiveVelocity(ee.jacobian, e_pos, e_rot, e_twist.linear_m_s,
                                 e_twist.angular_rad_s, state.q_rad,
                                 null_midpoint_rad_, null_centering_mask_,
                                 ramped_gains);
}
