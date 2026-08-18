// TrackingController integration tests against the real Pinocchio model.
// No Kortex connection and no robot execution.

#undef NDEBUG

#include <cassert>
#include <cmath>
#include <iostream>

#include "Config.h"
#include "Controller.h"
#include "ExecutionConfig.h"
#include "Frames.h"
#include "Kinematics.h"

namespace {

constexpr double kDegToRad = M_PI / 180.0;

ReactivePoseGains TestGains()
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

Eigen::Matrix<double, 7, 1> PositionLimitsRad()
{
    Eigen::Matrix<double, 7, 1> limits;
    for (int joint = 0; joint < 7; ++joint)
        limits[joint] = config::kJointSoftwareLimitDeg[joint] * kDegToRad;
    return limits;
}

RobotState MovingState()
{
    RobotState state{};
    state.q_rad << 0.3, -0.5, 0.4, 1.0, -0.2, 0.8, 0.1;
    state.qdot_rad_s << 0.2, -0.1, 0.3, -0.2, 0.1, -0.3, 0.2;
    state.t_s = 2.0; // null-space ramp fully settled
    state.world_p_mountseg = Eigen::Vector3d(1.0, 2.0, 3.0);
    state.world_R_mountseg =
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    state.world_v_mountseg_m_s = Eigen::Vector3d(0.4, -0.2, 0.1);
    state.world_w_mountseg_rad_s = Eigen::Vector3d(0.3, -0.1, 0.2);
    state.world_mount_twist_valid = true;
    state.world_fresh = true;
    return state;
}

void CheckMeasurementAndBaseMotionOnce()
{
    RobotModel robot_model(GEN3_DUAL_URDF_PATH);
    DualArmKinematics model(robot_model, Arm::kRight, config::kLeftNominalRad,
                            config::kRightBaseFrame,
                            config::kRightEndEffectorFrame);
    TrackingController controller(model);
    const RobotState state = MovingState();
    const MeasuredCartesianState measured = controller.Measure(state);

    KinematicsWorkspace workspace(model.robot_model());
    const PoseJacobian base =
        model.ControlledPoseAndJacobian(state.q_rad, workspace);
    const pinocchio::SE3& mount_T_base =
        model.MountFromBase(model.controlled_arm());
    const Eigen::Matrix3d world_R_base =
        state.world_R_mountseg * mount_T_base.rotation();
    const Eigen::Vector3d world_p_base =
        state.world_p_mountseg +
        state.world_R_mountseg * mount_T_base.translation();
    const Eigen::Vector3d expected_position =
        world_p_base + world_R_base * base.position;
    const Eigen::Matrix3d expected_rotation = world_R_base * base.rotation;
    Eigen::Matrix<double, 6, 7> expected_jacobian;
    expected_jacobian.topRows<3>() =
        world_R_base * base.jacobian.topRows<3>();
    expected_jacobian.bottomRows<3>() =
        world_R_base * base.jacobian.bottomRows<3>();
    const Eigen::Matrix<double, 6, 1> arm_twist =
        expected_jacobian * state.qdot_rad_s;
    const Eigen::Vector3d expected_linear =
        state.world_v_mountseg_m_s +
        state.world_w_mountseg_rad_s.cross(expected_position -
                                            state.world_p_mountseg) +
        arm_twist.head<3>();
    const Eigen::Vector3d expected_angular =
        state.world_w_mountseg_rad_s + arm_twist.tail<3>();

    assert((measured.ee_pose_world.position_m - expected_position).norm() < 1e-12);
    assert((measured.ee_pose_world.rotation - expected_rotation).norm() < 1e-12);
    assert((measured.jacobian_world - expected_jacobian).norm() < 1e-12);
    assert((measured.ee_twist_world.linear_m_s - expected_linear).norm() < 1e-12);
    assert((measured.ee_twist_world.angular_rad_s - expected_angular).norm() <
           1e-12);

    PoseReference reference;
    reference.ee_pose_world = measured.ee_pose_world;
    reference.ee_twist_world = Twist{}; // stationary world reference
    reference.trajectory_id = 1;
    reference.arrival_eligible = true;

    ControllerStatus status;
    const Eigen::Matrix<double, 7, 1> actual = controller.DesiredVelocity(
        state, measured, reference, 0.002, status);
    const ReactiveSolution expected = SolveReactiveVelocityDetailed(
        measured.jacobian_world, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), -measured.ee_twist_world.linear_m_s,
        -measured.ee_twist_world.angular_rad_s, state.q_rad,
        PositionLimitsRad(), config::kLimitAvoidZoneDeg * kDegToRad,
        TestGains());
    assert((actual - (expected.qdot_task_rad_s + expected.qdot_null_rad_s))
               .norm() < 1e-12 &&
           "Mount motion must enter exactly once through measured twist error");
    assert((status.qdot_task_rad_s - expected.qdot_task_rad_s).norm() < 1e-12);
    assert((status.qdot_null_rad_s - expected.qdot_null_rad_s).norm() < 1e-12);
}

void CheckStaleMountTwistDecayUsesSampleAge()
{
    using world_frames::EffectiveStaleDurationS;
    using world_frames::StaleTwistDecayMultiplier;
    const double threshold = config::kWorldFreshMaxAgeS;
    const double tau = config::kViconMountTwistFilterTauS;
    assert(StaleTwistDecayMultiplier(threshold, threshold, tau) == 1.0);
    assert(std::abs(StaleTwistDecayMultiplier(threshold + 0.002,
                                              threshold, tau) -
                    std::exp(-0.002 / tau)) < 1e-12);
    assert(std::abs(StaleTwistDecayMultiplier(threshold + 0.100,
                                              threshold, tau) -
                    std::exp(-0.100 / tau)) < 1e-12 &&
           "decay must follow Vicon sample age, not number of loop calls");
    assert(std::abs(EffectiveStaleDurationS(0.005, 0.100, threshold) -
                    0.100) < 1e-12 &&
           "continued invalid Vicon frames must not reset Mount staleness");
    assert(std::abs(EffectiveStaleDurationS(0.120, 0.002, threshold) -
                    0.070) < 1e-12 &&
           "a delayed cycle must honor source age beyond the threshold");
}

// A physically valid ExecutionConfig whose every controller-consumed field
// differs from production, with the two arrival tolerances strongly
// asymmetric (0.02 m vs 0.40 rad; production has both at 0.001, which
// cannot distinguish a swapped initializer). Fields the controller does not
// consume keep their production values so ValidateExecutionConfig still
// accepts the snapshot.
ExecutionConfig CustomExecutionConfig()
{
    ExecutionConfig custom = ProductionExecutionConfig();
    custom.gains.kp_position_s_inv = 2.5;      // production 10.0
    custom.gains.kp_rotation_s_inv = 1.5;      // production 10.0
    custom.gains.kd_position = 0.3;            // production 0.5
    custom.gains.kd_rotation = 0.2;            // production 0.5
    custom.gains.limit_avoid_gain_s_inv = 4.0; // production 2.0
    custom.gains.dls_lambda = 0.15;            // production 0.1
    custom.gains.orientation_enabled = true;
    custom.gains.velocity_enabled = true;
    custom.gains.null_space_enabled = true;
    // Only joint 3 bounded (60 deg); MovingState puts q3 = 1.0 rad inside
    // the 0.2 rad activation zone below 60 deg = 1.0472 rad, so the
    // null-space term is nonzero and limit/zone/ramp wiring is observable.
    custom.software_limit_deg = {0.0, 0.0, 0.0, 60.0, 0.0, 0.0, 0.0};
    custom.limit_avoid_zone_rad = 0.2;                // production ~0.349
    custom.arrival_position_tolerance_m = 0.02;       // production 0.001
    custom.arrival_orientation_tolerance_rad = 0.40;  // production 0.001
    custom.arrival_dwell_s = 0.005;                   // production 0.15
    custom.target_hold_s = 0.009;                     // production 2.0
    custom.null_ramp_duration_s = 8.0;                // production 1.0
    return custom;
}

// Review fix (Plan 01 Task 2): construct TrackingController from a
// NON-production snapshot and verify every initializer-list field reaches
// the behaviour it governs. A swapped or mis-copied field in
// Controller.cpp's constructor fails at least one prediction below.
void CheckCustomConfigInjection()
{
    const ExecutionConfig custom = CustomExecutionConfig();
    ValidateExecutionConfig(custom); // must not throw: values are physical

    RobotModel robot_model(GEN3_DUAL_URDF_PATH);
    DualArmKinematics model(robot_model, Arm::kRight, config::kLeftNominalRad,
                            config::kRightBaseFrame,
                            config::kRightEndEffectorFrame);
    TrackingController controller(model, custom);
    const RobotState state = MovingState(); // t_s = 2.0 s
    const MeasuredCartesianState measured = controller.Measure(state);
    const double dt_s = 0.002;

    // Case A: position error 0.015 m (within 0.02 m), rotation error
    // 0.10 rad (within 0.40 rad). 0.10 sits BETWEEN the two tolerances,
    // so swapping them in the constructor turns this arrival into a miss.
    PoseReference near;
    near.trajectory_id = 1;
    near.arrival_eligible = true;
    near.ee_pose_world.position_m = measured.ee_pose_world.position_m +
                                    Eigen::Vector3d(0.009, -0.012, 0.0);
    near.ee_pose_world.rotation =
        Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitY())
            .toRotationMatrix() *
        measured.ee_pose_world.rotation;
    const double near_rot_error =
        RotationLog(near.ee_pose_world.rotation *
                    measured.ee_pose_world.rotation.transpose())
            .norm();
    assert(near_rot_error > custom.arrival_position_tolerance_m &&
           near_rot_error < custom.arrival_orientation_tolerance_rad &&
           "case A rotation error must sit between the two tolerances or "
           "it cannot detect a swapped initializer");

    // Dwell 0.005 s at dt 0.002 s: in tolerance from cycle 1, settled
    // time 0.002/0.004/0.006 -> the arrival edge fires exactly on cycle 3.
    for (int cycle = 1; cycle <= 3; ++cycle) {
        ControllerStatus status;
        controller.DesiredVelocity(state, measured, near, dt_s, status);
        assert(!status.not_reached_edge);
        if (cycle < 3) {
            assert(!status.arrived_edge &&
                   "injected dwell must gate arrival until 0.005 s settled");
        } else {
            assert(status.arrived_edge &&
                   "arrival must fire once settled past the injected dwell");
            assert(std::abs(status.arrival_error_m - 0.015) < 1e-12);
        }
    }

    // Case B: position error 0.10 m sits BETWEEN the tolerances (misses
    // the 0.02 m position gate, would pass a swapped 0.40 one); rotation
    // error 0.01 rad is inside both. Correct mapping never arrives and
    // the injected 0.009 s timeout fires exactly on cycle 5, one-shot.
    PoseReference far;
    far.trajectory_id = 2; // new trajectory rearms both monitors
    far.arrival_eligible = true;
    far.ee_pose_world.position_m = measured.ee_pose_world.position_m +
                                   Eigen::Vector3d(0.06, -0.08, 0.0);
    far.ee_pose_world.rotation =
        Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX())
            .toRotationMatrix() *
        measured.ee_pose_world.rotation;
    const Eigen::Vector3d far_e_pos =
        far.ee_pose_world.position_m - measured.ee_pose_world.position_m;
    assert(far_e_pos.norm() > custom.arrival_position_tolerance_m &&
           far_e_pos.norm() < custom.arrival_orientation_tolerance_rad &&
           "case B position error must sit between the two tolerances or "
           "it cannot detect a swapped initializer");

    // Prediction P3 (first cycle of case B): the returned velocity is the
    // reactive law under the INJECTED gains, deg->rad limit vector, zone,
    // and the limit-avoid gain ramped by UnitRamp(2 s, 8 s) = 0.25.
    Eigen::Matrix<double, 7, 1> custom_limits_rad;
    for (int joint = 0; joint < 7; ++joint)
        custom_limits_rad[joint] =
            custom.software_limit_deg[joint] * kDegToRad;
    ReactivePoseGains ramped = custom.gains;
    ramped.limit_avoid_gain_s_inv *=
        UnitRamp(state.t_s, custom.null_ramp_duration_s);
    const Eigen::Vector3d far_e_rot =
        RotationLog(far.ee_pose_world.rotation *
                    measured.ee_pose_world.rotation.transpose());
    const ReactiveSolution expected = SolveReactiveVelocityDetailed(
        measured.jacobian_world, far_e_pos, far_e_rot,
        -measured.ee_twist_world.linear_m_s,
        -measured.ee_twist_world.angular_rad_s, state.q_rad,
        custom_limits_rad, custom.limit_avoid_zone_rad, ramped);
    assert(expected.qdot_null_rad_s.norm() > 1e-6 &&
           "joint 3 must be inside the custom activation zone or the "
           "limit/zone/ramp fields are unobservable");
    const ReactiveSolution unramped = SolveReactiveVelocityDetailed(
        measured.jacobian_world, far_e_pos, far_e_rot,
        -measured.ee_twist_world.linear_m_s,
        -measured.ee_twist_world.angular_rad_s, state.q_rad,
        custom_limits_rad, custom.limit_avoid_zone_rad, custom.gains);
    assert((unramped.qdot_null_rad_s - expected.qdot_null_rad_s).norm() >
               1e-9 &&
           "the ramp factor must change the answer or a mis-copied "
           "null_ramp_duration_s is unobservable");

    for (int cycle = 1; cycle <= 6; ++cycle) {
        ControllerStatus status;
        const Eigen::Matrix<double, 7, 1> actual = controller.DesiredVelocity(
            state, measured, far, dt_s, status);
        assert(!status.arrived_edge &&
               "a 0.10 m error must never satisfy the 0.02 m tolerance "
               "(a swapped initializer would arrive here)");
        if (cycle == 1) {
            assert((actual - (expected.qdot_task_rad_s +
                              expected.qdot_null_rad_s))
                           .norm() < 1e-12 &&
                   "the law must consume the injected gains, limits, zone "
                   "and ramp exactly");
        }
        // Timeout 0.009 s at dt 0.002 s: waited 0.002..0.008 on cycles
        // 1-4, 0.010 >= 0.009 on cycle 5, one-shot thereafter.
        assert(status.not_reached_edge == (cycle == 5) &&
               "injected target_hold_s must fire the timeout exactly on "
               "cycle 5, one-shot");
    }
}

} // namespace

int main()
{
    CheckMeasurementAndBaseMotionOnce();
    CheckStaleMountTwistDecayUsesSampleAge();
    CheckCustomConfigInjection();
    std::cout << "all controller tests passed\n";
    return 0;
}
