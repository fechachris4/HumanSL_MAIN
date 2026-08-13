//
// Hardware-free tests for TrackingController's channel handling — the part
// that needs the real Pinocchio model (forward kinematics of the hold pose),
// but no robot.
//

#include <cmath>
#include <iostream>
#include <string>

#include "Config.h"
#include "Controller.h"
#include "Kinematics.h"
#include "State.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    RobotState StateAt(const Eigen::Matrix<double, 7, 1>& q_rad)
    {
        RobotState state{};
        state.q_rad = q_rad;
        state.qdot_rad_s.setZero();
        return state;
    }

    Eigen::Vector3d ForwardPosition(DualArmKinematics& model,
                                    const Eigen::Matrix<double, 7, 1>& q_rad)
    {
        KinematicsWorkspace workspace(model.dynamics());
        return model.ControlledPoseAndJacobian(q_rad, workspace).position;
    }

    // After a joint trajectory has moved the arm, the takeover hold pose is
    // stale by the whole trajectory. The first pose-channel cycle must re-seat
    // it from the CURRENT measurement, or "hold here" would command the arm
    // back to where the run started.
    //
    // The re-seat is a one-shot latch, not a per-cycle refresh: a second
    // pose-channel cycle keeps the pose captured by the first. (Alternating
    // channels every cycle would therefore Reset every other cycle, doubling
    // the FK work and never letting an arrival latch — statically impossible
    // here, because the Runner has exactly one source and no source alternates
    // channels.)
    void TestHoldPoseReseatsAfterJointTracking()
    {
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, Arm::kRight, config::kLeftNominalRad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);
        TrackingController controller(model);

        Eigen::Matrix<double, 7, 1> q_start = Eigen::Matrix<double, 7, 1>::Zero();
        q_start(1) = 0.3;
        q_start(3) = -0.6;
        Eigen::Matrix<double, 7, 1> q_after = q_start;
        q_after(1) = 0.5;
        q_after(3) = -0.9;
        Eigen::Matrix<double, 7, 1> q_later = q_after;
        q_later(1) = 0.7;

        const Eigen::Vector3d p_start = ForwardPosition(model, q_start);
        const Eigen::Vector3d p_after = ForwardPosition(model, q_after);
        Check((p_start - p_after).norm() > 1e-3,
              "the test's two configurations are distinguishable in task space");

        controller.Reset(StateAt(q_start));

        // An empty reference before any joint tracking holds the takeover pose.
        {
            ControllerStatus status;
            controller.DesiredVelocity(StateAt(q_start), Reference{}, 0.002,
                                       status);
            Check((status.p_desired - p_start).norm() < 1e-9,
                  "before any joint tracking the hold pose is the takeover pose");
        }

        // One joint-channel cycle: the arm is now at q_after.
        {
            Reference reference;
            JointReference joint;
            joint.q_rad = q_after;
            joint.qdot_rad_s.setZero();
            reference.joint = joint;
            ControllerStatus status;
            controller.DesiredVelocity(StateAt(q_after), reference, 0.002,
                                       status);
            Check(!std::isfinite(status.p_desired(0)),
                  "the joint channel computes no Cartesian target");
        }

        // First pose-channel cycle back: the hold pose is the CURRENT pose.
        {
            ControllerStatus status;
            controller.DesiredVelocity(StateAt(q_after), Reference{}, 0.002,
                                       status);
            Check((status.p_desired - p_after).norm() < 1e-9,
                  "the first pose cycle after joint tracking re-seats the hold pose");
            Check((status.p_desired - p_start).norm() > 1e-3,
                  "the stale takeover pose is not commanded back");
        }

        // Second pose-channel cycle: no further re-seat, even though the
        // measured position has moved on again.
        {
            ControllerStatus status;
            controller.DesiredVelocity(StateAt(q_later), Reference{}, 0.002,
                                       status);
            Check((status.p_desired - p_after).norm() < 1e-9,
                  "the re-seat is one-shot: later pose cycles keep that pose");
        }
    }

} // namespace

namespace
{
    // The world-hold engage path, end to end through the real model — the
    // test the first hardware run showed was missing: production always
    // carries a joint reference, so the engage must happen THROUGH an
    // idle joint hold, not only through an empty reference.
    void WorldHoldEngageTests()
    {
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, Arm::kRight, config::kLeftNominalRad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);
        TrackingController controller(model);
        Eigen::Matrix<double, 7, 1> q;
        q << 0.3, -0.5, 0.4, 1.0, -0.2, 0.8, 0.1;
        RobotState state = StateAt(q);
        controller.Reset(state);
        ControllerStatus status;

        // Idle joint reference (production's permanent companion).
        Reference idle;
        JointReference joint;
        joint.q_rad = q;
        joint.qdot_rad_s.setZero();
        idle.joint = joint;
        idle.joint_is_idle_hold = true;

        // No fresh world sample: the idle joint hold is followed exactly
        // as before this slice — joint telemetry set, no hold engage.
        state.world_fresh = false;
        status = ControllerStatus{};
        Eigen::Matrix<double, 7, 1> qdot =
            controller.DesiredVelocity(state, idle, 0.002, status);
        Check(status.hold_state == 0,
              "no fresh sample: world hold stays inactive through idle hold");
        Check(std::isfinite(status.joint_following_error_deg),
              "no fresh sample: the idle JOINT hold is what runs");
        Check(qdot.cwiseAbs().maxCoeff() < 1e-9,
              "holding at the reference joints commands ~zero velocity");

        // Fresh sample arrives: the SAME idle reference must now engage
        // the world hold (this exact transition was unreachable before
        // the 2026-08-13 fix and the first hardware run proved it).
        state.world_fresh = true;
        state.t_s = 0.1;
        status = ControllerStatus{};
        qdot = controller.DesiredVelocity(state, idle, 0.002, status);
        Check(status.hold_state == 1,
              "fresh sample engages the world hold through the idle hold");
        Check(status.world_err_m == 0.0,
              "engage anchors at the current pose: zero world error");
        Check(qdot.cwiseAbs().maxCoeff() < 1e-6,
              "engage at the current pose commands ~zero velocity");

        // An ACTIVE trajectory still owns the cycle outright.
        Reference active = idle;
        active.joint_is_idle_hold = false;
        status = ControllerStatus{};
        controller.DesiredVelocity(state, active, 0.002, status);
        Check(status.hold_state == 0,
              "an actively sampled trajectory pre-empts the world hold");
    }
} // namespace

int main()
{
    
    TestHoldPoseReseatsAfterJointTracking();
    WorldHoldEngageTests();
    if (failures == 0) {
        std::cout << "all controller tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
