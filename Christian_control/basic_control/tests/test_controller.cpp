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

    // Slice 1: Reference.posture is SECONDARY guidance for the reactive
    // law's null space — consumed on pose/hold cycles, ignored on joint
    // cycles, and inert while config::kPostureEnabled is false (the
    // compiled default until a source actually publishes it). The branches
    // below test whichever way the flag is currently compiled, so this
    // test stays meaningful when slice 2 turns the term on.
    void TestPostureGuidanceComposition()
    {
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, Arm::kRight, config::kLeftNominalRad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);
        TrackingController controller(model);

        Eigen::Matrix<double, 7, 1> q_start = Eigen::Matrix<double, 7, 1>::Zero();
        q_start(1) = 0.3;
        q_start(3) = -0.6;
        Eigen::Matrix<double, 7, 1> q_now = q_start;
        q_now(1) = 0.4;
        q_now(3) = -0.7;

        controller.Reset(StateAt(q_start));

        // The arm has drifted from the hold pose, and the null ramp is over:
        // the task term is active and any null-space term is at full gain.
        RobotState drifted = StateAt(q_now);
        drifted.t_s = config::kNullRampDurationS + 1.0;

        JointReference nominal;
        nominal.q_rad = q_start;
        nominal.qdot_rad_s.setZero();

        Reference plain; // hold cycle, no guidance
        Reference guided = plain;
        guided.posture = nominal;

        ControllerStatus s_plain, s_guided;
        const auto qdot_plain =
            controller.DesiredVelocity(drifted, plain, 0.002, s_plain);
        const auto qdot_guided =
            controller.DesiredVelocity(drifted, guided, 0.002, s_guided);

        // Telemetry reports the guidance error whenever guidance was
        // supplied on a pose cycle, and stays NaN when none was.
        Check(std::isnan(s_plain.posture_error_deg),
              "no posture supplied -> posture error telemetry stays NaN");
        Check(std::isfinite(s_guided.posture_error_deg),
              "posture supplied on a pose cycle -> posture error reported");
        const double kDeg = M_PI / 180.0;
        const double expected_error_deg =
            WrappedJointError(nominal.q_rad, q_now).cwiseAbs().maxCoeff() /
            kDeg;
        Check(std::abs(s_guided.posture_error_deg - expected_error_deg) < 1e-9,
              "posture error telemetry is the worst-joint wrapped error");

        if (config::kPostureEnabled) {
            // Guidance biases the redundant motion but never the task part.
            Check((s_guided.qdot_task_rad_s - s_plain.qdot_task_rad_s).norm()
                      == 0.0,
                  "posture guidance leaves the task solution untouched");
            Check((qdot_guided - qdot_plain).norm() > 0.0,
                  "posture guidance produces null-space motion at full ramp");

            // Before the ramp, the guidance contributes nothing: takeover
            // can never begin with a projected posture transient.
            RobotState at_takeover = StateAt(q_now); // t_s = 0
            ControllerStatus s_ramp_plain, s_ramp_guided;
            const auto ramp_plain = controller.DesiredVelocity(
                at_takeover, plain, 0.002, s_ramp_plain);
            const auto ramp_guided = controller.DesiredVelocity(
                at_takeover, guided, 0.002, s_ramp_guided);
            Check((ramp_guided - ramp_plain).norm() == 0.0,
                  "posture guidance is fully ramped out at takeover");
        } else {
            // Compiled off: supplying guidance changes NOTHING — the
            // behaviour-preservation proof for the current configuration.
            Check((qdot_guided - qdot_plain).norm() == 0.0,
                  "kPostureEnabled=false -> guidance leaves the command "
                  "bit-identical");
        }

        // On a joint-channel cycle the joint tracking law owns every joint:
        // posture guidance is ignored regardless of configuration.
        Reference joint_ref;
        JointReference joint;
        joint.q_rad = q_now;
        joint.qdot_rad_s.setZero();
        joint_ref.joint = joint;
        Reference joint_guided = joint_ref;
        joint_guided.posture = nominal;

        ControllerStatus s_joint, s_joint_guided;
        const auto qdot_joint = controller.DesiredVelocity(
            StateAt(q_start), joint_ref, 0.002, s_joint);
        const auto qdot_joint_guided = controller.DesiredVelocity(
            StateAt(q_start), joint_guided, 0.002, s_joint_guided);
        Check((qdot_joint_guided - qdot_joint).norm() == 0.0,
              "the joint channel ignores posture guidance");
        Check(std::isnan(s_joint_guided.posture_error_deg),
              "no posture telemetry on a joint-channel cycle");
    }

} // namespace

int main()
{
    TestHoldPoseReseatsAfterJointTracking();
    TestPostureGuidanceComposition();
    if (failures == 0) {
        std::cout << "all controller tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
