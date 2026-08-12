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

int main()
{
    TestHoldPoseReseatsAfterJointTracking();
    if (failures == 0) {
        std::cout << "all controller tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
