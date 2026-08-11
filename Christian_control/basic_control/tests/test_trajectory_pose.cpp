//
// Hardware-free tests for TrajectoryPoseSource — the pose-primary wrapper
// that turns the joint-trajectory source's sampled nominal into a Cartesian
// reference (FK of q_nom, twist J·q̇_nom) with the nominal itself riding
// along as null-space posture guidance. Needs the real Pinocchio model for
// the FK cross-checks, but no robot.
//

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "Config.h"
#include "JointTrajectory.h"
#include "Kinematics.h"
#include "Targets.h"
#include "TrajectoryPoseSource.h"

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

    // A three-point trajectory from `start`: j2/j4 move out and back in
    // velocity-consistent steps. Small enough that every sample stays well
    // inside the joint limits and the splice guard accepts point zero.
    JointTrajectory MakeTrajectory(const Eigen::Matrix<double, 7, 1>& start)
    {
        JointTrajectory traj;
        JointTrajectoryPoint a{};
        a.t_s = 0.0;
        a.q_rad = start;
        a.qdot_rad_s.setZero();
        JointTrajectoryPoint b = a;
        b.t_s = 0.5;
        b.q_rad(1) += 0.05;
        b.q_rad(3) -= 0.04;
        b.qdot_rad_s(1) = 0.1;
        b.qdot_rad_s(3) = -0.08;
        JointTrajectoryPoint c = b;
        c.t_s = 1.0;
        c.q_rad(1) += 0.025;
        c.q_rad(3) -= 0.02;
        c.qdot_rad_s.setZero();
        traj.points = {a, b, c};
        return traj;
    }

    void TestPosePrimaryPhases()
    {
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, Arm::kRight, config::kLeftNominalRad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);
        KinematicsWorkspace check_ws(model.dynamics());

        Eigen::Matrix<double, 7, 1> q_start = Eigen::Matrix<double, 7, 1>::Zero();
        q_start(1) = 0.3;
        q_start(3) = -0.6;

        JointTrajectoryMailbox mailbox;
        TrajectoryPoseSource source(q_start, mailbox, model);

        // Phase 1 — initial hold: no pose channel (the controller keeps its
        // own takeover hold, identical behaviour to today), no joint channel
        // (the joint law must not run), posture pinned at the hold joints.
        {
            ControllerStatus status;
            const Reference r = source.Get(StateAt(q_start), 0.25, status);
            Check(!r.pose.has_value(), "initial hold publishes no pose channel");
            Check(!r.joint.has_value(), "pose-primary never fills the joint channel");
            Check(r.posture.has_value(), "initial hold pins the redundancy via posture");
            Check(r.posture &&
                      (r.posture->q_rad - q_start).norm() == 0.0 &&
                      r.posture->qdot_rad_s.norm() == 0.0,
                  "initial hold posture is the hold q at zero velocity");
        }

        // Phase 2 — activation. The reference trajectory for cross-checks.
        const JointTrajectory expected_traj = MakeTrajectory(q_start);
        mailbox.Publish(std::make_unique<JointTrajectory>(MakeTrajectory(q_start)));

        std::uint64_t first_sequence = 0;
        {
            ControllerStatus status;
            const Reference r = source.Get(StateAt(q_start), 0.25, status);
            Check(status.joint_traj_activated, "the splice guard accepts the trajectory");
            Check(r.pose.has_value(), "an active trajectory publishes a pose reference");
            if (r.pose) {
                const JointTrajectorySample s =
                    SampleJointTrajectory(expected_traj, 0.0);
                const PoseJacobian ee =
                    model.ControlledPoseAndJacobian(s.q_rad, check_ws);
                Check((r.pose->p_desired - ee.position).norm() < 1e-12,
                      "activation pose is FK of the first nominal sample");
                Check(r.pose->rotation.has_value() &&
                          (*r.pose->rotation - ee.rotation).norm() < 1e-12,
                      "activation orientation is FK of the first nominal sample");
                Check(!r.pose->arrival_eligible,
                      "a moving profile is not arrival-eligible");
                first_sequence = r.pose->sequence;
                Check(r.posture.has_value() &&
                          (r.posture->q_rad - s.q_rad).norm() == 0.0,
                      "posture carries exactly the sampled nominal");
            }
        }

        // Phase 3 — mid-profile: pose/twist equal FK and J·q̇_nom of the
        // Hermite sample at the source's own clock (one dt has elapsed).
        {
            ControllerStatus status;
            const Reference r = source.Get(StateAt(q_start), 0.25, status);
            const JointTrajectorySample s =
                SampleJointTrajectory(expected_traj, 0.25);
            const PoseJacobian ee =
                model.ControlledPoseAndJacobian(s.q_rad, check_ws);
            Check(r.pose.has_value() &&
                      (r.pose->p_desired - ee.position).norm() < 1e-12,
                  "mid-profile pose is FK of the Hermite sample");
            if (r.pose) {
                const Eigen::Matrix<double, 6, 1> expected_twist =
                    ee.jacobian * s.qdot_rad_s;
                Check((r.pose->twist.linear_m_s - expected_twist.head<3>())
                              .norm() < 1e-12 &&
                          (r.pose->twist.angular_rad_s - expected_twist.tail<3>())
                              .norm() < 1e-12,
                      "mid-profile reference twist is J(q_nom) * qdot_nom");
                Check(!r.pose->arrival_eligible,
                      "mid-profile samples stay arrival-ineligible");
                Check(r.pose->sequence == first_sequence,
                      "one trajectory keeps one sequence");
                Check(r.posture.has_value() &&
                          (r.posture->qdot_rad_s - s.qdot_rad_s).norm() == 0.0,
                      "posture velocity is the sampled nominal velocity");
            }
        }

        // Phase 4 — completion: step the clock past the end. The reference
        // becomes the stationary terminal pose, NOW arrival-eligible — the
        // moving reference has become the hold reference with no mode switch.
        bool complete_seen = false;
        for (int i = 0; i < 4; ++i) {
            ControllerStatus status;
            const Reference r = source.Get(StateAt(q_start), 0.5, status);
            if (status.joint_traj_complete_edge) {
                Check(!complete_seen, "the completion edge fires exactly once");
                complete_seen = true;
            }
            if (complete_seen && r.pose) {
                const PoseJacobian ee = model.ControlledPoseAndJacobian(
                    expected_traj.points.back().q_rad, check_ws);
                Check((r.pose->p_desired - ee.position).norm() < 1e-12,
                      "terminal pose is FK of the final nominal point");
                Check(r.pose->arrival_eligible,
                      "the stationary terminal reference is arrival-eligible");
                Check(r.pose->twist.linear_m_s.norm() == 0.0 &&
                          r.pose->twist.angular_rad_s.norm() == 0.0,
                      "the terminal reference twist is zero");
                Check(r.pose->sequence == first_sequence,
                      "the terminal hold keeps the trajectory's sequence");
            }
        }
        Check(complete_seen, "the trajectory completes");

        // Phase 5 — a rejected trajectory changes nothing: the splice guard
        // refuses a far start, and the terminal hold continues untouched.
        {
            Eigen::Matrix<double, 7, 1> q_far = q_start;
            q_far(1) += 1.0; // ~57 deg from the arm, far past the guard
            mailbox.Publish(
                std::make_unique<JointTrajectory>(MakeTrajectory(q_far)));
            ControllerStatus status;
            const Reference r = source.Get(StateAt(q_start), 0.25, status);
            Check(status.joint_traj_rejected, "the far trajectory is rejected");
            Check(r.pose.has_value() && r.pose->arrival_eligible &&
                      r.pose->sequence == first_sequence,
                  "a rejection leaves the terminal hold untouched");
        }

        // Phase 6 — a second valid trajectory (from the terminal joints) gets
        // a NEW sequence and drops arrival eligibility again.
        {
            mailbox.Publish(std::make_unique<JointTrajectory>(
                MakeTrajectory(expected_traj.points.back().q_rad)));
            ControllerStatus status;
            const Reference r = source.Get(
                StateAt(expected_traj.points.back().q_rad), 0.25, status);
            Check(status.joint_traj_activated, "the follow-up trajectory activates");
            Check(r.pose.has_value() && !r.pose->arrival_eligible,
                  "a new trajectory is again arrival-ineligible");
            Check(r.pose && r.pose->sequence != first_sequence,
                  "a new trajectory gets a new sequence");
        }
    }

    // Slice 3: world anchoring. A scripted provider plays the wearer.
    struct ScriptedBase : BaseMotionSource {
        BaseMotionEstimate next;
        BaseMotionEstimate Latest() override { return next; }
    };

    void TestWorldAnchoring()
    {
        Dynamics dynamics(GEN3_DUAL_URDF_PATH);
        DualArmKinematics model(dynamics, Arm::kRight, config::kLeftNominalRad,
                                config::kRightBaseFrame,
                                config::kRightEndEffectorFrame);

        Eigen::Matrix<double, 7, 1> q_start = Eigen::Matrix<double, 7, 1>::Zero();
        q_start(1) = 0.3;
        q_start(3) = -0.6;

        // Reference outputs: one source with NO provider, one with the
        // STATIC provider, one with the scripted provider — same trajectory
        // published to each (mailboxes are consumed, so three of them).
        JointTrajectoryMailbox mb_none, mb_static, mb_scripted;
        StaticBaseMotionSource static_base;
        ScriptedBase scripted;
        scripted.next.valid = true; // starts at world == mount
        TrajectoryPoseSource src_none(q_start, mb_none, model);
        TrajectoryPoseSource src_static(q_start, mb_static, model, &static_base);
        TrajectoryPoseSource src_scripted(q_start, mb_scripted, model, &scripted);

        mb_none.Publish(std::make_unique<JointTrajectory>(MakeTrajectory(q_start)));
        mb_static.Publish(std::make_unique<JointTrajectory>(MakeTrajectory(q_start)));
        mb_scripted.Publish(std::make_unique<JointTrajectory>(MakeTrajectory(q_start)));

        // Cycle 1 (activation, base still at the anchor): all three agree.
        ControllerStatus s_none, s_static, s_scripted;
        const Reference r_none = src_none.Get(StateAt(q_start), 0.25, s_none);
        const Reference r_static = src_static.Get(StateAt(q_start), 0.25, s_static);
        Reference r_scripted = src_scripted.Get(StateAt(q_start), 0.25, s_scripted);
        Check(r_none.pose && r_static.pose &&
                  (r_static.pose->p_desired - r_none.pose->p_desired).norm() == 0.0 &&
                  (*r_static.pose->rotation - *r_none.pose->rotation).norm() == 0.0,
              "the static provider is bit-identical to no provider");
        Check(s_static.base_comp_m == 0.0,
              "static provider reports exactly zero compensation");
        Check(std::isnan(s_none.base_comp_m),
              "no provider -> compensation telemetry stays NaN");

        // The anchored world pose: T_world_mount(anchor) is identity here,
        // so world_ref = T_mount_base * p_ref at the anchor instant.
        const auto& T_mb_pin = model.MountFromBase(Arm::kRight);
        Eigen::Isometry3d T_mb = Eigen::Isometry3d::Identity();
        T_mb.linear() = T_mb_pin.rotation();
        T_mb.translation() = T_mb_pin.translation();
        const Eigen::Vector3d world_ref_at_anchor =
            T_mb * r_scripted.pose->p_desired;

        // Cycle 2: the wearer steps 5 cm along world X. The invariant: the
        // compensated base-frame reference, pushed back through the MOVED
        // mount pose, is the SAME world point the anchor had (for the same
        // trajectory sample — the trajectory itself also advances, so
        // compare against the uncompensated source at the same clock).
        scripted.next.world_from_mount.translation() =
            Eigen::Vector3d(0.05, 0.0, 0.0);
        ControllerStatus s2_none, s2_scripted;
        const Reference r2_none = src_none.Get(StateAt(q_start), 0.25, s2_none);
        const Reference r2 = src_scripted.Get(StateAt(q_start), 0.25, s2_scripted);
        const Eigen::Vector3d world_now =
            scripted.next.world_from_mount * (T_mb * r2.pose->p_desired);
        const Eigen::Vector3d world_planned =
            /* anchor pose is identity */ T_mb * r2_none.pose->p_desired;
        Check((world_now - world_planned).norm() < 1e-12,
              "compensation holds the planned WORLD pose under base motion");
        Check(s2_scripted.base_comp_m > 0.04 && s2_scripted.base_comp_m < 0.06,
              "a 5 cm base step reports ~5 cm of compensation");
        Check(s2_scripted.base_estimate_fresh,
              "a valid estimate reports fresh");

        // Cycle 3: the estimate goes invalid. Compensation FREEZES at the
        // last good correction (graded degradation), telemetry says stale.
        scripted.next.valid = false;
        scripted.next.world_from_mount.translation() =
            Eigen::Vector3d(9.0, 9.0, 9.0); // must be ignored
        ControllerStatus s3_none, s3_scripted;
        const Reference r3_none = src_none.Get(StateAt(q_start), 0.25, s3_none);
        const Reference r3 = src_scripted.Get(StateAt(q_start), 0.25, s3_scripted);
        const Eigen::Vector3d world_frozen =
            Eigen::Translation3d(0.05, 0.0, 0.0) * (T_mb * r3.pose->p_desired);
        Check((world_frozen - (T_mb * r3_none.pose->p_desired)).norm() < 1e-12,
              "an invalid estimate freezes compensation at the last good value");
        Check(!s3_scripted.base_estimate_fresh,
              "an invalid estimate reports stale");
    }

} // namespace

int main()
{
    TestPosePrimaryPhases();
    TestWorldAnchoring();
    if (failures == 0) {
        std::cout << "all trajectory-pose tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
