//
// TrajectoryPoseSource implementation. See the header for the phase
// contract; every decision here is channel translation, not trajectory
// logic — that stays in the wrapped JointTrajectorySource.
//

#include "TrajectoryPoseSource.h"

TrajectoryPoseSource::TrajectoryPoseSource(
    const Eigen::Matrix<double, 7, 1>& hold_q_rad,
    JointTrajectoryMailbox& mailbox, DualArmKinematics& model,
    BaseMotionSource* base_motion)
    : inner_(hold_q_rad, mailbox), model_(model),
      workspace_(model.dynamics()), base_motion_(base_motion)
{
    // Both mounts are fixed joints, so these are constants (Kinematics.h).
    const auto& mount_from_base = model.MountFromBase(model.controlled_arm());
    mount_from_base_.linear() = mount_from_base.rotation();
    mount_from_base_.translation() = mount_from_base.translation();
    base_from_mount_ = mount_from_base_.inverse();
}

Reference TrajectoryPoseSource::Get(const RobotState& state, double dt_s,
                                    ControllerStatus& status)
{
    // The wrapped source does everything trajectory-shaped: mailbox Take,
    // splice guard, Hermite sample, completion. Its status edges are the
    // phase signals, reported to the Runner unchanged.
    const Reference inner = inner_.Get(state, dt_s, status);
    if (status.joint_traj_activated) {
        tracking_ = true;
        terminal_ = false;
        anchored_ = false; // each trajectory re-anchors its world reference
        ++sequence_;
    }
    if (status.joint_traj_complete_edge)
        terminal_ = true;

    // The sampled nominal is ALWAYS the posture guidance — including the
    // holds, where it pins the redundant degree of freedom the 6-DoF pose
    // task leaves free.
    Reference out;
    out.posture = inner.joint;

    // Before any trajectory: no pose channel, so the controller keeps the
    // takeover hold pose it captured at Reset — identical behaviour to the
    // joint mode's startup hold, without commanding all seven joints.
    if (!tracking_ || !inner.joint)
        return out;

    // FK and Jacobian of the NOMINAL (not the measured q — the controller
    // computes that itself): where the plan says the tool should be, and
    // how fast it should be moving there.
    const PoseJacobian ee =
        model_.ControlledPoseAndJacobian(inner.joint->q_rad, workspace_);
    const Eigen::Matrix<double, 6, 1> twist =
        ee.jacobian * inner.joint->qdot_rad_s;

    PoseReference pose;
    pose.p_desired = ee.position;
    pose.rotation = ee.rotation;
    pose.twist.linear_m_s = twist.head<3>();
    pose.twist.angular_rad_s = twist.tail<3>();

    // Slice 3: hold the WORLD pose against base motion. The reference the
    // plan describes is anchored in the world at activation; if the mount
    // has moved since, re-express it in the (moved) base frame:
    //
    //   correction (mount frame)  C = T_world_mount(now)^-1 · T_world_mount(anchor)
    //   compensated reference     T' = T_base_mount · C · T_mount_base · T
    //
    // With a static provider (or none) C is exactly identity and the branch
    // below leaves the reference bit-identical to slice 2. An invalid
    // estimate freezes C at the last good value — the arm keeps tracking the
    // best-known world pose rather than stopping (degrade, don't veto).
    if (base_motion_ != nullptr) {
        const BaseMotionEstimate estimate = base_motion_->Latest();
        status.base_estimate_fresh = estimate.valid;
        if (estimate.valid) {
            last_world_from_mount_ = estimate.world_from_mount;
            have_estimate_ = true;
            if (!anchored_) {
                // Anchor on the first valid estimate of this trajectory, so
                // compensation is continuous from zero (no startup jump).
                anchor_world_from_mount_ = estimate.world_from_mount;
                anchored_ = true;
            }
        }
        if (have_estimate_ && anchored_) {
            const Eigen::Isometry3d correction_mount =
                last_world_from_mount_.inverse() * anchor_world_from_mount_;
            if (!correction_mount.matrix().isIdentity(0.0)) {
                const Eigen::Isometry3d correction_base =
                    base_from_mount_ * correction_mount * mount_from_base_;
                const Eigen::Vector3d p_comp =
                    correction_base * pose.p_desired;
                status.base_comp_m = (p_comp - pose.p_desired).norm();
                pose.p_desired = p_comp;
                pose.rotation = correction_base.linear() * *pose.rotation;
                pose.twist.linear_m_s =
                    correction_base.linear() * pose.twist.linear_m_s;
                pose.twist.angular_rad_s =
                    correction_base.linear() * pose.twist.angular_rad_s;
            } else {
                status.base_comp_m = 0.0;
            }
        }
    }

    pose.sequence = sequence_;
    // Past the end the sampler holds the final point at zero velocity, so
    // the twist above is genuinely zero: the terminal reference is the same
    // stationary pose reference an operator target would be.
    pose.arrival_eligible = terminal_;
    out.pose = pose;
    return out;
}
