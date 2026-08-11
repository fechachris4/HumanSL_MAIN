//
// TrajectoryPoseSource — pose-primary trajectory following (slice 2 of the
// world-frame architecture). Wraps the joint-trajectory source and flips
// which channel owns the command: the planner's sampled nominal q_nom(t)
// becomes a Cartesian reference (FK of q_nom, reference twist J·q̇_nom) for
// the reactive law, while the nominal itself rides along as null-space
// posture guidance. GPMP2's trajectory guides; the Cartesian law commands.
//
// Composition, not reimplementation: activation, the splice guard,
// rejection, Hermite sampling, and the completion edge are all the wrapped
// JointTrajectorySource's, unchanged. This class only translates the
// sampled joint reference into a pose reference — it is the one place the
// reference path meets the Pinocchio model, mirroring how Controller.cpp is
// the one place the laws do.
//
// Phase behaviour:
//   before any trajectory  -> no pose channel (the controller holds its own
//                             takeover pose, exactly as today) + posture
//                             pinned at the hold joints
//   while tracking         -> pose = FK(q_nom(t)), twist = J·q̇_nom(t),
//                             arrival_eligible = false (a moving profile
//                             must not fire arrival), one sequence per
//                             activated trajectory
//   after completion       -> the stationary terminal pose, twist zero,
//                             arrival_eligible = true — the moving reference
//                             BECOMES the hold reference, no mode switch
//
// Called from the control loop: one FK + Jacobian per cycle on the
// preallocated workspace (the same call the controller already makes for
// the measured pose), no I/O, no allocation beyond the wrapped source's
// accepted bounded mailbox exception.
//

#pragma once

#include <cstdint>

#include <Eigen/Dense>

#include "Kinematics.h"
#include "State.h"
#include "Targets.h"

class TrajectoryPoseSource : public ReferenceSource
{
public:
    // `base_motion` (optional, slice 3): a world-frame estimate of the mount.
    // When supplied, the pose reference is ANCHORED in the world at each
    // trajectory's activation: if the mount then moves, the reference is
    // re-expressed in the arm's base frame every cycle so the tool holds its
    // WORLD pose. An invalid estimate freezes compensation at the last good
    // value (graded degradation, never a stop; base_estimate_fresh telemetry
    // records it). With no provider — or the static provider, whose world IS
    // the mount — the compensation is exactly identity and the output is
    // bit-identical to slice 2. The nominal posture guidance stays the
    // planner's uncompensated q_nom: guidance, small gain, documented drift.
    TrajectoryPoseSource(const Eigen::Matrix<double, 7, 1>& hold_q_rad,
                         JointTrajectoryMailbox& mailbox,
                         DualArmKinematics& model,
                         BaseMotionSource* base_motion = nullptr);

    // Sequencing is the planner's job (as for the wrapped source), so the
    // inherited no-op OnArrivalEdge is deliberate here too.
    Reference Get(const RobotState& state, double dt_s,
                  ControllerStatus& status) override;

private:
    JointTrajectorySource inner_;
    DualArmKinematics& model_;
    KinematicsWorkspace workspace_;

    // One sequence per activated trajectory, so the controller re-arms its
    // arrival notice exactly once per plan, never per cycle.
    std::uint64_t sequence_ = 0;
    bool tracking_ = false; // a trajectory has been activated
    bool terminal_ = false; // the active trajectory has completed

    // Slice 3: world anchoring. The base/mount transforms are cached as
    // Eigen isometries at construction (both mounts are fixed joints).
    BaseMotionSource* base_motion_ = nullptr;
    Eigen::Isometry3d mount_from_base_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d base_from_mount_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d anchor_world_from_mount_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d last_world_from_mount_ = Eigen::Isometry3d::Identity();
    bool have_estimate_ = false; // any valid estimate ever received
    bool anchored_ = false;      // anchor captured for the active trajectory
};
