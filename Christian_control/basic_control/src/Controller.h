//
// Controller — THE controller: it drives the arm toward this cycle's
// Reference, whichever channel the source filled.
//
//   pose channel   -> the reactive law (ReactiveLaw.h) — FK + 6×7 Jacobian
//   joint channel  -> the joint tracking law below — no model at all
//   no reference   -> the reactive law toward the pose captured at Reset,
//                     i.e. "hold here": a silent source never causes motion
//
// Reference.posture (secondary, optional) rides along on pose/hold cycles:
// the planner's nominal joint state enters the reactive law's null space as
// guidance (PostureObjective). Joint cycles ignore it.
//
// This file contains composition only. It does no frame conversion, command
// integration, timing, hardware access, printing, or allocation. The
// equations live in ReactiveLaw.h; the reference sources
// in Targets.h; gains in Config.h.
//
// Implementation in Controller.cpp — the one place the laws meet the
// Pinocchio model, so this header stays Pinocchio-free.
//

#pragma once

#include <cstdint>
#include <limits>
#include <memory>

#include <Eigen/Dense>

#include "Arrival.h"
#include "Feasibility.h"
#include "ReactiveLaw.h"
#include "State.h"

class DualArmKinematics;      // Kinematics.h — only the .cpp needs them
struct KinematicsWorkspace;

// The joint tracking law's output, before the per-joint clip.
struct JointTrackingCommand {
    Eigen::Matrix<double, 7, 1> qdot_rad_s = Eigen::Matrix<double, 7, 1>::Zero();
    double max_abs_error_rad = 0.0;
    bool following_error_stop = false;
};

// Joint-space tracking: feed-forward reference velocity plus proportional
// correction on the position error,
//
//     q̇_cmd = q̇_ref + kp wrap(q_ref - q_meas)
//
// and the worst joint's wrapped |q_ref - q_meas| against the stop gate. The
// error is wrapped (WrappedJointError, State.h) because measured positions
// arrive on [0, 360) while the reference is signed — unwrapped, a joint
// either side of zero would be commanded a full turn the wrong way. Pure
// arithmetic on fixed-size vectors: no allocation, no model, no I/O. The
// gains arrive as arguments so this header stays Config-free; the caller
// passes config::kKpJointTracking and kTrajFollowingErrorStopDeg in radians.
// A non-positive gate disables the stop request.
inline JointTrackingCommand
SolveJointTracking(const JointReference& reference,
                   const Eigen::Matrix<double, 7, 1>& q_meas, double kp_s_inv,
                   double following_error_stop_rad)
{
    const Eigen::Matrix<double, 7, 1> error =
        WrappedJointError(reference.q_rad, q_meas);
    JointTrackingCommand command;
    command.qdot_rad_s = reference.qdot_rad_s + kp_s_inv * error;
    // Eigen's maxCoeff SKIPS a NaN, so a non-finite error on any joint would
    // otherwise report 0.0 and leave the stop unrequested while the command
    // itself is NaN. A safety flag fails toward stopping.
    command.max_abs_error_rad = error.allFinite()
                                    ? error.cwiseAbs().maxCoeff()
                                    : std::numeric_limits<double>::infinity();
    command.following_error_stop =
        following_error_stop_rad > 0.0 &&
        !(command.max_abs_error_rad <= following_error_stop_rad);
    return command;
}

class TrackingController
{
public:
    // The model adapter is validated before any hardware connection and
    // exposes only the CONTROLLED arm's 6x7 Jacobian (DualArmKinematics
    // fixes which arm that is at construction). Every gain, term switch and
    // tolerance comes straight from Config.h, read in the .cpp — there is
    // nothing to pass in and nothing to forward.
    explicit TrackingController(DualArmKinematics& model);
    ~TrackingController();

    // T5 of takeover: captures the CURRENT end-effector pose as the hold
    // pose and disarms the arrival notice.
    void Reset(const RobotState& state);

    // Desired joint velocity BEFORE clamping, rad/s. dt_s is the Runner's
    // measured, clamped cycle time.
    Eigen::Matrix<double, 7, 1> DesiredVelocity(const RobotState& state,
                                                const Reference& reference,
                                                double dt_s,
                                                ControllerStatus& status);

private:
    DualArmKinematics& model_;
    std::unique_ptr<KinematicsWorkspace> workspace_; // sized in the .cpp

    // Built once from Config.h in the constructor, because the null-space
    // targets need a deg->rad conversion and the loop must not repeat it.
    ReactivePoseGains gains_;
    Eigen::Matrix<double, 7, 1> limit_rad_;
    double zone_rad_ = 0.0;

    // The takeover pose: the target when a reference has no pose channel or
    // no orientation.
    Eigen::Vector3d hold_position_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d hold_rotation_ = Eigen::Matrix3d::Identity();

    // True once a joint reference has been followed, cleared by the re-seat
    // it triggers on the first pose-channel cycle after it. Without that
    // re-seat the takeover hold pose would still be the one captured before
    // the trajectory ran.
    bool followed_joint_reference_ = false;

    // Arrival notice: armed only when the pose-reference sequence changes,
    // so the hold pose never fires and each target fires once.
    std::uint64_t last_pose_sequence_ = 0;
    bool pose_sequence_seen_ = false;
    bool arrival_reported_ = true;

    // Positive/negative arrival gates. Constructed from Config.h in the .cpp
    // (this header stays Config-free). Declared last so init order matches.
    ArrivalSettlingMonitor arrival_monitor_;
    ArrivalTimeoutMonitor timeout_monitor_;

    // Slice 4: graded feasibility supervision (Feasibility.h) — measures
    // and a debounced replan advisory in the status, never a stop.
    FeasibilityThresholds feasibility_;
    ReplanAdvisor replan_advisor_;
};
