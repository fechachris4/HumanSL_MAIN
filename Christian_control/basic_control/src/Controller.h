//
// Controller — THE controller: it drives the arm toward this cycle's
// Reference, whichever channel the source filled.
//
//   joint channel  -> the joint law (Trajectory.h) — the planner's exact path
//   pose channel   -> the reactive law (ReactiveLaw.h) — FK + 6×7 Jacobian
//   no reference   -> the reactive law toward the pose captured at Reset,
//                     i.e. "hold here": a silent source never causes motion
//
// This file contains composition only. It does no frame conversion, command
// integration, timing, hardware access, printing, or allocation. The
// equations live in ReactiveLaw.h and Trajectory.h; the reference sources
// in Targets.h and Trajectory.h; gains in Config.h.
//
// Implementation in Controller.cpp — the one place the laws meet the
// Pinocchio model, so this header stays Pinocchio-free.
//

#pragma once

#include <cstdint>
#include <memory>

#include <Eigen/Dense>

#include "ReactiveLaw.h"
#include "State.h"

class DualArmKinematics;      // Kinematics.h — only the .cpp needs them
struct KinematicsWorkspace;

class TrackingController
{
public:
    // The model adapter is validated before any hardware connection and
    // exposes only the selected right-arm 6x7 Jacobian. Every gain, term
    // switch and tolerance comes straight from Config.h, read in the .cpp —
    // there is nothing to pass in and nothing to forward.
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
    Eigen::Matrix<double, 7, 1> null_midpoint_rad_;
    Eigen::Matrix<double, 7, 1> null_centering_mask_;

    // The takeover pose: the target when a reference has no pose channel or
    // no orientation.
    Eigen::Vector3d hold_position_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d hold_rotation_ = Eigen::Matrix3d::Identity();

    // Arrival notice: armed only when the pose-reference sequence changes,
    // so the hold pose never fires and each target fires once.
    std::uint64_t last_pose_sequence_ = 0;
    bool pose_sequence_seen_ = false;
    bool arrival_reported_ = true;
};
