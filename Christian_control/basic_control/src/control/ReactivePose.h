//
// ReactivePose: the reactive 6-DoF pose controller — PD on pose (+ optional
// twist) error resolved to joint velocities via damped least squares, with
// optional null-space joint centering. The mathematical policy lives in
// control/ReactiveLaw.h (ported from the simulation, msc_project
// controller/reactive_controller.py); this class binds it to the Controller
// interface and the robot model. Design: docs/decisions/reactive-pose-port.md
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_REACTIVEPOSE_H
#define HUMANSL_MASTERS_PROJECT_2025_REACTIVEPOSE_H

#include <cstdint>
#include <string>

#include "control/Controller.h"
#include "control/ReactiveLaw.h"
#include "control/Target.h"
#include "math/Kinematics.h"
#include "Dynamics.h"

class ReactivePose : public Controller
{
public:
    // Validates the frame name against the model (throws, before any
    // takeover can happen) and preallocates the kinematics workspace.
    // midpoint/mask: the null-space centering configuration (radians; mask
    // 1 = joint centers, 0 = never — the Gen3's continuous joints).
    ReactivePose(Dynamics& dynamics, PoseTargetStore& targets,
                 const ReactivePoseGains& gains, double arrival_tolerance_m,
                 const std::string& ee_frame_name,
                 const Eigen::Matrix<double, 7, 1>& null_midpoint_rad,
                 const Eigen::Matrix<double, 7, 1>& null_centering_mask);

    // Seeds the desired pose with the CURRENT end-effector pose (position
    // AND orientation), so the controller holds until the operator types a
    // target (anything typed before takeover is discarded); disarms the
    // arrival notice.
    void Reset(const RobotState& state) override;

    // FK + full 6x7 Jacobian from the SAME q -> pose/twist error -> task
    // twist -> damped least squares (+ null-space). dt_s is unused by this
    // control law.
    Eigen::Matrix<double, 7, 1> DesiredVelocity(const RobotState& state, double dt_s,
                                                ControllerStatus& status) override;

private:
    Dynamics& dynamics_;
    PoseTargetStore& targets_;
    ReactivePoseGains gains_;
    double arrival_tolerance_m_;
    pinocchio::FrameIndex ee_frame_;
    KinematicsWorkspace workspace_;
    Eigen::VectorXd q_measured_rad_; // preallocated deg->model boundary buffer
    Eigen::Matrix<double, 7, 1> null_midpoint_rad_;
    Eigen::Matrix<double, 7, 1> null_centering_mask_;

    // Arrival notice state: armed only when the target sequence changes, so
    // the seeded hold target never fires and each typed target fires once.
    std::uint64_t last_target_sequence_ = 0;
    bool arrival_reported_ = true;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_REACTIVEPOSE_H
