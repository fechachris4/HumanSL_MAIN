//
// ResolvedRate: the Cartesian resolved-rate controller — Kp position error
// resolved to joint velocities via damped least squares. Design, math:
// docs/decisions/resolved-rate-position-integration.md
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_RESOLVEDRATE_H
#define HUMANSL_MASTERS_PROJECT_2025_RESOLVEDRATE_H

#include <cstdint>
#include <string>

#include "control/Controller.h"
#include "control/Target.h"
#include "math/Kinematics.h"
#include "Dynamics.h"

class ResolvedRate : public Controller
{
public:
    // Validates the frame name against the model (throws, before any
    // takeover can happen) and preallocates the kinematics workspace.
    ResolvedRate(Dynamics& dynamics, TargetStore& targets, double kp,
                 double dls_lambda, double arrival_tolerance_m,
                 const std::string& ee_frame_name);

    // Seeds the desired position with the CURRENT end-effector position, so
    // the controller holds until the operator types a target (anything
    // typed before takeover is discarded); disarms the arrival notice.
    void Reset(const RobotState& state) override;

    // FK + translational Jacobian from the SAME q -> v_d = Kp e -> damped
    // least squares. dt_s is unused by this control law.
    Eigen::Matrix<double, 7, 1> DesiredVelocity(const RobotState& state, double dt_s,
                                                ControllerStatus& status) override;

private:
    Dynamics& dynamics_;
    TargetStore& targets_;
    double kp_;
    double dls_lambda_;
    double arrival_tolerance_m_;
    pinocchio::FrameIndex ee_frame_;
    KinematicsWorkspace workspace_;
    Eigen::VectorXd q_measured_rad_; // preallocated deg->model boundary buffer

    // Arrival notice state: armed only when the target sequence changes, so
    // the seeded hold target never fires and each typed target fires once.
    std::uint64_t last_target_sequence_ = 0;
    bool arrival_reported_ = true;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_RESOLVEDRATE_H
