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
#include "control/Controller.h"
#include "control/ReactiveLaw.h"
#include "control/Target.h"
#include "math/DualArmKinematics.h"

class ReactivePose : public Controller
{
public:
    // The model adapter is validated before any hardware connection and
    // exposes only the selected right-arm 6x7 Jacobian.
    // midpoint/mask: the null-space centering configuration (radians; mask
    // 1 = joint centers, 0 = never — the Gen3's continuous joints).
    ReactivePose(DualArmKinematics& model, PoseTargetStore& targets,
                 const ReactivePoseGains& gains, double arrival_tolerance_m,
                 const Eigen::Matrix<double, 7, 1>& null_midpoint_rad,
                 const Eigen::Matrix<double, 7, 1>& null_centering_mask,
                 double null_ramp_duration_s = 0.0);

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
    DualArmKinematics& model_;
    PoseTargetStore& targets_;
    ReactivePoseGains gains_;
    double arrival_tolerance_m_;
    KinematicsWorkspace workspace_;
    Eigen::Matrix<double, 7, 1> null_midpoint_rad_;
    Eigen::Matrix<double, 7, 1> null_centering_mask_;
    double null_ramp_duration_s_;

    // Arrival notice state: armed only when the target sequence changes, so
    // the seeded hold target never fires and each typed target fires once.
    std::uint64_t last_target_sequence_ = 0;
    bool arrival_reported_ = true;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_REACTIVEPOSE_H
