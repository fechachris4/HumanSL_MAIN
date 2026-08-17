//-------------------------------------------------
// RobotModel: loads the URDF into a Pinocchio model and owns the
// model/data pair every kinematics call reads and writes.
//
// This is the whole job. `model_` is the parsed robot description
// (joints, frames, limits); `data_` is Pinocchio's scratch workspace,
// which its algorithms — forward kinematics and Jacobians included —
// fill in place. DualArmKinematics borrows both by reference.
//
// History: this class was Dynamics, copied from TrajectoryExecution,
// where its mass/Coriolis/gravity methods fed task-impedance control.
// The velocity-level controller here never called them, so they were
// removed with the 2026-08-17 rename; git history of Dynamics.cpp has
// the implementations should torque, impedance or operational-space
// control return.
//-------------------------------------------------

#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <string>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

class RobotModel {
public:
    pinocchio::Data data_;
    pinocchio::Model model_;

    // Parses the URDF at urdf_path and sizes data_ for it. Gravity is
    // world z-up, -9.81 m/s²; diagnostics go to stderr so planner and
    // controller stdout stay clean.
    explicit RobotModel(const std::string& urdf_path);
};

#endif // ROBOT_MODEL_H
