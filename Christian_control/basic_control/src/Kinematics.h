//
// Kinematics: forward kinematics via Pinocchio (model already loaded in Dynamics).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_KINEMATICS_H
#define HUMANSL_MASTERS_PROJECT_2025_KINEMATICS_H

#include <string>
#include <ostream>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include "Dynamics.h"

namespace k_api = Kinova::Api;

// Pose of one frame of the robot, expressed in the base frame.
struct Pose {
    Eigen::Vector3d position; // meters
    Eigen::Matrix3d rotation; // orientation as a rotation matrix
};

// Forward kinematics: where is `frame_name` when the joints are at q_pin?
// q_pin is the Pinocchio configuration vector (from measure_configuration).
// Frame names come from the URDF, e.g. "EndEffector_Link", "gripper_link",
// "Bracelet_Link".
Pose forward_kinematics(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                        const std::string& frame_name = "EndEffector_Link");

// Sanity check at startup: our FK (URDF + Pinocchio) vs the robot's own
// reported tool pose. Prints both to `out`. Read-only.
void report_fk_vs_robot(Dynamics& dynamics, k_api::Base::BaseClient* base,
                        k_api::BaseCyclic::BaseCyclicClient* base_cyclic, std::ostream& out);

#endif // HUMANSL_MASTERS_PROJECT_2025_KINEMATICS_H
