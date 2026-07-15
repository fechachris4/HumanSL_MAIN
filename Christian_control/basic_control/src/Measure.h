//
// Measure: read sensor data from the arm (no motion commands).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_MEASURE_H
#define HUMANSL_MASTERS_PROJECT_2025_MEASURE_H

#include <vector>

#include <BaseCyclicClientRpc.h>

#include "Dynamics.h"

namespace k_api = Kinova::Api;

// One snapshot of the joint positions, in both representations we use.
struct JointReading
{
    std::vector<double> deg;  // raw robot angles, degrees [0, 360)
    Eigen::VectorXd q_pin;    // Pinocchio configuration vector (from radians)
};

// Returns the current angle of each joint in degrees [0, 360),
// read from the real-time feedback channel.
std::vector<double> measure_joint_angles(k_api::BaseCyclic::BaseCyclicClient* base_cyclic);

// Reads the joints once and also converts them into the model's
// configuration vector, ready for dynamics/kinematics calls.
JointReading measure_configuration(k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                                   Dynamics& dynamics);

#endif //HUMANSL_MASTERS_PROJECT_2025_MEASURE_H
