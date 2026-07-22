//
// Measure: read sensor data from the arm (no motion commands).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_MEASURE_H
#define HUMANSL_MASTERS_PROJECT_2025_MEASURE_H

#include <vector>

#include <BaseCyclicClientRpc.h>

#include "Dynamics.h"

namespace k_api = Kinova::Api;

// Joint position and velocity from one feedback frame, in Kortex boundary
// units. Use this when both quantities must refer to the same instant.
struct JointMeasurements {
    std::vector<double> position_deg;   // raw robot angles [0, 360)
    std::vector<double> velocity_deg_s; // measured angular velocities
};

// One snapshot of the joint state, including the Pinocchio configuration.
struct JointReading {
    std::vector<double> deg;            // raw robot angles [0, 360)
    std::vector<double> velocity_deg_s; // measured angular velocities
    Eigen::VectorXd q_pin;               // Pinocchio configuration vector (from radians)
};

// THE robot state reader: the single place in the program that fetches a
// standalone feedback frame (RefreshFeedback). Everything that needs the
// arm's state outside a command loop calls this. The cyclic loop does not:
// they use the feedback frame that send_positions' Refresh(command) returns
// from the same exchange — same data, no extra round trip.
k_api::BaseCyclic::Feedback read_feedback(k_api::BaseCyclic::BaseCyclicClient* base_cyclic);

// Returns position and measured velocity for every joint from one feedback
// frame. Values remain in the Kortex boundary units: degrees and degrees/s.
JointMeasurements measure_joints(k_api::BaseCyclic::BaseCyclicClient* base_cyclic);

// Reads the joints once and also converts them into the model's
// configuration vector, ready for dynamics/kinematics calls.
JointReading measure_configuration(k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                                   Dynamics& dynamics);

#endif // HUMANSL_MASTERS_PROJECT_2025_MEASURE_H
