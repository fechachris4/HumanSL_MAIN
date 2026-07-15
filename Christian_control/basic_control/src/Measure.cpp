//
// Measure: read sensor data from the arm (no motion commands).
//

#include "Measure.h"

#include <cmath>

std::vector<double> measure_joint_angles(k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    // One feedback frame from the real-time channel: positions, velocities,
    // torques, currents... — we extract just the joint positions.
    k_api::BaseCyclic::Feedback feedback = base_cyclic->RefreshFeedback();

    std::vector<double> angles_deg(feedback.actuators_size());
    for (int i = 0; i < feedback.actuators_size(); ++i)
        angles_deg[i] = feedback.actuators(i).position();
    return angles_deg;
}

JointReading measure_configuration(k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                                   Dynamics& dynamics)
{
    JointReading reading;
    reading.deg = measure_joint_angles(base_cyclic);

    // Robot reports degrees; Pinocchio works in radians, and the model has
    // its own configuration layout (handled by convertJointAnglesToConfig).
    Eigen::VectorXd q_rad(reading.deg.size());
    for (size_t i = 0; i < reading.deg.size(); ++i)
        q_rad(i) = reading.deg[i] * M_PI / 180.0;

    reading.q_pin = dynamics.convertJointAnglesToConfig(q_rad);
    return reading;
}
