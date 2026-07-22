//
// Measure: read sensor data from the arm (no motion commands).
//

#include "hardware/Measure.h"

#include <cmath>
#include <utility>

k_api::BaseCyclic::Feedback read_feedback(k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    // The ONLY RefreshFeedback call in the program (see Measure.h).
    return base_cyclic->RefreshFeedback();
}

JointMeasurements measure_joints(k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    // Position and velocity come from the same feedback frame, so they describe
    // one robot-state snapshot rather than two time-separated reads.
    k_api::BaseCyclic::Feedback feedback = read_feedback(base_cyclic);

    JointMeasurements measurements;
    measurements.position_deg.resize(feedback.actuators_size());
    measurements.velocity_deg_s.resize(feedback.actuators_size());
    for (int i = 0; i < feedback.actuators_size(); ++i)
    {
        const auto& actuator = feedback.actuators(i);
        measurements.position_deg[i] = actuator.position();
        measurements.velocity_deg_s[i] = actuator.velocity();
    }
    return measurements;
}

JointReading measure_configuration(k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                                   Dynamics& dynamics)
{
    JointMeasurements measurements = measure_joints(base_cyclic);

    JointReading reading;
    reading.deg = std::move(measurements.position_deg);
    reading.velocity_deg_s = std::move(measurements.velocity_deg_s);

    // Robot reports degrees; Pinocchio works in radians, and the model has
    // its own configuration layout (handled by convertJointAnglesToConfig).
    Eigen::VectorXd q_rad(reading.deg.size());
    for (size_t i = 0; i < reading.deg.size(); ++i)
        q_rad(i) = reading.deg[i] * M_PI / 180.0;

    reading.q_pin = dynamics.convertJointAnglesToConfig(q_rad);
    return reading;
}
