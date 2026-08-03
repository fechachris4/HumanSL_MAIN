//
// Cyclic: the UDP cyclic exchange (see Cyclic.h).
//
// Pattern follows TrajectoryExecution/src/KinovaTrajectory.cpp
// (joint_position_control): LOW_LEVEL_SERVOING + one command frame per cycle.
//

#include "hardware/Cyclic.h"

#include "hardware/Measure.h"

#include <cmath>
#include <tuple>

namespace
{
    // The number of physical joints is defined by the JointVector type.
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;

    // The robot expects command angles in [0, 360).
    double wrap_0_360(double angle_deg)
    {
        double wrapped = std::fmod(angle_deg, 360.0);
        return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
    }
} // namespace

CyclicSession::CyclicSession(k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
    : base_cyclic_(base_cyclic)
{
}

k_api::BaseCyclic::Feedback CyclicSession::Seed()
{
    // Read the robot's current state before sending the first low-level command.
    k_api::BaseCyclic::Feedback feedback = read_feedback(base_cyclic_);

    // Make one persistent command slot for each physical actuator.
    for (int i = 0; i < NUM_JOINTS; ++i)
        command_.add_actuators();
    return feedback;
}

k_api::BaseCyclic::Feedback CyclicSession::Send(const JointVector& setpoints_deg)
{
    // Put the next desired absolute angle into each joint's existing command slot.
    for (int i = 0; i < NUM_JOINTS; ++i)
        command_.mutable_actuators(i)->set_position(wrap_0_360(setpoints_deg[i]));

    // Advance the packet sequence number so this cyclic exchange is identifiable.
    command_.set_frame_id((command_.frame_id() + 1) % 65536);

    // Give every actuator command the same packet ID for this control cycle.
    for (int i = 0; i < NUM_JOINTS; ++i)
        command_.mutable_actuators(i)->set_command_id(command_.frame_id());

    // Send the complete seven-joint packet and return the robot's feedback reply.
    return base_cyclic_->Refresh(command_, 0);
}
