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
    k_api::BaseCyclic::Feedback feedback = read_feedback(base_cyclic_);
    for (int i = 0; i < NUM_JOINTS; ++i)
        command_.add_actuators();
    return feedback;
}

k_api::BaseCyclic::Feedback CyclicSession::Send(const JointVector& setpoints_deg)
{
    for (int i = 0; i < NUM_JOINTS; ++i)
        command_.mutable_actuators(i)->set_position(wrap_0_360(setpoints_deg[i]));
    command_.set_frame_id((command_.frame_id() + 1) % 65536);
    for (int i = 0; i < NUM_JOINTS; ++i)
        command_.mutable_actuators(i)->set_command_id(command_.frame_id());
    return base_cyclic_->Refresh(command_, 0);
}
