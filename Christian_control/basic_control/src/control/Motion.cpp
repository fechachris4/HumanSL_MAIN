//
// Motion: cyclic command primitives for BaseCyclic — all 7 joints at once.
//
// Pattern follows TrajectoryExecution/src/KinovaTrajectory.cpp
// (joint_position_control): LOW_LEVEL_SERVOING + one command frame per cycle.
//

#include "control/Motion.h"

#include <cmath>
#include <iostream>
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

k_api::BaseCyclic::Feedback send_positions(k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                                           k_api::BaseCyclic::Command& command,
                                           const JointVector& angles)
{
    for (int i = 0; i < NUM_JOINTS; ++i)
        command.mutable_actuators(i)->set_position(wrap_0_360(angles[i]));
    command.set_frame_id((command.frame_id() + 1) % 65536);
    for (int i = 0; i < NUM_JOINTS; ++i)
        command.mutable_actuators(i)->set_command_id(command.frame_id());
    return base_cyclic->Refresh(command, 0);
}

void enter_low_level_servoing(k_api::Base::BaseClient* base)
{
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
}

bool restore_single_level_servoing(k_api::Base::BaseClient* base)
{
    try
    {
        auto servoing_mode = k_api::Base::ServoingModeInformation();
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        return true;
    }
    catch (...)
    {
        std::cout << "WARNING: could not restore SINGLE_LEVEL servoing — the arm may "
            "still be in low-level mode; check it before running anything else\n";
        return false;
    }
}
