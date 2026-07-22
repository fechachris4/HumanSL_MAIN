//
// ServoingGuard: RAII ownership of the base's servoing mode (see header).
//

#include "safety/ServoingGuard.h"

#include <iostream>

ServoingGuard::ServoingGuard(k_api::Base::BaseClient* base)
    : base_(base)
{
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base_->SetServoingMode(servoing_mode);
}

ServoingGuard::~ServoingGuard()
{
    try
    {
        auto servoing_mode = k_api::Base::ServoingModeInformation();
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base_->SetServoingMode(servoing_mode);
    }
    catch (...)
    {
        std::cout << "WARNING: could not restore SINGLE_LEVEL servoing — the arm may "
            "still be in low-level mode; check it before running anything else\n";
    }
}
