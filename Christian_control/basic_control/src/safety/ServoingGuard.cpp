//
// ServoingGuard: RAII ownership of the base's servoing mode (see header).
//

#include "safety/ServoingGuard.h"

#include <chrono>
#include <iostream>
#include <thread>

#include <KDetailedException.h>

ServoingGuard::ServoingGuard(k_api::Base::BaseClient* base)
    : base_(base)
{
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base_->SetServoingMode(servoing_mode);
}

ServoingGuard::~ServoingGuard()
{
    if (restored_)
        return;
    if (!Restore(std::cerr))
        std::cerr << "WARNING: could not restore SINGLE_LEVEL servoing — the arm may "
            "still be in low-level mode; check it before running anything else\n";
}

bool ServoingGuard::Restore(std::ostream& out) noexcept
{
    try
    {
        auto servoing_mode = k_api::Base::ServoingModeInformation();
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base_->SetServoingMode(servoing_mode);
        // TrajectoryExecution waits after this transition. Keep the session
        // and client alive while the base settles before any teardown.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        restored_ = true;
        return true;
    }
    catch (k_api::KDetailedException& ex)
    {
        out << "SINGLE_LEVEL restore Kortex error: " << ex.what()
            << " (sub-code "
            << k_api::SubErrorCodes_Name(static_cast<k_api::SubErrorCodes>(
                   ex.getErrorInfo().getError().error_sub_code()))
            << ")\n";
    }
    catch (std::exception& ex)
    {
        out << "SINGLE_LEVEL restore error: " << ex.what() << "\n";
    }
    catch (...)
    {
        out << "SINGLE_LEVEL restore error: unknown exception type\n";
    }
    return false;
}
