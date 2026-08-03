//
// ServoingGuard: RAII ownership of the base's servoing mode — the ONLY code
// allowed to call SetServoingMode. Constructing enters LOW_LEVEL_SERVOING
// (from here until Restore WE are the controller and must stream, or at
// least seed, commands); explicit Restore returns to SINGLE_LEVEL and the
// destructor retries by unwinding on every exit path if that call failed.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_SERVOINGGUARD_H
#define HUMANSL_MASTERS_PROJECT_2025_SERVOINGGUARD_H

#include <ostream>

#include <BaseClientRpc.h>

namespace k_api = Kinova::Api;

class ServoingGuard
{
public:
    // Throws (Kortex) if the mode switch fails.
    explicit ServoingGuard(k_api::Base::BaseClient* base);

    // Guarded restore: the restore is a network call that can fail if the
    // link died — on failure it warns the operator instead of throwing
    // (a throwing destructor would abort the program).
    ~ServoingGuard();

    // Restore immediately after cyclic streaming ends, report the exact
    // Kortex failure when possible, and wait for the mode transition to
    // settle. The destructor retries as a final RAII backstop if this fails.
    bool Restore(std::ostream& out) noexcept;

    ServoingGuard(const ServoingGuard&) = delete;
    ServoingGuard& operator=(const ServoingGuard&) = delete;

private:
    k_api::Base::BaseClient* base_;
    bool restored_ = false;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_SERVOINGGUARD_H
