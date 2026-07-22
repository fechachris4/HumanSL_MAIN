//
// ServoingGuard: RAII ownership of the base's servoing mode — the ONLY code
// allowed to call SetServoingMode. Constructing enters LOW_LEVEL_SERVOING
// (from here until destruction WE are the controller and must stream, or at
// least seed, commands); destruction restores SINGLE_LEVEL_SERVOING by
// unwinding, so the restore runs on every exit path, exceptions of any
// type included.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_SERVOINGGUARD_H
#define HUMANSL_MASTERS_PROJECT_2025_SERVOINGGUARD_H

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

    ServoingGuard(const ServoingGuard&) = delete;
    ServoingGuard& operator=(const ServoingGuard&) = delete;

private:
    k_api::Base::BaseClient* base_;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_SERVOINGGUARD_H
