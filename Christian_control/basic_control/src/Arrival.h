//
// Arrival — pure debouncing of target arrival, both directions.
//
// Both monitors are fed a per-cycle boolean and the measured cycle dt, and own
// only a scalar time accumulator: no I/O, no allocation, no blocking, safe
// inside the 500 Hz loop. Style mirrors Freshness.h. Time is accumulated from
// the Runner's measured, clamped dt so a scheduler stall stretches wall-clock,
// not the logical window; a non-finite or non-positive dt is a no-op.
//

#pragma once

#include <cmath>

// Positive signal. Reports arrival only after the pose has stayed within
// tolerance for a continuous hold_s window, so a single noisy in-tolerance
// cycle cannot latch arrival. A non-positive hold_s disables the debounce
// (arrival on the first in-tolerance cycle — the pre-debounce behaviour).
class ArrivalSettlingMonitor
{
public:
    explicit ArrivalSettlingMonitor(double hold_s) : hold_s_(hold_s) {}

    // in_tolerance: pose within arrival tolerance AND arrival-eligible this
    // cycle. dt_s: measured, clamped cycle time (s). Returns whether arrival
    // is currently reported.
    bool Update(bool in_tolerance, double dt_s)
    {
        if (!in_tolerance) {
            settled_s_ = 0.0;
            return false;
        }
        if (hold_s_ <= 0.0)
            return true;
        if (std::isfinite(dt_s) && dt_s > 0.0)
            settled_s_ += dt_s;
        return settled_s_ >= hold_s_;
    }

    double settled_s() const { return settled_s_; }
    void Rearm() { settled_s_ = 0.0; }

private:
    double hold_s_;
    double settled_s_ = 0.0;
};
