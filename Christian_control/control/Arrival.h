//
// Arrival — pure debouncing of target arrival, both directions.
//
// Both monitors are fed a per-cycle boolean and the measured cycle dt, and own
// only a scalar time accumulator: no I/O, no allocation, no blocking, safe
// inside the 500 Hz loop. Style mirrors Freshness.h. Time is accumulated from
// the controller's fixed control dt so the logical window follows control
// cycles; a non-finite or non-positive dt is a no-op.
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
    // cycle. dt_s: fixed control step (s). Returns whether arrival
    // is currently reported.
    bool Update(bool in_tolerance, double dt_s)
    {
        if (!in_tolerance) {
            settled_s_ = 0.0;
            return false;
        }
        if (hold_s_ <= 0.0)
            return true;
        settled_s_ += dt_s;
        return settled_s_ >= hold_s_;
    }

    double settled_s() const { return settled_s_; }
    void Rearm() { settled_s_ = 0.0; }

private:
    double hold_s_;
    double settled_s_ = 0.0;
};

// Negative signal. While the arm is parked at a target and waiting to arrive,
// accumulates time and fires a one-shot edge once the wait reaches timeout_s.
// It drives no motion — the caller only reports it. A non-positive timeout_s
// disables the gate.
class ArrivalTimeoutMonitor
{
public:
    explicit ArrivalTimeoutMonitor(double timeout_s) : timeout_s_(timeout_s) {}

    // waiting: arrival-eligible AND not yet arrived this cycle. dt_s: fixed
    // control step (s). Returns true only on the single cycle the wait first
    // reaches timeout_s.
    bool Update(bool waiting, double dt_s)
    {
        if (!waiting) {
            waited_s_ = 0.0;
            return false;
        }
        if (timeout_s_ <= 0.0)
            return false;
        waited_s_ += dt_s;
        if (!fired_ && waited_s_ >= timeout_s_) {
            fired_ = true;
            return true;
        }
        return false;
    }

    double waited_s() const { return waited_s_; }
    void Rearm()
    {
        waited_s_ = 0.0;
        fired_ = false;
    }

private:
    double timeout_s_;
    double waited_s_ = 0.0;
    bool fired_ = false;
};
