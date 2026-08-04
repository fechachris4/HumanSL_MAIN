# Arrival Settling Debounce and Non-Arrival Timeout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the controller's "arrived" signal trustworthy (debounce it over a settling window) and add an operator-facing "target NOT reached" report when the arm parks short of a target, without changing any motion.

**Architecture:** Two pure, header-only monitors in a new `src/Arrival.h` (mirroring `src/Freshness.h`) each accumulate time from the per-cycle `dt`. `ArrivalSettlingMonitor` reports arrival only after the pose has held within tolerance continuously for `kArrivalDwellS` (0.15 s). `ArrivalTimeoutMonitor` fires a one-shot edge when the arm has been parked-and-waiting at a target for `kTargetHoldS` (2.0 s, reused — no new constant). `TrackingController` owns both, feeds them the existing in-tolerance boolean and `dt_s`, and sets edge flags on `ControllerStatus`. `Runner` prints "target reached" or "target NOT reached" from those flags. The timeout path issues no command and no stop; the arm keeps holding exactly as it does today.

**Tech Stack:** C++17, Eigen, CMake, the repo's hand-rolled `Check(...)` test harness in `tests/test_control_logic.cpp` (no GoogleTest), CTest.

## Global Constraints

- Real-time loop runs at 500 Hz. Both monitors are called inside it: **no I/O, no allocation, no blocking, no locks** — only fixed scalar state, matching `Freshness.h`.
- Internally radians; operator-facing quantities are metres and seconds. The new constant `kArrivalDwellS` is **seconds** (the `S` suffix), reused `kTargetHoldS` is **seconds**.
- **No hardware run.** Building the `controller` target is allowed; executing it is never a test step (`CLAUDE.md`).
- **No new CSV column.** `log_format` stays 7; the Python parsers are untouched.
- **Exactly one new Config constant** (`kArrivalDwellS`). The non-arrival timeout reuses `kTargetHoldS`.
- Do not weaken any existing guard: `kArrivalToleranceM`, the following-error stop, joint-limit guard, freshness stop, and `kStopOnFault` stay exactly as they are.
- **Commits: do NOT append a `Co-Authored-By` trailer** (repo convention). Match the existing lowercase-prefix commit style (`safety:`, `test:`).
- Match the surrounding commented code style (e.g. `Freshness.h`); these files are not comment-free.

**Build / test commands** (run from `Christian_control/basic_control`, a configured `build/` dir already exists):

```bash
cd /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control
cmake --build build --target test_control_logic 2>&1 | tail -30
ctest --test-dir build -R control_logic --output-on-failure
```

The project compiles with `-w` (warnings suppressed globally), so add an explicit warning-checked syntax pass on the new pure header:

```bash
g++ -std=c++17 -Wall -Wextra -fsyntax-only -I src src/Arrival.h
```

---

### Task 1: `ArrivalSettlingMonitor` (positive signal)

The failing test **already exists** in the working tree (`TestArrivalSettling`, added but never called, and it `#include "Arrival.h"` which does not exist — so the target currently fails to compile). This task creates the header and wires the test in.

**Files:**
- Create: `src/Arrival.h`
- Modify: `tests/test_control_logic.cpp` (add the `main()` call at the list ending line 713)
- Test: `tests/test_control_logic.cpp::TestArrivalSettling` (lines 629-700, already written)

**Interfaces:**
- Produces: `class ArrivalSettlingMonitor` with `explicit ArrivalSettlingMonitor(double hold_s)`, `bool Update(bool in_tolerance, double dt_s)`, `double settled_s() const`, `void Rearm()`.

- [ ] **Step 1: Wire the existing test into `main()`**

In `tests/test_control_logic.cpp`, inside `main()` (the call list at lines 705-713), add after `TestStaleAcknowledgementGuard();`:

```cpp
    TestArrivalSettling();
```

- [ ] **Step 2: Build to verify it fails (red)**

Run: `cmake --build build --target test_control_logic 2>&1 | tail -30`
Expected: FAIL — `fatal error: Arrival.h: No such file or directory`.

- [ ] **Step 3: Create `src/Arrival.h` with `ArrivalSettlingMonitor`**

```cpp
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
```

- [ ] **Step 4: Build and run to verify it passes (green)**

Run:
```bash
cmake --build build --target test_control_logic 2>&1 | tail -30
ctest --test-dir build -R control_logic --output-on-failure
```
Expected: build succeeds; `control_logic` passes, output includes "all control-logic tests passed".

- [ ] **Step 5: Warning-checked compile of the new header**

Run: `g++ -std=c++17 -Wall -Wextra -fsyntax-only -I src src/Arrival.h`
Expected: no output (clean).

- [ ] **Step 6: Commit**

```bash
git add src/Arrival.h tests/test_control_logic.cpp
git commit -m "safety: debounce target arrival over a settling window

Add ArrivalSettlingMonitor (pure, header-only) and wire its already-written
test into main(). Arrival is reported only after the pose holds within
tolerance for a continuous window, so a single noisy in-tolerance cycle can
no longer latch arrival. Not yet wired into the controller."
```

---

### Task 2: `ArrivalTimeoutMonitor` (negative signal)

**Files:**
- Modify: `src/Arrival.h` (add the second class)
- Modify: `tests/test_control_logic.cpp` (add `TestArrivalTimeout`, and its `main()` call)
- Test: `tests/test_control_logic.cpp::TestArrivalTimeout` (new)

**Interfaces:**
- Produces: `class ArrivalTimeoutMonitor` with `explicit ArrivalTimeoutMonitor(double timeout_s)`, `bool Update(bool waiting, double dt_s)`, `double waited_s() const`, `void Rearm()`.

- [ ] **Step 1: Write the failing test**

In `tests/test_control_logic.cpp`, add a new function immediately after `TestArrivalSettling()` (after line 700, before the closing `}` of the anonymous namespace at line 702). It mirrors the settling test's structure:

```cpp
    void TestArrivalTimeout()
    {
        // 2 ms cycles at 500 Hz; a 0.1 s timeout is 50 cycles. The production
        // wiring uses kTargetHoldS (2.0 s); 0.1 s here is only test arithmetic.
        constexpr double kDt = 0.002;
        constexpr double kTimeout = 0.1;

        // Below the threshold, the non-arrival edge must not fire.
        ArrivalTimeoutMonitor early(kTimeout);
        for (int i = 0; i < 49; ++i)
            Check(!early.Update(true, kDt),
                  "no non-arrival edge before the timeout elapses");

        // A single one-shot edge exactly when the wait reaches the threshold,
        // and no re-fire while still waiting.
        ArrivalTimeoutMonitor fire(kTimeout);
        int fired_cycle = 0;
        for (int i = 0; i < 100; ++i) {
            if (fire.Update(true, kDt)) {
                fired_cycle = i + 1;
                break;
            }
        }
        Check(fired_cycle == 50,
              "100 ms of waiting at 2 ms fires the non-arrival edge on cycle 50");
        Check(!fire.Update(true, kDt),
              "the non-arrival edge does not re-fire while still waiting");

        // Reaching the target before the timeout (waiting goes false) cancels
        // the alarm and resets the accumulator.
        ArrivalTimeoutMonitor arrived(kTimeout);
        for (int i = 0; i < 40; ++i)
            arrived.Update(true, kDt);
        Check(!arrived.Update(false, kDt),
              "reaching the target before the timeout cancels the alarm");
        Check(arrived.waited_s() == 0.0,
              "leaving the waiting state resets the timeout accumulator");

        // Rearm clears both the accumulator and the fired latch for a new
        // target, and the full timeout must then be re-earned.
        ArrivalTimeoutMonitor rearmed(kTimeout);
        for (int i = 0; i < 60; ++i)
            rearmed.Update(true, kDt);
        rearmed.Rearm();
        Check(rearmed.waited_s() == 0.0, "Rearm clears the timeout accumulator");
        for (int i = 0; i < 49; ++i)
            Check(!rearmed.Update(true, kDt),
                  "the full timeout is re-earned after Rearm");
        Check(rearmed.Update(true, kDt),
              "the non-arrival edge fires again once the re-earned timeout elapses");

        // Broken timing neither accumulates nor resets.
        ArrivalTimeoutMonitor bad_dt(kTimeout);
        for (int i = 0; i < 40; ++i)
            bad_dt.Update(true, kDt);
        const double waited_before = bad_dt.waited_s();
        bad_dt.Update(true, 0.0);
        bad_dt.Update(true, -kDt);
        bad_dt.Update(true, std::numeric_limits<double>::quiet_NaN());
        bad_dt.Update(true, std::numeric_limits<double>::infinity());
        Check(bad_dt.waited_s() == waited_before,
              "non-finite and non-positive dt neither accumulate nor reset the timeout");

        // A non-positive timeout disables the gate entirely.
        ArrivalTimeoutMonitor disabled(0.0);
        for (int i = 0; i < 100; ++i)
            Check(!disabled.Update(true, kDt),
                  "a non-positive timeout never fires");
    }
```

Then add its call in `main()` immediately after `TestArrivalSettling();`:

```cpp
    TestArrivalTimeout();
```

(`<limits>` is already included at line 10; the `Check` helper and `failures` counter already exist.)

- [ ] **Step 2: Build to verify it fails (red)**

Run: `cmake --build build --target test_control_logic 2>&1 | tail -30`
Expected: FAIL — `'ArrivalTimeoutMonitor' was not declared in this scope`.

- [ ] **Step 3: Add `ArrivalTimeoutMonitor` to `src/Arrival.h`**

Append, after `ArrivalSettlingMonitor`:

```cpp
// Negative signal. While the arm is parked at a target and waiting to arrive,
// accumulates time and fires a one-shot edge once the wait reaches timeout_s.
// It drives no motion — the caller only reports it. A non-positive timeout_s
// disables the gate.
class ArrivalTimeoutMonitor
{
public:
    explicit ArrivalTimeoutMonitor(double timeout_s) : timeout_s_(timeout_s) {}

    // waiting: arrival-eligible AND not yet arrived this cycle. dt_s: measured
    // cycle time (s). Returns true only on the single cycle the wait first
    // reaches timeout_s.
    bool Update(bool waiting, double dt_s)
    {
        if (!waiting) {
            waited_s_ = 0.0;
            return false;
        }
        if (timeout_s_ <= 0.0)
            return false;
        if (std::isfinite(dt_s) && dt_s > 0.0)
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
```

- [ ] **Step 4: Build and run to verify it passes (green)**

Run:
```bash
cmake --build build --target test_control_logic 2>&1 | tail -30
ctest --test-dir build -R control_logic --output-on-failure
```
Expected: "all control-logic tests passed".

- [ ] **Step 5: Warning-checked compile**

Run: `g++ -std=c++17 -Wall -Wextra -fsyntax-only -I src src/Arrival.h`
Expected: clean.

- [ ] **Step 6: Commit**

```bash
git add src/Arrival.h tests/test_control_logic.cpp
git commit -m "safety: add non-arrival timeout monitor

Add ArrivalTimeoutMonitor (pure, header-only) and its test. It fires a
one-shot edge when the arm has been parked at a target and not arrived for
timeout_s. Reporting-only; not yet wired into the controller."
```

---

### Task 3: Controller integration + Config/State additions

Wire both monitors into `TrackingController` and add the one new constant and the status flag they need. Verified by compiling `controller` and re-running the full hardware-free suite (the controller itself has no unit test — its pure pieces are tested in Tasks 1-2).

**Files:**
- Modify: `src/Config.h` (add `kArrivalDwellS` at line 250; comment `kTargetHoldS` at line 105)
- Modify: `src/State.h` (add `not_reached_edge` at line 42)
- Modify: `src/Controller.h` (include `Arrival.h` at line 26; two members after line 71)
- Modify: `src/Controller.cpp` (constructor init list; `Reset`; the arrival block in `DesiredVelocity`)

**Interfaces:**
- Consumes: `ArrivalSettlingMonitor`, `ArrivalTimeoutMonitor` (Tasks 1-2); `config::kArrivalDwellS`, `config::kTargetHoldS`; `ControllerStatus::arrived_edge`, `::not_reached_edge`, `::arrival_error_m`.
- Produces: `ControllerStatus::not_reached_edge` set true on the timeout edge; `arrived_edge` now edge-fires off the debounced settling monitor.

- [ ] **Step 1: Add the Config constant and document the reuse**

In `src/Config.h`, after line 250 (`kArrivalOrientationToleranceRad`), add:

```cpp

    // Arrival settling debounce: the arrival notice fires only after the
    // end-effector holds within kArrivalToleranceM continuously for this long,
    // s. Sized above the ~144 ms measured closed-loop response lag
    // (whole-path-validation.md) so a debounced arrival confirms the physical
    // arm settled, not just the command. Non-positive disables the debounce.
    inline constexpr double kArrivalDwellS = 0.15;
```

Replace the `kTargetHoldS` line (line 105) with the same value plus a comment recording the reuse:

```cpp
    // Dwell held at each reached target before the queue advances, s. Also
    // reused as the non-arrival timeout: if the arm is parked at a target and
    // has not arrived within this long, the run reports "target NOT reached"
    // and keeps holding (Controller.cpp / Runner.cpp). Shortening this also
    // shortens that timeout.
    inline constexpr double kTargetHoldS = 2.0;
```

- [ ] **Step 2: Add the status flag**

In `src/State.h`, after line 42 (`double arrival_error_m = 0.0;`), add:

```cpp
    bool not_reached_edge = false; // set the cycle the non-arrival timeout fires
```

- [ ] **Step 3: Declare the monitors on the controller**

In `src/Controller.h`, add the include after line 26 (`#include "State.h"`):

```cpp
#include "Arrival.h"
```

Then, after line 71 (`bool arrival_reported_ = true;`) and before the closing `};`, add:

```cpp

    // Positive/negative arrival gates. Constructed from Config.h in the .cpp
    // (this header stays Config-free). Declared last so init order matches.
    ArrivalSettlingMonitor arrival_monitor_;
    ArrivalTimeoutMonitor timeout_monitor_;
```

- [ ] **Step 4: Construct the monitors and rearm on Reset**

In `src/Controller.cpp`, extend the constructor init list (lines 31-34) so it ends:

```cpp
TrackingController::TrackingController(DualArmKinematics& model)
    : model_(model),
      workspace_(std::make_unique<KinematicsWorkspace>(model.dynamics())),
      gains_(ConfiguredGains()),
      arrival_monitor_(config::kArrivalDwellS),
      timeout_monitor_(config::kTargetHoldS)
{
```

In `Reset` (lines 44-54), after `arrival_reported_ = true;` (line 53), add:

```cpp
    arrival_monitor_.Rearm();
    timeout_monitor_.Rearm();
```

- [ ] **Step 5: Rearm on a new target and drive both gates**

In `DesiredVelocity`, in the sequence-change block (lines 71-78), after `arrival_reported_ = false;` (line 77), add:

```cpp
        arrival_monitor_.Rearm();
        timeout_monitor_.Rearm();
```

Then replace the arrival block (lines 102-111, from `const bool arrival_eligible` through the closing brace of the `if`) with:

```cpp
    const bool arrival_eligible = reference.pose && reference.pose->arrival_eligible;
    const bool position_arrived = e_pos.norm() <= config::kArrivalToleranceM;
    const bool orientation_arrived = !gains_.orientation_enabled ||
        e_rot.norm() <= config::kArrivalOrientationToleranceRad;
    const bool in_tolerance =
        arrival_eligible && position_arrived && orientation_arrived;

    // Positive: debounce arrival over kArrivalDwellS, then edge-fire once per
    // target so Runner/Targets keep their once-per-target semantics.
    const bool reported = arrival_monitor_.Update(in_tolerance, dt_s);
    if (!arrival_reported_ && reported) {
        arrival_reported_ = true;
        status.arrived_edge = true;
        status.arrival_error_m = e_pos.norm();
    }

    // Negative: while parked at a target and not yet arrived, fire a one-shot
    // non-arrival edge after kTargetHoldS. Reporting only — no motion change.
    const bool waiting = arrival_eligible && !arrival_reported_;
    if (timeout_monitor_.Update(waiting, dt_s)) {
        status.not_reached_edge = true;
        status.arrival_error_m = e_pos.norm();
    }
```

- [ ] **Step 6: Build the controller and the test target (green)**

Run:
```bash
cmake --build build --target controller 2>&1 | tail -30
cmake --build build --target test_control_logic 2>&1 | tail -30
ctest --test-dir build -R control_logic --output-on-failure
```
Expected: both targets build; `control_logic` still passes. (Building `controller` only — never run it.)

- [ ] **Step 7: Commit**

```bash
git add src/Config.h src/State.h src/Controller.h src/Controller.cpp
git commit -m "safety: gate arrival on settling and detect non-arrival

Wire ArrivalSettlingMonitor and ArrivalTimeoutMonitor into TrackingController.
Arrival now edge-fires off the debounced settling gate; a new not_reached_edge
status flag fires once when the arm is parked at a target and has not arrived
within kTargetHoldS. Adds the single new constant kArrivalDwellS (0.15 s) and
documents the kTargetHoldS reuse. No motion, guard, or CSV-format change."
```

---

### Task 4: Runner non-arrival report

Print the operator-facing "target NOT reached" line from the new flag, next to the existing "target reached" print. No queue or motion action — the arm keeps holding.

**Files:**
- Modify: `src/Runner.cpp` (extend the arrival-notice block at lines 362-371)

**Interfaces:**
- Consumes: `ControllerStatus::not_reached_edge`, `::arrival_error_m`, `::p_desired`, `::p_current`; `config::kTargetHoldS`.

- [ ] **Step 1: Add the report branch**

In `src/Runner.cpp`, the arrival-notice block currently ends at line 371 (after `NotifyPoseTargetSourceOnArrivalEdge(reference, status);` and its closing `}`). Immediately after that closing brace, add:

```cpp
            else if (status.not_reached_edge)
            {
                std::cout << "target NOT reached: "
                    << status.arrival_error_m * 1000.0 << " mm short after "
                    << config::kTargetHoldS
                    << " s (holding; Ctrl+C to abort)\n";
            }
```

The `if (status.arrived_edge)` above becomes the first arm of the `if / else if`. The two edges are mutually exclusive: `waiting` requires `!arrival_reported_`, so once arrival fires the timeout cannot. `config::` is already used in this file (line 91), and `status.arrival_error_m` is set in the timeout block from Task 3.

- [ ] **Step 2: Build the controller (green)**

Run: `cmake --build build --target controller 2>&1 | tail -30`
Expected: builds cleanly. (Build only — do not run.)

- [ ] **Step 3: Full hardware-free suite**

Run: `ctest --test-dir build --output-on-failure`
Expected: all registered tests pass. Confirm each `add_test` target is hardware-free before running (they are: control logic, dual-arm model, reactive law, trajectory profile, process lock, supervisor, runlog compat).

- [ ] **Step 4: Commit**

```bash
git add src/Runner.cpp
git commit -m "safety: report when the arm does not reach a target

Print a one-shot 'target NOT reached: N mm short after H s (holding)' line
when the controller raises not_reached_edge, beside the existing 'target
reached' notice. Reporting only: the arm keeps holding and the queue does not
advance, so the operator can inspect, wait, or abort."
```

---

## Post-implementation (out of plan scope, human-gated)

- Review the full diff against the spec.
- With Christian's explicit confirmation, merge `codex/restore-controller` into `master` (only once green; the branch must not reach `master` while it does not compile).
- Supervised hardware validation of the end-to-end behaviour remains a separate, later step. Neither change makes motion more aggressive (the positive gate can only *delay* arrival; the negative gate only *reports*), so no guard is loosened, but physical behaviour is not claimed proven by this plan.

## Self-review notes

- **Spec coverage:** `ArrivalSettlingMonitor` (T1), `ArrivalTimeoutMonitor` (T2), one new constant + `kTargetHoldS` reuse + `not_reached_edge` + controller wiring + rearm-on-new-target + rearm-on-Reset (T3), Runner report with report-and-hold behaviour (T4), no-new-CSV-column (unchanged, honored), Level-2 safety framing and no-hardware-run (Global Constraints + post-implementation). All spec sections map to a task.
- **Types consistent across tasks:** `Update(bool, double)->bool`, `settled_s()/waited_s()->double`, `Rearm()->void`, `ControllerStatus::not_reached_edge` (bool) used identically in T3 (set) and T4 (read).
- **No placeholders:** every code and command step is concrete.
