# Safety.cpp + Safety.h — line-by-line read

**Entry points, in the order the program hits them:**

1. `RobotReadyForTakeover(initial, std::cout)` — Main.cpp:257, on a
   standalone read, BEFORE any mode switch (gate T1).
2. `ServoingGuard servoing_guard(base)` — Runner.cpp:120 (T2, enters
   low-level servoing); `servoing_guard.Restore(std::cout)` —
   Runner.cpp:391 (D2); the destructor as the retry backstop.
3. `FollowingErrorExceeded` / `HasLiveFault` — facts for each completed
   Runner sample; `FeedbackFreshnessMonitor` supplies the stale-ack fact.
4. `ResolveStopPriority` (StopPriority.h) — the Runner's actual precedence.
5. `PrintFaultChange` / `PrintStopReport` / decode helpers — edge-triggered
   and post-loop printing (Runner.cpp:289, 392).
6. `StopReasonName(result.reason)` — Main.cpp:411, the CSV exit trailer.

The file is three modules merged into one (the section banners are the seams
of that merge): the **Supervisor** (policy: when to stop, when to refuse the
takeover), **FaultReport** (printing: decode banks, explain stops), and the
**ServoingGuard** (the RAII mode owner). Policy never prints inside the
cycle; printing never decides anything.

---

## Safety.h

### Lines 1–19 — banner and includes
Kortex client headers (the fault enums live in the protobuf-generated API),
plus Config and Hardware (for the `LoopLogSample` the fact helpers read).

### Lines 21–34 — section banner, `namespace k_api`
`namespace k_api = Kinova::Api;` — a namespace alias. It appears again
verbatim at line 179. **[unnecessary, lines 30–34]** The duplicated alias
and the runs of blank comment lines (also 139–141, 151–154 in the .cpp) are
leftovers of merging three files into one; harmless, but noise. One alias at
the top would do.

### Lines 36–54 — `enum class LoopStop`
An `enum class` is a strongly-typed set of named constants — unlike a plain
`enum`, its values do not implicitly convert to int and must be written
`LoopStop::kUserStop`, so you cannot accidentally compare it against the
wrong enum. Ten reasons a loop ends; the comment defines each. Only
`kUserStop` is success. `kFollowingError` = the arm stopped following the
integrated command; `kLeftLowLevel` = the base dropped out of low-level
servoing on its own; `kJointLimitWarning` = the software held the complete
last-safe command frame before an outward warning crossing;
`kStaleFeedback` = an actuator acknowledgement remained unchanged for its
configured window; and `kNonFiniteCommand` / `kOverrun` are counter stops.

### Lines 56–69 — `struct LoopResult`
What `RunControlLoop` returns to Main. `faults_observed` is the taint flag:
a live fault seen at ANY point forces a nonzero exit even if the
fault-ignoring policy kept the loop running — you cannot run through a fault
and still report success. `stop_t_s` / `cycles` come from the last logged
sample so the console report lines up with the CSV.

### Freshness.h — `FeedbackFreshnessMonitor`
Each actuator's feedback carries the ID of the last cyclic command it
processed; a healthy stream advances every joint's ID every cycle. This
class counts, per joint, how many CONSECUTIVE completed replies repeat the
ID. `StaleAcknowledgementJoint` is an active Runner stop at
`kStaleFeedbackStopCycles = 25` (50 ms at 500 Hz). The first reply seeds the
monitor at zero; a new ID resets that joint to zero. This proves cyclic
acknowledgement progress only, never physical motion or setpoint acceptance.

### Lines 98–107 — `struct CycleCounters`
Plain data: the consecutive non-finite and overrun counters plus the
whole-run overrun tally. The Runner owns the update logic; counter stops are
checked after the completed-sample `ResolveStopPriority` decision.

### Lines 109–112 — `kJointFaultBit`
The base's JOINT_FAULT summary bit, named once. `inline constexpr` means one
compile-time constant shared across all files that include this header.
**[edit-hazard, lines 109–112]** This single constant encodes the project's
hardest-won fault-handling judgement: the latched summary bit alone is a
stale historical aggregate, not a live interlock. It is masked out by
`HasLiveFault` and tolerated in
`RobotReadyForTakeover` (127). Change either use without the other and the
gate and the loop disagree about what a fault is.

### Policy fact and compatibility declarations
`FollowingErrorExceeded` and `HasLiveFault` expose loop-safe facts used by
the Runner and StopPriority. `ClassifyStop` is a compatibility helper for
callers that need only following-error / low-level / live-fault
classification; the Runner does not call it. `RobotReadyForTakeover` is the
pre-takeover gate, prints its findings to the passed `std::ostream&` (an
output stream by reference — passing `std::cout` in production, a
`std::ostringstream` in a test).

### Lines 128–162 — FaultReport declarations
`DecodeBaseBank` / `DecodeActuatorBank` ("16 (JOINT_FAULT)"-style),
`kMaxFaultChangePrints` (20 — the flood cap), `StopReasonName` (the stable
machine token for the CSV trailer — Main writes it, offline tooling parses
it, so the strings are an interface: **[edit-hazard, line 155]** renaming a
token breaks every script that greps run CSVs), `PrintStopReport`,
`PrintFaultChange`.

### Lines 164–203 — `class ServoingGuard`
The RAII owner of the servoing mode, and by rule the ONLY code allowed to
call `SetServoingMode`. Constructor enters LOW_LEVEL_SERVOING (throws on
failure — if the mode switch fails you never get a guard, and the Runner
never enters its try block). `Restore` is `noexcept` — the C++ promise that
a function never throws; here it is enforced by catching everything
internally, because it is also called from the destructor, and an exception
escaping a destructor during exception unwinding terminates the whole
program. Copying is deleted (lines 197–198): two guards for one arm would
fight over the mode. `restored_` makes restore idempotent.

---

## Safety.cpp

### Lines 5–30 — includes and `NUM_JOINTS`
`ActuatorConfig.pb.h` supplies the actuator-bank name lookup;
`KDetailedException.h` the Kortex error type `Restore` decodes. `NUM_JOINTS`
derived from `JointVector` as in Runner.cpp.

### Lines 32–80 — sample facts and compatibility classification
`FollowingErrorExceeded` checks the wrapped-to-command position gap unless
the compile-time override disables it. `HasLiveFault` checks every actuator
bank plus every base bit except the latched JOINT_FAULT summary.

`ClassifyStop` composes those facts for compatibility callers in the order
following error → left low-level → live fault. It does not include held
joint warnings, stale acknowledgements, fault-stop policy, or counter stops,
and the Runner does not use it. The Runner's authoritative order is
`ResolveStopPriority`: following error → left low-level → enabled live fault
→ held joint warning → stale acknowledgement, followed by nonfinite/overrun
counters.

### Lines 98–142 — `RobotReadyForTakeover` (T1, Main.cpp:257)
Runs on one standalone feedback frame, before the mode switch — a live
fault means the program never takes over at all.

- **100–115** — read the base bank; print the arm state and the decoded
  base bank; walk every actuator, print and remember any non-zero bank.
- **117–124** — the 2026-07-31 lesson, in code: `in_fault_state` is true
  when the base reports ARMSTATE_IN_FAULT. The arm's own state outranks any
  bank decoding — a run once proceeded to takeover while the arm printed
  IN_FAULT, because the bank heuristics all looked clear.
- **126–129** — the stale-summary tolerance: if the ONLY thing wrong is the
  latched JOINT_FAULT summary (no actuator fault, not in fault state),
  print a note and continue. **[edit-hazard, lines 126–129]** This is the
  gate's one deliberate leniency; widening it (e.g. tolerating an actuator
  bank too) removes the barrier that keeps a genuinely faulted arm from
  being taken over, and it must stay consistent with `HasLiveFault`'s
  masking of the same bit.
- **130–140** — refuse if: any actuator fault, any base bit besides the
  summary, or the IN_FAULT state — with a message telling the operator to
  clear deliberately via the web dashboard (this program never clears
  faults here; Main's `ClearFaults` runs earlier and unconditionally, which
  is a different, pre-gate step). Returns true only when nothing live is
  wrong.

### Lines 155–173 — `DecodeBank` (file-local helper)
A safety bank is a bitmask: each set bit is one named safety event. The
helper walks all 32 bits (`for (bit = 1; bit != 0; bit <<= 1)` — shifting
left until the 1 falls off the top), and for each set bit asks `name_of` for
its name. `name_of` is a *function pointer* parameter
(`const std::string& (*name_of)(int)`): the caller passes which enum's
name-lookup to use, so one loop serves both bank types. An empty name (an
unknown bit) prints as "bit N" rather than nothing. Output looks like
`"18 (JOINT_FAULT | bit 2)"`.

### Lines 175–191 — `DecodeBaseBank` / `DecodeActuatorBank`
Each passes `DecodeBank` a lambda that wraps the protobuf-generated
`*_Name` lookup for its enum. The lambda has a `-> const std::string&`
return type: protobuf's name tables are static strings, so returning a
reference is safe here (the referenced string outlives the call). A
capture-free lambda converts to a plain function pointer, which is why this
compiles against the function-pointer parameter.

### Lines 193–207 — `StopReasonName` (Main.cpp:411)
The enum→token switch for the CSV exit trailer. No `default:` case — on
purpose: with all ten enumerators handled, a newly added `LoopStop` value
makes the compiler warn about the switch, pointing you here. The trailing
`return "unknown"` silences the "control reaches end" warning.

### Lines 209–277 — `PrintStopReport` (D3, Runner.cpp:392)
One human-readable explanation per stop reason, printed once, after the
loop:

- **214–269** — the per-reason headline. The `kFollowingError` case
  (221–239) re-scans the final sample to find and name the WORST joint and
  its gap — the single most useful line in a bad run's output.
- **265–267** — desired vs current end-effector position, always printed.
- **268–269** — a clean user stop ends here; only failure reports go on to
  the full dump.
- **270–276** — the full per-joint dump: decoded fault bank, commanded
  position and velocity, measured (wrapped-to-command) and raw measured
  positions — everything needed to reason about the stop without opening
  the CSV.

### Lines 279–292 — `PrintFaultChange` (edge-triggered, Runner.cpp:289)
Prints only what CHANGED: base bank old → new if it differs, and each joint
whose bank differs. Called at most `kMaxFaultChangePrints` times per run
(the cap lives in the Runner). Printing inside the cycle is tolerated here
precisely because it is edge-triggered and capped.

### Lines 305–311 — `ServoingGuard` constructor (T2)
Builds the protobuf `ServoingModeInformation`, sets LOW_LEVEL_SERVOING,
sends it. **THIS CALL CHANGES THE ROBOT'S MODE.** No try/catch: if it
throws, the guard object never finishes constructing, its destructor will
never run (C++ only destroys fully-constructed objects), and the exception
propagates to Main's handler — correct, because there is nothing to
restore: the mode never changed.

### Lines 313–320 — the destructor
If `Restore` already succeeded (`restored_`), do nothing. Otherwise call
`Restore(std::cerr)` — the RAII backstop for every exit path the explicit
call did not cover (including stack unwinding after an exception) — and if
even that fails, print the one warning that really matters: the arm may
still be in low-level mode; check it before running anything else.

### Lines 322–352 — `Restore(out) noexcept`
The explicit restore (D2): set SINGLE_LEVEL_SERVOING, then
**line 331** sleep 100 ms. **[hides-work, lines 326–332]** The sleep is not
decoration: it keeps the session and clients alive while the base settles
the mode transition (mirroring Kinova's own TrajectoryExecution example);
remove it and teardown can race the transition. Then `restored_ = true` so
the destructor stays quiet. The three catch clauses (335–351) decode a
Kortex failure with its sub-code, then any `std::exception`, then anything —
returning false instead of throwing, which is what makes the `noexcept`
promise true and the destructor call safe.
