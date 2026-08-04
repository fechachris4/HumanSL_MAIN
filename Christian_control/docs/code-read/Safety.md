# Safety.cpp + Safety.h — line-by-line read

**Entry points, in the order the program hits them:**

1. `RobotReadyForTakeover(initial, std::cout)` — Main.cpp:257, on a
   standalone read, BEFORE any mode switch (gate T1).
2. `ServoingGuard servoing_guard(base)` — Runner.cpp:120 (T2, enters
   low-level servoing); `servoing_guard.Restore(std::cout)` —
   Runner.cpp:391 (D2); the destructor as the retry backstop.
3. `ClassifyStop(sample, limit, reason)` — Runner.cpp:305, every cycle.
4. `FeedbackFreshnessMonitor::Update` — Runner.cpp:320, every cycle.
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
plus Config and Hardware (for `LoopLogSample`, which `ClassifyStop` reads).

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
wrong enum. Eight reasons a loop ends; the comment defines each. Only
`kUserStop` is success. `kFollowingError` = the arm stopped following the
integrated command; `kLeftLowLevel` = the base dropped out of low-level
servoing on its own; `kNonFiniteCommand` / `kOverrun` are the
consecutive-cycle counter stops.

### Lines 56–69 — `struct LoopResult`
What `RunControlLoop` returns to Main. `faults_observed` is the taint flag:
a live fault seen at ANY point forces a nonzero exit even if the
fault-ignoring policy kept the loop running — you cannot run through a fault
and still report success. `stop_t_s` / `cycles` come from the last logged
sample so the console report lines up with the CSV.

### Lines 71–96 — `FeedbackFreshnessMonitor`
Each actuator's feedback carries the ID of the last cyclic command it
processed; a healthy stream advances every joint's ID every cycle. This
class counts, per joint, how many CONSECUTIVE cycles the ID repeated.
The block comment is the important part: **TELEMETRY ONLY** since
2026-08-03 — this used to be a stop, and the comment exists so nobody
re-reads the counters as policy. Genuine communication failure still
surfaces as a throwing `Refresh` → `kCommunication`.
**[unnecessary, line 88]** `Reset()` is dead code in the executed program:
the Runner constructs a fresh monitor per run and never calls `Reset` (the
constructor defaults do the same job). Kept presumably for tests/reuse.

### Lines 98–107 — `struct CycleCounters`
Plain data: the consecutive non-finite and overrun counters plus the
whole-run overrun tally. The Runner owns the update logic; this header only
defines the record and documents the priority rule (counter stops are
checked AFTER `ClassifyStop`).

### Lines 109–112 — `kJointFaultBit`
The base's JOINT_FAULT summary bit, named once. `inline constexpr` means one
compile-time constant shared across all files that include this header.
**[edit-hazard, lines 109–112]** This single constant encodes the project's
hardest-won fault-handling judgement: the latched summary bit alone is a
stale historical aggregate, not a live interlock. It is masked out in
`ClassifyStop` (line 85 of the .cpp) and tolerated in
`RobotReadyForTakeover` (127). Change either use without the other and the
gate and the loop disagree about what a fault is.

### Lines 114–126 — the two policy function declarations
`ClassifyStop`: no printing (loop-safe), following-error checked FIRST so no
fault-ignoring policy can mask it, returns whether to stop via `bool` and
why via the `LoopStop& reason` out-parameter. `RobotReadyForTakeover`: the
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

### Lines 32–57 — `FeedbackFreshnessMonitor` (Update runs every cycle)
`Reset` (32–36): back to the uninitialized state. `Update` (38–57): on the
first call there is nothing to compare against, so it seeds `previous_` and
reports zeros; every later call, per joint: same ID as last cycle →
increment the counter, new ID → reset it to 0, then remember the ID. Zero
therefore means "advanced this cycle". Pure bookkeeping, no I/O, loop-safe.

### Lines 59–96 — `ClassifyStop` (every cycle, Runner.cpp:305)
Reads only the just-filled `LoopLogSample` — the classifier judges exactly
what was logged, so the CSV always contains the evidence for the stop. The
checks, in deliberate priority order (first match returns):

1. **71–78 — following error.** Unless disabled by
   `kDisableFollowingErrorStop`, any joint whose
   `|measured_deg − commanded_deg|` exceeds the limit (3°) stops the loop.
   Checked FIRST so the fault-ignoring experiment policy can never mask it
   (a faulted joint that stops moving trips this even when fault bits are
   ignored). **[hides-work, lines 71–78]** The comparison is only correct
   because `FillSample` (Runner.cpp:55–56) already shifted `measured_deg`
   to within ±180° of the command — the wrap that makes this subtraction
   meaningful lives in another file. The comment explains why that is sound
   (the gap grows well under a degree per cycle), but an edit to FillSample
   changes this guard's meaning without touching this function.
2. **79–84 — any actuator fault bit** → `kRobotFault`.
3. **85–89 — any base fault bit EXCEPT the JOINT_FAULT summary**
   (`base_fault_bank & ~kJointFaultBit`) → `kRobotFault`. The `~` is
   bitwise-NOT: "all bits except that one". This is the stale-summary
   masking decision (see Safety.h:109–112).
4. **90–94 — the arm state itself.** If the base is no longer in
   ARMSTATE_SERVOING_LOW_LEVEL, we are no longer the controller —
   `kLeftLowLevel`. With every other guard disabled by config, this is the
   one automatic stop that always remains.

Returns false = keep running. **[edit-hazard, lines 59–96]** The order IS
the policy: reordering these checks changes which reason wins when several
are true at once, and the comments record that the current order was chosen
on purpose (guard before faults, faults before mode).

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
  being taken over, and it must stay consistent with `ClassifyStop`'s
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
purpose: with all eight enumerators handled, a newly added `LoopStop` value
makes the compiler warn about the switch, pointing you here. The trailing
`return "unknown"` silences the "control reaches end" warning.

### Lines 209–277 — `PrintStopReport` (D3, Runner.cpp:392)
One human-readable explanation per stop reason, printed once, after the
loop:

- **214–263** — the per-reason headline. The `kFollowingError` case
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
