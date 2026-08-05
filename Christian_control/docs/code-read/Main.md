# Main.cpp — line-by-line, in execution order

File: `basic_control/src/Main.cpp`, at HEAD after commit f64325c0 ("remove
trajectory playback: fixed-target-only controller"). The program's entry
point. Everything below follows the order the CPU actually visits things
when you run `./controller`, not the order they appear in the file. Helper
functions are explained at the point main() first calls them.

The program now connects, performs the mandatory robot safety checks, takes
over while holding the measured current pose, follows a bounded profile to the
compiled terminal target, accepts queued stdin targets, holds, and stops on
Ctrl+C. There is no exact registered-startup-pose requirement.

Flag categories used inline: **unnecessary**, **hides-work**,
**mixed-jobs**, **edit-hazard**.

---

## Before main() runs: file-scope setup

### Lines 1–5 — file banner comment
Says what the file is: parse `--log`, load the model, connect, check
readiness, wire the reference source to the controller, run the loop,
report. The control loop itself is in Runner.h/.cpp, not here. Pure
documentation; no runtime effect.

### Lines 7–23 — standard-library includes
Each `#include` pastes in declarations the file needs: `<atomic>` for the
stop flag, `<csignal>` for Ctrl+C handling, `<filesystem>` for creating the
runs directory, `<fstream>` for the CSV file, `<optional>` for "a value
that may be absent", and so on. Note `<thread>` (line 21) is still needed —
not for any thread main() starts (there is no stdin thread any more; the
only worker thread lives inside LoopLogWriter), but for
`std::this_thread::sleep_for` at line 251.

### Line 25 — `#include <KDetailedException.h>`
Kinova's rich exception type. Needed so the `catch` at line 433 can name the
exact Kortex sub-error code instead of a generic message.

### Lines 27–35 — project includes
The other modules of this program. Main.cpp is the only file that sees all
of them, because main() is where they get wired together. (Trajectory.h is
gone from this list since the playback removal.)

### Line 37 — `namespace k_api = Kinova::Api;`
A namespace alias: from here on, `k_api::` is shorthand for `Kinova::Api::`.
Purely a typing convenience.

### Lines 39–46 — comment block about the config echo
Explains that `--log` is the only runtime argument and that the same
`key = value` lines serve as "the startup echo and the CSV preamble".
**FLAG unnecessary (lines 43–46):** the "startup echo" half of this claim
is stale — `WriteConfigLines` is only ever called from `WriteCsvPreamble`
(line 116), and the comment at lines 222–224 inside main() says the startup
print was deliberately dropped ("without printing thirty lines at every
start"). A reader trusting this comment would look for a console echo that
does not exist.

### Line 48 — `namespace {`
An *anonymous namespace*: everything inside is private to this .cpp file
(other files cannot link against it). The C++ way of saying "these helpers
are internal".

### Lines 49–55 — `UsageAndExit`
Prints an optional error plus the usage text to stderr, then `std::exit(2)`.
`[[noreturn]]` is a promise to the compiler that this function never returns
(it always exits), which silences "missing return" warnings at call sites.
Exit code 2 is the conventional "bad command line" code, distinct from 1
(run failed) and 0 (clean stop). Note `std::exit` skips destructors of
locals — fine here because nothing is open yet at parse time, but this
function must never be called after the hardware connection exists.

### Lines 57–61 — `FormatDouble`
Formats a double via an `ostringstream` with default formatting, which
prints the shortest faithful representation (e.g. `0.5`, not `0.500000` like
`std::to_string` would). Exists so config values in the CSV preamble look
like what was typed in Config.h.

### Lines 63–101 — `WriteConfigLines`
Writes every Config.h value as `prefix + "key = value (Config.h)"` lines to
a stream. Called once per run, from `WriteCsvPreamble`, with prefix `"# "`.

- **Line 64** — `const auto line = [&](...)`: a *lambda*, a small unnamed
  function stored in a variable. `[&]` means it can use surrounding
  variables (`out`, `prefix`) by reference. Exists to avoid repeating the
  `out << prefix << key ...` boilerplate ~20 times.
- **Lines 67–75** — the reactive-law gains and feature toggles.
- **Lines 76–84** — the terminal target and, only when `kFixedTargetUseRpy` is on, its
  rpy. The `if` mirrors the config's own dependency: the rpy line only
  appears when it will actually be commanded.
- **Lines 85–97** — the following-error limit, the guard overrides and the
  stop counters, recorded "so a log can be read back knowing which
  protections were active". This is a safety-audit feature: never remove
  lines here without understanding you are erasing evidence from future
  run records.
- **Lines 98–99** — the log_file line. **FLAG edit-hazard (lines 98–99):**
  the value written is whatever string was *passed in* — and main() passes
  `log_file_arg` (line 312), the raw `--log` argument, not the resolved
  path. On a default run the preamble therefore says
  `log_file = <timestamped> (default)` and never records the actual
  filename. Harmless today (the file knows its own name on disk), but
  anyone "fixing" the preamble to show the real path must change the
  caller, not this function — and anyone parsing preambles must not expect
  a real path here.
- **Line 100** — control period, last so it sits next to the data that
  follows.

### Lines 103–111 — `ParseLogFileArg`
Walks `argv` (the command-line words). The only accepted option is `--log
<file>`; anything else calls `UsageAndExit`. Returns the filename, or `""`
if `--log` was not given.
**FLAG edit-hazard (lines 105–108):** the loop increments `i` in *two*
places — the `for` header and `++i` inside the body (line 107) to consume
the filename. The `if (++i >= argc)` both advances and bounds-checks in one
expression. Correct, but compact enough that inserting a new option or
reordering the checks could easily walk off the end of `argv` or eat the
wrong word.

### Lines 113–117 — `WriteCsvPreamble`
Writes two header comment lines then delegates to `WriteConfigLines` with
the `"# "` prefix. CSV parsers skip `#` lines, so the preamble rides inside
the data file without breaking it.
**FLAG edit-hazard (line 115):** `# log_format = 8 (compiled)` is a
hand-maintained schema version, bumped from 7 with the reactive-law
decomposition columns (taskvel_j*, nullvel_j*, null_leak_mps). If someone changes the CSV
columns or preamble contract without bumping this number — or bumps it
without changing them — every analysis script keyed on the format number
silently misreads data.

### Line 118 — `}` closes the first anonymous namespace.

### Lines 120–139 — the "story of the program" comment
A prose walkthrough of main() from parse to exit, updated for the
current-pose startup design (the reference source is now named as "the
current-start reference source"). Worth reading once; it is the file's own
table of contents. The all-caps "MOVES THE ARM" marks where hardware motion
begins.

### Lines 144–202 — second anonymous namespace: the print helpers

- **Lines 147–148** — column widths for the joint tables, named constants
  so header and rows stay aligned if either changes.
- **Lines 150–156 — `PrintJointHeader`**: prints `joint   1   2 ... 7`.
  `std::tuple_size_v<JointVector>` reads the array's size (7) at compile
  time, so the header automatically matches the joint count.
  `std::left/std::right/std::setw` are stream formatting: pad the label
  left-aligned to 16 chars, then each number right-aligned to 10.
- **Lines 158–165 — `PrintRow`**: one labeled row of 7 numbers, fixed 2
  decimals. `std::defaultfloat` at the end *restores* the stream's normal
  float formatting — forget that and every later number in the program
  prints with this row's formatting (stream formatting flags are sticky
  global state). Both helpers now have a single caller, `PrintRobotState`
  (lines 181–183); they survived the playback removal because the state
  printout still needs them.
- **Lines 170–200 — `PrintRobotState`**: prints the measured joint state
  and the FK end-effector pose.
  - Lines 176–180: copies each actuator's position (deg) and velocity
    (deg/s) out of the Kortex feedback message, and builds `q_rad`, the
    same positions converted to radians for the kinematics.
  - Lines 181–183: the joint table.
  - Lines 185–186: **FLAG hides-work:** despite the "Print" name, this
    constructs a `KinematicsWorkspace` (allocates Pinocchio scratch
    buffers) and runs full forward kinematics + Jacobian
    (`RightPoseAndJacobian`) just to print one pose. Cheap enough at
    startup, but this is real computation inside a print function — calling
    it from a fast path, or assuming it only formats text, would be wrong.
  - Line 190: `ee.rotation.eulerAngles(2, 1, 0)` decomposes the rotation
    matrix as R = Rz·Ry·Rx. **FLAG edit-hazard (lines 190–198):** Eigen
    returns the angles *in the order requested* — `zyx[0]` is the Z angle
    (yaw), `zyx[2]` is the X angle (roll) — so the print deliberately emits
    `zyx.z(), zyx.y(), zyx.x()` to get roll-pitch-yaw order. Swapping those
    accessors "to tidy up" would print yaw where roll belongs, and the
    printed triple is explicitly meant to be pasted into
    `kFixedTargetRpyRad` — a wrong paste rotates the real tool the wrong
    way on the next compiled run.
  - Why this function exists at all: the comment at 167–169 says it — the
    printed pose is what "hold here" looks like, and under the
    current-start design it is the *only* in-band way to choose the
    next compiled target: read this printout, edit Config.h, recompile.

### Lines 204–212 — the stop flag and signal handler
- **Line 208** — `std::atomic<bool> g_stop{false};` A global flag. *atomic*
  means reads and writes are indivisible and safely visible across threads
  — the signal handler and the 500 Hz control loop both touch this one
  bool without a mutex. A plain `bool` here would be undefined behaviour
  (a data race, and also not legal to write from a signal handler).
- **Lines 209–212** — `on_stop_signal` just sets the flag. Signal handlers
  are allowed to do almost nothing safely; setting a `std::atomic` (of a
  lock-free type) is one of the few legal actions. The comment explains why
  SIGTERM is included: CLion's Stop button sends SIGTERM, and both signals
  must leave through the same graceful path (stop report, CSV drain,
  servoing restore) or an IDE stop kills the process mid-takeover.

(The `InputThreadJoiner` RAII struct that used to live here left with the
stdin input thread — the log writer's thread manages its own lifetime
inside `LoopLogWriter`, so main() no longer owns any thread object.)

---

## main() — lines 214 to 443

### Lines 216–217 — install signal handlers
`std::signal(SIGINT, on_stop_signal)` routes Ctrl+C to the flag-setter;
same for SIGTERM. Done first so even a very early Ctrl+C is graceful.

### Line 219 — parse the command line
`ParseLogFileArg` (explained above). Empty string means "use the default
timestamped file". This is deliberately *outside* the try block: parse
errors exit(2) before anything is constructed.

### Line 221 — `try {`
Everything hardware-touching lives in one try. The matching catches
(lines 433–442) turn any exception into a printed error and exit code 1,
instead of a crash. Because all the hardware objects are locals inside the
try, stack unwinding runs their destructors (RAII teardown of sessions,
the writer thread, the CSV) on the way out.

### Lines 225–232 — build the kinematic model
`Dynamics dynamics(GEN3_DUAL_URDF_PATH)` loads the mounted dual-arm URDF
into Pinocchio (`GEN3_DUAL_URDF_PATH` is a macro baked in by CMake — the
path is compiled into the binary). `DualArmKinematics` wraps it as the
explicit 14-DoF-model / 7-joint-controller adapter: it validates the model
has exactly 14 velocity variables and that the named right-arm frames
(`kRightBaseFrame`, `kRightEndEffectorFrame`) exist. Constructed *before*
the connection so a wrong frame name or broken URDF fails with zero
hardware involvement.

### Line 236 — `Connect connection(config::kRightRobotIp);`
The program's sole hardware connection: TCP + UDP sessions to the right arm
(Hardware.h). RAII — its destructor closes the sessions on any exit path.
The left arm never gets a connection; it exists only inside the model.

### Lines 238–248 — clear faults
`connection.base()->ClearFaults()` unconditionally clears any *latched*
fault left over from a previous run. It commands no motion. The comment
explains the logic: a stale latch clears; a genuinely live fault simply
re-latches and the readiness gate below catches it on a fresh read.
**FLAG hides-work (lines 243–248):** `catch (...)` catches *every* possible
exception and discards it, printing only "Unable to clear robot faults".
The actual reason (network down? wrong IP? Kortex error code?) is thrown
away — the one early-exit path in the program where you learn nothing about
why. Debugging a failure here means adding the detail back.

### Line 251 — `sleep_for(500ms)`
**FLAG edit-hazard:** a magic delay. After a fault clear the base needs a
moment to re-arm the actuators; read too soon and the readiness gate sees a
frame from *before* the clear and refuses a healthy robot (or, worse,
passes on stale data). There is no completion signal to wait on, so this is
a tuned constant — shortening it to "speed up startup" reintroduces a race
with the firmware.

### Line 255 — first feedback frame
`read_feedback` performs one standalone Feedback exchange. This single
frame (`initial`) is reused for the readiness gate, state printout, and FK
startup-pose measurement — deliberately the same snapshot so startup facts
refer to one instant.

### Lines 257–259 — readiness gate (T1)
`RobotReadyForTakeover` inspects the frame for live faults and prints its
verdict. Fails → `return 1` with the arm never entering low-level
servoing. The `robot_ready` bool is also threaded into `RunControlLoop`
later (line 404) as proof the gate ran.

### Mandatory read-only hard-speed gate
Immediately after readiness, `VerifyKinematicHardLimits` queries the
Connect-owned `ControlConfigClient`. It requires seven finite positive live
hard joint speeds and refuses takeover if the compiled per-joint
`kQdotLimitDegS` clip exceeds any of them. This gate always runs: the
`kSkipStartupGates` override skips configuration writes, not hard-speed
verification. Bundled Kortex 2.7.0 exposes no live joint-position fields in
this schema, so the gate makes no position-limit claim.

### Restore and verify JOINT_LIMIT thresholds
The long comment is load-bearing history: firmware limit bands do not
survive a power cycle, and a degenerate 0/0 band makes the firmware fault
any motion away from zero. `EnsureJointLimits` re-writes the configured
bands — but only for joints with non-zero entries in
`kJointLimitWarnDeg/kJointLimitErrorDeg` (currently bounded joints 2, 4,
and 6; continuous joints 1/3/5/7 remain untouched). It runs only after the
readiness and mandatory hard-speed gates. The parenthetical records
why the old control-mode verification gate was deleted on 2026-08-04. If
`kSkipStartupGates` is set
the restore is skipped with a loud warning instead — the warning text
spells out exactly what protection is lost.

### Line 277 — `PrintRobotState(initial, controlled_model);`
The state printout described earlier. Its position/rpy lines are the
reference for choosing the next compiled target.

### Lines 279–284 — allocate the log
`LoopLog` is the in-memory handoff queue between the 500 Hz loop and the
CSV writer thread — not the run's record (the CSV is; it's written live).
**FLAG edit-hazard (lines 283–284):** the capacity arithmetic
`kLogBufferSeconds * (1'000'000 / kCyclePeriod.count())` uses *integer
division* to get cycles-per-second (500 at 2000 µs). It is exact at the
current period, but a period that doesn't divide 1 000 000 evenly (e.g.
3 ms = 333.33 Hz) silently truncates and under-sizes the buffer. All
allocation happens here, before the loop, on purpose: no allocations in
the real-time path.

### Lines 286–287 — the control-rate banner
One line naming the law and the rate, pointing at the CSV preamble for the
full settings.

### Lines 288–312 — open the CSV before takeover
The ordering rule in the comment is the point: *a hardware run must never
end with zero evidence* because file creation failed after motion started.
- **Lines 295–306** — default path: `<repo>/runs/YYYY-MM-DD/` via
  `dated_run_dir(RUNS_ROOT_DIR)` (another CMake-baked macro) +
  `timestamped_csv_name`. `create_directories` uses the `std::error_code`
  overload — errors come back in `dir_error` instead of an exception, so
  the failure message can name the directory and reason, then `return 1`.
  An explicit `--log` filename is used verbatim, no directory creation.
- **Lines 307–311** — open the file; an unopenable file refuses the run.
- **Line 312** — `WriteCsvPreamble(log_file_arg, csv)`. Passes the *raw
  argument*, not the resolved `log_file` — see the edit-hazard flag on
  WriteConfigLines lines 98–99.

### Line 319 — `LoopLogWriter log_writer(log, csv, kLogDrainInterval);`
Starts the background writer thread that drains the log queue to the CSV
every 100 ms for the rest of the run — the only thread this program starts.
**FLAG edit-hazard (line 319):** its position is load-bearing, as the
comment says: declared *after* `csv` and before the loop. C++ destroys
locals in reverse declaration order, so on every exit path — including an
exception mid-run — the writer is torn down (final drain, thread joined)
*before* the `ofstream` closes. Move this declaration above `csv` and the
teardown order inverts: the writer's last drain writes into a closed file.
Nothing in the compiler will warn about that.

### Line 323 — `TrackingController controller(controlled_model);`
The controller. Constructed here so a bad end-effector frame name fails
before any takeover.

### Lines 325–404 — build the current-start reference source
- **Line 325** — `std::unique_ptr<ReferenceSource> reference;`
  *unique_ptr* is an owning smart pointer: exactly one owner, deletes its
  object automatically. **FLAG unnecessary (lines 325, 376):** this
  indirection is a leftover of the removed two-source design. There is now
  exactly one concrete source (`PoseTargetSource`), known at compile time,
  so the pointer + virtual-interface hop buys nothing a plain local
  `PoseTargetSource` would not — unless a second source is expected back.
  Harmless, but it makes the code look more dynamic than it is.
- **Line 328** — `std::optional<PoseTarget> fixed_target;` *std::optional*
  is a box that either holds a value or is empty — the modern replacement
  for "pointer that might be null". **FLAG unnecessary (lines 328, 375):**
  under fixed-target-only the optional is assigned unconditionally at line
  375, so it is never empty at the point of use. It survives only as
  plumbing matching `PoseTargetSource`'s parameter type from the era when
  interactive runs passed an empty one. If that constructor signature is
  ever tightened to take a plain `PoseTarget`, this wrapper goes with it.
- **Lines 379–397 — measure the startup pose.** Converts the fresh initial
  actuator feedback to radians, computes FK in the right-arm model, rejects a
  non-finite result, records the measured startup pose in the CSV, and prints
  it for operator visibility. There is no comparison to a compiled joint
  configuration or Cartesian pose.
- **Build the PoseTargetSource (Stage 1.5).** `kFixedTargetM` is deleted;
  the terminal target's position is the measured FK position itself
  (`target.p_desired = ee_now.position`), so the constructor's profile
  start and terminal target are the same point — the arm's first profile
  is zero-distance. Rotation is left unset, meaning "keep the takeover
  orientation," unconditionally (`kFixedTargetUseRpy` no longer gates
  this — see `../decisions/stage15-bridge-workflow.md`).

### Line 377 — `PositionIntegration actuation(kCommandLeadLimitDeg);`
The actuation stage: integrates commanded velocities into position
setpoints, never letting the setpoint lead measured position by more than
1°. See Config.md's flag on why that 1° interacts with the following-error
guard.

### Lines 401–407 — the terminal-target banner
The banner states that takeover holds the measured startup pose before the
bounded profile moves to the terminal target. It pairs the measured startup
position with the configured target so the operator can review both before
motion.

### Lines 398–404 — run the loop. THE ARM MOVES HERE.
`RunControlLoop` (Runner.cpp) is the entire run: the takeover sequence,
then the 500 Hz cycle of feedback → reference source → controller →
actuation → command, until `g_stop` or a stop condition. Everything
constructed above is passed in by reference/pointer; main() owns, the
runner uses. The comment's promise — servoing mode restored on *every*
exit path (T2/D1) — is the runner's contract, which is why main() never
touches servoing modes itself. Returns a `LoopResult` (stop reason, time,
cycle count, whether faults were observed).

### Lines 406–416 — final drain and exit trailer
`log_writer.Stop()` performs the final drain — every earlier row was
already on disk, written live during the run. Then the trailer: four `#`
lines recording how (`StopReasonName`), when (`stop_t_s`), at which cycle
the run ended, and whether faults were observed. Same `#` convention as the
preamble, so parsers skip it; a CSV alone now tells the whole story of its
run without the console output. `csv.flush()` pushes it to the OS
immediately rather than waiting for the destructor.

### Lines 417–424 — sample accounting
Prints rows written; if `log.dropped() > 0`, a warning that the writer
could not keep up and there is a visible gap in `time_s`. Then the full log
path on its own line — the comment says why: ready to paste into an
analysis request.

### Lines 426–432 — the exit code
The success condition: reason was a clean operator stop
(`LoopStop::kUserStop`) AND no faults were observed → 0; anything else →
1. Note the deliberate strictness on lines 428–429: with
With the validation configuration, `kStopOnFault = true`, so a live fault
ends the loop and also taints the exit code. If that compile-time policy is
ever disabled for a separately approved experiment, observed faults still
taint the exit code — "the run finished" and "the run was clean" remain
different claims.

### Lines 433–442 — the catch blocks
Order matters: `KDetailedException` first (it derives from
`std::exception`, so the general catch would swallow it), printing Kinova's
sub-error code by name via `SubErrorCodes_Name` — the difference between
"error" and "error: INVALID_DEVICE". Then the generic `std::exception`
catch for everything else (bad URDF, bad frame names, hardware setup
failures). Both return 1. Reaching either means stack unwinding already
ran every destructor: writer drained and joined, sessions closed — the
RAII payoff.

### Line 443 — end of main. No code after the try/catch; every path has
already returned an explicit exit code.

---

## Flag summary for this file

| Lines | Flag | Reason |
|---|---|---|
| 43–46 | unnecessary | comment claims a startup config echo that was removed |
| 98–99 | edit-hazard | preamble records the raw --log arg, not the resolved path |
| 105–108 | edit-hazard | double increment of `i` while parsing --log |
| 115 | edit-hazard | hand-maintained log_format version number (now 6) |
| 185–186 | hides-work | PrintRobotState runs full FK, not just printing |
| 190–198 | edit-hazard | reversed eulerAngles indexing; printed rpy feeds the compiled target |
| 243–248 | hides-work | catch(...) discards the real ClearFaults error |
| 251 | edit-hazard | magic 500 ms re-arm delay, no completion signal to wait on |
| 283–284 | edit-hazard | integer division sizing the log buffer |
| 312 | edit-hazard | passes log_file_arg (see 98–99) |
| 319 | edit-hazard | declaration order = teardown order for writer vs csv |
| 325, 376 | unnecessary | unique_ptr/interface indirection with only one concrete source left |
| 328, 375 | unnecessary | optional<PoseTarget> is always engaged; vestige of the two-source design |
