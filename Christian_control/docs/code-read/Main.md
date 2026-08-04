# Main.cpp — line-by-line, in execution order

File: `basic_control/src/Main.cpp`. The program's entry point. Everything below
follows the order the CPU actually visits things when you run `./controller`,
not the order they appear in the file. Helper functions are explained at the
point main() first calls them.

Flag categories used inline: **trajectory-only**, **unnecessary**,
**hides-work**, **mixed-jobs**, **edit-hazard**.

---

## Before main() runs: file-scope setup

### Lines 1–5 — file banner comment
Says what the file is: parse `--log`, load the model, connect, check
readiness, wire reference source to controller, run the loop, report. The
control loop itself is in Runner.h/.cpp, not here. Pure documentation; no
runtime effect.

### Lines 7–23 — standard-library includes
Each `#include` pastes in declarations the file needs: `<atomic>` for the
stop flag, `<csignal>` for Ctrl+C handling, `<filesystem>` for creating the
runs directory, `<fstream>` for the CSV file, `<thread>` for the stdin input
thread, `<optional>` for "a value that may be absent", and so on. Removing
one breaks compilation of whatever uses it — nothing subtle here.

### Line 25 — `#include <KDetailedException.h>`
Kinova's rich exception type. Needed so the `catch` at line 627 can name the
exact Kortex sub-error code instead of a generic message.

### Lines 27–36 — project includes
The nine other modules of this program. Main.cpp is the only file that sees
all of them, because main() is where they get wired together.

### Line 38 — `namespace k_api = Kinova::Api;`
A namespace alias: from here on, `k_api::` is shorthand for `Kinova::Api::`.
Purely a typing convenience.

### Lines 40–47 — comment block about the config echo
Explains that `--log` is the only runtime argument and that the same
`key = value` lines serve as "the startup echo and the CSV preamble".
**FLAG unnecessary (lines 44–47):** the "startup echo" half of this claim is
stale — `WriteConfigLines` is only ever called from `WriteCsvPreamble`
(line 124), and the comment at lines 247–249 inside main() says the startup
print was deliberately dropped ("without printing thirty lines at every
start"). A reader trusting this comment would look for a console echo that
does not exist.

### Line 49 — `namespace {`
An *anonymous namespace*: everything inside is private to this .cpp file
(other files cannot link against it). The C++ way of saying "these helpers
are internal".

### Lines 50–56 — `UsageAndExit`
Prints an optional error plus the usage text to stderr, then `std::exit(2)`.
`[[noreturn]]` is a promise to the compiler that this function never returns
(it always exits), which silences "missing return" warnings at call sites.
Exit code 2 is the conventional "bad command line" code, distinct from 1
(run failed) and 0 (clean stop). Note `std::exit` skips destructors of
locals — fine here because nothing is open yet at parse time, but this
function must never be called after the hardware connection exists.

### Lines 58–62 — `FormatDouble`
Formats a double via an `ostringstream` with default formatting, which
prints the shortest faithful representation (e.g. `0.5`, not `0.500000` like
`std::to_string` would). Exists so config values in the CSV preamble look
like what was typed in Config.h.

### Lines 64–109 — `WriteConfigLines`
Writes every Config.h value as `prefix + "key = value (Config.h)"` lines to
a stream. Called once per run, from `WriteCsvPreamble`, with prefix `"# "`.

- **Line 65** — `const auto line = [&](...)`: a *lambda*, a small unnamed
  function stored in a variable. `[&]` means it can use surrounding
  variables (`out`, `prefix`) by reference. Exists to avoid repeating the
  `out << prefix << key ...` boilerplate ~25 times.
- **Lines 68–71** — reference source, trajectory file, playback gains.
  **FLAG trajectory-only (lines 69–71):** `trajectory_file`, `playback_kp`
  and `start_mismatch_limit_deg` only matter for trajectory playback; on an
  operator run they are recorded but unused.
- **Lines 72–93** — the reactive-law gains, feature toggles, and (only when
  `kUseFixedTarget` is on) the fixed target and its optional rpy. The `if`
  nesting mirrors the config's own dependency: rpy is only meaningful when a
  fixed target exists.
- **Lines 94–105** — the guard overrides and stop counters, recorded "so a
  log can be read back knowing which protections were active". This is a
  safety-audit feature: never remove lines here without understanding you
  are erasing evidence from future run records.
- **Lines 106–107** — the log_file line. **FLAG edit-hazard (lines
  106–107):** the value written is whatever string was *passed in* — and
  main() passes `log_file_arg` (line 460), the raw `--log` argument, not the
  resolved path. On a default run the preamble therefore says
  `log_file = <timestamped> (default)` and never records the actual
  filename. Harmless today (the file knows its own name on disk), but
  anyone "fixing" the preamble to show the real path must change the caller,
  not this function — and anyone parsing preambles must not expect a real
  path here.
- **Line 108** — control period, last so it sits next to the data that
  follows.

### Lines 111–119 — `ParseLogFileArg`
Walks `argv` (the command-line words). The only accepted option is `--log
<file>`; anything else calls `UsageAndExit`. Returns the filename, or `""`
if `--log` was not given.
**FLAG edit-hazard (lines 113–116):** the loop increments `i` in *two*
places — the `for` header and `++i` inside the body (line 115) to consume
the filename. The `if (++i >= argc)` both advances and bounds-checks in one
expression. Correct, but compact enough that inserting a new option or
reordering the checks could easily walk off the end of `argv` or eat the
wrong word.

### Lines 121–125 — `WriteCsvPreamble`
Writes two header comment lines then delegates to `WriteConfigLines` with
the `"# "` prefix. CSV parsers skip `#` lines, so the preamble rides inside
the data file without breaking it.
**FLAG edit-hazard (line 123):** `# log_format = 5 (compiled)` is a
hand-maintained schema version. The actual row layout is written elsewhere
(the Hardware.h log writer). If someone changes the CSV columns without
bumping this number — or bumps it without changing them — every analysis
script keyed on the format number silently misreads data.

### Line 126 — `}` closes the first anonymous namespace.

### Lines 128–149 — the "story of the program" comment
A prose walkthrough of main() from parse to exit. Worth reading once; it is
the file's own table of contents. The all-caps "MOVES THE ARM" marks where
hardware motion begins.

### Lines 154–212 — second anonymous namespace: the print helpers

- **Lines 157–158** — column widths for the joint tables, named constants so
  header and rows stay aligned if either changes.
- **Lines 160–166 — `PrintJointHeader`**: prints `joint   1   2 ... 7`.
  `std::tuple_size_v<JointVector>` reads the array's size (7) at compile
  time, so the header automatically matches the joint count.
  `std::left/std::right/std::setw` are stream formatting: pad the label
  left-aligned to 16 chars, then each number right-aligned to 10.
- **Lines 168–175 — `PrintRow`**: one labeled row of 7 numbers, fixed 2
  decimals. `std::defaultfloat` at the end *restores* the stream's normal
  float formatting — forget that and every later number in the program
  prints with this row's formatting (stream formatting flags are sticky
  global state).
- **Lines 180–210 — `PrintRobotState`**: prints the measured joint state
  and the FK end-effector pose.
  - Lines 185–189: copies each actuator's position (deg) and velocity
    (deg/s) out of the Kortex feedback message, and builds `q_rad`, the
    same positions converted to radians for the kinematics.
  - Lines 191–193: the joint table.
  - Lines 195–196: **FLAG hides-work:** despite the "Print" name, this
    constructs a `KinematicsWorkspace` (allocates Pinocchio scratch
    buffers) and runs full forward kinematics + Jacobian
    (`RightPoseAndJacobian`) just to print one pose. Cheap enough at
    startup, but this is real computation inside a print function — calling
    it from a fast path, or assuming it only formats text, would be wrong.
  - Line 200: `ee.rotation.eulerAngles(2, 1, 0)` decomposes the rotation
    matrix as R = Rz·Ry·Rx. **FLAG edit-hazard (lines 200–208):** Eigen
    returns the angles *in the order requested* — `zyx[0]` is the Z angle
    (yaw), `zyx[2]` is the X angle (roll) — so the print deliberately emits
    `zyx.z(), zyx.y(), zyx.x()` to get roll-pitch-yaw order. Swapping those
    accessors "to tidy up" would print yaw where roll belongs, and the
    printed triple is explicitly meant to be pasted into
    `kFixedTargetRpyRad` or typed as a target — a wrong paste moves the
    real arm the wrong way.
  - Why this function exists at all: the comment at 177–179 says it — the
    printed pose is what "hold here" looks like, so the operator can copy it
    and edit one coordinate.

### Lines 214–222 — the stop flag and signal handler
- **Line 218** — `std::atomic<bool> g_stop{false};` A global flag. *atomic*
  means reads and writes are indivisible and safely visible across threads
  — the signal handler, the input thread, and the 500 Hz control loop all
  touch this one bool without a mutex. A plain `bool` here would be
  undefined behaviour (a data race, and also not legal to write from a
  signal handler).
- **Lines 219–222** — `on_stop_signal` just sets the flag. Signal handlers
  are allowed to do almost nothing safely; setting a `std::atomic` (of a
  lock-free type) is one of the few legal actions. The comment explains why
  SIGTERM is included: CLion's Stop button sends SIGTERM, and both signals
  must leave through the same graceful path (stop report, CSV drain,
  servoing restore) or an IDE stop kills the process mid-takeover.

### Lines 224–237 — `InputThreadJoiner`
A tiny RAII helper. *RAII* = tie cleanup to a destructor, so it runs on
every scope exit, including exceptions. It holds a reference to the input
thread; its destructor sets `g_stop` and joins the thread.
Why: if an exception flies between thread start and the normal join,
`std::thread`'s own destructor on a still-joinable thread calls
`std::terminate` — instant abort, skipping *all* cleanup (no stop report, no
CSV drain). This struct guarantees the join happens first.
**FLAG hides-work (lines 231–236):** the name says "Joiner" but the
destructor also *sets the global stop flag* (line 233). That side effect is
essential — the input thread blocks on stdin and only exits when it sees
`g_stop` — but it means destroying this object silently commands the whole
program to stop. Reusing it as "just a joiner" elsewhere would be a
surprise.

---

## main() — lines 239 to 637

### Lines 241–242 — install signal handlers
`std::signal(SIGINT, on_stop_signal)` routes Ctrl+C to the flag-setter;
same for SIGTERM. Done first so even a very early Ctrl+C is graceful.

### Line 244 — parse the command line
`ParseLogFileArg` (explained above). Empty string means "use the default
timestamped file". This is deliberately *outside* the try block: parse
errors exit(2) before anything is constructed.

### Line 246 — `try {`
Everything hardware-touching lives in one try. The matching catches
(lines 627–636) turn any exception into a printed error and exit code 1,
instead of a crash. Because all the hardware objects are locals inside the
try, stack unwinding runs their destructors (RAII teardown of sessions,
threads, the CSV) on the way out.

### Lines 250–255 — validate the compiled reference source
Reads `config::kReferenceSource` and derives two bools: `playback`
(trajectory run) and `operator_targets`. Anything else, or a trajectory run
with no file configured, throws — caught at line 633, exit 1. This is the
compile-time config's only runtime sanity check; without it a typo in
Config.h would silently fall through to neither branch.

### Lines 257–294 — load and validate the trajectory (playback only)
**FLAG trajectory-only (lines 257–294):** this entire block exists solely
for trajectory playback; on an operator run `playback` is false and it is
skipped.
- **Line 260** — `std::optional<Trajectory> trajectory;` *std::optional* is
  a box that either holds a value or is empty — the modern replacement for
  "pointer that might be null". Used because a Trajectory only exists on
  playback runs; `*trajectory` accesses the value (undefined if empty, so
  every use is guarded by `if (playback)`).
- **Lines 262–263** — `LoadTrajectoryCsv` parses the file and *throws* on
  any structural violation, so a malformed file dies here, before any
  hardware session — that ordering is the point of the whole block.
- **Lines 265–268** — builds the per-joint velocity gate: 0.9 × the command
  clip, so a validated file can never make the Runner's own clamp engage.
- **Lines 269–271** — `ValidateTrajectory` checks velocity, acceleration
  (Kinova Table 43) and position ranges, filling `trajectory_summary` and
  returning a list of human-readable violations.
- **Lines 272–281** — prints the summary table (samples, dt, duration,
  per-joint displacement and peaks) and the file's metadata lines. The
  `for (const auto& [key, value] : ...)` is a *structured binding*: it
  unpacks each map entry into two named variables.
- **Lines 282–290** — if any violation, print them all and `return 1` —
  refusing to run. Printing *in full* is deliberate: the operator must see
  every reason, not just the first.
- **Lines 291–293** — the PASS line names both gates so the console record
  says what was checked.

### Lines 296–303 — build the kinematic model
`Dynamics dynamics(GEN3_DUAL_URDF_PATH)` loads the mounted dual-arm URDF
into Pinocchio (`GEN3_DUAL_URDF_PATH` is a macro baked in by CMake — the
path is compiled into the binary). `DualArmKinematics` wraps it as the
explicit 14-DoF-model / 7-joint-controller adapter: it validates the model
has exactly 14 velocity variables and that the named right-arm frames
(`kRightBaseFrame`, `kRightEndEffectorFrame`) exist. Constructed *before*
the connection so a wrong frame name or broken URDF fails with zero
hardware involvement.

### Line 307 — `Connect connection(config::kRightRobotIp);`
The program's sole hardware connection: TCP + UDP sessions to the right arm
(Hardware.h). RAII — its destructor closes the sessions on any exit path.
The left arm never gets a connection; it exists only inside the model.

### Lines 309–319 — clear faults
`connection.base()->ClearFaults()` unconditionally clears any *latched*
fault left over from a previous run. It commands no motion. The comment
explains the logic: a stale latch clears; a genuinely live fault simply
re-latches and the readiness gate below catches it on a fresh read.
**FLAG hides-work (lines 314–319):** `catch (...)` catches *every* possible
exception and discards it, printing only "Unable to clear robot faults".
The actual reason (network down? wrong IP? Kortex error code?) is thrown
away — the one early-exit path in the program where you learn nothing about
why. Debugging a failure here means adding the detail back.

### Line 322 — `sleep_for(500ms)`
**FLAG edit-hazard:** a magic delay. After a fault clear the base needs a
moment to re-arm the actuators; read too soon and the readiness gate sees a
frame from *before* the clear and refuses a healthy robot (or, worse,
passes on stale data). There is no completion signal to wait on, so this is
a tuned constant — shortening it to "speed up startup" reintroduces a race
with the firmware.

### Line 326 — first feedback frame
`read_feedback` performs one standalone Feedback exchange. This single
frame (`initial`) is reused three times below: the readiness gate, the
state printout, and (playback) the start-state gate — deliberately the
*same* snapshot so all three judge the same instant.

### Lines 328–330 — readiness gate (T1)
`RobotReadyForTakeover` inspects the frame for live faults and prints its
verdict. Fails → `return 1` with the arm never entering low-level
servoing. The `robot_ready` bool is also threaded into `RunControlLoop`
later (line 579) as proof the gate ran.

### Lines 331–347 — restore JOINT_LIMIT thresholds
The long comment is load-bearing history: firmware limit bands do not
survive a power cycle, and a degenerate 0/0 band makes the firmware fault
any motion away from zero. `EnsureJointLimits` re-writes the configured
bands — but only for joints with non-zero entries in
`kJointLimitWarnDeg/kJointLimitErrorDeg` (currently joint 4 only; joint 6
is deliberately zeroed because its config service is wedged and every RPC
to it times out). The parenthetical records why the old control-mode
verification gate was deleted on 2026-08-04. If `kSkipStartupGates` is set
the restore is skipped with a loud warning instead — the warning text spells
out exactly what protection is lost.

### Line 348 — `PrintRobotState(initial, controlled_model);`
The state printout described earlier. Its position/rpy lines are the
operator's reference for typing targets.

### Lines 350–407 — playback start gate + FK cross-check
**FLAG trajectory-only (lines 350–407):** playback only.
- **Lines 353–376 — start-state gate.** For each joint, the difference
  between measured position and the trajectory's first row, wrapped into
  (−180, 180] by `std::remainder(x, 360.0)` (which returns x minus the
  nearest multiple of 360 — this is how "359.3° vs −0.7°" correctly counts
  as a 0° gap between the arm's [0,360) feedback and the file's continuous
  angles). Any joint over `kStartMismatchLimitDeg` (0.2°) → print the
  per-joint table, refuse, `return 1`. Why so strict: the playback law
  drives reference-minus-measured error, so starting displaced means the
  arm *jumps*. The comment notes `TrajectorySource::Reset` re-checks at the
  actual takeover — this early copy exists to refuse *before* any hardware
  takeover, with a better error message.
- **Lines 378–406 — FK cross-check.** Runs the controller's own URDF
  kinematics on the trajectory's first and last rows and prints start EE,
  final EE and the displacement vector/norm. Why: the trajectory came from
  the *planner's* DH model; this is an independent second opinion from the
  controller's URDF model, shown to the operator before anything moves.
  Purely informational — nothing is gated on it. Note lines 384–391 reuse
  one `q_row_rad` buffer for both rows (overwritten in place).

### Lines 409–415 — allocate the log
`LoopLog` is the in-memory handoff queue between the 500 Hz loop and the
CSV writer thread — not the run's record (the CSV is; it's written live).
**FLAG edit-hazard (lines 413–414):** the capacity arithmetic
`kLogBufferSeconds * (1'000'000 / kCyclePeriod.count())` uses *integer
division* to get cycles-per-second (500 at 2000 µs). It is exact at the
current period, but a period that doesn't divide 1 000 000 evenly (e.g.
3 ms = 333.33 Hz) silently truncates and under-sizes the buffer. All
allocation happens here, before the loop, on purpose: no allocations in
the real-time path.

### Line 415 — `PoseTargetStore pose_targets;`
The thread-safe mailbox the stdin thread writes targets into and the loop
reads from (operator source only). Constructed unconditionally even on
playback runs where nothing uses it — harmless (it is just a small struct),
but note it is not playback state.

### Lines 417–435 — mode banner
Playback: an explicit warning that the arm moves the moment takeover
completes, with duration and the Ctrl+C behaviour (**FLAG trajectory-only,
lines 417–423**). Operator: the typed-target grammar (`x y z` or
`x y z roll pitch yaw`) — skipped when `kUseFixedTarget`, whose own louder
banner comes later — plus the control-rate line.

### Lines 437–460 — open the CSV before takeover
The ordering rule in the comment is the point: *a hardware run must never
end with zero evidence* because file creation failed after motion started.
- **Lines 443–454** — default path: `<repo>/runs/YYYY-MM-DD/` via
  `dated_run_dir(RUNS_ROOT_DIR)` (another CMake-baked macro) +
  `timestamped_csv_name`. `create_directories` uses the `std::error_code`
  overload — errors come back in `dir_error` instead of an exception, so
  the failure message can name the directory and reason, then `return 1`.
  An explicit `--log` filename is used verbatim, no directory creation.
- **Lines 455–459** — open the file; an unopenable file refuses the run.
- **Line 460** — `WriteCsvPreamble(log_file_arg, csv)`. Passes the *raw
  argument*, not the resolved `log_file` — see the edit-hazard flag on
  WriteConfigLines lines 106–107.

### Line 467 — `LoopLogWriter log_writer(log, csv, kLogDrainInterval);`
Starts the background writer thread that drains the log queue to the CSV
every 100 ms for the rest of the run.
**FLAG edit-hazard (line 467):** its position is load-bearing, as the
comment says: declared *after* `csv` and before the loop. C++ destroys
locals in reverse declaration order, so on every exit path — including an
exception mid-run — the writer is torn down (final drain, thread joined)
*before* the `ofstream` closes. Move this declaration above `csv` and the
teardown order inverts: the writer's last drain writes into a closed file.
Nothing in the compiler will warn about that.

### Line 473 — `TrackingController controller(controlled_model);`
THE controller — one implementation regardless of run mode; only the
reference source ahead of it differs. Constructed before the input thread
so a bad end-effector frame name fails before any takeover.

### Lines 475–542 — build the reference source
`std::unique_ptr<ReferenceSource> reference;` — *unique_ptr* is an owning
smart pointer: exactly one owner, deletes its object automatically. Needed
because the concrete type (TrajectorySource vs PoseTargetSource) is chosen
at runtime but the loop only sees the common `ReferenceSource` interface
(*polymorphism*: `RunControlLoop` calls virtual methods without knowing
which source it has).
**FLAG mixed-jobs (lines 485–542):** the operator branch packs three jobs
into one `else`: a safety gate (freshness check), target construction, and
source wiring. It works, but a future edit to any one job wades through the
other two; the freshness gate in particular is a *safety check* living
inside object-construction plumbing.

- **Lines 477–484 — playback branch.** **FLAG trajectory-only.** Builds
  `PlaybackSettings` from config, then a `TrajectorySource` owning the
  trajectory. `std::move(*trajectory)` transfers the loaded trajectory's
  guts into the source without copying — after this the optional still
  "has a value" but it is an empty husk; touching `*trajectory` later would
  be a bug. Line 483 keeps a raw `trajectory_source` pointer *before*
  ownership moves into `reference` — needed at lines 606–615 to ask the
  source how playback ended. A raw non-owning pointer next to a unique_ptr
  is a standard pattern, but only valid while `reference` is alive.
- **Lines 490–527 — fixed-target freshness gate.** Only when
  `kUseFixedTarget`. Computes FK on the *current* measured joints (from
  `initial`), takes the straight-line gap to the compiled target, and
  refuses the run if it exceeds `kMaxFixedTargetDistanceM` (0.15 m). The
  comment records the incident that created it: on 2026-08-04 the arm was
  jogged 37 cm between compile and run — the controller would have driven
  the whole gap at clip speed from cycle one. The extra `{ ... }` braces
  around 498–527 are a deliberate scope so the gate's temporaries
  (`q_now_rad`, workspace, `ee_now`) die immediately after the check.
- **Lines 528–538 — build the PoseTarget.** Position from `kFixedTargetM`;
  rotation only if `kFixedTargetUseRpy` — an unset rotation means "keep the
  takeover orientation" (that convention lives in Targets.h; the comment on
  line 532 restates it here because getting it wrong rotates a real tool).
- **Lines 540–541** — `PoseTargetSource` reads the shared store each cycle;
  `fixed_target` (a `std::optional<PoseTarget>`, empty on interactive runs)
  pre-loads the compiled target so it applies from the first cycle after
  takeover, deliberately bypassing the store — whose pre-takeover content
  is discarded on purpose (stale typed targets from before the run must not
  fire at takeover).

### Line 543 — `PositionIntegration actuation(kCommandLeadLimitDeg);`
The actuation stage: integrates commanded velocities into position
setpoints, never letting the setpoint lead measured position by more than
1°. See Config.md's flag on why that 1° interacts with the following-error
guard.

### Lines 545–571 — the input thread (or not)
Three cases:
- Fixed target (lines 550–566): **no** stdin thread; instead a shouty
  banner — target coordinates, "THE ARM MOVES THERE IMMEDIATELY", and the
  rotation (or "orientation target unchanged"). The banner *is* the safety
  feature: the operator's last chance to notice a wrong compiled target.
- Interactive operator (lines 567–569): starts `RunPoseTargetInput` on a
  new thread, reading stdin lines into the store. `std::ref`/`std::cref`
  are required because `std::thread` copies its arguments by default —
  these wrappers say "pass the actual object by reference" (the store must
  be shared, not copied; `cref` = const reference to the stop flag).
- Playback: neither — Ctrl+C via the signal handler is the only input.

**Line 571** — `InputThreadJoiner input_thread_joiner{input_thread};` arms
the RAII joiner described earlier. Note it wraps the thread object even in
the two cases where no thread was started — joining a default-constructed
thread is skipped by the `joinable()` check, so this is safe by design.

### Lines 573–579 — run the loop. THE ARM MOVES HERE.
`RunControlLoop` (Runner.cpp) is the entire run: takeover sequence T1–T5,
then the 500 Hz cycle of feedback → reference source → controller →
actuation → command, until `g_stop` or a stop condition. Everything
constructed above is passed in by reference/pointer; main() owns, the
runner uses. The comment's promise — servoing mode restored on *every* exit
path — is the runner's contract, which is why main() never touches
servoing modes itself. Returns a `LoopResult` (stop reason, time, cycle
count, whether faults were observed).

### Lines 581–583 — stop the input thread
`g_stop = true` first — the loop may have exited on a fault, not Ctrl+C, and
without this the stdin thread would block forever waiting for input. Then an
explicit join. This *duplicates* what the joiner's destructor would do, but
deliberately: the report lines below (trailer, sample counts) should print
after the input thread is fully quiet, not interleaved with it, and the
joiner still covers the exception path.

### Lines 585–595 — final drain and exit trailer
`log_writer.Stop()` performs the final drain — every earlier row was
already on disk, written live during the run. Then the trailer: four `#`
lines recording how (`StopReasonName`), when (`stop_t_s`), at which cycle
the run ended, and whether faults were observed. Same `#` convention as the
preamble, so parsers skip it; a CSV alone now tells the whole story of its
run without the console output. `csv.flush()` pushes it to the OS
immediately rather than waiting for the destructor.

### Lines 596–603 — sample accounting
Prints rows written; if `log.dropped() > 0`, a warning that the writer
could not keep up and there is a visible gap in `time_s`. Then the full log
path on its own line — the comment says why: ready to paste into an
analysis request.

### Lines 605–615 — playback outcome
**FLAG trajectory-only (lines 605–615):** reads the raw
`trajectory_source` pointer (null on operator runs, so the whole block is
skipped): refused at takeover (no motion commanded), completed, or stopped
early. One plain line for the run record.

### Lines 617–626 — the exit code
The success condition, spelled out: reason was a clean operator stop
(`LoopStop::kUserStop`) AND no faults were observed AND playback was not
refused → 0; anything else → 1. Note the deliberate strictness on line
619–620: with `kStopOnFault = false` the loop *keeps running* through
faults, but they still taint the exit code — "the run finished" and "the
run was clean" are different claims, and scripts keying on the exit code
get the honest one. **FLAG trajectory-only (lines 621–622):** the
`playback_refused` term.

### Lines 627–636 — the catch blocks
Order matters: `KDetailedException` first (it derives from
`std::exception`, so the general catch would swallow it), printing Kinova's
sub-error code by name via `SubErrorCodes_Name` — the difference between
"error" and "error: INVALID_DEVICE". Then the generic `std::exception`
catch for everything else (config errors, trajectory contract violations,
bad frame names). Both return 1. Reaching either means stack unwinding
already ran every destructor: writer drained, threads joined, sessions
closed — the RAII payoff.

### Line 637 — end of main. No code after the try/catch; every path has
already returned an explicit exit code.

---

## Flag summary for this file

| Lines | Flag | Reason |
|---|---|---|
| 44–47 | unnecessary | comment claims a startup echo that was removed |
| 69–71 | trajectory-only | playback config lines in the preamble |
| 106–107 | edit-hazard | preamble records the raw --log arg, not the resolved path |
| 113–116 | edit-hazard | double increment of `i` while parsing --log |
| 123 | edit-hazard | hand-maintained log_format version number |
| 195–196 | hides-work | PrintRobotState runs full FK, not just printing |
| 200–208 | edit-hazard | reversed eulerAngles indexing; printed rpy feeds real targets |
| 231–236 | hides-work | InputThreadJoiner destructor also sets the global stop flag |
| 257–294 | trajectory-only | trajectory load + validation block |
| 314–319 | hides-work | catch(...) discards the real ClearFaults error |
| 322 | edit-hazard | magic 500 ms re-arm delay, no completion signal to wait on |
| 350–407 | trajectory-only | start-state gate + FK cross-check |
| 413–414 | edit-hazard | integer division sizing the log buffer |
| 417–423 | trajectory-only | playback banner |
| 460 | edit-hazard | passes log_file_arg (see 106–107) |
| 467 | edit-hazard | declaration order = teardown order for writer vs csv |
| 477–484 | trajectory-only | TrajectorySource construction |
| 485–542 | mixed-jobs | freshness gate + target build + source wiring in one branch |
| 605–615 | trajectory-only | playback outcome report |
| 621–622 | trajectory-only | playback_refused exit-code term |
