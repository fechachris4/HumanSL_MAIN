# Hardware.cpp / Hardware.h — line-by-line read

*(Updated for commit f64325c0, "remove trajectory playback": log_format is
now 6 with 121 columns; the nine playback columns and fields are gone.)*

Hardware is everything that touches the robot's network link plus everything
that records what happened: four unrelated-looking jobs in one file pair —
(1) the two Kortex sessions (`Connect`), (2) the per-cycle command exchange
(`CyclicSession`), (3) the one standalone feedback read (`read_feedback`),
and (4) the run log (`LoopLogSample`, `LoopLog`, `LoopLogWriter`, CSV
helpers, filename helpers).

**FLAG `Hardware.h:177-393` + `Hardware.cpp:253-481` | mixed-jobs** — the
Recording section (log sample, queue, CSV writer, filename helpers) shares a
file with robot I/O only because "records what the hardware did" was judged
close enough to "hardware". It never touches the robot and could be its own
file pair without changing a line of behaviour.

Execution order (what Main/Runner actually call, in the order they call it):

1. `Connect connection(ip)` — Main.cpp:236
2. `read_feedback(...)` — Main.cpp:255
3. `connection.EnsureJointLimits(std::cout)` — Main.cpp:274
4. `LoopLog log(capacity)` — Main.cpp:283-284
5. `dated_run_dir(...)` / `timestamped_csv_name(...)` — Main.cpp:297/305
6. `LoopLogWriter log_writer(log, csv, interval)` — Main.cpp:319 (starts a thread)
7. Inside the Runner: `CyclicSession cyclic(...)` (Runner.cpp:117),
   `cyclic.Seed()` (Runner.cpp:126), `cyclic.Send(...)` every cycle
   (Runner.cpp:155/165/263), `log.push(sample)` every cycle
8. After the loop: `log_writer.Stop()` (Main.cpp:407), `log.dropped()`,
   destructors.

---

## 1. Connect — opening the two sessions

### The port constants (Hardware.cpp:29-37)

```cpp
constexpr unsigned kTcpPort = 10000;
constexpr unsigned kUdpPort = 10001;
```

- **What**: the two ports the Gen3 base always listens on. `constexpr`
  means "a value fully known at compile time" — like `#define` but typed
  and scoped. They sit in an *anonymous namespace* (`namespace { ... }`),
  which is C++ for "visible only inside this .cpp file".
- **Why here and not Config.h**: the comment says it — the robot's firmware
  fixes these; they are a fact, not a choice.
- **If changed**: the transports would connect to nothing; `Channel`'s
  constructor would throw "could not reach the arm".

### `Connect::Channel::Channel` (Hardware.cpp:39-62) — one channel = socket + router + login

Called twice, from `Connect`'s constructor: once with a TCP transport, once
with a UDP transport. This is the real entry into the Kortex stack.

- **Line 39-41** — the constructor takes ownership of an *unconnected*
  transport via `std::unique_ptr`. First-time C++ note: `std::unique_ptr<T>`
  is a pointer that owns its object — when the unique_ptr is destroyed, the
  object is deleted automatically, and ownership can only be *moved*
  (`std::move`), never copied. The member initializer
  `transport(std::move(transport_in))` transfers that ownership into the
  member before the body runs.
- **Line 46-48** — `transport->connect(ip, port)` actually opens the socket.
  **edit-hazard note built into the comment**: Kortex's `connect` reports
  failure by returning `false`, *not* by throwing. The `if (!...) throw`
  converts that into an exception with a readable message naming the channel
  ("TCP"/"UDP"). Deleting this check would not make failure go away — it
  would just move it to a confusing error inside `CreateSession` later.
- **Line 50-52** — builds the `RouterClient`, the Kortex message
  dispatcher that sits on top of the socket. The lambda
  `[](k_api::KError err) { ... }` is the router's error callback: any
  asynchronous API error gets printed. A *lambda* is an unnamed function
  written inline; `[]` means it captures no outside variables.
- **Line 54-58** — fills a `CreateSessionInfo` protobuf with the
  username/password and the two inactivity timeouts from Config.h. The
  connection timeout (2000 ms) is what the base waits before dropping a
  silent connection — relevant because the control loop must keep talking.
- **Line 60-61** — `SessionManager::CreateSession` logs in. From this point
  the robot considers this process a client.
- **If this constructor throws** partway: members already constructed
  (transport, router) are destroyed in reverse order automatically — that is
  the RAII guarantee the header comment leans on. *RAII* ("resource
  acquisition is initialization") = tie a resource's lifetime to an object's
  lifetime so cleanup is automatic on every exit path, including exceptions.

### `Connect::Channel::~Channel` (Hardware.cpp:64-75) — logout in reverse

- **Line 68-72** — `CloseSession()` talks to the robot over the network, so
  it can throw if the link died. A destructor that throws while the stack is
  already unwinding calls `std::terminate` (instant abort, no cleanup), so
  the `try/catch(...)` swallows it and prints a warning instead.
  `catch (...)` means "catch anything, of any type".
- **Line 73-74** — deactivate the router, then close the socket. The header
  (Hardware.h:97-99) explains the member declaration order
  socket → router → session exists *so that* destruction (always reverse
  order in C++) logs out first and closes the socket last. **FLAG
  `Hardware.h:100-111` | edit-hazard** — reordering the three `Channel`
  members looks cosmetic but silently changes destruction order; a session
  could then be "closed" over an already-destroyed router.

### `Connect::Connect` (Hardware.cpp:77-86) — the whole connection in an initializer list

- **Lines 78-83** — everything happens in the member initializer list, in
  declaration order: TCP channel, UDP channel, then three service clients
  (`BaseClient` and `DeviceConfigClient` on the TCP router,
  `BaseCyclicClient` on the UDP router). If the UDP channel fails, the
  already-built TCP channel is destroyed properly on the way out — no
  session leaks on the robot.
- **Line 85** — one console line confirming the connection. The body does
  nothing else.
- **Why `BaseClient` vs `BaseCyclicClient`**: `base()` (TCP) is for
  configuration and high-level commands — servoing-mode switches, fault
  clearing; `base_cyclic()` (UDP) is the 1 kHz real-time streaming channel
  the loop uses. Main hands both pointers to the Runner.

### Accessors (Hardware.h:56-85)

- `base()` / `base_cyclic()` — plain getters returning raw pointers; the
  `Connect` object keeps ownership (unique_ptr members), callers just borrow.
- `tcp_router()` (Hardware.h:74-77) — exposes the raw router so *external
  tools* can build additional service clients. Nothing in the controller
  binary calls it. **FLAG `Hardware.h:74-77` | unnecessary** — in the
  control path this accessor is dead; it exists for the read-only
  diagnostics tools that share this header. Safe to keep, but know it is
  not part of the run.
- `device_config()` (Hardware.h:82-85) — shares the already-owned
  DeviceConfig client. The long CAUTION comment above it is real: a Kortex
  router refuses a second notification-callback registration for the same
  service, so building a *second* DeviceConfigClient on this router throws.
  That is why the client is shared instead of rebuilt. **FLAG
  `Hardware.h:65-94` | edit-hazard** — "just build your own client" is the
  natural refactor and it fails at runtime with a cryptic router error.

### `Connect::EnsureJointLimits` (Hardware.cpp:88-171) — the joint-limit gate

Called from Main.cpp:274, before any takeover, unless
`config::kSkipStartupGates`. **FLAG `Hardware.cpp:88-171` | hides-work** —
the name says "ensure", which sounds like a check; this function *writes
safety configuration into the robot's firmware* (SetSafetyConfiguration)
whenever the read-back does not match Config.h. It is the fix for the
degenerate 0/0 JOINT_LIMIT band problem (bands do not survive a power
cycle; a 0/0 band makes the firmware fault any motion away from zero).

- **Lines 91-94** — the two safety identifiers, HIGH and LOW joint limit,
  from the Kortex enum. Actuator device ids are 1..7 in joint order.
- **Lines 97-102** — loop over the seven joints; a joint whose Config.h
  warn *and* error entries are both 0.0 is skipped entirely ("leave
  alone"). Today that is every joint except joint 4 — joint 6 was zeroed
  on 2026-08-04 because its config service is wedged and every RPC to it
  only times out. So in the current configuration this loop does real work
  for exactly one joint. **FLAG `Hardware.cpp:99-102` | edit-hazard** —
  the 0.0/0.0 "skip" sentinel is load-bearing: setting a joint's config
  entries to zero doesn't mean "limit at zero", it means "never touch this
  joint's band". Easy to misread when tuning Config.h.
- **Lines 104-110** — the per-joint `try` block. Deliberate: before
  2026-08-04 one unreachable actuator (joint 6) aborted the whole gate and
  left joint 4 on its degenerate band. Now a timeout on one joint cannot
  block restoring the others. Note the unusual indentation — the `try {`
  wraps the following `for` without re-indenting it (lines 110-146), which
  reads as if the `for` were outside the try. **FLAG
  `Hardware.cpp:109-147` | edit-hazard** — the non-indented try body makes
  it easy to move code "into" or "out of" the exception scope by accident.
- **Lines 110-115** — for HIGH then LOW: the configured magnitude gets its
  sign applied here (+warn for HIGH, −warn for LOW), cast to `float`
  because that is what the protobuf stores.
- **Lines 117-134** — read the current configuration
  (`GetSafetyConfiguration`), compare warning and error thresholds, and if
  either differs: print old→new, write the corrected configuration
  (`SetSafetyConfiguration`), remember `changed`.
- **Lines 136-145** — read back *again* and verify the error threshold
  stuck. If not, print "robot NOT ready" and return false — Main then
  refuses the run. Write-then-verify is the pattern because a write that is
  silently ignored is exactly the failure the wedged joint 6 produces.
- **Lines 148-166** — the catch: the joint's config service did not answer.
  With `kAllowUnverifiedActuators` false (the default, and current value)
  this refuses the run; with it true, it prints a WARNING and continues
  with that joint's band unknown. This is one of the three guard overrides
  echoed into every CSV preamble.
- **Lines 168-170** — the PASS line, noting whether corrections were
  applied, and `return true`.

---

## 2. CyclicSession — the per-cycle exchange (Hardware.cpp:178-236)

Constructed inside `RunControlLoop` (Runner.cpp:117), *before* the
`ServoingGuard` (Runner.cpp:120) switches the arm to LOW_LEVEL_SERVOING.

### helpers (Hardware.cpp:192-203)

- `NUM_JOINTS` — derived from `JointVector`'s size at compile time
  (`std::tuple_size_v<std::array<double,7>>` is 7), so "seven joints" is
  written down exactly once, in Config.h.
- `wrap_0_360` (198-202) — the robot expects command angles in [0, 360).
  `std::fmod(a, 360)` keeps the sign of `a`, so a negative result gets 360
  added. This is where the controller's *continuous* (unwrapped) command
  meets the robot's wrapped convention.

### `CyclicSession::CyclicSession` (Hardware.cpp:205-208)

Stores the client pointer. The `command_` protobuf member starts empty —
`Seed()` populates it.

### `CyclicSession::Seed` (Hardware.cpp:210-219) — Runner.cpp:126, T3 of takeover

- **Line 213** — one `read_feedback` (see below): the only standalone read
  inside a command loop; its round trip also gives the base time to finish
  entering LOW_LEVEL_SERVOING (the Runner calls Seed *after* the mode
  switch for exactly that reason).
- **Lines 216-217** — `command_.add_actuators()` seven times: protobuf
  repeated fields start empty, so the seven command slots are created once
  here and *reused* every cycle (`mutable_actuators(i)` later) — no
  allocation inside the loop.
- **If Seed were skipped**: the first `Send` would index empty actuator
  slots — protobuf would either crash or send a malformed frame.

### `CyclicSession::Send` (Hardware.cpp:221-236) — the one exchange per cycle

Called from Runner.cpp:155/165 (takeover holding frames) and Runner.cpp:263
(every control cycle). SENDS COMMANDS TO THE ARM.

**FLAG `Hardware.cpp:221-236` | hides-work** — the name says "send", but
this function does four distinct things: wraps each angle to [0,360),
advances the 16-bit frame id, stamps every actuator's command id, and
performs the blocking network round trip whose *reply* is the next cycle's
feedback. Two of those (wrapping, id stamping) silently define semantics
the log analysis depends on (`measured_raw_deg` vs continuous commands;
`command_frame_id` vs `actuator_command_ack`).

- **Lines 224-225** — write each setpoint, wrapped, into its persistent
  slot.
- **Line 228** — `frame_id = (frame_id + 1) % 65536`: the id the base uses
  to reject stale/duplicate packets. **FLAG `Hardware.cpp:228` |
  edit-hazard** — the `% 65536` matches the 16-bit field the base actually
  compares; "simplifying" to a plain increment changes the wraparound
  behaviour of the freshness evidence (`ack_unchanged_j*`) after ~2 minutes
  at 500 Hz.
- **Lines 231-232** — every actuator's `command_id` gets the same frame id;
  actuator feedback later echoes the id of the last command it *processed*
  — that echo is what `FeedbackFreshnessMonitor` counts.
- **Line 235** — `Refresh(command_, 0)`: the blocking UDP exchange. Its
  return value is the same cycle's feedback. `0` is the device id
  (the base).
- `last_command_frame_id()` (Hardware.h:153) — read by the Runner's
  `FillSample` (Runner.cpp:272) so each log row records what id was just
  sent.

---

## 3. `read_feedback` (Hardware.cpp:247-251)

One line: `return base_cyclic->RefreshFeedback();` — a feedback-only
exchange, no command. Deliberately THE only `RefreshFeedback` call in the
program ("single reader" decision): everywhere else, feedback arrives as
the reply to `Send`. Called from Main.cpp:255 (the pre-takeover readiness
read) and from `Seed()`. Wrapping one SDK call in a named function looks
redundant, but it is what makes "there is exactly one standalone read"
checkable by grep — leave it.

---

## 4. Recording — sample, queue, writer

### `LoopLogSample` (Hardware.h:256-304)

A plain struct: one control cycle, 121 CSV columns' worth of values. The
long comment block (Hardware.h:199-255) is the log-format contract —
column order (log_format = 6; the "(121 columns)" count is at
Hardware.h:213), the requested/sent/measured distinction, timestamp
semantics, and the warning that a row mixes *two* exchanges (controller
inputs from last cycle's reply, measurements from this cycle's). The
format history (Hardware.h:214-223) records that format 6 (2026-08-04)
removed the nine trajectory-playback columns (ref_j1..7, playback_t_s,
playback_state) with the playback feature: tooling that read them by name
no longer finds them, and every column index after pd_beyond_reach
shifted. Field notes:

- `measured_deg` vs `measured_raw_deg` (h:265-269) — the raw feedback is
  wrapped to [0,360); `measured_deg` is shifted by whole turns to sit
  within ±180° of the command so plots share an axis. The comment flags the
  ambiguity: once command and measurement drift more than half a turn
  apart, the shift picks the wrong turn. **FLAG `Hardware.h:263-269` |
  edit-hazard** — two fields that look like duplicates are not; analysis
  scripts rely on each one's exact convention.
- The format-4 freshness fields (h:288-296) and format-5 requested-vs-sent
  fields (h:298-303) are diagnostics for the joint-6 investigation; all are
  filled every cycle by the Runner.

### `LoopLog` — the single-producer/single-consumer ring (Hardware.cpp:265-314)

Why this exists at all: the header comment (Hardware.h:186-193) records
that 16 of the first 60 runs left *zero-byte* CSVs because the old design
wrote the log once at the end and any hard kill skipped it. Now rows reach
the kernel every 100 ms while the run happens.

First-time C++ note — `std::atomic<std::size_t>`: a variable two threads
may read/write simultaneously without a mutex. Plain variables shared
across threads are undefined behaviour; atomics make each load/store
indivisible and let you specify *memory ordering* — how much other memory
traffic is synchronized around the atomic operation.

- **Constructor (265-268)** — `samples_.resize(capacity)` allocates every
  slot up front; nothing in `push` ever allocates. Capacity comes from
  Main.cpp:283-284: `kLogBufferSeconds * cycles-per-second` = 30 s ×
  500 Hz = 15 000 slots.
- **`push` (270-285)** — producer side, called once per cycle from the
  loop.
  - Line 275: `head_` loaded *relaxed* — only this thread writes it, so no
    synchronization needed to read our own counter.
  - Line 276: `tail_` loaded *acquire* — pairs with the consumer's
    *release* store in `Drain`; acquire/release is the C++ idiom meaning
    "everything the other thread wrote before its release store is visible
    to us after our acquire load". Here it guarantees a slot the writer
    released really is finished being read.
  - Lines 277-280: if the ring is full (`head - tail >= size`), drop *this*
    sample and count it — never overwrite a slot the consumer may still be
    copying. Dropping the newest (not the oldest) is the price of never
    racing.
  - Line 281: copy the sample into its slot (`head % size` — the ring).
  - Line 284: publish with a *release* store of `head + 1`: the sample's
    bytes are guaranteed written before the consumer can observe the new
    head. **FLAG `Hardware.cpp:270-299` | edit-hazard** — the
    acquire/release pairing is the entire correctness argument of this
    queue; changing any `memory_order`, or adding a second producer or
    consumer, silently breaks it. Note the head/tail counters never wrap
    to the buffer size — they count forever and `% size` happens at
    indexing; `head - tail` relies on unsigned arithmetic.
- **`Drain` (287-299)** — consumer side, writer thread only. Mirror image:
  own `tail_` relaxed, other's `head_` acquire, copy `count` samples out,
  then release-store `tail_ = head` — only *after* the copy, because until
  that store the producer treats those slots as in use.
- **`capacity`/`total_pushed`/`dropped` (301-314)** — `dropped_` is a plain
  (non-atomic) member: only the producer writes it and it is read after
  the loop has stopped, so no race. Reading it *during* the run from
  another thread would be a bug.

### `WriteCsvHeader` / `WriteCsvRow` (Hardware.cpp:316-398)

The authority for log_format 6: the header's (316-352) column order and
the row's (354-398) field order must match each other and the comment in
Hardware.h. Straight streaming of every field, booleans as 0/1. The
frame-id columns now follow pd_beyond_reach directly (header line 335, row
line 381 — the nine playback columns that used to sit between them are
gone). Relies on the stream's default six-significant-digit formatting —
the comment (Hardware.h:341-343) says every parser assumes that, so adding
`std::setprecision` here would change every future log's resolution.
**FLAG `Hardware.cpp:316-398` | edit-hazard** — the two functions are
position-coupled to each other and to external analysis scripts; any
reorder/insert must bump the log_format number in Main's preamble
(Main.cpp:115, currently 6). One leftover from the removal: the
declaration comment at **Hardware.h:341** still says "the authority for
log_format = 5" while the struct comment (h:205) and Main's preamble say
6 — a stale comment, not a behaviour bug, but worth fixing before it
misleads a parser author.

### `LoopLogWriter` (Hardware.cpp:400-461)

- **Constructor (400-410)** — writes the CSV header and flushes
  immediately (a run killed while connecting still leaves a
  self-describing file), reserves the staging vector to full capacity (so
  drains never allocate), then starts the writer thread:
  `std::thread(&LoopLogWriter::Run, this)`. First-time C++ note —
  `std::thread` starts running its function immediately on construction;
  you must `join()` (wait for it) before destroying the `std::thread`
  object, or the program aborts.
- **`Run` (422-428)** — sleep `interval_` (100 ms), drain, repeat until
  `stop_` is set. Worst case a hard kill costs the last 100 ms of rows.
- **`DrainOnce` (430-445)** — pulls everything new out of the ring, formats
  the whole batch into one `std::ostringstream` in memory, then writes it
  with a single `csv_.write(...)` and flushes. The single-write-per-batch
  is what guarantees a kill can only cut the file *between* rows, and the
  flush is what moves the bytes from the process's buffer into the
  kernel's (so they survive the process dying, though not a power cut).
- **`Stop` (447-456)** — idempotent via the `joinable()` check: set the
  flag, join the thread, then one final `DrainOnce`. The comment order
  matters: this final drain is only "final" because the producer
  (the loop) has already stopped pushing — Main calls `Stop()`
  (Main.cpp:407) after `RunControlLoop` returns. **FLAG
  `Hardware.cpp:447-456` | edit-hazard** — calling `Stop()` while the loop
  still pushes would silently lose whatever is pushed after the final
  drain; the safety is an ordering convention in Main, not something this
  class enforces.
- **Destructor (412-420)** — `Stop()` inside try/catch so an unwinding
  path (exception in Main) still drains, and a failing drain cannot
  terminate the program. Main declares the writer *after* the `csv` stream
  and *before* the loop runs, so on any exception the writer is destroyed
  (final drain) before the stream it writes to.
- **`rows_written` (458-461)** — valid only after `Stop()`; before that the
  writer thread is still incrementing it unsynchronized.

### Filename helpers (Hardware.cpp:463-481)

- `timestamped_csv_name` (463-471) — `<prefix>_YYYYMMDD_HHMMSS.csv` in
  local time via `localtime_r` (the thread-safe variant of `localtime`) and
  `std::put_time`. One file per run so a failed run's evidence is never
  overwritten.
- `dated_run_dir` (473-481) — `<runs_root>/YYYY-MM-DD`; Main creates the
  directory and the plot scripts search this layout. Two functions instead
  of one because Main also honours an explicit `--log` path that bypasses
  the directory scheme.
