# Targets.cpp / Targets.h — line-by-line read

Targets is the operator's side of the reference pipeline: parse a typed
pose line, keep the latest one in a thread-safe store, and serve it to the
controller as a `ReferenceSource`. The header's own first line states the
boundary: never talks to the robot, does no control math.

> **Removal note, up front.** Under the fixed-target-only design the
> operator input machinery is slated for removal:
>
> - **stdin input thread**: `RunPoseTargetInput`, **Targets.cpp:82-116**
>   (declaration Targets.h:64-68; started at Main.cpp:567-570, joined at
>   Main.cpp:581-583 and by `InputThreadJoiner`).
> - **file-watch thread**: **already gone** — removed 2026-08-04. Only the
>   tombstone comment at **Targets.cpp:118-122** remains (it explains the
>   watcher served the deleted TOML `target_file` key and points to git
>   history). There is no code to delete, but the header's opening comment
>   (Targets.h:3) still advertises "the stdin and file-watch input
>   threads" — that line is stale.
>
> What else falls with stdin-only removal: `ParsePoseTarget`
> (Targets.cpp:29-58), both `PoseTargetStore` store methods
> (Targets.cpp:60-74), and the store-sequence branch of
> `PoseTargetSource::Get` (Targets.cpp:150-152). What stays:
> `RotationFromRpy` (the fixed target's rpy uses it, Main.cpp:535) and the
> `initial_target_` path of `PoseTargetSource`.

Execution order in a run with `kUseFixedTarget = false` (today's default):

1. `PoseTargetStore pose_targets;` — Main.cpp:415
2. `PoseTargetSource(...)` constructed — Main.cpp:540-541
3. `std::thread(RunPoseTargetInput, ...)` — Main.cpp:568 (the stdin thread)
4. Inside the loop: `Reset(state)` once at takeover (Runner.cpp:168), then
   `Get(state, dt, status)` every cycle (Runner.cpp:218).
5. With `kUseFixedTarget = true`, steps 1-3 change: no thread starts, and
   Main builds a `PoseTarget` (using `RotationFromRpy` if rpy is enabled)
   and passes it as `initial_target`.

---

## `RotationFromRpy` (Targets.cpp:19-27)

```cpp
return (Eigen::AngleAxisd(yaw, UnitZ()) *
        Eigen::AngleAxisd(pitch, UnitY()) *
        Eigen::AngleAxisd(roll, UnitX())).toRotationMatrix();
```

- **What**: build a rotation matrix from roll/pitch/yaw radians, composed
  R = Rz(yaw)·Ry(pitch)·Rx(roll). `Eigen::AngleAxisd` is "rotate by this
  angle about this axis"; multiplying them composes right-to-left, so the
  roll about X is applied first.
- **Why it exists**: the header (Targets.h:26-30) calls it "the one place
  that convention is written down" — it mirrors the Python sim's
  `rotation_from_rpy`, so a compiled target, a typed 6-number line, and
  the sim all agree on what an rpy triple means. Main's startup print
  decomposes the measured orientation with `eulerAngles(2,1,0)` to match.
- **If changed**: swapping the multiplication order silently changes every
  orientation target *and* desynchronizes from the Python sim and from the
  startup printout the operator copies numbers from. **FLAG
  `Targets.cpp:19-27` | edit-hazard** — three call sites in two languages
  depend on this exact composition order.

## `ParsePoseTarget` (Targets.cpp:29-58)

Parses one line into 3 numbers (position only) or 6 (position + rpy),
returning `std::optional<PoseTarget>` — a box that holds either a value or
nothing (`std::nullopt`); "nothing" here means "rejected", with the reason
written into the `error` out-parameter.

- **Lines 31-34** — `std::istringstream in(line)` treats the string as a
  stream; `in >> values[count]` extracts whitespace-separated doubles,
  stopping at 6 or at the first non-number.
- **Lines 35-38** — reject non-finite numbers (`inf`/`nan` parse as valid
  doubles!). Without this an operator typo could feed NaN into the control
  law. First place the program defends against it.
- **Line 41** — `in.clear()`: subtle. If the loop stopped because
  extraction *failed* (e.g. 4th token was "abc"), the stream's fail bit is
  set and every further `>>` is a no-op — including the trailing-text
  check below, which would then falsely pass. `clear()` resets the error
  flags first. **FLAG `Targets.cpp:41-45` | edit-hazard** — deleting the
  "redundant-looking" `clear()` makes `1 2 3 abc` parse as a valid
  3-number target.
- **Lines 42-45** — any trailing token rejects the line.
- **Lines 47-51** — exactly 3 or exactly 6; anything else rejects with a
  message that restates the units (meters; radians).
- **Lines 53-57** — build the target; only the 6-number form sets
  `rotation`, so a 3-number line leaves it `nullopt` = "keep the current
  orientation target". Note the header (Targets.h:34-36): there is
  deliberately **no reachability check** — an unreachable target makes the
  controller push until something stops it.
- **FLAG `Targets.cpp:29-58` | unnecessary** *(under the fixed-target-only
  design)* — its only caller is the stdin thread below; it goes when the
  thread goes.

## `PoseTargetStore` (Targets.h:44-62, Targets.cpp:60-80)

The shared mailbox between the input thread (writer) and the control loop's
source (reader). First-time note — `std::mutex` + `std::lock_guard`: a
mutex is a lock only one thread can hold at a time; `lock_guard` locks it
on construction and unlocks on destruction (RAII), so the lock can never be
forgotten on an early return. The header's promise: the mutex is held only
for the copy, so the 2 ms loop's worst-case wait is tiny.

- `Store` (cpp:60-67) — position + rotation, and `++sequence_`. The
  sequence number is how the source detects "a *new* target arrived",
  including re-sends of the same coordinates.
- `StorePosition` (cpp:69-74) — position only; note it does **not** clear
  `rotation_` — the previously stored orientation target survives, which
  is exactly the "orientation target unchanged" behaviour the prompt
  advertises. **FLAG `Targets.cpp:69-74` | edit-hazard** — "clearing the
  stale rotation" would look like a cleanup and would change behaviour:
  a later position-only target would silently drop an orientation the
  operator had commanded.
- `Get` (cpp:76-80) — copy all three fields out under the lock and return
  them as a `Snapshot`. Returning a copy (not references) is what makes
  the caller race-free after the lock is released.
- **Removal note**: with stdin gone, nothing ever calls `Store`/
  `StorePosition`; the store shrinks to a vestige that only ever reports
  sequence 0. Main still constructs one (Main.cpp:415) and the source
  holds a reference to it.

## `RunPoseTargetInput` (Targets.cpp:82-116) — THE STDIN INPUT THREAD

**FLAG `Targets.cpp:82-116` | unnecessary** *(slated for removal —
fixed-target-only design; this is the entire stdin thread body)*.

Thread body started by Main (only when not playback and not fixed-target).
Runs until Ctrl+C (`stop`, a `std::atomic<bool>` — see State.md/Hardware.md
for what an atomic is; here it is the cross-thread stop flag the signal
handler sets) or stdin EOF.

- **Lines 85-91** — the poll dance. A plain `std::getline(std::cin, ...)`
  blocks with no portable way to interrupt it, so the thread would survive
  Ctrl+C until the next Enter. Instead: `poll()` (a POSIX call) waits up
  to 100 ms for stdin to become readable; on timeout (`ready <= 0`) loop
  around and re-check `stop`. `getline` only runs when a line is actually
  waiting. **edit-hazard within an already-flagged range** — replacing
  poll with blocking getline re-introduces the hang-on-exit; this is also
  POSIX-only code (`poll.h`, `unistd.h`), the one platform-specific corner
  of the module.
- **Lines 92-93** — `getline` failing means EOF (stdin closed): break, no
  more targets can ever arrive; the loop keeps running with the last
  target.
- **Lines 94-95** — skip empty lines (a stray Enter is not an error).
- **Lines 97-101** — parse; rejected lines print the reason and continue.
  Printing from this thread is fine — it is not the control loop.
- **Lines 103-114** — accepted: full pose → `Store`, position-only →
  `StorePosition`; echo what was accepted, restating units and frame, and
  for the position-only case that the orientation target is unchanged.

## The file-watch thread tombstone (Targets.cpp:118-122)

A comment only: the watched-target-file input (`FirstTargetLine` /
`RunPoseTargetFileInput`) was removed 2026-08-04; it served the TOML
front-end's `target_file` key, deleted 2026-08-03, and had been dead
(compiled-empty `kTargetFile`) since. Git history has the code. Nothing to
remove here beyond the stale mention of "file-watch input threads" in
Targets.h:3.

## `PoseTargetSource` (Targets.cpp:128-161) — the ReferenceSource

The bridge from the store to the controller. Inherits `ReferenceSource`
(State.h:114-127); the loop only ever sees the base-class interface.

### Constructor (cpp:128-132)

Keeps a `const` reference to the store (read-only access — this class can
never write targets) and moves in the optional `initial_target`
(the compiled fixed target, or `nullopt`).

### `Reset` (cpp:134-139) — called once at takeover (T5)

```cpp
baseline_sequence_ = store_.Get().sequence;
```

- **What**: snapshot the store's sequence number *at takeover*.
- **Why**: anything typed **before** takeover is deliberately discarded —
  `Get` below only serves store content whose sequence has moved past this
  baseline. A target typed during the connection dance will not yank the
  arm the instant servoing starts. The deliberate exception is
  `initial_target_`, which Main passes on purpose.

### `Get` (cpp:141-161) — called every cycle

- **Line 144** — one locked snapshot per cycle.
- **Lines 146-149 (comment)** — operator targets are **stationary**: the
  reference `Twist{}` stays zero so the controller's Kd term is pure
  damping. A future source that *moves* its target must fill the twist or
  the damping term fights the motion.
- **Lines 150-152** — if the store's sequence moved past the baseline, an
  operator target exists: serve it, carrying the store's sequence so the
  controller's arrival notice can fire once per distinct target.
  *(Removal note: this branch is the stdin path — dead once the thread
  goes.)*
- **Lines 153-159** — otherwise, if there is an initial (fixed) target,
  serve that with **sequence 0**. The comment explains the trick: live
  store sequences continue from the takeover baseline and are therefore
  never 0, so 0 uniquely identifies the fixed target and its arrival
  notice also fires exactly once. **FLAG `Targets.cpp:150-159` |
  edit-hazard** — the branch *order* (store wins over initial) and the
  sequence-0 convention interlock with the controller's arrival
  edge-detection; swapping the branches or "normalizing" the sequence
  changes which target the arm chases after the first typed line, and how
  often arrival prints.
- **Line 160** — neither branch taken → empty `Reference` → the controller
  holds the takeover pose (State.h's "NEITHER set means no reference").

### What Get never does

No I/O, no allocation beyond the snapshot copy, no blocking beyond the
store's brief mutex — the `ReferenceSource` contract (State.h:112) that
keeps the 500 Hz loop honest.
