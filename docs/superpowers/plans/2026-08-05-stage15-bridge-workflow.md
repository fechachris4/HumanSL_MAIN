# Stage 1.5 Bridge Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Kill the three-terminal FIFO ceremony and hand-typed start state: the controller reads targets from a self-reopening named pipe and holds its measured pose at startup; the bridge finds its own start state; the pipe format can carry orientation (transport only); one launcher script runs a session.

**Architecture:** Spec: `docs/superpowers/specs/2026-08-05-stage15-bridge-workflow.md`. Four code changes (two bridge-side, two controller-side) plus a launcher script and docs. Controller changes reuse the tested `RunPoseTargetInputFromFd` core and the existing settle/dwell machinery — no reactive-law, safety, or actuation changes anywhere.

**Tech Stack:** C++17, POSIX (`mkfifo`, `open`, `poll`), bash, existing test harnesses (basic_control's `Check`-based tests in `tests/test_reactive_law.cpp`, planner_bridge's assert-based ctest).

## Global Constraints

- Never run `controller`, root `main`, or any Kortex-linked binary; building is fine, hardware validation is a separate authorized step (project CLAUDE.md).
- No change to ReactiveLaw, Safety, Actuation, Runner, or any guard/limit constant.
- Orientation is transport-only: `config::kAcceptOrientationTargets = false` compiled in; a 7-field line is rejected loudly while false — never silently truncated to a position target.
- Quaternion order is xyzw (matches telemetry `quat_x..quat_w`), unit norm `|n−1| ≤ 1e-3`, base_link.
- Pipe path constant: `config::kTargetPipePath = "/tmp/humansl_bridge_targets"`.
- Units/frames unchanged: metres, base_link, Kortex actuator order 1..7, radians internal / degrees at CSV boundary.
- TDD per task; compile failure counts as the expected RED. basic_control tests follow the existing `Check(...)`/`TestXxx()` style in `tests/test_reactive_law.cpp`; bridge tests follow the assert style with `#undef NDEBUG` first line.
- planner_bridge CMake conventions: new test targets copy the existing pattern, and the test-name list in the LD_LIBRARY_PATH `foreach` loop gains any new test.

## File Structure

```
Bridge:   src/StartState.{h,cpp} (+FindLatestRunCsv), src/BridgeMain.cpp (auto-state,
          --runs-root, --emit-orientation), src/Waypoints.{h,cpp} (FormatTargetLine
          overload), tests/test_start_state.cpp, tests/test_bridge_main.cpp,
          tests/test_waypoints.cpp
Control:  src/Config.h (kTargetPipePath, kAcceptOrientationTargets; delete kFixedTargetM),
          src/Targets.{h,cpp} (FromPipe; extended parser; delete stdin wrapper),
          src/Main.cpp (pipe thread, hold-at-startup), tests/test_reactive_law.cpp
Launcher: Christian_control/planner_bridge/scripts/run_session.sh
Docs:     Christian_control/docs/decisions/stage15-bridge-workflow.md,
          Christian_control/basic_control/README.md (runbook rewrite)
```

---

### Task 1: Bridge finds its own start state

**Files:**
- Modify: `Christian_control/planner_bridge/src/StartState.h`, `src/StartState.cpp`
- Modify: `Christian_control/planner_bridge/src/BridgeMain.cpp` (arg handling + default resolution)
- Test: `Christian_control/planner_bridge/tests/test_start_state.cpp`, `tests/test_bridge_main.cpp`

**Interfaces:**
- Consumes: existing `ReadLatestMeasuredQ(csv_path, error)`.
- Produces:

```cpp
// StartState.h addition
// Newest loop_log*.csv by modification time under <runs_root>/<subdir>/
// (the controller's dated-directory layout). nullopt + error when the
// root is missing or holds no matching file.
std::optional<std::string> FindLatestRunCsv(const std::string& runs_root,
                                            std::string& error);
```

- `RunBridge` behavior change: when NEITHER `--state-csv` nor `--start-deg` is given, resolve `<repo>/runs` (via the existing `/proc/self/exe`-relative default-path helper, like `DefaultDhPath`), call `FindLatestRunCsv`, then `ReadLatestMeasuredQ` on the result; `--runs-root PATH` overrides the root. On failure: exit 2, diagnostics `"no run log found under <root> — start the controller first (it creates the log), or pass --state-csv/--start-deg"`. `--start-deg` help text gains `(test-only)`.

- [ ] **Step 1: Write the failing tests.** In `test_start_state.cpp` append to `main` (temp dirs under the working directory, cleaned up like the existing temp CSV):

```cpp
    // FindLatestRunCsv: newest dated subdir wins by mtime.
    std::filesystem::create_directories("tsr_tmp/2026-08-04");
    std::filesystem::create_directories("tsr_tmp/2026-08-05");
    { std::ofstream("tsr_tmp/2026-08-04/loop_log_a.csv") << "x\n"; }
    { std::ofstream("tsr_tmp/2026-08-05/loop_log_b.csv") << "x\n"; }
    // Ensure strictly increasing mtimes regardless of filesystem resolution.
    std::filesystem::last_write_time("tsr_tmp/2026-08-04/loop_log_a.csv",
        std::filesystem::file_time_type::clock::now() - std::chrono::hours(1));
    std::string find_error;
    const auto latest = FindLatestRunCsv("tsr_tmp", find_error);
    assert(latest.has_value() && find_error.empty());
    assert(latest->find("loop_log_b.csv") != std::string::npos);
    { std::ofstream("tsr_tmp/2026-08-05/notes.txt") << "x\n"; }  // non-matching ignored
    assert(FindLatestRunCsv("tsr_tmp", find_error) == latest);
    assert(!FindLatestRunCsv("tsr_missing", find_error).has_value() && !find_error.empty());
    std::filesystem::remove_all("tsr_tmp");
```

In `test_bridge_main.cpp` add: `RunBridge` with `--goal` only and `--runs-root` pointing at a fixture dir containing one dated subdir with a valid 13-column CSV (reuse the fixture-writing pattern from `test_start_state.cpp`) → exit 0, ≥1 parseable line; same call with an empty runs root → exit 2, empty targets, diagnostics mentions "start the controller".

- [ ] **Step 2: Run to verify failure** — `cmake --build Christian_control/planner_bridge/build -j"$(nproc)"` → compile error (`FindLatestRunCsv` undeclared).

- [ ] **Step 3: Implement.**

```cpp
// StartState.cpp addition
#include <filesystem>

std::optional<std::string> FindLatestRunCsv(const std::string& runs_root,
                                            std::string& error) {
    error.clear();
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::is_directory(runs_root, ec)) {
        error = "runs root not found: " + runs_root;
        return std::nullopt;
    }
    std::optional<std::string> newest;
    fs::file_time_type newest_time{};
    for (const auto& day : fs::directory_iterator(runs_root, ec)) {
        if (ec || !day.is_directory()) continue;
        for (const auto& entry : fs::directory_iterator(day.path(), ec)) {
            if (ec || !entry.is_regular_file()) continue;
            const std::string name = entry.path().filename().string();
            if (name.rfind("loop_log", 0) != 0 ||
                entry.path().extension() != ".csv")
                continue;
            const auto written = entry.last_write_time(ec);
            if (ec) continue;
            if (!newest || written > newest_time) {
                newest = entry.path().string();
                newest_time = written;
            }
        }
    }
    if (!newest)
        error = "no loop_log*.csv under " + runs_root;
    return newest;
}
```

In `BridgeMain.cpp`: add `--runs-root` to `ParseArgs` (one path argument, default empty); in `RunBridge`'s start-state resolution branch, replace the current "must pass one of the two" error with the auto-discovery path described in Interfaces (default root = repo root + `/runs` via the same helper `DefaultDhPath` uses). Update `kUsageText` (grammar line, `--runs-root`, `(test-only)` on `--start-deg`).

- [ ] **Step 4: Run to verify pass** — full `ctest --test-dir Christian_control/planner_bridge/build --output-on-failure`, all green.

- [ ] **Step 5: Commit** — `git commit -m "planner_bridge: auto-discover latest run CSV as default start state"`

---

### Task 2: Controller — named-pipe target input, stdin path deleted

**Files:**
- Modify: `Christian_control/basic_control/src/Config.h` (add `kTargetPipePath`)
- Modify: `Christian_control/basic_control/src/Targets.h`, `src/Targets.cpp`
- Modify: `Christian_control/basic_control/src/Main.cpp`
- Test: `Christian_control/basic_control/tests/test_reactive_law.cpp`

**Interfaces:**
- Produces:

```cpp
// Targets.h — replaces the RunPoseTargetInput declaration
// Reads target lines from a named pipe, surviving writer disconnects:
// on EOF the pipe is reopened, so each bridge invocation may open, write,
// and close independently. `stop` is the only exit. The fd-level loop is
// the tested RunPoseTargetInputFromFd, unchanged.
void RunPoseTargetInputFromPipe(PoseTargetMailbox& mailbox,
                                const std::atomic<bool>& stop,
                                const std::string& pipe_path);
```

```cpp
// Config.h, next to the input constants
// The named pipe the controller reads targets from. Created by Main at
// startup if missing. Writers (planner_bridge, echo) open/write/close per
// plan; the reader reopens after every EOF.
inline constexpr const char* kTargetPipePath = "/tmp/humansl_bridge_targets";
```

- [ ] **Step 1: Write the failing test.** In `test_reactive_law.cpp`, next to the existing fd-input tests (~line 518), add `TestPipeInputSurvivesWriterReconnect()` and register it where the other tests run:

```cpp
    void TestPipeInputSurvivesWriterReconnect()
    {
        const std::string pipe_path = "./test_target_pipe_tmp";
        unlink(pipe_path.c_str());
        Check(mkfifo(pipe_path.c_str(), 0600) == 0, "mkfifo succeeds");

        PoseTargetMailbox mailbox;
        std::atomic<bool> stop{false};
        std::thread reader([&] {
            RunPoseTargetInputFromPipe(mailbox, stop, pipe_path);
        });

        // Two independent writer sessions — the Stage 1 EOF failure mode.
        for (int session = 0; session < 2; ++session) {
            std::ofstream writer(pipe_path);   // blocks until reader has the pipe open
            Check(static_cast<bool>(writer), "writer opens pipe");
            writer << "0.1 0.2 0.3\n";
        }                                       // close = EOF for the reader

        // Both targets must arrive despite the intervening EOF.
        Eigen::Vector3d seen[2];
        int received = 0;
        for (int attempt = 0; attempt < 200 && received < 2; ++attempt) {
            if (const auto target = mailbox.TryDequeue())
                seen[received++] = target->p_desired;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        Check(received == 2, "targets from both writer sessions arrive");

        stop.store(true);
        const auto join_start = std::chrono::steady_clock::now();
        reader.join();
        Check(std::chrono::steady_clock::now() - join_start
                  < std::chrono::seconds(1), "reader joins promptly on stop");
        unlink(pipe_path.c_str());
    }
```

- [ ] **Step 2: Run to verify failure** — build the basic_control test target (see `tests/` registration in `Christian_control/basic_control/CMakeLists.txt`; the reactive-law test is hardware-free) → compile error (`RunPoseTargetInputFromPipe` undeclared).

- [ ] **Step 3: Implement.** In `Targets.cpp` (uses existing `kInputPollTimeoutMs`; add `<fcntl.h>`, `<sys/stat.h>` includes):

```cpp
void RunPoseTargetInputFromPipe(PoseTargetMailbox& mailbox,
                                const std::atomic<bool>& stop,
                                const std::string& pipe_path)
{
    while (!stop.load(std::memory_order_relaxed)) {
        // Non-blocking open: a blocking O_RDONLY open would wedge teardown
        // until a writer appears. With no writer yet, poll below sees
        // nothing and the FromFd loop idles at its own poll cadence.
        const int fd = open(pipe_path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(kInputPollTimeoutMs));
            continue;
        }
        RunPoseTargetInputFromFd(mailbox, stop, fd);
        close(fd);
        // FromFd returned: stop (outer loop exits) or EOF (writer left) —
        // brief pause so a vanished writer cannot spin this loop hot.
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kInputPollTimeoutMs));
    }
}
```

Delete `RunPoseTargetInput` from `Targets.h`/`Targets.cpp`. In `Main.cpp`: create the pipe before the thread launch (error out if the path exists and is not a FIFO):

```cpp
        struct stat pipe_stat{};
        if (stat(config::kTargetPipePath, &pipe_stat) == 0) {
            if (!S_ISFIFO(pipe_stat.st_mode)) {
                std::cerr << "Error: " << config::kTargetPipePath
                          << " exists and is not a FIFO — not starting\n";
                return 1;
            }
        } else if (mkfifo(config::kTargetPipePath, 0600) != 0) {
            std::cerr << "Error: cannot create target pipe "
                      << config::kTargetPipePath << " — not starting\n";
            return 1;
        }
        std::thread input_thread(RunPoseTargetInputFromPipe,
                                 std::ref(pose_targets), std::cref(g_stop),
                                 std::string(config::kTargetPipePath));
```

Replace the "type x y z …" banner with one line naming the pipe: `targets: write "x y z" lines to <kTargetPipePath> (planner_bridge does this; validation happens bridge-side)`.

- [ ] **Step 4: Run to verify pass** — basic_control test suite (hardware-free targets confirmed from its CMake `add_test` entries) green, including the new pipe test; then build the `controller` target itself to prove Main compiles. Do not run `controller`.

- [ ] **Step 5: Commit** — `git commit -m "controller: named-pipe target input replaces stdin, reader survives writer EOF"`

---

### Task 3: Controller — hold measured pose at startup

**Files:**
- Modify: `Christian_control/basic_control/src/Main.cpp:398-421` (target injection + banners)
- Modify: `Christian_control/basic_control/src/Config.h` (delete `kFixedTargetM`; preamble)
- Modify: `Christian_control/basic_control/src/Main.cpp:76-78` (preamble line)
- Test: existing suites (characterization; no new behavior to unit-test — the change is one assignment plus deletions)

**Interfaces:** none new. `PoseTargetSource` is constructed with `terminal_target == startup position`; the existing settle/dwell phases handle a zero-length profile.

- [ ] **Step 1: Make the change.**

```cpp
        // The terminal target IS the measured startup pose: the arm holds
        // where it woke up until the first validated pipe waypoint arrives.
        PoseTarget target;
        target.p_desired = ee_now.position;
```

Delete `kFixedTargetM` from `Config.h`. Replace the `fixed_target_m` preamble line (`Main.cpp:76-78`) with `line("startup_hold", "measured");` and the `TERMINAL TARGET` banner with: `HOLD AT START: the arm holds the measured startup pose until the first pipe waypoint; Ctrl+C to stop`.

- [ ] **Step 2: Grep for stragglers** — `grep -rn "kFixedTargetM\|fixed_target_m" Christian_control/basic_control scripts docs` and fix every live reference (analysis scripts reading the preamble key: update `scripts/plot_run.py`-family only if they parse that key — check, and note the finding either way).

- [ ] **Step 3: Build controller + run the full basic_control hardware-free suite** — green, zero warnings in changed files.

- [ ] **Step 4: Commit** — `git commit -m "controller: hold measured startup pose; delete compiled terminal target"`

---

### Task 4: Orientation transport (parse + emit, consumption off)

**Files:**
- Modify: `Christian_control/basic_control/src/Config.h` (add flag), `src/Targets.cpp` (parser)
- Modify: `Christian_control/planner_bridge/src/Waypoints.h/.cpp` (emit overload), `src/BridgeMain.cpp` (`--emit-orientation`)
- Test: `Christian_control/basic_control/tests/test_reactive_law.cpp` (parser cases), `Christian_control/planner_bridge/tests/test_waypoints.cpp` (round-trip)

**Interfaces:**

```cpp
// Config.h
// Stage 1.6 gate: while false, a 7-field target line (x y z qx qy qz qw)
// is REJECTED loudly — orientation is never silently dropped. Enabling
// consumption is a separate reviewed change, blocked on j6 health.
inline constexpr bool kAcceptOrientationTargets = false;
```

```cpp
// Waypoints.h
// "x y z qx qy qz qw", 6 decimals, xyzw — the orientation-carrying form.
std::string FormatTargetLine(const Eigen::Vector3d& position_m,
                             const Eigen::Quaterniond& rotation_xyzw);
```

- [ ] **Step 1: Failing parser tests** (basic_control, `TestParsePoseTarget`, after the existing cases):

```cpp
        // 7-field grammar: parses only when orientation targets are on.
        Check(!ParsePoseTarget("0.4 0.1 0.3 0 0 0 1", error).has_value(),
              "7-field line rejected while kAcceptOrientationTargets is false");
        Check(error.find("orientation targets disabled") != std::string::npos,
              "rejection names the disabled gate");
        Check(!ParsePoseTarget("0.4 0.1 0.3 0 0 0 0.5", error).has_value(),
              "non-unit quaternion rejected");
        Check(!ParsePoseTarget("0.4 0.1 0.3 0 0 1", error).has_value(),
              "six fields rejected");
```

Bridge side, `test_waypoints.cpp`: `FormatTargetLine(p, Quaterniond::Identity())` has 7 space-separated fields, fields 4–7 are `0 0 0 1` to 6 decimals, and the first three match the 3-field overload exactly.

- [ ] **Step 2: RED** — parser test fails ("7-field line" case: current parser rejects with the *wrong message*, so the `error.find` check fails — confirm that exact failure), bridge overload fails to compile.

- [ ] **Step 3: Implement.** New `ParsePoseTarget` body (replaces the trailing-token check):

```cpp
std::optional<PoseTarget> ParsePoseTarget(const std::string& line, std::string& error)
{
    std::istringstream input(line);
    std::vector<double> fields;
    double value = 0.0;
    while (input >> value)
        fields.push_back(value);
    std::string trailing;
    input.clear();
    if (input >> trailing || (fields.size() != 3 && fields.size() != 7)) {
        error = "expected 'x y z' or 'x y z qx qy qz qw'";
        return std::nullopt;
    }
    for (const double field : fields)
        if (!std::isfinite(field)) {
            error = "all target values must be finite";
            return std::nullopt;
        }

    PoseTarget target;
    target.p_desired = Eigen::Vector3d(fields[0], fields[1], fields[2]);
    if (fields.size() == 7) {
        if (!config::kAcceptOrientationTargets) {
            error = "orientation targets disabled "
                    "(config::kAcceptOrientationTargets)";
            return std::nullopt;
        }
        const Eigen::Quaterniond q(fields[6], fields[3], fields[4], fields[5]);
        if (std::abs(q.norm() - 1.0) > 1e-3) {
            error = "quaternion must be unit norm (xyzw)";
            return std::nullopt;
        }
        target.rotation = q.normalized().toRotationMatrix();
    }
    return target;
}
```

(Note `Eigen::Quaterniond`'s constructor takes w first — fields arrive xyzw, so the argument order above is deliberate. The existing "position-only at runtime" comment on `PoseTarget::rotation` in `Targets.h` is updated to name the gate instead.) Keep `ProcessPoseTargetLine`'s queued-message "position-only" wording accurate by printing "(+orientation)" when `target->rotation` is set. Bridge: the overload formats 7 fields with `%.6f`; `RunBridge` uses it when `--emit-orientation` is passed, sourcing each waypoint's quaternion from `ToolPoseInBaseLink(model, q_support).rotation()` at the same support states the sampler kept — simplest wiring: `SampleCartesianWaypoints` gains an optional out-parameter `std::vector<Eigen::Quaterniond>* rotations_xyzw = nullptr` filled at each kept index.

- [ ] **Step 4: GREEN** — both suites fully green. Confirm default bridge output is byte-identical to before (run the e2e test's stdout comparison — no `--emit-orientation`, no 7-field lines).

- [ ] **Step 5: Commit** — `git commit -m "targets: orientation-carrying line grammar behind a disabled gate; bridge can emit it"`

---

### Task 5: Session launcher

**Files:**
- Create: `Christian_control/planner_bridge/scripts/run_session.sh` (mode 755)

**Interfaces:** consumes everything above; no code changes.

- [ ] **Step 1: Write the script.**

```bash
#!/usr/bin/env bash
# One-terminal supervised session for the Stage 1.5 bridge workflow.
# Sequences what the operator previously did by hand; bypasses nothing.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONTROLLER="$REPO/Christian_control/basic_control/build/controller"
BRIDGE="$REPO/Christian_control/planner_bridge/build/planner_bridge"
PIPE="/tmp/humansl_bridge_targets"
ALLOW_STALE=0; DRY_RUN=0
for arg in "$@"; do case "$arg" in
    --allow-stale) ALLOW_STALE=1 ;;
    --dry-run) DRY_RUN=1 ;;
    *) echo "usage: run_session.sh [--allow-stale] [--dry-run]"; exit 1 ;;
esac; done

fresh_or_die() { # $1 binary, $2 source dir
    [[ -x "$1" ]] || { echo "missing binary: $1 — build it first"; exit 1; }
    local stale
    stale=$(find "$2" -name '*.cpp' -o -name '*.h' | xargs -r ls -t | head -1)
    if [[ "$stale" -nt "$1" ]]; then
        echo "STALE: $1 is older than $stale"
        [[ $ALLOW_STALE = 1 ]] || { echo "rebuild, or pass --allow-stale"; exit 1; }
    fi
}
fresh_or_die "$CONTROLLER" "$REPO/Christian_control/basic_control/src"
fresh_or_die "$BRIDGE"     "$REPO/Christian_control/planner_bridge/src"

echo "== Supervised session checklist (project CLAUDE.md) =="
echo "  - Christian present, workspace clear, e-stop in reach"
echo "  - Kinova web dashboard CLOSED (it blocks SetServoingMode)"
echo "  - This run is explicitly authorized"
read -r -p "Type GO to start the controller: " confirm
[[ "$confirm" == "GO" ]] || { echo "aborted"; exit 1; }
[[ $DRY_RUN = 1 ]] && { echo "dry-run: would start $CONTROLLER now"; exit 0; }

SESSION_MARK=$(mktemp)   # anything newer than this was created by THIS session
"$CONTROLLER" & CONTROLLER_PID=$!   # no --log: timestamped default under runs/
trap 'kill -INT $CONTROLLER_PID 2>/dev/null || true; wait $CONTROLLER_PID 2>/dev/null || true; rm -f "$SESSION_MARK"' EXIT

echo "waiting for the controller's run log..."
for _ in $(seq 1 60); do
    LATEST=$(find "$REPO/runs" -name 'loop_log*.csv' -newer "$SESSION_MARK" 2>/dev/null | head -1)
    [[ -n "${LATEST:-}" ]] && break
    kill -0 $CONTROLLER_PID 2>/dev/null || { echo "controller exited during startup"; exit 1; }
    sleep 1
done
[[ -n "${LATEST:-}" ]] || { echo "no run log appeared after 60 s"; exit 1; }
echo "state source: $LATEST"

while read -r -p "bridge> " cmd args; do case "$cmd" in
    goal) # goal X Y Z [box CX CY CZ HX HY HZ]
        read -r gx gy gz maybe_box rest <<<"$args"
        extra=()
        [[ "${maybe_box:-}" == "box" ]] && extra=(--box $rest)
        "$BRIDGE" --goal "$gx" "$gy" "$gz" "${extra[@]}" > "$PIPE" \
            || echo "bridge exited $? — nothing was sent"
        ;;
    quit|q) break ;;
    *) echo "commands: goal X Y Z [box CX CY CZ HX HY HZ] | quit" ;;
esac; done
echo "stopping controller..."
```

The bridge's auto-state discovery (Task 1) will independently find the same newest CSV, so `$LATEST` is informational; the `bridge>` loop deliberately does not pass `--state-csv`.

- [ ] **Step 2: Verify offline** — `bash -n` (syntax), then `--dry-run` twice: once with a deliberately touched source file (must refuse as STALE), once fresh (must reach the GO prompt; type something ≠ GO, confirm abort). Never type GO during verification.

- [ ] **Step 3: Commit** — `git commit -m "planner_bridge: single-terminal session launcher with freshness and GO gates"`

---

### Task 6: Decision record + runbook rewrite

**Files:**
- Create: `Christian_control/docs/decisions/stage15-bridge-workflow.md`
- Modify: `Christian_control/basic_control/README.md` (replace the Stage 1 FIFO runbook)
- Modify: `Christian_control/docs/decisions/stage1-planner-bridge.md` (superseded-by note on the transport section)

**Interfaces:** documentation of Tasks 1–5, validated against final source.

- [ ] **Step 1: Decision record** — why stdin died (unvalidated path + EOF ceremony), why hold-at-startup (planner owns all motion intent), why orientation is transport-only (j6 gate, cite the spec), the pipe reopen design, and the launcher's role (sequencing, not authority — authorization stays with Christian).
- [ ] **Step 2: README runbook** — new flow: `scripts/run_session.sh` → checklist → GO → `goal X Y Z`; keep the supervised-run preconditions verbatim in spirit; note `--dry-run` and `--allow-stale`; delete the `exec 3>` instructions.
- [ ] **Step 3: Verify every cited name/path/flag against built source**, then commit — `git commit -m "docs: stage 1.5 workflow decision record and runbook rewrite"`

---

## Verification of the whole plan (after Task 6)

1. Both suites green: planner_bridge ctest and basic_control's hardware-free tests.
2. `grep -rn "RunPoseTargetInput\b" Christian_control` → only `FromFd`/`FromPipe` remain.
3. `grep -rn "kFixedTargetM" Christian_control` → nothing.
4. Default bridge stdout unchanged vs Stage 1 (no orientation fields without the flag).
5. `run_session.sh --dry-run` behaves per Task 5 Step 2.
6. Supervised hardware validation (hold at startup, pipe survives two bridge runs) — PENDING explicit authorization; report as pending, never as done.
