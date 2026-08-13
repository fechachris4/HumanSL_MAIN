# Read-only Vicon Monitor Implementation Plan

> **Status (2026-08-13): shelved — do not execute this plan.** See the
> banner on its spec (`../specs/2026-08-11-vicon-monitor-design.md`):
> the separate `vicon_monitor` process was dismissed for the control
> path by the 2026-08-12 integration design, and the panel's Vicon
> display now comes from the controller's log (stage 1.5). Kept for the
> panel-diagnostics ideas only.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Build a read-only Vicon monitor in the existing browser panel, backed by a validated sensing-only C++ executable and deterministic replay states.

**Architecture:** vicon_monitor reuses ViconInterface in a dedicated acquisition boundary, validates and publishes one immutable JSON snapshot through atomic file replacement, and never links to Kinova/controller code. The existing Python panel server starts the sensing-only helper, exposes only GET /api/vicon, and renders a new VICON tab with a dependency-free canvas visualisation.

**Tech Stack:** C++17, vendored Vicon DataStream SDK, CMake/CTest, Python 3 standard library, existing http.server panel, browser DOM/Canvas, and existing unittest/static-test conventions.

## Global Constraints

- Reuse ViconInterface; do not duplicate DataStream SDK connection logic.
- Keep Vicon acquisition outside all Kinova control loops and outside UI rendering.
- The monitor is sensing-only: no Kortex/Kinova/controller/session command calls.
- Convert Vicon millimetres to metres exactly once at the validated Vicon data boundary.
- Required segment poses are authoritative for readiness; labelled markers alone never produce READY.
- Default stale threshold is exactly 100 ms and is shown in the UI diagnostics.
- Invalid, occluded, missing, stale, non-finite, or non-normalised poses never appear as valid zero poses.
- Preserve raw quaternion order (x, y, z, w) and display derived roll/pitch/yaw in degrees.
- Expected subject, segment names, marker names, endpoint, and stale threshold are explicit configuration values.
- Do not alter the controller, Kinova safety path, arm commands, Nexus configuration, or marker labels.
- Preserve unrelated uncommitted changes; do not commit or push without explicit project approval.

---

## File map

- Modify vicon/src/ViconInterface.h/.cpp: quiet diagnostics and labelled/unlabelled count accessors.
- Create vicon/src/ViconMonitorModel.h/.cpp: pure configuration, validation, readiness, replay, RPY, unit conversion, frame gaps, and JSON functions.
- Create vicon/tools/vicon_monitor.cpp: live acquisition/retry loop, replay loop, and atomic snapshot publisher.
- Modify vicon/CMakeLists.txt; create vicon/tests/test_vicon_monitor_model.cpp.
- Modify tools/panel/paths.py, tools/panel/server.py, and tools/control_panel.py; create tools/panel/vicon.py and tools/panel/tests/test_vicon.py.
- Modify tools/panel/static/index.html, panel.js, and panel.css; create tools/panel/static/vicon.js.
- Modify tools/panel/tests/test_static.py.

---

## Task 1: Build and test the pure validated-sample model

Files:

- Create vicon/src/ViconMonitorModel.h
- Create vicon/src/ViconMonitorModel.cpp
- Create vicon/tests/test_vicon_monitor_model.cpp
- Modify vicon/CMakeLists.txt

Interfaces:

- Produces MonitorConfig, RawViconFrame, FrameTracker, ViconSample, validateFrame(), makeDisconnectedSample(), makeReplayFrame(), readinessText(), and sampleToJson().
- The model library is plain C++ and must not include or link the DataStream SDK.

### Step 1: Define the model contract

Add explicit configuration defaults:

- endpoint: 192.168.128.206:801
- subject_name: Dr Octopus Christian
- stale_threshold: 100 ms
- expected_segments: Mount, LeftBase, RightBase, LeftEE, RightEE
- expected_markers: Mount1, Mount2; LeftBase1-4; RightBase1-4; LeftEE1-5; RightEE1-5

Define Readiness as NoConnection, SegmentsUnavailable, and
ReadyForPoseCalibration. Define ReplayState as Disconnected, Occluded, Valid,
and Stale; the executable's all mode cycles those four values. Define
FrameTracker with an optional previous frame number, cumulative skipped-frame
count, and an advancing flag.

Raw inputs must contain connected, frame number, server rate, local receive timestamp, labelled/unlabelled totals, raw markers, and raw segments. Raw marker positions and raw segment translations remain millimetres. Raw segment data includes subject/segment names, present/occluded flags, x/y/z millimetres, quaternion x/y/z/w, and optional sample age.

Validated segment data uses optional position_m, quaternion_xyzw, and rpy_deg arrays. Invalid optionals remain empty. Each row also stores present, visible, fresh, valid, sample_age, and an explicit reason. ViconSample stores connection/readiness, frame/rate/age/gap counters, timestamp, configuration echo, marker groups, and all five expected segments.

The serialized ViconSample fields are stable: connected, readiness,
diagnostic_reason, endpoint, subject_name, server_frame_rate_hz,
frame_number, received_frame_rate_hz, received_at_ms, frame_age_ms,
skipped_frames, stale_threshold_ms, labelled_marker_count,
unlabeled_marker_count, markers, and segments. Each marker includes name,
group, present, occluded, position_m, and reason. Each segment includes name,
present, visible, fresh, valid, position_m, quaternion_xyzw, rpy_deg,
sample_age_ms, and reason. Numeric values that are not valid are JSON null.

### Step 2: Write failing tests

Test these before implementing the model:

1. An occluded segment is invalid, has no pose optionals, and says occluded.
2. A connected segment aged 101 ms under a 100 ms threshold is stale and invalid.
3. Translation 1000.0, -250.0, 12.5 mm becomes 1.0, -0.25, 0.0125 m.
4. NaN, zero-norm, and non-normalised quaternions are rejected.
5. Normalised quaternion 0,0,0,1 is accepted with finite zero RPY degrees.
6. Frame numbers 100, 101, 104, 104 produce exactly two cumulative skipped frames.
7. Connected with all five segments occluded produces NOT READY: segments unavailable.
8. Connected with all five valid/fresh/advancing segments produces READY FOR POSE CALIBRATION.
9. Disconnected produces NOT READY: no connection.
10. Invalid pose fields serialize as JSON null and raw quaternion order is x/y/z/w.

Run the intended failing baseline:

    cmake -S vicon -B vicon/build
    cmake --build vicon/build --target test_vicon_monitor_model --parallel 2

Expected: compilation fails because the new model target does not exist.

### Step 3: Implement and test the model

Implement:

- validateFrame(config, raw, tracker)
- makeDisconnectedSample(config, now_ms, reason)
- makeReplayFrame(config, replay_state, now_ms, frame_number)
- readinessText(readiness)
- sampleToJson(sample)

Require present, not occluded, finite translation/quaternion, quaternion norm within 1e-3 of one, and age no greater than the configured threshold. Do not silently normalise. Convert millimetres to metres in one helper only. Use ZYX yaw-pitch-roll formulas for x/y/z/w, clamp the pitch input to [-1,1], and convert radians to degrees. READY requires connection, advancing frames, fresh data, and all five valid segments; marker labels never override an unavailable segment pose.

Register the vicon_monitor_model static library and CTest test, then run:

    cmake --build vicon/build --target test_vicon_monitor_model --parallel 2
    ctest --test-dir vicon/build --output-on-failure -R '^vicon_monitor_model$'

Expected: PASS without contacting Vicon.

---

## Task 2: Extend the Vicon wrapper and create vicon_monitor

Files:

- Modify vicon/src/ViconInterface.h
- Modify vicon/src/ViconInterface.cpp
- Create vicon/tools/vicon_monitor.cpp
- Modify vicon/CMakeLists.txt
- Modify vicon/tests/test_segment_data.cpp

### Step 1: Add the wrapper seam

Change the constructor to explicit ViconInterface(bool quiet = false). Gate only the existing connection/disconnection messages with quiet; default false preserves connect_vicon output.

Add read-only methods:

    unsigned int getLabeledMarkerCount() const;
    unsigned int getUnlabeledMarkerCount() const;

Return zero while disconnected; otherwise forward the vendor SDK count methods. Add no SDK write/configuration calls. Extend the existing offline wrapper test to assert both counts are zero and getSegmentPoses() is empty.

### Step 2: Define executable options

Support:

- --host HOST:PORT, default 192.168.128.206:801
- --subject SUBJECT, default Dr Octopus Christian
- --stale-ms N, default 100, positive only
- --snapshot PATH, default /tmp/humansl_vicon_snapshot.json
- --replay STATE, disconnected|occluded|valid|stale|all
- --interval-ms N, default 100 for replay, positive only
- --frames N, optional finite replay count

Live mode is default. Replay mode must not instantiate or call ViconInterface.

### Step 3: Implement live acquisition

Use one dedicated loop. Publish an initial disconnected sample, retry failed connections after one second, call getFrame(), then read frame number/rate, the configured expected marker list, labelled/unlabelled totals, and existing segment results. Filter segments by configured subject/name. Timestamp each received frame with system_clock. On acquisition failure disconnect and publish a disconnected reason. Do not print raw marker output.

### Step 4: Implement replay

Use makeReplayFrame() and the same validation/serialization path:

- disconnected: no connection
- occluded: connected, 20 labelled markers, all five segments occluded
- valid: all five valid and moving with finite normalised quaternions
- stale: LeftBase sample age 250 ms, all others valid
- all: ten frames of each state, repeating unless --frames ends it

Replay emits no terminal stream; it only updates the snapshot.

### Step 5: Publish atomically and register targets

Write JSON to a sibling temporary file, flush and close it, then replace the snapshot with filesystem rename. Use a process-specific temporary name and create the parent directory if needed. A UI reader must see either the previous complete snapshot or the new complete snapshot.

Add vicon_monitor linking only vicon_monitor_model and vicon_interface, with the existing Vicon SDK RPATH. Keep connect_vicon and existing tests.

Run:

    cmake -S vicon -B vicon/build
    cmake --build vicon/build --target vicon_monitor connect_vicon test_segment_data test_vicon_monitor_model --parallel 2
    ctest --test-dir vicon/build --output-on-failure

---

## Task 3: Add the read-only Python bridge and lifecycle

Files:

- Modify tools/panel/paths.py
- Create tools/panel/vicon.py
- Modify tools/panel/server.py
- Modify tools/control_panel.py
- Create tools/panel/tests/test_vicon.py

### Step 1: Test the bridge without launching a process

Define and test:

- configure(replay, endpoint, subject, stale_ms)
- start()
- stop()
- snapshot(now_ms=None)
- replay_active()

Patch subprocess.Popen in tests. Assert missing/malformed snapshots return NOT READY: no connection; fresh JSON is returned; an old helper snapshot is downgraded to disconnected; Popen receives an argument list with no shell; --replay is passed only when requested; and stop terminates only the stored Vicon helper process.

### Step 2: Add paths and implementation

Add:

- VICON_PROJECT = CHRISTIAN_CONTROL / vicon
- VICON_BIN = VICON_PROJECT / build / vicon_monitor
- VICON_SNAPSHOT = /tmp/humansl_vicon_snapshot.json

Use Popen([...], stdout=DEVNULL, stderr=DEVNULL, start_new_session=True). Pass host, subject, stale threshold, snapshot path, and optional replay. Read bounded UTF-8 JSON and reject non-object values. If snapshot age exceeds max(stale_ms * 3, 1000), return disconnected instead of retaining a valid pose. If the executable is missing, return disconnected with the build command.

### Step 3: Add endpoint and launcher options

Route only this GET in server.py:

    elif route == "/api/vicon":
        self._json(vicon.snapshot())

Do not add a Vicon POST route. Start/configure the helper before serve_forever() and stop it in the existing finally block.

Add to control_panel.py:

- --vicon-host, default 192.168.128.206:801
- --vicon-subject, default Dr Octopus Christian
- --vicon-stale-ms, default 100, reject values <= 0
- --vicon-replay, choices disconnected|occluded|valid|stale|all

Preserve the existing controller --replay argument and pass both replay systems independently to server.serve().

Run:

    python3 -m unittest tools.panel.tests.test_vicon -v

Expected: all tests pass without Vicon, panel, controller, or robot processes.

---

## Task 4: Add the VICON graphical tab

Files:

- Modify tools/panel/static/index.html
- Modify tools/panel/static/panel.js
- Create tools/panel/static/vicon.js
- Modify tools/panel/static/panel.css
- Modify tools/panel/tests/test_static.py

### Step 1: Add stable markup

Add a VICON tab and view-vicon with IDs:

- vicon-readonly-mark
- vicon-connection
- vicon-state
- vicon-reason
- vicon-endpoint
- vicon-server-rate
- vicon-frame
- vicon-received-rate
- vicon-age
- vicon-skipped
- vicon-threshold
- vicon-segment-table
- vicon-marker-diagnostics
- vicon-frame-diagnostics
- vicon-canvas
- vicon-canvas-note

The view has a sticky header with connection, endpoint, server rate, latest frame, receive rate, age, skipped frames, threshold, and exact readiness text. Use details for marker and frame diagnostics. Add no Vicon form, write, start, stop, or controller action; retain the existing global STOP control.

### Step 2: Poll only the read endpoint

Import renderVicon from ./vicon.js, add state.vicon, and poll GET /api/vicon every 250 ms. On fetch failure use a disconnected fallback. Do not connect Vicon polling to arm SSE or any POST action.

### Step 3: Render the required data

renderVicon(sample) must use textContent, show exact connection/readiness strings, calculate age from received_at_ms, render all five rows in expected order, and show metres/RPY/raw x-y-z-w quaternion. Invalid rows show OCCLUDED, NO SEGMENT POSE, STALE, INVALID QUATERNION, or the reason; they never show zero as a pose.

Render marker groups/counts/positions, labelled and unlabelled totals, frame diagnostics, the Vicon-world-frame statement, and T_world_cluster. Use green for valid/fresh, amber for connected stale/incomplete, and red for disconnected/occluded/missing/invalid. Keep invalid segment geometry out of the canvas.

### Step 4: Render the visual aid

Use a dependency-free 2D canvas with a documented orthographic/isometric projection. Draw the Vicon world origin and X/Y/Z axes. For every valid segment, compute a labelled triad from metre position and raw x/y/z/w quaternion, with distinct segment accents. Fit to valid positions. If none are valid, draw axes only and the note no valid segment poses; never draw an invalid segment at the origin. Resize when the view appears and on window resize; do not run an animation loop.

### Step 5: Add CSS and static checks

Add scoped Vicon table/canvas/status/sticky-header styles, explicit green/amber/red tokens, tabular numbers, responsive collapse at the existing 900px breakpoint, and no external dependencies.

Update test_static.py to assert ./vicon.js wiring, matching tab/view, all Vicon IDs, /api/vicon routing, GET-only Vicon access, exact readiness and read-only text, no external URLs, and no silent invalid-to-zero rendering.

Run:

    python3 -m unittest tools.panel.tests.test_static -v

---

## Task 5: Integrate and verify replay/live behaviour

### Step 1: Run complete hardware-free verification

    cmake -S vicon -B vicon/build
    cmake --build vicon/build --target vicon_monitor connect_vicon test_segment_data test_vicon_monitor_model --parallel 2
    ctest --test-dir vicon/build --output-on-failure
    python3 -m unittest discover -s tools/panel/tests -p 'test_*.py' -v
    git diff --check

Expected: all tests pass; no robot, Kinova arm, or Nexus write operation is performed.

### Step 2: Verify replay through the named launch path

Start:

    python3 tools/control_panel.py --port 8876 --vicon-replay all

Poll from another terminal:

    curl --fail --silent http://127.0.0.1:8876/api/vicon

Repeat the curl at 250 ms intervals while all runs. Observed states must include disconnected, occluded/segments-unavailable, valid/ready, and stale LeftBase/segments-unavailable. Open http://127.0.0.1:8876/, select VICON, and confirm the table/canvas render without raw-marker terminal output.

Run each fixed state separately with --vicon-replay disconnected, --vicon-replay occluded, --vicon-replay valid, and --vicon-replay stale. Confirm occluded has 20 markers, five red rows, NOT READY: segments unavailable, and no valid zero pose. Confirm valid has five green rows, metres, RPY, and triads. Confirm stale has amber LeftBase and no READY.

### Step 3: Verify the live sensing-only path

    timeout 15s vicon/build/vicon_monitor --host 192.168.128.206:801 --subject 'Dr Octopus Christian' --stale-ms 100 --snapshot /tmp/humansl_vicon_live_test.json
    python3 -m json.tool /tmp/humansl_vicon_live_test.json

If the live Nexus state is unchanged, report 100 Hz, 20 labelled markers, and five occluded segments as NOT READY: segments unavailable; do not fabricate readiness or alter Nexus settings.

### Step 4: Trace the no-command boundary and review the diff

    rg -n "Kinova|Kortex|controller|session/start|session/stop|target_pipe|write|Enable|Set|Command" vicon/tools/vicon_monitor.cpp vicon/src/ViconMonitorModel.* tools/panel/vicon.py
    ldd vicon/build/vicon_monitor
    git status --short
    git diff --stat
    git diff --check
    git diff -- vicon tools/control_panel.py tools/panel docs/superpowers

The source trace must end at Vicon read calls and snapshot/file operations; the executable must link to vicon_interface and the Vicon SDK, not Kortex or controller libraries. Final handoff must list files, launch commands, readiness meanings, exact commands/results, live result, and any remaining Nexus blocker.

Do not commit, push, start a controller, or operate either arm.
